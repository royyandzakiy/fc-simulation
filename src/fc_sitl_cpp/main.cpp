// fc_min.cpp - host side: SDL3 gamepad, CLI11 args, fmt logging.
//
//   conan install . --output-folder=build --build=missing -s compiler.cppstd=17
//   cmake --preset conan-default
//   cmake --build build --config Release
//
//   fc_min                          pipes, first pad SDL finds
//   fc_min --throttle trigger       right trigger instead of left stick
//   fc_min --dev COM7               serial instead of pipes
//   fc_min --list                   show detected pads and exit
//
// Portable across clang-cl, MSVC, GCC and Clang. All OS-specific I/O is in
// platform.hpp; all flight logic is in fc_core.hpp. This file is glue.

#include "fc_core.hpp"
#include "platform.hpp"
#include <SDL3/SDL_error.h>
#include <SDL3/SDL_events.h>
#include <SDL3/SDL_gamepad.h>
#include <SDL3/SDL_hints.h>
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_joystick.h>
#include <SDL3/SDL_stdinc.h>
#include <SDL3/SDL_timer.h>
#include <array>
#include <cstdint>
#include <cstdio>
#include <fmt/base.h>
#include <optional>
#include <utility>

// SDL3 hijacks main() unless told not to. We want a plain console entry
// point, so we handle main ourselves and call SDL_SetMainReady().
#define SDL_MAIN_HANDLED
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <CLI/CLI.hpp>
#include <fmt/core.h>

#include <string>

namespace {

// -------------------------------------------------------------------
// Gamepad - SDL3.
//
// SDL remaps every known pad onto one virtual layout via its controller
// database, so LEFTX is LEFTX whether the pad is xpad, xone, XInput,
// wireless, or a PlayStation clone. No per-driver calibration step.
//
// SDL3 renamed most of this from SDL2: SDL_GameController -> SDL_Gamepad,
// SDL_CONTROLLER_BUTTON_B -> SDL_GAMEPAD_BUTTON_EAST, and SDL_Init now
// returns bool (true on success) rather than int (0 on success).
// -------------------------------------------------------------------

// Debug-only thresholds, same as the standalone dpad.cpp printer.
constexpr float kFull = 0.90f; // "at max" threshold (0..1)
constexpr float kDead = 0.20f; // ignore noise below this
constexpr int kAxisMax = 32767;

// Button + last state, so we only print on change.
struct Btn {
	const char *name;
	SDL_GamepadButton id;
	bool last = false;
};

class Gamepad {
  public:
	static constexpr float kDeadband = 0.08f;

	enum class Throttle {
		LeftStick,
		RightTrigger
	};

	explicit Gamepad(Throttle mode) noexcept : mode_{mode} {
		// No window here, so SDL would otherwise ignore the pad for lack
		// of input focus. This hint is what makes headless work.
		SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");

		// SDL enumerates joysticks through udev on Linux, and WSL runs no udev,
		// so without this it finds nothing at all even though /dev/input/js0 is
		// sitting right there. Makes SDL scan /dev/input directly. Ignored on
		// platforms where it does not apply.
		SDL_SetHint(SDL_HINT_JOYSTICK_LINUX_CLASSIC, "1");

		SDL_SetMainReady();

		if (!SDL_Init(SDL_INIT_GAMEPAD)) { // SDL3: true means success
			fmt::print(stderr, "sdl: init failed: {}\n", SDL_GetError());
			return;
		}
		sdl_ready_ = true;

		int count = 0;
		SDL_JoystickID *ids = SDL_GetGamepads(&count);
		if (ids) {
			for (int i = 0; i < count; ++i) {
				pad_ = SDL_OpenGamepad(ids[i]);
				if (pad_) {
					fmt::print(stderr, "pad: {}\n", SDL_GetGamepadName(pad_));
					break;
				}
			}
			SDL_free(ids);
		}
		if (!pad_)
			fmt::print(stderr, "pad: none found, staying disarmed\n");
	}

	~Gamepad() {
		if (pad_)
			SDL_CloseGamepad(pad_);
		if (sdl_ready_)
			SDL_Quit();
	}

	Gamepad(const Gamepad &) = delete;
	Gamepad &operator=(const Gamepad &) = delete;

	[[nodiscard]] bool present() const noexcept {
		return pad_ != nullptr;
	}

	// Refresh device state. SDL_UpdateGamepads() on its own is not enough
	// on Windows: the RAWINPUT driver receives device data as window
	// messages, and only SDL_PumpEvents() drains the message queue, so
	// without this the axes sit at zero forever. Draining the event queue
	// also stops it growing without bound over a long run.
	static void pump() noexcept {
		SDL_Event e;
		while (SDL_PollEvent(&e)) { // implies SDL_PumpEvents()
		}
		SDL_UpdateGamepads();
	}

	// Refresh pad state without an event loop. Never blocks, so the
	// lockstep control loop keeps its cadence regardless of pad activity.
	fc::Sticks poll() noexcept {
		fc::Sticks s{};

		if (!pad_) {
			gate_.force_disarm();
			return s;
		}

		pump();

		if (!SDL_GamepadConnected(pad_)) { // unplugged mid-flight
			fmt::print(stderr, "pad: disconnected, failsafe\n");
			SDL_CloseGamepad(pad_);
			pad_ = nullptr;
			gate_.force_disarm();
			return s;
		}

		s.roll = norm(axis(SDL_GAMEPAD_AXIS_RIGHTX));
		s.pitch = -norm(axis(SDL_GAMEPAD_AXIS_RIGHTY)); // up is negative
		s.yaw = norm(axis(SDL_GAMEPAD_AXIS_LEFTX));

		if (mode_ == Throttle::RightTrigger) {
			// triggers are one-sided 0..32767 and stay where you put them
			s.throttle = clamp01(axis(SDL_GAMEPAD_AXIS_RIGHT_TRIGGER) / 32767.0f);
		} else {
			// spring-centered: rests at 0.5, near CF2X hover thrust
			s.throttle = clamp01((1.0f - axis(SDL_GAMEPAD_AXIS_LEFTY) / 32767.0f) * 0.5f);
		}

		// EAST is the Xbox B button in SDL3's cardinal naming
		s.armed = gate_.update(button(SDL_GAMEPAD_BUTTON_LEFT_SHOULDER), button(SDL_GAMEPAD_BUTTON_EAST), s.throttle);
		if (gate_.refused()) {
			fmt::print(stderr, "arm refused - throttle is {:.0f}%, pull it all the way down\n",
					   static_cast<double>(s.throttle) * 100.0);
		}
		if (!s.armed)
			s.throttle = 0.0f;

		return s;
	}

	// ---------------------------------------------------------------
	// Debug printer: mirrors the standalone dpad.cpp behavior.
	// Call once per frame from the main loop instead of / in addition
	// to poll(). Reads raw SDL state directly, bypassing fc::Sticks,
	// so it can show every button and every axis edge.
	// ---------------------------------------------------------------
	void debug_print() {
		if (!pad_)
			return;

		pump();

		if (!SDL_GamepadConnected(pad_)) {
			fmt::println("\npad: disconnected");
			return;
		}

		// ---- buttons: report on change ----
		for (auto &b : buttons_) {
			const bool now = SDL_GetGamepadButton(pad_, b.id);
			if (now != b.last) {
				fmt::println("button {:<14} {}", b.name, now ? "DOWN" : "up");
				b.last = now;
			}
		}

		// ---- triggers: report on full squeeze ----
		const Sint16 lt = SDL_GetGamepadAxis(pad_, SDL_GAMEPAD_AXIS_LEFT_TRIGGER);
		const Sint16 rt = SDL_GetGamepadAxis(pad_, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER);
		if (static_cast<float>(lt) > kAxisMax * kFull && !axis_reported_.at(SDL_GAMEPAD_AXIS_LEFT_TRIGGER)) {
			fmt::println("trigger left       MAX ({})", lt);
			axis_reported_.at(SDL_GAMEPAD_AXIS_LEFT_TRIGGER) = true;
		} else if (static_cast<float>(lt) < kAxisMax * kDead) {
			axis_reported_.at(SDL_GAMEPAD_AXIS_LEFT_TRIGGER) = false;
		}
		if (static_cast<float>(rt) > kAxisMax * kFull && !axis_reported_.at(SDL_GAMEPAD_AXIS_RIGHT_TRIGGER)) {
			fmt::println("trigger right      MAX ({})", rt);
			axis_reported_.at(SDL_GAMEPAD_AXIS_RIGHT_TRIGGER) = true;
		} else if (static_cast<float>(rt) < kAxisMax * kDead) {
			axis_reported_.at(SDL_GAMEPAD_AXIS_RIGHT_TRIGGER) = false;
		}

		// ---- sticks: report per-axis when pushed to max ----
		struct Sticks {
			const char *name;
			SDL_GamepadAxis axis;
		};

		std::array<Sticks, 4> sticks{
			Sticks{.name = "left stick  X", .axis = SDL_GAMEPAD_AXIS_LEFTX},
			Sticks{.name = "left stick  Y", .axis = SDL_GAMEPAD_AXIS_LEFTY},
			Sticks{.name = "right stick X", .axis = SDL_GAMEPAD_AXIS_RIGHTX},
			Sticks{.name = "right stick Y", .axis = SDL_GAMEPAD_AXIS_RIGHTY},
		};
		for (auto &s : sticks) {
			const float v = static_cast<float>(SDL_GetGamepadAxis(pad_, s.axis)) / 32767.0f;
			const bool full = (v > kFull) || (v < -kFull);
			const int idx = s.axis; // unique per physical axis
			if (full && !axis_reported_.at(idx)) {
				const char *dir = (v > 0) ? "+" : "-";
				fmt::println("axis    {:<14} MAX {} ({:.2f})", s.name, dir, v);
				axis_reported_.at(idx) = true;
			} else if (!full && v > -kDead && v < kDead) {
				axis_reported_.at(idx) = false;
			}
		}
	}

	static void list() {
		SDL_SetMainReady();
		if (!SDL_Init(SDL_INIT_GAMEPAD)) {
			fmt::print(stderr, "sdl: {}\n", SDL_GetError());
			return;
		}
		int count = 0;
		SDL_JoystickID *ids = SDL_GetJoysticks(&count);
		if (ids) {
			for (int i = 0; i < count; ++i) {
				fmt::print("{}: {}{}\n", i, SDL_GetJoystickNameForID(ids[i]),
						   SDL_IsGamepad(ids[i]) ? "" : "  (not a gamepad)");
			}
			SDL_free(ids);
		}
		SDL_Quit();
	}

  private:
	[[nodiscard]] float axis(SDL_GamepadAxis a) const noexcept {
		return static_cast<float>(SDL_GetGamepadAxis(pad_, a));
	}
	[[nodiscard]] bool button(SDL_GamepadButton b) const noexcept {
		return SDL_GetGamepadButton(pad_, b);
	}

	static float clamp01(float v) noexcept {
		return (v < 0.0f) ? 0.0f : (v > 1.0f) ? 1.0f : v;
	}

	static float norm(float raw) noexcept // -32768..32767 -> -1..1
	{
		float f = raw / 32767.0f;
		if (f > 1.0f)
			f = 1.0f;
		if (f < -1.0f)
			f = -1.0f;
		if (f > -kDeadband && f < kDeadband)
			return 0.0f;
		// rescale so the stick still reaches full travel past the deadband
		return (f > 0.0f) ? (f - kDeadband) / (1.0f - kDeadband) : (f + kDeadband) / (1.0f - kDeadband);
	}

	SDL_Gamepad *pad_{nullptr};
	bool sdl_ready_{false};
	Throttle mode_;
	fc::ArmingGate gate_{};

	// Debug printer state.
	std::array<Btn, 21> buttons_{Btn{.name = "south (A)", .id = SDL_GAMEPAD_BUTTON_SOUTH},
								 Btn{.name = "east (B)", .id = SDL_GAMEPAD_BUTTON_EAST},
								 Btn{.name = "west (X)", .id = SDL_GAMEPAD_BUTTON_WEST},
								 Btn{.name = "north (Y)", .id = SDL_GAMEPAD_BUTTON_NORTH},
								 Btn{.name = "back", .id = SDL_GAMEPAD_BUTTON_BACK},
								 Btn{.name = "guide", .id = SDL_GAMEPAD_BUTTON_GUIDE},
								 Btn{.name = "start", .id = SDL_GAMEPAD_BUTTON_START},
								 Btn{.name = "left stick", .id = SDL_GAMEPAD_BUTTON_LEFT_STICK},
								 Btn{.name = "right stick", .id = SDL_GAMEPAD_BUTTON_RIGHT_STICK},
								 Btn{.name = "left shoulder", .id = SDL_GAMEPAD_BUTTON_LEFT_SHOULDER},
								 Btn{.name = "right shoulder", .id = SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER},
								 Btn{.name = "dpad up", .id = SDL_GAMEPAD_BUTTON_DPAD_UP},
								 Btn{.name = "dpad down", .id = SDL_GAMEPAD_BUTTON_DPAD_DOWN},
								 Btn{.name = "dpad left", .id = SDL_GAMEPAD_BUTTON_DPAD_LEFT},
								 Btn{.name = "dpad right", .id = SDL_GAMEPAD_BUTTON_DPAD_RIGHT},
								 Btn{.name = "misc1", .id = SDL_GAMEPAD_BUTTON_MISC1},
								 Btn{.name = "paddle1", .id = SDL_GAMEPAD_BUTTON_RIGHT_PADDLE1},
								 Btn{.name = "paddle2", .id = SDL_GAMEPAD_BUTTON_LEFT_PADDLE1},
								 Btn{.name = "paddle3", .id = SDL_GAMEPAD_BUTTON_RIGHT_PADDLE2},
								 Btn{.name = "paddle4", .id = SDL_GAMEPAD_BUTTON_LEFT_PADDLE2},
								 Btn{.name = "touchpad", .id = SDL_GAMEPAD_BUTTON_TOUCHPAD}};
	std::array<bool, SDL_GAMEPAD_AXIS_COUNT> axis_reported_{};
};

// ===================================================================
// Radio - a transmitter in USB HID joystick mode.
//
// Not a gamepad. SDL_GetGamepads() only returns devices it has a mapping
// for in its controller database, and a radio has no entry there, so it
// never shows up that way. The joystick API sees it as what it is: a
// pile of unlabelled axes.
//
// Which axis is which depends on the model config in the radio (AETR vs
// TAER and so on), so the mapping is flags rather than a guess. Use
// --debug-pad --input joystick to find out what yours reports.
// ===================================================================

class Radio {
  public:
	struct Map {
		int roll{0};
		int pitch{1};
		int throttle{2};
		int yaw{3};
		int arm{4};
		float arm_threshold{0.5f};
		float arm_hold_s{0.2f};
	};

	explicit Radio(Map m) noexcept : map_{m} {
		gate_.set_arm_hold(map_.arm_hold_s);
		SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");

		// SDL enumerates joysticks through udev on Linux, and WSL runs no udev,
		// so without this it finds nothing at all even though /dev/input/js0 is
		// sitting right there. Makes SDL scan /dev/input directly. Ignored on
		// platforms where it does not apply.
		SDL_SetHint(SDL_HINT_JOYSTICK_LINUX_CLASSIC, "1");
		SDL_SetMainReady();

		if (!SDL_Init(SDL_INIT_JOYSTICK)) {
			fmt::print(stderr, "sdl: init failed: {}\n", SDL_GetError());
			return;
		}
		sdl_ready_ = true;

		int count = 0;
		SDL_JoystickID *ids = SDL_GetJoysticks(&count);
		if (ids) {
			for (int i = 0; i < count; ++i) {
				js_ = SDL_OpenJoystick(ids[i]);
				if (js_) {
					fmt::print(stderr, "radio: {} ({} axes)\n", SDL_GetJoystickName(js_),
							   SDL_GetNumJoystickAxes(js_));
					break;
				}
			}
			SDL_free(ids);
		}
		if (!js_)
			fmt::print(stderr, "radio: none found, staying disarmed\n");
	}

	~Radio() {
		if (js_)
			SDL_CloseJoystick(js_);
		if (sdl_ready_)
			SDL_Quit();
	}

	Radio(const Radio &) = delete;
	Radio &operator=(const Radio &) = delete;

	[[nodiscard]] bool present() const noexcept {
		return js_ != nullptr;
	}

	fc::Sticks poll(float dt) noexcept {
		fc::Sticks s{};
		if (!js_) {
			gate_.force_disarm();
			return s;
		}

		pump();

		if (!SDL_JoystickConnected(js_)) {
			fmt::print(stderr, "radio: disconnected, failsafe\n");
			SDL_CloseJoystick(js_);
			js_ = nullptr;
			gate_.force_disarm();
			return s;
		}

		s.roll = norm(axis(map_.roll));
		s.pitch = norm(axis(map_.pitch));
		s.yaw = norm(axis(map_.yaw));

		// A radio throttle does not self-centre, so it gets the full travel
		// with no centre dead band: stick down is zero, stick up is full.
		s.throttle = clamp01((axis(map_.throttle) + 1.0f) * 0.5f);

		// Schmitt trigger on the arm channel. A plain "> threshold" compare
		// chatters when the axis happens to rest near the threshold: one
		// count of noise either way flips it, and at 240 Hz that reads as
		// ARMED/disarmed alternating several times a second. Two separate
		// levels with a gap between them means noise smaller than the gap
		// cannot cross both.
		const float v = axis(map_.arm);
		if (v > map_.arm_threshold + kArmHysteresis) {
			arm_sw_ = true;
		} else if (v < map_.arm_threshold - kArmHysteresis) {
			arm_sw_ = false;
		} // in between: hold whatever it was

		s.armed = gate_.update_level(arm_sw_, false, s.throttle, dt);
		if (gate_.refused()) {
			fmt::print(stderr, "arm refused - throttle is {:.0f}%, bring it down first\n",
					   static_cast<double>(s.throttle) * 100.0);
		}
		if (!s.armed)
			s.throttle = 0.0f;

		return s;
	}

	// Every axis, live. This is how you find out which channel is which.
	void debug_print() noexcept {
		if (!js_)
			return;
		pump();
		if (!SDL_JoystickConnected(js_)) {
			fmt::println("radio: disconnected");
			return;
		}
		const int n = SDL_GetNumJoystickAxes(js_);
		std::string line;
		for (int i = 0; i < n; ++i) {
			line += fmt::format("a{}={:+.2f}  ", i, static_cast<double>(axis(i)));
		}
		fmt::print("\r{}", line);
		std::fflush(stdout);
	}

	static void pump() noexcept {
		SDL_Event e;
		while (SDL_PollEvent(&e)) {
		}
		SDL_UpdateJoysticks();
	}

  private:
	// Raw axis as -1..1. Out-of-range indices read as centred rather than
	// crashing, so a bad --ch-* flag is a dead channel and not a fault.
	[[nodiscard]] float axis(int idx) const noexcept {
		if (!js_ || idx < 0 || idx >= SDL_GetNumJoystickAxes(js_))
			return 0.0f;
		float f = static_cast<float>(SDL_GetJoystickAxis(js_, idx)) / 32767.0f;
		if (f > 1.0f)
			f = 1.0f;
		if (f < -1.0f)
			f = -1.0f;
		return f;
	}

	static float clamp01(float v) noexcept {
		return (v < 0.0f) ? 0.0f : (v > 1.0f) ? 1.0f : v;
	}

	// The radio has already calibrated and trimmed its own sticks, so this
	// only needs to kill the last bit of jitter around centre.
	static float norm(float f) noexcept {
		constexpr float kDead = 0.03f;
		if (f > -kDead && f < kDead)
			return 0.0f;
		return (f > 0.0f) ? (f - kDead) / (1.0f - kDead) : (f + kDead) / (1.0f - kDead);
	}

	// Gap either side of the arm threshold, in axis units. Wide enough to
	// swallow channel noise, far narrower than a real switch throw.
	static constexpr float kArmHysteresis = 0.15f;

	SDL_Joystick *js_{nullptr};
	bool sdl_ready_{false};
	bool arm_sw_{false};
	Map map_;
	fc::ArmingGate gate_{};
};

// ===================================================================
// Scripted sticks: a canned sequence driven by simulated time, so the
// loop can be flown end to end with no hands and no gamepad. Time comes
// from the sensor packet's dt, not a wall clock, so it stays in lockstep
// and reproduces exactly.
// ===================================================================

class ScriptedSticks {
  public:
	explicit ScriptedSticks(std::string name) noexcept : name_{std::move(name)} {
	}

	fc::Sticks poll(float dt) noexcept {
		t_ += dt;
		fc::Sticks s{};

		// A tap, not a hold: press at 0.5 s, release at 0.55 s. Arming toggles
		// on the press edge, so it stays armed after the release - which is
		// the whole point of the tap, and what hold-to-arm got wrong.
		const bool arm_btn = (t_ >= 0.5f && t_ < 0.55f);
		float throttle = 0.0f;

		if (name_ == "arm-hover") {
			throttle = (t_ < 0.6f) ? 0.0f : 0.62f;
		} else { // arm-climb-roll
			throttle = (t_ < 0.6f) ? 0.0f : 0.75f;
			if (t_ >= 3.0f)
				s.roll = 0.5f;
		}

		s.throttle = throttle;
		s.armed = gate_.update(arm_btn, false, throttle);
		if (!s.armed)
			s.throttle = 0.0f;
		return s;
	}

  private:
	std::string name_;
	float t_{0.0f};
	fc::ArmingGate gate_{};
};

} // namespace

// ===================================================================

int main(int argc, char **argv) {
	// Must happen before any byte moves. On Windows the CRT would
	// otherwise translate 0x0A to 0x0D 0x0A and corrupt every packet
	// containing that byte, which floats do constantly.
	plat::set_binary_stdio();

	CLI::App app{"minimum viable flight controller, lockstep HIL"};

	std::string io_dev;
	std::string throttle = "stick";
	std::string script;
	std::string input = "auto";
	int baud = 115200;
	Radio::Map chmap{};
	int arm_hold_ms = 200;
	bool do_list = false;
	bool debug_pad = false;
	bool no_pad = false;

	app.add_option("--dev", io_dev,
				   "serial device, e.g. COM7 or /dev/ttyUSB0 "
				   "(default: stdin/stdout pipes)");
	app.add_option("--throttle", throttle, "throttle source")->check(CLI::IsMember({"stick", "trigger"}));
	app.add_option("--baud", baud,
				   "serial rate for --dev. 115200 cannot carry 240 Hz lockstep: "
				   "22 B out plus 39 B back serialises to 5.3 ms per exchange, "
				   "against a 4.17 ms period. 230400 is the first rate that fits")
		->capture_default_str();
	app.add_flag("--list", do_list, "list detected gamepads and exit");
	app.add_flag("--debug-pad", debug_pad,
				 "stream gamepad state to the console instead of flying");
	app.add_flag("--no-pad", no_pad, "ignore any gamepad: neutral sticks, never arms");
	app.add_option("--input", input,
				   "stick source: auto (gamepad, else radio), gamepad, joystick")
		->check(CLI::IsMember({"auto", "gamepad", "joystick"}));
	app.add_option("--ch-roll", chmap.roll, "radio axis carrying roll")->capture_default_str();
	app.add_option("--ch-pitch", chmap.pitch, "radio axis carrying pitch")->capture_default_str();
	app.add_option("--ch-throttle", chmap.throttle, "radio axis carrying throttle")->capture_default_str();
	app.add_option("--ch-yaw", chmap.yaw, "radio axis carrying yaw")->capture_default_str();
	app.add_option("--ch-arm", chmap.arm, "radio axis carrying the arm switch")->capture_default_str();
	app.add_option("--arm-threshold", chmap.arm_threshold,
				   "arm switch counts as up above this (-1..1)")
		->capture_default_str();
	app.add_option("--arm-hold-ms", arm_hold_ms,
				   "hold the arm switch up this long before it arms. Disarm is "
				   "always immediate: delaying that would keep the motors running "
				   "after the switch is already off")
		->capture_default_str();
	app.add_option("--script", script,
				   "fly a canned stick sequence instead of a gamepad "
				   "(arm-climb-roll, arm-hover), for testing without hands")
		->check(CLI::IsMember({"arm-climb-roll", "arm-hover"}));

	CLI11_PARSE(app, argc, argv);
	chmap.arm_hold_s = static_cast<float>(arm_hold_ms) / 1000.0f;

	if (do_list) {
		Gamepad::list();
		return 0;
	}

	// Pick the stick source. Only one of these is ever alive, because each
	// owns its own SDL init and teardown.
	//
	// auto prefers a mapped gamepad and falls back to a bare joystick, which
	// is what a transmitter in USB HID mode looks like.
	std::optional<Gamepad> pad;
	std::optional<Radio> radio;

	if (!no_pad && script.empty()) {
		if (input != "joystick") {
			pad.emplace(throttle == "trigger" ? Gamepad::Throttle::RightTrigger
											  : Gamepad::Throttle::LeftStick);
			if (!pad->present() && input == "auto") {
				pad.reset(); // releases SDL before the radio claims it
				radio.emplace(chmap);
			}
		} else {
			radio.emplace(chmap);
		}
	}

	fmt::println(stderr, "throttle: {}", throttle);

	// ---------------------------------------------------------------
	// Debug mode: no sensor I/O and no motor output, just gamepad state
	// on the console. Identical in effect to the standalone dpad.cpp
	// printer, and the quickest way to tell a dead pad from a dead
	// control loop.
	// ---------------------------------------------------------------
	if (debug_pad) {
		fmt::println(stderr, "press buttons / push sticks to max, Ctrl+C to quit\n");
		for (;;) {
			if (radio)
				radio->debug_print(); // every axis, to find your channel numbers
			else if (pad)
				pad->debug_print();
			SDL_Delay(8); // ~120 Hz poll for snappy button edges
		}
	}

	// ---------------------------------------------------------------
	// Lockstep loop: one motor packet out per sensor packet in.
	// ---------------------------------------------------------------

	plat::ByteStream io = io_dev.empty() ? plat::ByteStream{} : plat::ByteStream{io_dev, baud};
	if (!io.valid()) {
		fmt::print(stderr, "io: cannot open {}\n", io_dev);
		return 1;
	}

	fmt::print(stderr, "io: {}   throttle: {}\n", io_dev.empty() ? "stdin/stdout" : io_dev, throttle);

	std::optional<ScriptedSticks> scripted;
	if (!script.empty()) {
		scripted.emplace(script);
		fmt::print(stderr, "sticks: script {}\n", script);
	} else if (no_pad) {
		fmt::print(stderr, "sticks: --no-pad, neutral, never arms\n");
	} else if (radio) {
		fmt::print(stderr,
				   "sticks: radio  roll=a{} pitch=a{} throttle=a{} yaw=a{} arm=a{} (>{:.2f})\n",
				   chmap.roll, chmap.pitch, chmap.throttle, chmap.yaw, chmap.arm,
				   static_cast<double>(chmap.arm_threshold));
		fmt::print(stderr, "flip the arm switch up with the throttle down to arm, "
						   "down to disarm\n");
	} else if (pad) {
		fmt::print(stderr, "press LB to arm with the throttle down, press again to disarm. "
						   "B is a hard disarm\n");
	}

	fc::Controller controller;
	bool was_armed = false;

	for (;;) {
		// resync on the sync byte, then pull the rest of the frame
		std::uint8_t b{};
		if (!io.read_exact(&b, 1))
			break;
		if (b != fc::kSyncSensor)
			continue;

		fc::SensorPacket s{};
		s.sync = b;
		if (!io.read_exact(reinterpret_cast<std::uint8_t *>(&s) + 1, sizeof(s) - 1)) {
			break;
		}

		if (fc::packet_crc(s) != s.crc) {
			fmt::print(stderr, "rx: bad crc, dropping\n");
			continue;
		}

		const fc::Sticks rc = scripted  ? scripted->poll(s.dt)
							  : radio   ? radio->poll(s.dt)
							  : pad     ? pad->poll()
										: fc::Sticks{}; // --no-pad, or nothing plugged in

		if (rc.armed != was_armed) {
			fmt::print(stderr, "{}\n", rc.armed ? "ARMED" : "disarmed");
			was_armed = rc.armed;
		}

		const fc::MotorPacket out = controller.step(s, rc);
		if (!io.write_all(&out, sizeof(out)))
			break;
	}

	return 0;
}
