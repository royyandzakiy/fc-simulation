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

	bool present() const noexcept {
		return pad_ != nullptr;
	}

	// Refresh pad state without an event loop. Never blocks, so the
	// lockstep control loop keeps its cadence regardless of pad activity.
	fc::Sticks poll() noexcept {
		fc::Sticks s{};

		if (!pad_) {
			gate_.force_disarm();
			return s;
		}

		SDL_UpdateGamepads();

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

		SDL_UpdateGamepads();

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
		if (lt > kAxisMax * kFull && !axis_reported_[SDL_GAMEPAD_AXIS_LEFT_TRIGGER]) {
			fmt::println("trigger left       MAX ({})", lt);
			axis_reported_[SDL_GAMEPAD_AXIS_LEFT_TRIGGER] = true;
		} else if (lt < kAxisMax * kDead) {
			axis_reported_[SDL_GAMEPAD_AXIS_LEFT_TRIGGER] = false;
		}
		if (rt > kAxisMax * kFull && !axis_reported_[SDL_GAMEPAD_AXIS_RIGHT_TRIGGER]) {
			fmt::println("trigger right      MAX ({})", rt);
			axis_reported_[SDL_GAMEPAD_AXIS_RIGHT_TRIGGER] = true;
		} else if (rt < kAxisMax * kDead) {
			axis_reported_[SDL_GAMEPAD_AXIS_RIGHT_TRIGGER] = false;
		}

		// ---- sticks: report per-axis when pushed to max ----
		struct {
			const char *name;
			SDL_GamepadAxis axis;
		} sticks[] = {
			{"left stick  X", SDL_GAMEPAD_AXIS_LEFTX},
			{"left stick  Y", SDL_GAMEPAD_AXIS_LEFTY},
			{"right stick X", SDL_GAMEPAD_AXIS_RIGHTX},
			{"right stick Y", SDL_GAMEPAD_AXIS_RIGHTY},
		};
		for (auto &s : sticks) {
			const float v = SDL_GetGamepadAxis(pad_, s.axis) / 32767.0f;
			const bool full = (v > kFull) || (v < -kFull);
			const int idx = s.axis; // unique per physical axis
			if (full && !axis_reported_[idx]) {
				const char *dir = (v > 0) ? "+" : "-";
				fmt::println("axis    {:<14} MAX {} ({:.2f})", s.name, dir, v);
				axis_reported_[idx] = true;
			} else if (!full && v > -kDead && v < kDead) {
				axis_reported_[idx] = false;
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
	float axis(SDL_GamepadAxis a) const noexcept {
		return static_cast<float>(SDL_GetGamepadAxis(pad_, a));
	}
	bool button(SDL_GamepadButton b) const noexcept {
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
	Btn buttons_[21] = {
		{"south (A)", SDL_GAMEPAD_BUTTON_SOUTH},
		{"east (B)", SDL_GAMEPAD_BUTTON_EAST},
		{"west (X)", SDL_GAMEPAD_BUTTON_WEST},
		{"north (Y)", SDL_GAMEPAD_BUTTON_NORTH},
		{"back", SDL_GAMEPAD_BUTTON_BACK},
		{"guide", SDL_GAMEPAD_BUTTON_GUIDE},
		{"start", SDL_GAMEPAD_BUTTON_START},
		{"left stick", SDL_GAMEPAD_BUTTON_LEFT_STICK},
		{"right stick", SDL_GAMEPAD_BUTTON_RIGHT_STICK},
		{"left shoulder", SDL_GAMEPAD_BUTTON_LEFT_SHOULDER},
		{"right shoulder", SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER},
		{"dpad up", SDL_GAMEPAD_BUTTON_DPAD_UP},
		{"dpad down", SDL_GAMEPAD_BUTTON_DPAD_DOWN},
		{"dpad left", SDL_GAMEPAD_BUTTON_DPAD_LEFT},
		{"dpad right", SDL_GAMEPAD_BUTTON_DPAD_RIGHT},
		{"misc1", SDL_GAMEPAD_BUTTON_MISC1},
		{"paddle1", SDL_GAMEPAD_BUTTON_RIGHT_PADDLE1},
		{"paddle2", SDL_GAMEPAD_BUTTON_LEFT_PADDLE1},
		{"paddle3", SDL_GAMEPAD_BUTTON_RIGHT_PADDLE2},
		{"paddle4", SDL_GAMEPAD_BUTTON_LEFT_PADDLE2},
		{"touchpad", SDL_GAMEPAD_BUTTON_TOUCHPAD},
	};
	bool axis_reported_[SDL_GAMEPAD_AXIS_COUNT] = {};
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
	bool do_list = false;

	app.add_option("--dev", io_dev,
				   "serial device, e.g. COM7 or /dev/ttyUSB0 "
				   "(default: stdin/stdout pipes)");
	app.add_option("--throttle", throttle, "throttle source")->check(CLI::IsMember({"stick", "trigger"}));
	app.add_flag("--list", do_list, "list detected gamepads and exit");

	CLI11_PARSE(app, argc, argv);

	if (do_list) {
		Gamepad::list();
		return 0;
	}

	// ---------------------------------------------------------------
	// Debug mode: comment out all sensor I/O and motor output, and
	// just stream gamepad state to the console. Identical in effect
	// to the standalone dpad.cpp printer.
	// ---------------------------------------------------------------
	Gamepad pad{throttle == "trigger" ? Gamepad::Throttle::RightTrigger : Gamepad::Throttle::LeftStick};

	fmt::println(stderr, "throttle: {}", throttle);
	fmt::println("press buttons / push sticks to max, Ctrl+C to quit\n");

	for (;;) {
		pad.debug_print();
		SDL_Delay(8); // ~120 Hz poll for snappy button edges
	}

	// ---------------------------------------------------------------
	// Original lockstep loop - commented out while debugging buttons.
	// ---------------------------------------------------------------
	//
	// plat::ByteStream io = io_dev.empty() ? plat::ByteStream{} : plat::ByteStream{io_dev};
	// if (!io.valid()) {
	// 	fmt::print(stderr, "io: cannot open {}\n", io_dev);
	// 	return 1;
	// }
	//
	// fmt::print(stderr, "io: {}   throttle: {}\n", io_dev.empty() ? "stdin/stdout" : io_dev, throttle);
	// fmt::print(stderr, "hold LB to arm from low throttle, B to disarm\n");
	//
	// fc::Controller controller;
	// bool was_armed = false;
	//
	// for (;;) {
	// 	// resync on the sync byte, then pull the rest of the frame
	// 	std::uint8_t b{};
	// 	if (!io.read_exact(&b, 1))
	// 		break;
	// 	if (b != fc::kSyncSensor)
	// 		continue;
	//
	// 	fc::SensorPacket s{};
	// 	s.sync = b;
	// 	if (!io.read_exact(reinterpret_cast<std::uint8_t *>(&s) + 1, sizeof(s) - 1)) {
	// 		break;
	// 	}
	//
	// 	if (fc::packet_crc(s) != s.crc) {
	// 		fmt::print(stderr, "rx: bad crc, dropping\n");
	// 		continue;
	// 	}
	//
	// 	const fc::Sticks rc = pad.poll();
	//
	// 	if (rc.armed != was_armed) {
	// 		fmt::print(stderr, "{}\n", rc.armed ? "ARMED" : "disarmed");
	// 		was_armed = rc.armed;
	// 	}
	//
	// 	const fc::MotorPacket out = controller.step(s, rc);
	// 	if (!io.write_all(&out, sizeof(out)))
	// 		break;
	// }

	return 0;
}
