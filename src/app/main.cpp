// fc_min.cpp - host side: SDL2 gamepad, CLI11 args, fmt logging.
//
//   vcpkg install            (reads vcpkg.json)
//   cmake --preset default && cmake --build build
//
//   ./fc_min                          pipes, first pad SDL finds
//   ./fc_min --throttle trigger       RT for throttle instead of left stick
//   ./fc_min --dev /dev/pts/7         pty instead of pipes
//   ./fc_min --list                   show detected pads and exit
//
// All three libraries live here, never in fc_core.hpp. SDL gives a normalized
// controller abstraction, so there is no per-driver axis map to calibrate.

#include "fc_core.hpp"

#include <CLI/CLI.hpp>
#include <SDL.h>
#include <fmt/core.h>

#include <cerrno>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {

// -------------------------------------------------------------------
// Gamepad - SDL_GameController.
//
// SDL remaps every known pad onto one virtual layout via its controller
// database, so LEFTX is LEFTX whether the pad is xpad, xone, wireless,
// or a PlayStation clone. That removes the whole calibration step.
// -------------------------------------------------------------------
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

		if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
			fmt::print(stderr, "sdl: init failed: {}\n", SDL_GetError());
			return;
		}
		sdl_ready_ = true;

		for (int i = 0; i < SDL_NumJoysticks(); ++i) {
			if (!SDL_IsGameController(i))
				continue;
			pad_ = SDL_GameControllerOpen(i);
			if (pad_) {
				fmt::print(stderr, "pad: {}\n", SDL_GameControllerName(pad_));
				return;
			}
		}
		fmt::print(stderr, "pad: none found, staying disarmed\n");
	}

	~Gamepad() {
		if (pad_)
			SDL_GameControllerClose(pad_);
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

		SDL_GameControllerUpdate();

		if (!SDL_GameControllerGetAttached(pad_)) { // unplugged
			fmt::print(stderr, "pad: disconnected, failsafe\n");
			SDL_GameControllerClose(pad_);
			pad_ = nullptr;
			gate_.force_disarm();
			return s;
		}

		s.roll = norm(axis(SDL_CONTROLLER_AXIS_RIGHTX));
		s.pitch = -norm(axis(SDL_CONTROLLER_AXIS_RIGHTY)); // up is negative
		s.yaw = norm(axis(SDL_CONTROLLER_AXIS_LEFTX));

		if (mode_ == Throttle::RightTrigger) {
			// triggers are one-sided 0..32767 and stay where you put them
			s.throttle = clamp01(axis(SDL_CONTROLLER_AXIS_TRIGGERRIGHT) / 32767.0f);
		} else {
			// spring-centered: rests at 0.5, which is near CF2X hover thrust
			s.throttle = clamp01((1.0f - axis(SDL_CONTROLLER_AXIS_LEFTY) / 32767.0f) * 0.5f);
		}

		s.armed = gate_.update(button(SDL_CONTROLLER_BUTTON_LEFTSHOULDER), button(SDL_CONTROLLER_BUTTON_B), s.throttle);
		if (!s.armed)
			s.throttle = 0.0f;

		return s;
	}

	static void list() {
		SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
		if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
			fmt::print(stderr, "sdl: {}\n", SDL_GetError());
			return;
		}
		for (int i = 0; i < SDL_NumJoysticks(); ++i) {
			fmt::print("{}: {}{}\n", i, SDL_JoystickNameForIndex(i),
					   SDL_IsGameController(i) ? "" : "  (not a game controller)");
		}
		SDL_Quit();
	}

  private:
	float axis(SDL_GameControllerAxis a) const noexcept {
		return static_cast<float>(SDL_GameControllerGetAxis(pad_, a));
	}
	bool button(SDL_GameControllerButton b) const noexcept {
		return SDL_GameControllerGetButton(pad_, b) != 0;
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

	SDL_GameController *pad_{nullptr};
	bool sdl_ready_{false};
	Throttle mode_;
	fc::ArmingGate gate_{};
};

// -------------------------------------------------------------------
// byte stream: pipes by default, pty if a device is named.
//
// Deliberately not asio or libserialport. Lockstep means one blocking
// read and one blocking write per iteration, and async machinery would
// add concepts without removing lines.
// -------------------------------------------------------------------
class ByteStream {
  public:
	ByteStream() noexcept : rfd_{STDIN_FILENO}, wfd_{STDOUT_FILENO} {
	}

	explicit ByteStream(const std::string &dev) noexcept {
		const int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY);
		if (fd < 0)
			return;

		termios t{};
		if (::tcgetattr(fd, &t) != 0) {
			::close(fd);
			return;
		}
		::cfmakeraw(&t);
		::cfsetispeed(&t, B115200);
		::cfsetospeed(&t, B115200);
		t.c_cc[VMIN] = 1;
		t.c_cc[VTIME] = 0;
		if (::tcsetattr(fd, TCSANOW, &t) != 0) {
			::close(fd);
			return;
		}

		rfd_ = wfd_ = fd;
		owned_ = true;
	}

	~ByteStream() {
		if (owned_ && rfd_ >= 0)
			::close(rfd_);
	}

	ByteStream(const ByteStream &) = delete;
	ByteStream &operator=(const ByteStream &) = delete;

	bool valid() const noexcept {
		return rfd_ >= 0 && wfd_ >= 0;
	}

	bool read_exact(void *buf, std::size_t n) noexcept {
		auto *p = static_cast<std::uint8_t *>(buf);
		while (n) {
			const ssize_t r = ::read(rfd_, p, n);
			if (r <= 0)
				return false;
			p += r;
			n -= static_cast<std::size_t>(r);
		}
		return true;
	}

	bool write_all(const void *buf, std::size_t n) noexcept {
		const auto *p = static_cast<const std::uint8_t *>(buf);
		while (n) {
			const ssize_t w = ::write(wfd_, p, n);
			if (w <= 0)
				return false;
			p += w;
			n -= static_cast<std::size_t>(w);
		}
		return true;
	}

  private:
	int rfd_{-1};
	int wfd_{-1};
	bool owned_{false};
};

} // namespace

// ===================================================================

int main(int argc, char **argv) {
	CLI::App app{"minimum viable flight controller, lockstep HIL"};

	std::string io_dev;
	std::string throttle = "stick";
	bool do_list = false;

	app.add_option("--dev", io_dev, "serial/pty device (default: stdin/stdout pipes)");
	app.add_option("--throttle", throttle, "throttle source")->check(CLI::IsMember({"stick", "trigger"}));
	app.add_flag("--list", do_list, "list detected gamepads and exit");

	CLI11_PARSE(app, argc, argv);

	if (do_list) {
		Gamepad::list();
		return 0;
	}

	ByteStream io = io_dev.empty() ? ByteStream{} : ByteStream{io_dev};
	if (!io.valid()) {
		fmt::print(stderr, "io: cannot open {}\n", io_dev);
		return 1;
	}

	Gamepad pad{throttle == "trigger" ? Gamepad::Throttle::RightTrigger : Gamepad::Throttle::LeftStick};

	fmt::print(stderr, "io: {}   throttle: {}\n", io_dev.empty() ? "stdin/stdout" : io_dev, throttle);
	fmt::print(stderr, "hold LB to arm from low throttle, B to disarm\n");

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

		const fc::Sticks rc = pad.poll();

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
