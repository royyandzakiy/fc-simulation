// radio_test.cpp - see what an RC transmitter in USB HID joystick mode reports.
//
// The gamepad sample next door uses SDL's gamepad API, which only returns
// devices SDL has a mapping for in its controller database. A radio has no
// entry there, so it never shows up that way and you get "none found". This
// one uses the joystick API and takes the device as it is: a pile of
// unlabelled axes with no agreed meaning.
//
// That is the point. Which axis carries throttle, and which one your arm
// switch lands on, depends on the model config in the radio (AETR, TAER, and
// whatever you assigned the switches to). This tells you, so you can pass the
// numbers to fc_sitl_cpp --ch-roll / --ch-pitch / --ch-throttle / --ch-yaw /
// --ch-arm.
//
//   radio_test              live view of every axis, button and hat
//   radio_test --list       enumerate devices and exit
//   radio_test --raw        show the raw -32768..32767 instead of -1..1
//   radio_test --events     log changes as lines instead of redrawing
//
// No platform #ifdefs and no OS headers: SDL covers the device side, and the
// display uses plain ANSI escapes, which Linux, macOS, Windows Terminal and
// the VS Code terminal all handle. If your editor offers to "add missing
// includes" for things like HANDLE or DWORD, decline. Those pull internal
// Windows SDK headers (consoleapi.h, winbase.h, winnt.h) that cannot be
// included on their own and fail with #error "No Target Architecture".

#define SDL_MAIN_HANDLED
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <CLI/CLI.hpp>
#include <fmt/base.h>
#include <fmt/core.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <string>

namespace {

constexpr int kAxisMax = 32767;
constexpr float kMoved = 0.15f; // counts as "you touched that one"
constexpr int kMaxAxes = 32;	// radios expose a lot of channels
constexpr int kMaxButtons = 32;

void set_hints() noexcept {
	// No window here, so without this SDL ignores the device for lack of
	// input focus.
	SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");

	// SDL enumerates joysticks through udev on Linux, and WSL usually has no
	// udev running. This makes it scan /dev/input directly instead, which is
	// the difference between seeing the radio and seeing nothing at all.
	// Irrelevant elsewhere, and SDL ignores a hint that does not apply, so it
	// costs nothing to set unconditionally.
	SDL_SetHint(SDL_HINT_JOYSTICK_LINUX_CLASSIC, "1");
}

void list() {
	SDL_SetMainReady();
	set_hints();
	if (!SDL_Init(SDL_INIT_JOYSTICK)) {
		fmt::print(stderr, "sdl: {}\n", SDL_GetError());
		return;
	}
	int count = 0;
	SDL_JoystickID *ids = SDL_GetJoysticks(&count);
	if (!ids || count == 0) {
		fmt::println("no joysticks found");
		fmt::println("  is the radio plugged in and set to USB Joystick mode?");
		fmt::println("  on Linux, check ls -l /dev/input/ and whether you can read it");
		fmt::println("  under WSL the device needs usbipd attach first");
	} else {
		for (int i = 0; i < count; ++i) {
			fmt::println("{}: {}{}", i, SDL_GetJoystickNameForID(ids[i]),
						 SDL_IsGamepad(ids[i]) ? "  (SDL also has a gamepad mapping for this)"
											   : "  (no gamepad mapping, which is normal for a radio)");
		}
	}
	if (ids)
		SDL_free(ids);
	SDL_Quit();
}

// A 13-cell bar with the centre marked, so a stick at rest reads as centred
// and a switch at an extreme is obvious at a glance.
std::string bar(float v) {
	constexpr int kCells = 13;
	const int mid = kCells / 2;
	int pos = static_cast<int>(std::lround((v + 1.0F) * 0.5F * (kCells - 1)));
	if (pos < 0)
		pos = 0;
	if (pos >= kCells)
		pos = kCells - 1;

	std::string out = "[";
	for (int i = 0; i < kCells; ++i) {
		out += (i == pos) ? '#' : (i == mid ? '|' : '.');
	}
	out += ']';
	return out;
}

struct AxisState {
	float value{0.0F};
	float lo{0.0F};
	float hi{0.0F};
	bool touched{false};
};

} // namespace

int main(int argc, char **argv) {
	CLI::App app{"see what an RC transmitter reports over USB HID"};
	int index = 0;
	bool do_list = false;
	bool raw = false;
	bool events = false;
	app.add_option("--index", index, "which joystick to open")->capture_default_str();
	app.add_flag("--list", do_list, "enumerate joysticks and exit");
	app.add_flag("--raw", raw, "print raw -32768..32767 instead of -1..1");
	app.add_flag("--events", events, "log changes as lines instead of redrawing");
	CLI11_PARSE(app, argc, argv);

	if (do_list) {
		list();
		return 0;
	}

	SDL_SetMainReady();
	set_hints();

	if (!SDL_Init(SDL_INIT_JOYSTICK)) {
		fmt::print(stderr, "sdl: init failed: {}\n", SDL_GetError());
		return 1;
	}

	int count = 0;
	SDL_JoystickID *ids = SDL_GetJoysticks(&count);
	SDL_Joystick *js = nullptr;
	if (ids && index < count) {
		js = SDL_OpenJoystick(ids[index]);
	}
	if (ids)
		SDL_free(ids);

	if (!js) {
		fmt::print(stderr, "no joystick at index {}. Try --list.\n", index);
		fmt::print(stderr, "  the radio has to be in USB Joystick mode, not storage mode.\n");
		SDL_Quit();
		return 1;
	}

	const int n_axes = SDL_GetNumJoystickAxes(js);
	const int n_buttons = SDL_GetNumJoystickButtons(js);
	const int n_hats = SDL_GetNumJoystickHats(js);

	fmt::println("radio: {}", SDL_GetJoystickName(js));
	fmt::println("{} axes, {} buttons, {} hats", n_axes, n_buttons, n_hats);
	fmt::println("");
	fmt::println("Move one stick or flip one switch at a time and watch which");
	fmt::println("row reacts. That row number is what --ch-* wants. Ctrl+C quits.");
	fmt::println("");

	std::array<AxisState, kMaxAxes> ax{};
	std::array<bool, kMaxButtons> btn{};
	const int axes = (n_axes < kMaxAxes) ? n_axes : kMaxAxes;
	const int buttons = (n_buttons < kMaxButtons) ? n_buttons : kMaxButtons;

	// Sample once before drawing, so the resting position becomes the
	// baseline rather than showing up as movement on the first frame.
	SDL_UpdateJoysticks();
	for (int i = 0; i < axes; ++i) {
		const float v = static_cast<float>(SDL_GetJoystickAxis(js, i)) / kAxisMax;
		ax.at(static_cast<std::size_t>(i)) = AxisState{.value = v, .lo = v, .hi = v, .touched = false};
	}

	int drawn = 0;
	for (;;) {
		SDL_Event e;
		while (SDL_PollEvent(&e)) {
		}
		SDL_UpdateJoysticks();

		if (!SDL_JoystickConnected(js)) {
			fmt::println("\nradio disconnected");
			break;
		}

		for (int i = 0; i < axes; ++i) {
			auto &a = ax.at(static_cast<std::size_t>(i));
			const float v = static_cast<float>(SDL_GetJoystickAxis(js, i)) / kAxisMax;
			if (v < a.lo)
				a.lo = v;
			if (v > a.hi)
				a.hi = v;
			if (!a.touched && (a.hi - a.lo) > kMoved) {
				a.touched = true;
				if (events)
					fmt::println("axis {:2d} moved", i);
			}
			a.value = v;
		}

		for (int i = 0; i < buttons; ++i) {
			const bool down = SDL_GetJoystickButton(js, i);
			if (down != btn.at(static_cast<std::size_t>(i))) {
				btn.at(static_cast<std::size_t>(i)) = down;
				if (events)
					fmt::println("button {:2d} {}", i, down ? "DOWN" : "up");
			}
		}

		if (!events) {
			// Redraw in place: move the cursor back up over the block we drew
			// last time. \x1b[K clears to end of line so a shorter value does
			// not leave the tail of the previous one behind.
			if (drawn > 0)
				fmt::print("\x1b[{}A", drawn);
			drawn = 0;

			fmt::println(" axis   value        seen           \x1b[K");
			++drawn;
			for (int i = 0; i < axes; ++i) {
				const auto &a = ax.at(static_cast<std::size_t>(i));
				if (raw) {
					fmt::println("  a{:<2d} {:+7.0f}   {:+6.0f}..{:+6.0f}  {} {}\x1b[K", i,
								 static_cast<double>(a.value) * kAxisMax,
								 static_cast<double>(a.lo) * kAxisMax,
								 static_cast<double>(a.hi) * kAxisMax, bar(a.value),
								 a.touched ? "moved" : "");
				} else {
					fmt::println("  a{:<2d} {:+6.2f}   {:+5.2f}..{:+5.2f}   {} {}\x1b[K", i,
								 static_cast<double>(a.value), static_cast<double>(a.lo),
								 static_cast<double>(a.hi), bar(a.value),
								 a.touched ? "moved" : "");
				}
				++drawn;
			}

			if (buttons > 0) {
				std::string down;
				for (int i = 0; i < buttons; ++i) {
					if (btn.at(static_cast<std::size_t>(i)))
						down += fmt::format("b{} ", i);
				}
				fmt::println(" buttons down: {}\x1b[K", down.empty() ? "-" : down);
				++drawn;
			}

			for (int h = 0; h < n_hats; ++h) {
				const Uint8 v = SDL_GetJoystickHat(js, h);
				fmt::println(" hat {}: 0x{:02x}\x1b[K", h, v);
				++drawn;
			}
			std::fflush(stdout);
		}

		SDL_Delay(16); // ~60 Hz, smooth enough to watch without burning a core
	}

	SDL_CloseJoystick(js);
	SDL_Quit();
	return 0;
}
