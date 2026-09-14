// dpad.cpp - minimal SDL3 gamepad reader: D-pad, all buttons, stick axes at max
//
// Build (Linux/macOS):
//   c++ -std=c++17 dpad.cpp -o dpad $(pkg-config --cflags --libs sdl3) -lfmt
//
// Build (Windows w/ vcpkg):
//   cl /std:c++17 dpad.cpp /I path\to\SDL3\include /I path\to\fmt\include /link SDL3.lib fmt.lib

#define SDL_MAIN_HANDLED
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <fmt/base.h>
#include <fmt/core.h>

namespace {

constexpr float kFull = 0.90f; // "at max" threshold (0..1)
constexpr float kDead = 0.20f; // ignore noise below this
constexpr int kAxisMax = 32767;

// Button + last state, so we only print on change.
struct Btn {
	const char *name;
	SDL_GamepadButton id;
	bool last = false;
};

} // namespace

int main() {
	SDL_SetMainReady();

	if (!SDL_Init(SDL_INIT_GAMEPAD)) {
		fmt::println(stderr, "sdl: init failed: {}", SDL_GetError());
		return 1;
	}

	// Find and open the first gamepad SDL knows about.
	SDL_Gamepad *pad = nullptr;
	int count = 0;
	SDL_JoystickID *ids = SDL_GetGamepads(&count);
	if (ids) {
		for (int i = 0; i < count; ++i) {
			pad = SDL_OpenGamepad(ids[i]);
			if (pad) {
				fmt::println("pad: {}", SDL_GetGamepadName(pad));
				break;
			}
		}
		SDL_free(ids);
	}
	if (!pad) {
		fmt::println(stderr, "pad: none found");
		SDL_Quit();
		return 1;
	}

	// Every button SDL3 exposes. Order roughly matches Xbox layout.
	Btn buttons[] = {
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

	fmt::println("press buttons / push sticks to max, Ctrl+C to quit\n");

	// Track which axes were already reported, so we print on entering
	// "full deflection" only, not every poll while held.
	bool axis_reported[SDL_GAMEPAD_AXIS_COUNT] = {};

	for (;;) {
		SDL_UpdateGamepads();

		if (!SDL_GamepadConnected(pad)) {
			fmt::println("\npad: disconnected");
			break;
		}

		// ---- buttons: report on change ----
		for (auto &b : buttons) {
			const bool now = SDL_GetGamepadButton(pad, b.id);
			if (now != b.last) {
				fmt::println("button {:<14} {}", b.name, now ? "DOWN" : "up");
				b.last = now;
			}
		}

		// ---- triggers: report on full squeeze ----
		const Sint16 lt = SDL_GetGamepadAxis(pad, SDL_GAMEPAD_AXIS_LEFT_TRIGGER);
		const Sint16 rt = SDL_GetGamepadAxis(pad, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER);
		if (lt > kAxisMax * kFull && !axis_reported[SDL_GAMEPAD_AXIS_LEFT_TRIGGER]) {
			fmt::println("trigger left       MAX ({})", lt);
			axis_reported[SDL_GAMEPAD_AXIS_LEFT_TRIGGER] = true;
		} else if (lt < kAxisMax * kDead) {
			axis_reported[SDL_GAMEPAD_AXIS_LEFT_TRIGGER] = false;
		}
		if (rt > kAxisMax * kFull && !axis_reported[SDL_GAMEPAD_AXIS_RIGHT_TRIGGER]) {
			fmt::println("trigger right      MAX ({})", rt);
			axis_reported[SDL_GAMEPAD_AXIS_RIGHT_TRIGGER] = true;
		} else if (rt < kAxisMax * kDead) {
			axis_reported[SDL_GAMEPAD_AXIS_RIGHT_TRIGGER] = false;
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
			const float v = SDL_GetGamepadAxis(pad, s.axis) / 32767.0f;
			const bool full = (v > kFull) || (v < -kFull);
			const int idx = s.axis; // unique per physical axis
			if (full && !axis_reported[idx]) {
				const char *dir = (v > 0) ? "+" : "-";
				fmt::println("axis    {:<14} MAX {} ({:.2f})", s.name, dir, v);
				axis_reported[idx] = true;
			} else if (!full && v > -kDead && v < kDead) {
				axis_reported[idx] = false;
			}
		}

		SDL_Delay(8); // ~120 Hz poll for snappy button edges
	}

	SDL_CloseGamepad(pad);
	SDL_Quit();
	return 0;
}
