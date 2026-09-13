#!/usr/bin/env python3
"""Read an Xbox gamepad and print what it does. Windows, no dependencies.

Talks to XInput directly through ctypes - no pygame, no SDL, no venv. XInput
is part of Windows and polls the pad without needing a window, which is what
makes this work where the pygame version read nothing but zeros.

Output mirrors the C++ gamepad_btn_test: buttons on press and release, sticks
and triggers when they hit their extremes.

    python gamepad_test.py            # events, like the C++ tool
    python gamepad_test.py --live     # a single continuously updating line
    python gamepad_test.py --raw      # raw XInput numbers, no interpretation
    python gamepad_test.py --pad 1    # a different controller slot
"""

import argparse
import ctypes
import sys
import time
from ctypes import wintypes

# --- XInput ---------------------------------------------------------------

ERROR_SUCCESS = 0
ERROR_DEVICE_NOT_CONNECTED = 1167

# Which bit in wButtons is which control, named the way SDL names them so the
# output lines up with the C++ tool.
BUTTONS = [
    (0x1000, "south (A)"),
    (0x2000, "east (B)"),
    (0x4000, "west (X)"),
    (0x8000, "north (Y)"),
    (0x0100, "left shoulder"),
    (0x0200, "right shoulder"),
    (0x0020, "back"),
    (0x0010, "start"),
    (0x0040, "left stick"),
    (0x0080, "right stick"),
    (0x0001, "dpad up"),
    (0x0002, "dpad down"),
    (0x0004, "dpad left"),
    (0x0008, "dpad right"),
]

STICK_MAX = 32767.0
TRIGGER_MAX = 255.0
AT_MAX = 0.90       # counts as "pushed to the limit"
RELEASED = 0.55     # and must fall back below this before it can fire again


class Gamepad(ctypes.Structure):
    _fields_ = [("wButtons", wintypes.WORD),
                ("bLeftTrigger", ctypes.c_ubyte),
                ("bRightTrigger", ctypes.c_ubyte),
                ("sThumbLX", ctypes.c_short),
                ("sThumbLY", ctypes.c_short),
                ("sThumbRX", ctypes.c_short),
                ("sThumbRY", ctypes.c_short)]


class State(ctypes.Structure):
    _fields_ = [("dwPacketNumber", wintypes.DWORD), ("Gamepad", Gamepad)]


def load_xinput():
    for name in ("XInput1_4.dll", "XInput1_3.dll", "XInput9_1_0.dll"):
        try:
            return ctypes.windll.LoadLibrary(name), name
        except OSError:
            continue
    sys.exit("no XInput DLL found - is this Windows?")


def read(xinput, pad):
    """Current state, or None if nothing is plugged into that slot."""
    state = State()
    if xinput.XInputGetState(pad, ctypes.byref(state)) != ERROR_SUCCESS:
        return None
    return state


def decode(state):
    """XInput's raw numbers as the values a person cares about.

    Stick Y is flipped so up is negative, matching SDL and therefore the C++
    tool. XInput reports up as positive; everything else agrees already.
    """
    g = state.Gamepad
    return {
        "buttons": {name: bool(g.wButtons & bit) for bit, name in BUTTONS},
        "axes": {
            "left stick  X": max(-1.0, g.sThumbLX / STICK_MAX),
            "left stick  Y": -max(-1.0, g.sThumbLY / STICK_MAX),
            "right stick X": max(-1.0, g.sThumbRX / STICK_MAX),
            "right stick Y": -max(-1.0, g.sThumbRY / STICK_MAX),
        },
        "triggers": {"left ": g.bLeftTrigger / TRIGGER_MAX,
                     "right": g.bRightTrigger / TRIGGER_MAX},
    }


# --- output modes ---------------------------------------------------------

def events(xinput, pad):
    """Print transitions only: button up/down, sticks and triggers at max."""
    held = {name: False for _, name in BUTTONS}
    latched = {}

    while True:
        state = read(xinput, pad)
        if state is None:
            print("controller disconnected - waiting...")
            while read(xinput, pad) is None:
                time.sleep(0.5)
            print("reconnected")
            continue

        s = decode(state)

        for name, down in s["buttons"].items():
            if down != held[name]:
                print(f"button {name:<16} {'DOWN' if down else 'up'}")
                held[name] = down

        # Sticks fire once when pushed to an extreme, and rearm only after
        # coming back near centre - otherwise a held stick spams every poll.
        for name, v in s["axes"].items():
            key = f"axis:{name}"
            if abs(v) >= AT_MAX and not latched.get(key):
                print(f"axis    {name}  MAX {'+' if v > 0 else '-'} ({v:.2f})")
                latched[key] = True
            elif abs(v) < RELEASED:
                latched[key] = False

        for name, v in s["triggers"].items():
            key = f"trig:{name}"
            if v >= AT_MAX and not latched.get(key):
                # scaled to SDL's range so it reads like the C++ output
                print(f"trigger {name}      MAX ({int(v * STICK_MAX)})")
                latched[key] = True
            elif v < RELEASED:
                latched[key] = False

        time.sleep(0.008)


def live(xinput, pad):
    """One line, continuously updated. Good for watching drift and deadzones."""
    while True:
        state = read(xinput, pad)
        if state is None:
            print("\rcontroller disconnected           ", end="", flush=True)
            time.sleep(0.5)
            continue
        s = decode(state)
        a, t = s["axes"], s["triggers"]
        down = [n for n, v in s["buttons"].items() if v]
        print(f"\rL({a['left stick  X']:+.2f},{a['left stick  Y']:+.2f}) "
              f"R({a['right stick X']:+.2f},{a['right stick Y']:+.2f}) "
              f"LT {t['left ']:.2f} RT {t['right']:.2f}  "
              + (", ".join(down) or "-").ljust(40), end="", flush=True)
        time.sleep(0.03)


def raw(xinput, pad):
    """Exactly what XInput returns, uninterpreted."""
    last = None
    while True:
        state = read(xinput, pad)
        if state is None:
            time.sleep(0.5)
            continue
        g = state.Gamepad
        now = (g.wButtons, g.bLeftTrigger, g.bRightTrigger,
               g.sThumbLX, g.sThumbLY, g.sThumbRX, g.sThumbRY)
        if now != last:
            print(f"packet {state.dwPacketNumber:<10} buttons 0x{g.wButtons:04x}  "
                  f"LT {g.bLeftTrigger:3d} RT {g.bRightTrigger:3d}  "
                  f"LX {g.sThumbLX:+6d} LY {g.sThumbLY:+6d}  "
                  f"RX {g.sThumbRX:+6d} RY {g.sThumbRY:+6d}")
            last = now
        time.sleep(0.008)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pad", type=int, default=0, help="controller slot, 0-3")
    ap.add_argument("--live", action="store_true", help="one updating line")
    ap.add_argument("--raw", action="store_true", help="raw XInput numbers")
    args = ap.parse_args()

    xinput, dll = load_xinput()

    if read(xinput, args.pad) is None:
        found = [i for i in range(4) if read(xinput, i) is not None]
        sys.exit(f"nothing in slot {args.pad}." +
                 (f" Try --pad {found[0]}." if found else
                  " No XInput controller connected at all.\n"
                  "  Note XInput only sees Xbox-style pads; a DirectInput-only\n"
                  "  joystick will not show up here."))

    print(f"pad: XInput controller in slot {args.pad}  (via {dll})")
    print("press buttons / push sticks to max, Ctrl+C to quit\n")

    mode = raw if args.raw else live if args.live else events
    try:
        mode(xinput, args.pad)
    except KeyboardInterrupt:
        print("\nbye")
    return 0


if __name__ == "__main__":
    sys.exit(main())
