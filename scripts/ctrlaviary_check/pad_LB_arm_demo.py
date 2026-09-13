#!/usr/bin/env python3
"""The gamepad in the loop - but only arming and disarming. No flight control.

One rung past plant_demo.py. The plant is still gym-pybullet-drones'
CtrlAviary and the throttle is still a fixed number; the new thing is that a
human decides, live, whether the motors are live at all.

LB toggles armed state: press to arm, press again to disarm. Disarmed the
motors are hard zero and the drone sits (or falls). Armed they spin up and it
climbs. Sticks do nothing yet - that is the next step.

    python pad_demo.py                # window, LB arms and disarms
    python pad_demo.py --no-gui

Reads the pad through XInput directly, the same way scripts/gamepad_test does.
pygame was tried first and read nothing but zeros on this machine.

A real flight controller idles the motors on arm and waits for throttle. There
is no throttle here yet, so armed applies a gentle climb instead - otherwise
arming would be invisible.
"""

import argparse
import ctypes
import sys
import time
from ctypes import wintypes

import numpy as np

CTRL_HZ = 240
ARMED_THROTTLE = 1.02   # fraction of hover RPM while armed
LB = 0x0100             # XInput bit for the left shoulder button


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


def open_pad(slot):
    """Return a function giving the current button bitmask, or exit."""
    for name in ("XInput1_4.dll", "XInput1_3.dll", "XInput9_1_0.dll"):
        try:
            xinput = ctypes.windll.LoadLibrary(name)
            break
        except OSError:
            continue
    else:
        sys.exit("no XInput DLL found - is this Windows?")

    state = State()

    def buttons():
        if xinput.XInputGetState(slot, ctypes.byref(state)) != 0:
            return None                       # unplugged
        return state.Gamepad.wButtons

    if buttons() is None:
        found = [s for s in range(4)
                 if xinput.XInputGetState(s, ctypes.byref(State())) == 0]
        sys.exit(f"no controller in slot {slot}." +
                 (f" Try --pad {found[0]}." if found else
                  " Nothing connected.\n"
                  "  Check it with: python ..\\gamepad_test\\gamepad_test.py"))
    return buttons


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pad", type=int, default=0, help="controller slot, 0-3")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--seconds", type=float, default=60.0)
    args = ap.parse_args()

    buttons = open_pad(args.pad)

    from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
    from gym_pybullet_drones.utils.enums import DroneModel, Physics

    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        physics=Physics.PYB,
        pyb_freq=CTRL_HZ,
        ctrl_freq=CTRL_HZ,
        gui=not args.no_gui,
    )

    print(f"\npad: XInput controller in slot {args.pad}")
    print("press LB to arm, press again to disarm. Ctrl+C to quit.\n")

    armed = False
    was_pressed = False
    action = np.zeros((1, 4))

    try:
        obs, _ = env.reset()
        for i in range(int(CTRL_HZ * args.seconds)):
            held = buttons()
            pressed = bool(held is not None and held & LB)

            # Toggle on the rising edge only. Polling the level would flip the
            # state every frame the button is held - 240 times a second.
            if pressed and not was_pressed:
                armed = not armed
                print(f"t={i / CTRL_HZ:5.1f}s  {'ARMED' if armed else 'DISARMED'}"
                      f"   z={obs[0][2]:.2f} m")
            was_pressed = pressed

            action[0, :] = env.HOVER_RPM * ARMED_THROTTLE if armed else 0.0
            obs, _, _, _, _ = env.step(action)

            if not args.no_gui:
                time.sleep(1.0 / CTRL_HZ)
            if i % (CTRL_HZ * 2) == 0:
                print(f"t={i / CTRL_HZ:5.1f}s  {'ARM' if armed else '---'}  "
                      f"z={obs[0][2]:6.3f} m  rpm={action[0, 0]:8.1f}")

    except KeyboardInterrupt:
        print("\nbye")
    finally:
        env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
