#!/usr/bin/env python3
"""The gamepad in the loop - but only arming and disarming. No flight control.

One rung past plant_demo.py. The plant is still gym-pybullet-drones'
CtrlAviary and the throttle is still a fixed number; the new thing is that a
human decides, live, whether the motors are live at all.

A shoulder button toggles armed state: press to arm, press again to disarm.
Disarmed the motors are hard zero and the drone sits (or falls). Armed they
spin up and it climbs. Sticks do nothing yet - that is the next step.

    python pad_demo.py                # window, LB arms and disarms
    python pad_demo.py --list         # which button is which, press to find out
    python pad_demo.py --button 5     # use a different button
    python pad_demo.py --no-gui

A real flight controller idles the motors on arm and waits for throttle. There
is no throttle here yet, so armed applies a gentle climb instead - otherwise
arming would be invisible.
"""

import argparse
import os
import sys
import time

# The joystick subsystem does not need a window, and asking for one would put
# an empty pygame window next to the pybullet one.
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import numpy as np
import pygame

CTRL_HZ = 240
ARM_BUTTON = 4          # LB on an Xbox pad, and what the firmware already uses
ARMED_THROTTLE = 1.02   # fraction of hover RPM while armed


def open_pad(index):
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        sys.exit("no gamepad found. Plug one in, and check Windows sees it "
                 "under 'Set up USB game controllers'.")
    pad = pygame.joystick.Joystick(index)
    pad.init()
    return pad


def list_mode(pad):
    """Print what the pad is doing, so you can find the button you want."""
    print(f"\n{pad.get_name()}: {pad.get_numbuttons()} buttons, "
          f"{pad.get_numaxes()} axes, {pad.get_numhats()} hats")
    print("press buttons and move sticks - Ctrl+C to quit\n")
    prev = [0] * pad.get_numbuttons()
    rest = None
    try:
        while True:
            pygame.event.pump()
            now = [pad.get_button(b) for b in range(pad.get_numbuttons())]
            for b, (was, is_) in enumerate(zip(prev, now)):
                if is_ and not was:
                    print(f"  button {b:2d} pressed"
                          f"{'   <- this is the default arm button' if b == ARM_BUTTON else ''}")
            prev = now

            axes = [round(pad.get_axis(a), 2) for a in range(pad.get_numaxes())]
            if rest is None:
                rest = axes
            moved = [(a, v) for a, (v, r) in enumerate(zip(axes, rest)) if abs(v - r) > 0.4]
            if moved:
                print("  axes " + "  ".join(f"{a}={v:+.2f}" for a, v in moved))
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nbye")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true",
                    help="show pad activity and exit, to identify buttons")
    ap.add_argument("--pad", type=int, default=0, help="which gamepad")
    ap.add_argument("--button", type=int, default=ARM_BUTTON,
                    help=f"arm/disarm button (default {ARM_BUTTON}, LB)")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--seconds", type=float, default=60.0)
    args = ap.parse_args()

    pad = open_pad(args.pad)
    if args.list:
        return list_mode(pad)

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

    print(f"\npad: {pad.get_name()}")
    print(f"press button {args.button} to arm, press again to disarm. Ctrl+C to quit.\n")

    armed = False
    was_pressed = False
    action = np.zeros((1, 4))

    try:
        obs, _ = env.reset()
        for i in range(int(CTRL_HZ * args.seconds)):
            pygame.event.pump()
            pressed = bool(pad.get_button(args.button))

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
        pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
