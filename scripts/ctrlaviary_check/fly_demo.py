#!/usr/bin/env python3
"""Fly it. Arm on LB, then both sticks, in stabilised (angle) mode.

One rung past pad_demo.py, which could only arm and disarm. Now the sticks
mean something:

    left stick  up/down     throttle   (fully down = zero)
    left stick  left/right  yaw rate
    right stick left/right  roll angle
    right stick up/down     pitch angle
    LB                      arm / disarm

Stabilised, not acro: the right stick commands an *angle*, not a rate. Centre
it and the drone levels itself. In acro the same stick would command a
rotation rate and centring would only stop the rotation, leaving it tilted.

Arming is refused unless the throttle stick is all the way down, so a stick
left half up cannot spin the motors the instant you press LB.

    python fly_demo.py               # window
    python fly_demo.py --no-gui
    python fly_demo.py --seconds 120

Still a Python controller. The point of the exercise is that this whole
control block is what the C++ flight controller replaces later.
"""

import argparse
import ctypes
import sys
import time
from ctypes import wintypes

import numpy as np
from scipy.spatial.transform import Rotation

CTRL_HZ = 240

# --- stick feel -----------------------------------------------------------
MAX_TILT = np.radians(30.0)      # what full right-stick deflection asks for
MAX_YAW_RATE = np.radians(180.0)  # deg/s at full left-stick left/right
ARM_THROTTLE = 0.05              # throttle must be below this to arm

# --- control gains, in RPM per unit of error ------------------------------
KP_ANGLE = 9000.0       # angle error -> corrective RPM split
KD_RATE = 900.0         # body-rate damping, keeps the above from ringing
KP_YAW = 1200.0         # yaw is rate-controlled even in stabilised mode

# Motor mixing for the CF2X X-layout, taken from the same table
# gym-pybullet-drones' own DSLPIDControl uses, so the signs are not guesswork.
# Rows are motors 0-3, columns are [roll, pitch, yaw].
MIXER = np.array([[-0.5, -0.5, -1.0],
                  [-0.5, +0.5, +1.0],
                  [+0.5, +0.5, -1.0],
                  [+0.5, -0.5, +1.0]])

# XInput's own recommended deadzones - these sticks do not rest at exactly 0.
DEADZONE_L = 7849 / 32767.0
DEADZONE_R = 8689 / 32767.0

LB = 0x0100


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


def deadzone(v, dz):
    """Rescale so the stick still reaches 1.0 after the dead band is removed."""
    if abs(v) < dz:
        return 0.0
    return (abs(v) - dz) / (1.0 - dz) * (1.0 if v > 0 else -1.0)


def open_pad(slot):
    """Return a function giving (throttle, yaw, roll, pitch, lb_down)."""
    for name in ("XInput1_4.dll", "XInput1_3.dll", "XInput9_1_0.dll"):
        try:
            xinput = ctypes.windll.LoadLibrary(name)
            break
        except OSError:
            continue
    else:
        sys.exit("no XInput DLL found - is this Windows?")

    state = State()

    def read():
        if xinput.XInputGetState(slot, ctypes.byref(state)) != 0:
            return None                      # unplugged: caller disarms
        g = state.Gamepad
        ly = deadzone(max(-1.0, g.sThumbLY / 32767.0), DEADZONE_L)
        lx = deadzone(max(-1.0, g.sThumbLX / 32767.0), DEADZONE_L)
        rx = deadzone(max(-1.0, g.sThumbRX / 32767.0), DEADZONE_R)
        ry = deadzone(max(-1.0, g.sThumbRY / 32767.0), DEADZONE_R)
        # Stick fully down is zero throttle, fully up is full. A gamepad stick
        # springs back to centre, so letting go leaves you at half throttle.
        return {"throttle": (ly + 1.0) / 2.0,
                "yaw": lx, "roll": rx, "pitch": ry,
                "lb": bool(g.wButtons & LB)}

    if read() is None:
        sys.exit(f"no controller in slot {slot}.\n"
                 "  Check it with: python ..\\gamepad_test\\gamepad_test.py")
    return read


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pad", type=int, default=0, help="controller slot, 0-3")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--seconds", type=float, default=120.0)
    args = ap.parse_args()

    read = open_pad(args.pad)

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
    idle_rpm = env.HOVER_RPM * 0.25
    top_rpm = env.MAX_RPM * 0.95

    print("\n  left stick   up/down     throttle (fully down = zero)")
    print("  left stick   left/right  yaw rate")
    print("  right stick  left/right  roll angle")
    print("  right stick  up/down     pitch angle")
    print("  LB                       arm / disarm")
    print(f"\nstabilised mode, max tilt {np.degrees(MAX_TILT):.0f} deg. "
          "Hold throttle down to arm. Ctrl+C to quit.\n")

    armed = False
    was_lb = False
    action = np.zeros((1, 4))

    try:
        obs, _ = env.reset()
        for i in range(int(CTRL_HZ * args.seconds)):
            t = i / CTRL_HZ
            pad = read()
            if pad is None:                       # pad yanked out mid-flight
                if armed:
                    print(f"t={t:5.1f}s  DISARMED  (controller disconnected)")
                    armed = False
                pad = {"throttle": 0.0, "yaw": 0.0, "roll": 0.0,
                       "pitch": 0.0, "lb": False}

            if pad["lb"] and not was_lb:           # rising edge only
                if armed:
                    armed = False
                    print(f"t={t:5.1f}s  DISARMED   z={obs[0][2]:.2f} m")
                elif pad["throttle"] <= ARM_THROTTLE:
                    armed = True
                    print(f"t={t:5.1f}s  ARMED")
                else:
                    print(f"t={t:5.1f}s  arm refused - throttle is "
                          f"{pad['throttle'] * 100:.0f}%, pull it all the way down")
            was_lb = pad["lb"]

            s = obs[0]
            roll, pitch, _ = s[7:10]
            # obs gives angular velocity in the world frame; the controller
            # wants body rates, the same thing a real gyro measures
            p_rate, q_rate, r_rate = Rotation.from_quat(s[3:7]).inv().apply(s[13:16])

            if armed:
                # stabilised: the stick is an angle the drone should hold
                roll_sp = pad["roll"] * MAX_TILT
                pitch_sp = pad["pitch"] * MAX_TILT
                yaw_rate_sp = -pad["yaw"] * MAX_YAW_RATE

                u_roll = KP_ANGLE * (roll_sp - roll) - KD_RATE * p_rate
                u_pitch = KP_ANGLE * (pitch_sp - pitch) - KD_RATE * q_rate
                u_yaw = KP_YAW * (yaw_rate_sp - r_rate)

                base = idle_rpm + pad["throttle"] * (top_rpm - idle_rpm)
                action[0, :] = np.clip(
                    base + MIXER @ np.array([u_roll, u_pitch, u_yaw]),
                    0.0, env.MAX_RPM)
            else:
                action[0, :] = 0.0

            obs, _, _, _, _ = env.step(action)

            if not args.no_gui:
                time.sleep(1.0 / CTRL_HZ)
            if i % CTRL_HZ == 0:
                print(f"t={t:5.1f}s {'ARM' if armed else '---'} "
                      f"thr={pad['throttle']:.2f} "
                      f"z={obs[0][2]:6.2f} m  "
                      f"roll={np.degrees(roll):+6.1f} pitch={np.degrees(pitch):+6.1f} "
                      f"deg  rpm={action[0].mean():8.1f}")

    except KeyboardInterrupt:
        print("\nbye")
    finally:
        env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
