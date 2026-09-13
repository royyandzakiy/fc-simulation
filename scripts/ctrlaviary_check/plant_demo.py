#!/usr/bin/env python3
"""The real plant, flown open-loop. No controller, no gamepad, no pipe.

One step closer to hil_bridge/bridge.py than pybullet_check/hover_demo.py:
the drone is no longer a hand-made URDF pushed around with invented forces,
it is gym-pybullet-drones' CtrlAviary flying a real Crazyflie 2.X. What you
feed it is motor RPM; what it hands back is the 20-float obs array that
bridge.py indexes into. The mixer - RPM to per-rotor thrust and torque -
lives inside the environment.

Nothing stabilises the attitude here, so it climbs and then tips over. That
is the expected result, and it is the whole argument for the flight
controller that gets wired in later.

    python plant_demo.py              # window, watch it go
    python plant_demo.py --no-gui     # numbers only
    python plant_demo.py --seconds 10
"""

import argparse
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

CTRL_HZ = 240

# Open-loop throttle schedule, as a fraction of hover RPM. All four motors
# get the same value, so nothing is being asked to roll, pitch or yaw.
#   (until_seconds, fraction_of_hover)
SCHEDULE = ((2.0, 1.03),    # climb
            (4.0, 1.00),    # nominal hover thrust
            (6.0, 0.97))    # descend

# No real quad is perfectly trimmed. Half a percent extra on one motor is
# enough to make the point: open loop there is nothing to correct it, so the
# tilt integrates away. It also gives the gyro something to report - with a
# perfectly symmetric machine the body rates stay exactly zero and the
# world->body conversion below would be multiplying zeros.
IMBALANCE = 1.0001


def throttle_at(t):
    for until, frac in SCHEDULE:
        if t < until:
            return frac
    return SCHEDULE[-1][1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-gui", action="store_true", help="no pybullet window")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--perfect", action="store_true",
                    help="remove the motor imbalance (stays level, gyro reads zero)")
    args = ap.parse_args()

    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        physics=Physics.PYB,
        pyb_freq=CTRL_HZ,
        ctrl_freq=CTRL_HZ,
        gui=not args.no_gui,
    )

    print(f"\nCF2X: {env.M * 1000:.0f} g   arm {env.L * 1000:.1f} mm   "
          f"hover {env.HOVER_RPM:.0f} rpm   max {env.MAX_RPM:.0f} rpm")

    try:
        obs, _ = env.reset()
        start_z = obs[0][2]
        peak_z = start_z
        action = np.zeros((1, 4))

        for i in range(int(CTRL_HZ * args.seconds)):
            t = i / CTRL_HZ
            s = obs[0]
            # the layout bridge.py depends on, asserted here by using it
            pos, quat_xyzw, ang_v_world = s[0:3], s[3:7], s[13:16]

            # obs reports angular velocity in the world frame; a real gyro
            # measures body rates, so rotate before anyone treats it as one
            gyro = Rotation.from_quat(quat_xyzw).inv().apply(ang_v_world)

            # open loop: the schedule is the entire "controller"
            action[0, :] = env.HOVER_RPM * throttle_at(t)
            if not args.perfect:
                action[0, 0] *= IMBALANCE

            obs, _, _, _, _ = env.step(action)
            peak_z = max(peak_z, pos[2])

            if not args.no_gui:
                time.sleep(1.0 / CTRL_HZ)
            if i % CTRL_HZ == 0:
                print(f"t={t:4.1f}s  z={pos[2]:6.3f} m  "
                      f"gyro=[{gyro[0]:+6.2f} {gyro[1]:+6.2f} {gyro[2]:+6.2f}] rad/s  "
                      f"rpm={action[0, 0]:8.1f}  ({throttle_at(t):.2f}x hover)")

        tilt = np.degrees(abs(Rotation.from_quat(obs[0][3:7]).as_euler("xyz")[:2])).max()
        print(f"\nstart z {start_z:.3f} m   peak z {peak_z:.3f} m   "
              f"final z {obs[0][2]:.3f} m   final tilt {tilt:.1f} deg")

        # The check is "did the plant respond", NOT "did it stay level" -
        # open loop it cannot stay level, and asserting otherwise would be
        # asserting something false.
        if peak_z > start_z + 0.05:
            print("PASS: CtrlAviary stepped and the drone responded to RPM.")
            if tilt > 10:
                print(f"      It tipped {tilt:.0f} deg, as expected with nothing "
                      "controlling attitude - that is what the FC is for.")
            return 0

        print("FAIL: the drone never climbed - RPM is not reaching the plant.",
              file=sys.stderr)
        return 1

    finally:
        env.close()


if __name__ == "__main__":
    sys.exit(main())
