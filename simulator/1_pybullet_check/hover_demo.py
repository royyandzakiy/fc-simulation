#!/usr/bin/env python3
"""A drone taking off, hovering, and recovering from a shove. Standalone.

No C++, no serial, no gamepad, no gym-pybullet-drones - just PyBullet and a
~15 line controller, so the whole thing is readable in one sitting. This is
the picture; the real work later is replacing the controller below with the
flight controller talking over a pipe.

    python hover_demo.py                 # window, watch it fly
    python hover_demo.py --headless      # numbers only
    python hover_demo.py --seconds 20    # fly longer
"""

import argparse
import math
import sys
import time
from pathlib import Path

import pybullet as p
import pybullet_data

HZ = 240
TARGET_Z = 1.0
G = 9.81
MAX_TILT = 0.35             # rad, how far the drone may lean
GUST_AT = (3.0, 6.0)        # seconds at which a gust hits
GUST_S = 0.15               # how long it blows
GUST_N = (4.0, 2.0)         # newtons, sideways


def clamp(v, lim):
    return max(-lim, min(lim, v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--seconds", type=float, default=8.0)
    args = ap.parse_args()

    p.connect(p.DIRECT if args.headless else p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -G)
    p.setTimeStep(1.0 / HZ)

    p.loadURDF("plane.urdf")
    # resolved against this file, not the shell's current directory,
    # so the demo runs from anywhere
    drone = p.loadURDF(str(Path(__file__).parent / "quad.urdf"), [0, 0, 0.1])

    # total mass, so hover thrust is derived rather than guessed
    mass = p.getDynamicsInfo(drone, -1)[0] + sum(
        p.getDynamicsInfo(drone, j)[0] for j in range(p.getNumJoints(drone)))

    # spin the props purely for show - lift comes from the forces below
    for j in range(p.getNumJoints(drone)):
        p.setJointMotorControl2(drone, j, p.VELOCITY_CONTROL,
                                targetVelocity=60 * (1 if j % 2 else -1), force=0.01)

    if not args.headless:
        p.resetDebugVisualizerCamera(2.2, 50, -25, [0, 0, 0.8])

    for i in range(int(HZ * args.seconds)):
        t = i / HZ
        pos, quat = p.getBasePositionAndOrientation(drone)
        x, y, z = pos
        (vx, vy, vz), (wx, wy, _) = p.getBaseVelocity(drone)
        roll, pitch, _ = p.getEulerFromQuaternion(quat)

        # --- the entire flight controller ---------------------------------
        # altitude PD around hover thrust
        thrust = max(0.0, mass * G + 8.0 * (TARGET_Z - z) - 4.0 * vz)
        # outer loop: to move, a quad must lean, so position error becomes a
        # tilt request, clamped so it never asks for an absurd angle
        want_pitch = clamp(0.35 * -x - 0.25 * vx, MAX_TILT)
        want_roll = clamp(-(0.35 * -y - 0.25 * vy), MAX_TILT)
        # inner loop: chase that attitude
        tx = 0.9 * (want_roll - roll) - 0.12 * wx
        ty = 0.9 * (want_pitch - pitch) - 0.12 * wy
        # ------------------------------------------------------------------

        # LINK_FRAME: force in the drone's own axes, applied at its centre of
        # mass. With WORLD_FRAME, posObj is a world point, and [0,0,0] would
        # push at the origin instead - a lever arm that flips the drone.
        p.applyExternalForce(drone, -1, [0, 0, thrust], [0, 0, 0], p.LINK_FRAME)
        # torque has no application point, so rotate body -> world and use that
        m = p.getMatrixFromQuaternion(quat)
        p.applyExternalTorque(drone, -1,
                              [m[0] * tx + m[1] * ty, m[3] * tx + m[4] * ty, m[6] * tx + m[7] * ty],
                              p.WORLD_FRAME)

        # a sideways gust, to prove the controller is really holding it there
        if any(k <= t < k + GUST_S for k in GUST_AT):
            p.applyExternalForce(drone, -1, [GUST_N[0], GUST_N[1], 0], pos, p.WORLD_FRAME)
            if any(abs(t - k) < 0.5 / HZ for k in GUST_AT):
                print(f"  t={t:4.1f}s  <- gust")

        p.stepSimulation()

        if not args.headless:
            time.sleep(1.0 / HZ)
        if i % HZ == 0:
            print(f"  t={t:4.1f}s  z={z:5.2f} m  drift={math.hypot(x, y):4.2f} m  "
                  f"tilt={math.degrees(max(abs(roll), abs(pitch))):4.1f} deg")

    z = p.getBasePositionAndOrientation(drone)[0][2]
    print(f"\nfinal z {z:.2f} m (target {TARGET_Z:.2f})")
    print("PASS: it flew and held altitude." if abs(z - TARGET_Z) < 0.25
          else "FAIL: it did not hold altitude.")
    p.disconnect()
    return 0 if abs(z - TARGET_Z) < 0.25 else 1


if __name__ == "__main__":
    sys.exit(main())
