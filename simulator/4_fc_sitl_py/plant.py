#!/usr/bin/env python3
"""The world and the IMU. Owns the physics, owns nothing about control.

Runs gym-pybullet-drones, derives a gyro reading from the true state, hands it
to a flight controller running as a separate process, and applies whatever
motor demands come back. It has no control law of its own - stop the FC and
this just falls out of the sky.

    plant.py                              fc_sitl.py
    ----------------------                ----------------------
    pybullet world              --gyro-->  rate PID + mixer
    m -> RPM, steps physics     <--m[4]--  reads the gamepad itself

    python plant.py                  # launches fc_sitl.py itself
    python plant.py --no-gui --fc-args "--no-pad"

Strict lockstep: every physics step blocks until the controller answers, so
nothing runs in real time and runs are reproducible.
"""

import argparse
import json
import shlex
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

CTRL_HZ = 240
HERE = Path(__file__).resolve().parent


def start_fc(fc_path, extra_args):
    """Launch the controller with this same interpreter.

    sys.executable rather than 'python' so it works whether or not the venv is
    activated, and picks up the same packages this process has.
    """
    cmd = [sys.executable, str(fc_path)] + extra_args
    return subprocess.Popen(
        cmd,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        # stderr is inherited on purpose: the FC's own logging lands on your
        # terminal, and critically stays off stdout, which carries the data.
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fc", default=str(HERE / "fc_sitl.py"),
                    help="path to the flight controller script")
    ap.add_argument("--fc-args", default="",
                    help="extra arguments passed through to the controller")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--seconds", type=float, default=120.0)
    args = ap.parse_args()

    from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
    from gym_pybullet_drones.utils.enums import DroneModel, Physics

    fc = start_fc(args.fc, shlex.split(args.fc_args))

    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        physics=Physics.PYB,
        pyb_freq=CTRL_HZ,
        ctrl_freq=CTRL_HZ,
        gui=not args.no_gui,
    )

    dt = 1.0 / CTRL_HZ
    action = np.zeros((1, 4))
    print(f"\nplant: CF2X at {CTRL_HZ} Hz, controller = {Path(args.fc).name}")
    print("acro mode - it will NOT self-level. Ctrl+C to quit.\n")

    try:
        obs, _ = env.reset()
        for i in range(int(CTRL_HZ * args.seconds)):
            s = obs[0]
            # The IMU lives here. obs reports angular velocity in the world
            # frame; a real gyro measures body rates, so rotate before sending.
            gyro = Rotation.from_quat(s[3:7]).inv().apply(s[13:16])

            fc.stdin.write(json.dumps(
                {"seq": i, "dt": dt, "gyro": [round(float(v), 6) for v in gyro]}
            ).encode() + b"\n")
            fc.stdin.flush()

            # lockstep: block here until the controller answers
            reply = fc.stdout.readline()
            if not reply:
                code = fc.poll()
                raise RuntimeError(
                    f"controller stopped answering at step {i}"
                    + (f" (exit code {code})" if code is not None else ""))
            msg = json.loads(reply)
            if msg["seq"] != i:
                raise RuntimeError(f"sequence slip: got {msg['seq']}, expected {i}")

            # m is a thrust fraction per motor; thrust goes as rpm squared, so
            # normalised demand -> rpm is a square root. Same conversion
            # hil_bridge/bridge.py applies to the C++ controller's output.
            action[0, :] = env.MAX_RPM * np.sqrt(np.clip(msg["m"], 0.0, 1.0))
            obs, _, _, _, _ = env.step(action)

            if not args.no_gui:
                time.sleep(dt)
            if i % CTRL_HZ == 0:
                roll, pitch, _ = obs[0][7:10]
                rc = msg["rc"]
                print(f"t={i * dt:5.1f}s {'ARM' if msg['armed'] else '---'} "
                      f"thr={rc[3]:.2f} z={obs[0][2]:6.2f} m  "
                      f"roll={np.degrees(roll):+6.1f} pitch={np.degrees(pitch):+6.1f} deg  "
                      f"m={['%.2f' % v for v in msg['m']]}")

    except KeyboardInterrupt:
        print("\nbye")
    finally:
        env.close()
        # Closing stdin lets the controller's read loop hit EOF and exit on its
        # own; terminate() is the fallback if it does not take the hint.
        try:
            fc.stdin.close()
            fc.wait(timeout=2)
        except (subprocess.TimeoutExpired, OSError):
            fc.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
