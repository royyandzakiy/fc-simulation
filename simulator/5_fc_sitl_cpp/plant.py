#!/usr/bin/env python3
"""The world and the IMU, talking to the C++ flight controller.

Same split as scripts/fc_sitl, but the controller is the real
`fc_sitl_cpp` binary instead of a Python stand-in, and the link is the
binary CRC framing declared in src/app/fc_core.hpp rather than JSON.

    plant.py                                 fc_sitl_cpp.exe
    ---------------------------              ---------------------------
    pybullet world (CtrlAviary)              rate PID + mixer, acro
    the "IMU": derives body rates  --0xA5-->
    m -> RPM, steps physics        <--0x5A-- reads the gamepad over SDL3

    python plant.py                          # hold LB to arm
    python plant.py --no-gui --fc-args=--script=arm-hover

Strict lockstep: every physics step blocks on the controller's reply, so
nothing runs in real time and runs reproduce exactly.
"""

import argparse
import shlex
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import protocol

CTRL_HZ = 240
# scripts/sitl_cpp/plant.py -> repo root. Anchored to this file, not the cwd,
# so the script runs from anywhere.
REPO_ROOT = Path(__file__).resolve().parents[2]


def find_fc():
    """Newest fc_sitl_cpp binary under bin/ or build/.

    Newest rather than first so it picks up whichever toolchain you built
    last, instead of a stale binary from another compiler.
    """
    names = ("fc_sitl_cpp.exe", "fc_sitl_cpp")
    found = [p for root in ("bin", "build")
             for name in names
             for p in (REPO_ROOT / root).rglob(name) if p.is_file()]
    if not found:
        raise SystemExit(
            "fc_sitl_cpp not found. Build it first:\n"
            "  cmake --build build/clang-cl-debug\n"
            "(check add_subdirectory(src/app) is enabled in CMakeLists.txt)\n"
            "or point at it with --fc <path>.")
    return max(found, key=lambda p: p.stat().st_mtime)


def start_fc(fc_path, extra_args):
    return subprocess.Popen(
        [str(fc_path)] + extra_args,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        # stderr inherited: the controller's own logging lands on your
        # terminal and critically stays off stdout, which carries the frames
    )


def read_exact(stream, n):
    """Pipes are allowed to return short reads; the protocol is not."""
    buf = b""
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            return buf
        buf += chunk
    return buf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fc", default=None, help="path to the controller binary")
    ap.add_argument("--fc-args", default="",
                    help="arguments passed through to the controller "
                         "(use --fc-args=--flag when the value starts with a dash)")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--seconds", type=float, default=120.0)
    args = ap.parse_args()

    from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
    from gym_pybullet_drones.utils.enums import DroneModel, Physics

    fc_path = Path(args.fc) if args.fc else find_fc()
    fc = start_fc(fc_path, shlex.split(args.fc_args))

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
    print(f"\nplant: CF2X at {CTRL_HZ} Hz")
    print(f"controller: {fc_path}")
    print("acro mode - it will NOT self-level. Ctrl+C to quit.\n")

    try:
        obs, _ = env.reset()
        for i in range(int(CTRL_HZ * args.seconds)):
            s = obs[0]
            # The IMU lives here. obs reports angular velocity in the world
            # frame; a real gyro measures body rates, so rotate before sending.
            gyro = Rotation.from_quat(s[3:7]).inv().apply(s[13:16])

            fc.stdin.write(protocol.build_sensor(i, dt, gyro))
            fc.stdin.flush()

            # lockstep: block here until the controller answers
            frame = read_exact(fc.stdout, protocol.MOTOR_LEN)
            if len(frame) != protocol.MOTOR_LEN:
                code = fc.poll()
                raise RuntimeError(
                    f"short read at step {i}: got {len(frame)}B of "
                    f"{protocol.MOTOR_LEN}"
                    + (f" (controller exited, code {code})" if code is not None
                       else ""))

            msg = protocol.parse_motor(frame)       # raises on a bad CRC
            if msg["seq"] != i:
                raise RuntimeError(f"sequence slip: got {msg['seq']}, expected {i}")

            # m is a thrust fraction per motor; thrust goes as rpm squared, so
            # normalised demand -> rpm is a square root.
            action[0, :] = env.MAX_RPM * np.sqrt(np.clip(msg["m"], 0.0, 1.0))
            obs, _, _, _, _ = env.step(action)

            if not args.no_gui:
                time.sleep(dt)
            if i % CTRL_HZ == 0:
                roll, pitch, _ = obs[0][7:10]
                print(f"t={i * dt:5.1f}s {'ARM' if msg['armed'] else '---'} "
                      f"thr={msg['rc'][3]:.2f} z={obs[0][2]:6.2f} m  "
                      f"roll={np.degrees(roll):+6.1f} pitch={np.degrees(pitch):+6.1f} deg  "
                      f"m={['%.2f' % v for v in msg['m']]}")

    except KeyboardInterrupt:
        print("\nbye")
    finally:
        env.close()
        # Closing stdin lets the controller's read loop hit EOF and exit on
        # its own; terminate() is the fallback if it does not take the hint.
        try:
            fc.stdin.close()
            fc.wait(timeout=2)
        except (subprocess.TimeoutExpired, OSError):
            fc.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
