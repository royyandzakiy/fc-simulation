#!/usr/bin/env python3
"""
bridge.py - gym-pybullet-drones <-> fc_min, in lockstep.

    pip install gym-pybullet-drones construct crc scipy rerun-sdk

    Linux/macOS:  cmake --preset default && cmake --build build
    Windows:      cmake --preset windows && cmake --build --preset windows

    python3 bridge.py                  # plug the pad in first
    python3 bridge.py --no-rerun       # console only

Runs unchanged on Windows: subprocess pipes are binary by default there,
and fc_min puts its own stdio into binary mode on the C++ side.

Sticks are read by fc_min from the gamepad. This process is purely the
plant: it ships gyro, receives motor demands, and steps physics.

Nothing runs in realtime. Every physics step waits for the controller's
reply, so latency is irrelevant and runs are reproducible.

gym-pybullet-drones API, from commit 7ebad1e:
    CtrlAviary.step(action) takes (NUM_DRONES, 4) of RPM, 0..MAX_RPM
    obs[i] is 20 floats:
        [0:3]   position xyz       world
        [3:7]   quaternion xyzw    pybullet order, not wxyz
        [7:10]  roll/pitch/yaw     world
        [10:13] velocity xyz       world
        [13:16] angular velocity   world frame, must be rotated to body
        [16:20] last clipped action
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path
from pathlib import Path

import numpy as np
from construct import (Array, Checksum, Const, Float32l, Int8ul, Int32ul,
                       RawCopy, Struct, this)
from crc import Calculator, Configuration
from scipy.spatial.transform import Rotation

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

CTRL_HZ = 240
DURATION_S = 60

# --- wire format ---------------------------------------------------
# Declared once. construct computes the trailing CRC on build and
# verifies it on parse, so there is no offset arithmetic to get wrong.

CRC8 = Calculator(Configuration(width=8, polynomial=0xD5, init_value=0,
                                final_xor_value=0, reverse_input=False,
                                reverse_output=False))

SensorPacket = Struct(
    "body" / RawCopy(Struct(
        "sync" / Const(0xA5, Int8ul),
        "seq"  / Int32ul,
        "dt"   / Float32l,
        "gyro" / Array(3, Float32l),
    )),
    "crc" / Checksum(Int8ul, CRC8.checksum, this.body.data),
)

MotorPacket = Struct(
    "body" / RawCopy(Struct(
        "sync"  / Const(0x5A, Int8ul),
        "seq"   / Int32ul,
        "m"     / Array(4, Float32l),
        "rc"    / Array(4, Float32l),
        "armed" / Int8ul,
    )),
    "crc" / Checksum(Int8ul, CRC8.checksum, this.body.data),
)

MOTOR_LEN = MotorPacket.sizeof()


def find_fc(explicit):
    """Locate the controller binary.

    Single-config generators drop it in build/. Visual Studio is
    multi-config and uses build/Release or build/Debug unless told
    otherwise, so search both, and try the .exe suffix.
    """
    if explicit:
        return explicit
    roots = [Path("build"), Path("build/RelWithDebInfo"), Path("build/Release"),
             Path("build/Debug"), Path(".")]
    for root in roots:
        for name in ("fc_min.exe", "fc_min"):
            candidate = root / name
            if candidate.is_file():
                return str(candidate)
    raise SystemExit("fc_min not found. Build it, or pass --fc <path>.")


def find_fc():
    """Locate fc_min across generators and platforms.

    Single-config generators (Ninja, Makefiles) put it in build/bin.
    Multi-config ones (Visual Studio) add a per-config subdirectory, and
    Windows appends .exe. CMakeLists pins RUNTIME_OUTPUT_DIRECTORY to
    build/bin for every config, so the first candidate usually wins.
    """
    exe = "fc_min.exe" if sys.platform == "win32" else "fc_min"
    candidates = [
        Path("build") / "bin" / exe,
        Path("build") / "bin" / "Release" / exe,
        Path("build") / "Release" / exe,
        Path("build") / exe,
        Path(exe),
    ]
    for c in candidates:
        if c.is_file():
            return str(c.resolve())

    found = shutil.which("fc_min")
    if found:
        return found

    raise SystemExit(
        f"could not find {exe}. Build it first, or pass --fc <path>.\n"
        "  Looked in: " + ", ".join(str(c) for c in candidates))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--fc", default=None, help="path to the fc_min binary")
    ap.add_argument("--pad-throttle", choices=["stick", "trigger"], default="stick")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--no-rerun", action="store_true")
    args = ap.parse_args(argv)

    fc_path = args.fc or find_fc()

    rr = None
    if not args.no_rerun:
        try:
            import rerun as rr_mod
            rr = rr_mod
            rr.init("fc_min", spawn=True)
        except ImportError:
            print("rerun not installed, console output only", file=sys.stderr)

    fc = subprocess.Popen(
        [find_fc(args.fc), "--throttle", args.pad_throttle],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        # stderr inherited: firmware logs land on your terminal, and
        # critically stay off stdout, which carries the binary stream
    )

    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        physics=Physics.PYB,
        pyb_freq=CTRL_HZ,
        ctrl_freq=CTRL_HZ,
        gui=not args.no_gui,
    )

    obs, _ = env.reset()
    dt = 1.0 / CTRL_HZ
    action = np.zeros((1, 4))

    try:
        for i in range(CTRL_HZ * DURATION_S):
            s = obs[0]
            pos, quat_xyzw, ang_v_world = s[0:3], s[3:7], s[13:16]

            # obs gives angular velocity in the world frame; the gyro on a
            # real drone measures body rates, so rotate before sending
            gyro = Rotation.from_quat(quat_xyzw).inv().apply(ang_v_world)

            fc.stdin.write(SensorPacket.build(
                {"body": {"value": {"seq": i, "dt": dt, "gyro": list(gyro)}}}))
            fc.stdin.flush()

            # lockstep: block here until the controller answers
            reply = fc.stdout.read(MOTOR_LEN)
            if len(reply) != MOTOR_LEN:
                raise RuntimeError(f"short read at step {i}: {len(reply)}B")

            p = MotorPacket.parse(reply).body.value      # raises on bad CRC
            if p.seq != i:
                raise RuntimeError(f"sequence slip: got {p.seq}, expected {i}")

            # thrust goes as rpm^2, so normalized -> rpm is a square root
            action[0, :] = env.MAX_RPM * np.sqrt(np.clip(p.m, 0.0, 1.0))
            obs, _, _, _, _ = env.step(action)

            if rr is not None:
                rr.set_time_sequence("step", i)
                rr.log("world/drone", rr.Transform3D(
                    translation=pos,
                    rotation=rr.Quaternion(xyzw=quat_xyzw)))
                for k in range(4):
                    rr.log(f"motor/m{k}", rr.Scalar(p.m[k]))
                for k, name in enumerate(("roll", "pitch", "yaw", "throttle")):
                    rr.log(f"stick/{name}", rr.Scalar(p.rc[k]))
                rr.log("state/armed", rr.Scalar(float(p.armed)))
                rr.log("state/altitude", rr.Scalar(float(pos[2])))

            if i % CTRL_HZ == 0:
                print(f"t={i*dt:5.1f} {'ARM' if p.armed else '---'} "
                      f"z={pos[2]:6.3f}  "
                      f"gyro=[{gyro[0]:+6.2f} {gyro[1]:+6.2f} {gyro[2]:+6.2f}]  "
                      f"rc={['%+.2f' % v for v in p.rc]}  "
                      f"m={['%.2f' % v for v in p.m]}")
    finally:
        env.close()
        fc.terminate()


if __name__ == "__main__":
    sys.exit(main())