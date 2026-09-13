#!/usr/bin/env python3
"""
bridge.py - gym-pybullet-drones <-> fc_simulation, in lockstep.

    cd scripts/hil_bridge
    py -3.11 -m venv .venv
    .venv/Scripts/python.exe -m pip install -r requirements.txt

    Build the controller first, then:
        .venv/Scripts/python.exe bridge.py              # plug the pad in first
        .venv/Scripts/python.exe bridge.py --no-rerun   # console only
        .venv/Scripts/python.exe bridge.py --no-gui --seconds 5

Runs unchanged on Windows: subprocess pipes are binary by default there,
and fc_simulation puts its own stdio into binary mode on the C++ side.

Sticks are read by fc_simulation from the gamepad. This process is purely
the plant: it ships gyro, receives motor demands, and steps physics.

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

import numpy as np
from construct import (Array, Checksum, Const, Float32l, Int8ul, Int32ul,
                       RawCopy, Struct, this)
from crc import Calculator, Configuration
from scipy.spatial.transform import Rotation

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

CTRL_HZ = 240
DURATION_S = 60

# scripts/hil_bridge/bridge.py -> repo root. Anchoring to the file rather
# than the cwd means the script runs from anywhere.
REPO_ROOT = Path(__file__).resolve().parents[2]

# --- wire format ---------------------------------------------------
# Declared once. construct computes the trailing CRC on build and
# verifies it on parse, so there is no offset arithmetic to get wrong.
# Mirrors the #pragma pack(1) structs in src/app/fc_core.hpp.

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

# The C++ side static_asserts these same sizes, so a mismatch here means
# the two definitions have drifted apart.
assert SensorPacket.sizeof() == 22, SensorPacket.sizeof()
assert MOTOR_LEN == 39, MOTOR_LEN


def find_fc(explicit=None):
    """Locate the fc_simulation binary.

    Each toolchain writes to its own directory (bin/clang-cl, bin/msvc,
    bin/clang, ...) and multi-config generators add Debug/ or Release/
    below that. Rather than enumerate every combination, take the most
    recently built match - which is almost always the one you want.
    """
    if explicit:
        path = Path(explicit).expanduser()
        if not path.is_file():
            raise SystemExit(f"--fc: not a file: {path}")
        return str(path.resolve())

    exe = "fc_simulation.exe" if sys.platform == "win32" else "fc_simulation"
    matches = [p for d in ("bin", "build")
               for p in (REPO_ROOT / d).rglob(exe) if p.is_file()]
    if matches:
        return str(max(matches, key=lambda p: p.stat().st_mtime).resolve())

    on_path = shutil.which(exe)
    if on_path:
        return on_path

    raise SystemExit(
        f"could not find {exe} under {REPO_ROOT}. Build it first:\n"
        "    cmake --build build/clang-cl-debug\n"
        "or point at it with --fc <path>.")


def open_rerun(enabled):
    """Start rerun if wanted and installed, else return None.

    The logging API moved in rerun 0.23 (Scalar -> Scalars, and
    set_time_sequence folded into set_time), so pick the pair this install
    actually has instead of pinning users to one SDK version.
    """
    if not enabled:
        return None
    try:
        import rerun as rr
    except ImportError:
        print("rerun not installed, console output only", file=sys.stderr)
        return None

    try:
        rr.init("fc_simulation", spawn=True)
    except RuntimeError as exc:
        # Since 0.37 the SDK no longer ships the viewer binary, so spawn
        # fails unless it was installed separately. Plotting is a nicety,
        # not a reason to lose the run.
        print(f"rerun viewer not available ({exc.args[0].splitlines()[0]}), "
              "console output only\n"
              "  get it from https://github.com/rerun-io/rerun/releases",
              file=sys.stderr)
        return None

    if hasattr(rr, "Scalars"):
        return rr, rr.Scalars, lambda i: rr.set_time("step", sequence=i)
    return rr, rr.Scalar, lambda i: rr.set_time_sequence("step", i)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--fc", default=None, help="path to the fc_simulation binary")
    ap.add_argument("--pad-throttle", choices=["stick", "trigger"], default="stick")
    ap.add_argument("--no-gui", action="store_true", help="no pybullet window")
    ap.add_argument("--no-rerun", action="store_true")
    ap.add_argument("--seconds", type=float, default=DURATION_S,
                    help=f"simulated seconds (default: {DURATION_S})")
    args = ap.parse_args(argv)

    fc_path = find_fc(args.fc)
    print(f"fc:   {fc_path}", file=sys.stderr)

    rerun = open_rerun(not args.no_rerun)

    fc = subprocess.Popen(
        [fc_path, "--throttle", args.pad_throttle],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        # stderr inherited: firmware logs land on your terminal, and
        # critically stay off stdout, which carries the binary stream
    )

    env = None
    try:
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

        for i in range(int(CTRL_HZ * args.seconds)):
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
                if fc.poll() is not None:
                    raise RuntimeError(
                        f"controller exited (code {fc.returncode}) at step {i}")
                raise RuntimeError(f"short read at step {i}: {len(reply)}B")

            p = MotorPacket.parse(reply).body.value      # raises on bad CRC
            if p.seq != i:
                raise RuntimeError(f"sequence slip: got {p.seq}, expected {i}")

            # thrust goes as rpm^2, so normalized -> rpm is a square root
            action[0, :] = env.MAX_RPM * np.sqrt(np.clip(p.m, 0.0, 1.0))
            obs, _, _, _, _ = env.step(action)

            if rerun is not None:
                rr, scalar, set_step = rerun
                set_step(i)
                rr.log("world/drone", rr.Transform3D(
                    translation=pos,
                    rotation=rr.Quaternion(xyzw=quat_xyzw)))
                for k in range(4):
                    rr.log(f"motor/m{k}", scalar(p.m[k]))
                for k, name in enumerate(("roll", "pitch", "yaw", "throttle")):
                    rr.log(f"stick/{name}", scalar(p.rc[k]))
                rr.log("state/armed", scalar(float(p.armed)))
                rr.log("state/altitude", scalar(float(pos[2])))

            if i % CTRL_HZ == 0:
                print(f"t={i*dt:5.1f} {'ARM' if p.armed else '---'} "
                      f"z={pos[2]:6.3f}  "
                      f"gyro=[{gyro[0]:+6.2f} {gyro[1]:+6.2f} {gyro[2]:+6.2f}]  "
                      f"rc={['%+.2f' % v for v in p.rc]}  "
                      f"m={['%.2f' % v for v in p.m]}")
    finally:
        if env is not None:
            env.close()
        # Closing stdin lets the controller's read loop hit EOF and exit on
        # its own; terminate() is the fallback if it does not take the hint.
        if fc.stdin and not fc.stdin.closed:
            fc.stdin.close()
        try:
            fc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            fc.terminate()

    return 0


if __name__ == "__main__":
    sys.exit(main())
