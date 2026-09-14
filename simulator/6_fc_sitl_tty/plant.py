#!/usr/bin/env python3
"""Step 5, but over a tty instead of a pipe. Linux only.

Zephyr will not give you a pipe. On native_sim the UART is a pseudo-terminal,
and on real hardware it is a UART or a USB CDC port, so both ends open a device
path. This runs the existing C++ controller over exactly that, using a pty, so
the eventual move to Zephyr is a path change and nothing else.

    plant.py                                  fc_sitl_cpp
    ---------------------------               ---------------------------
    pybullet world (CtrlAviary)               rate PID + mixer, acro
    pty master                     --0xA5-->  opens /dev/pts/N with --dev
                                   <--0x5A--

    python plant.py --no-gui --fc-args=--script=arm-hover
    python plant.py --baud 115200 --no-gui --fc-args=--script=arm-hover

A pty is a memory buffer with no baud limit, so it will happily run thousands
of exchanges a second and tell you nothing about whether a real wire could keep
up. --baud emulates the serialisation time so the limit becomes visible here
rather than on hardware.
"""

import argparse
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

# The wire format is identical to step 5, so import it rather than copying it.
# Two copies of a packet definition drift, and the C++ side only has one.
REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "simulator" / "5_fc_sitl_cpp"))
import protocol  # noqa: E402

CTRL_HZ = 240
BITS_PER_BYTE = 10          # 8N1: one start bit, eight data, one stop


def find_fc():
    names = ("fc_sitl_cpp", "fc_sitl_cpp.exe")
    found = [p for root in ("bin", "build") for name in names
             for p in (REPO_ROOT / root).rglob(name) if p.is_file()]
    if not found:
        raise SystemExit(
            "fc_sitl_cpp not found. Build it for this platform first:\n"
            "  cmake --preset clang-linux-debug\n"
            "  cmake --build --preset clang-linux-debug")
    return max(found, key=lambda p: p.stat().st_mtime)


def serialisation_s(n_bytes, baud):
    """Wall time those bytes would take on the wire. 0 baud means no limit."""
    return 0.0 if baud <= 0 else n_bytes * BITS_PER_BYTE / baud


def spin(seconds):
    """Busy-wait.

    time.sleep on Linux rounds up to roughly a millisecond, which is coarser
    than the 0.95 ms a 22 byte frame takes at 230400, so sleeping would swamp
    the thing being measured.
    """
    if seconds <= 0:
        return
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        pass


def read_exact(fd, n):
    buf = b""
    while len(buf) < n:
        chunk = os.read(fd, n - len(buf))
        if not chunk:
            return buf
        buf += chunk
    return buf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fc", default=None, help="path to the controller binary")
    ap.add_argument("--fc-args", default="",
                    help="arguments for the controller (use --fc-args=--flag "
                         "when the value starts with a dash)")
    ap.add_argument("--baud", type=int, default=230400,
                    help="emulated wire speed; 0 runs the pty unthrottled")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()

    if not hasattr(os, "openpty"):
        raise SystemExit("ptys are POSIX only. Run this under WSL or Linux.")

    from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
    from gym_pybullet_drones.utils.enums import DroneModel, Physics

    fc_path = Path(args.fc) if args.fc else find_fc()

    master, slave = os.openpty()
    slave_path = os.ttyname(slave)

    fc = subprocess.Popen(
        [str(fc_path), "--dev", slave_path, "--baud", str(args.baud or 115200)]
        + shlex.split(args.fc_args),
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    # The controller has its own handle now. Holding this end open means EOF
    # never arrives when it exits, and the read below would block forever.
    os.close(slave)

    env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=1,
                     physics=Physics.PYB, pyb_freq=CTRL_HZ, ctrl_freq=CTRL_HZ,
                     gui=not args.no_gui)

    dt = 1.0 / CTRL_HZ
    action = np.zeros((1, 4))

    tx = serialisation_s(protocol.SENSOR_LEN, args.baud)
    rx = serialisation_s(protocol.MOTOR_LEN, args.baud)
    budget = tx + rx

    print(f"\nplant: CF2X at {CTRL_HZ} Hz over {slave_path}")
    print(f"controller: {fc_path}")
    if args.baud:
        print(f"wire: {args.baud} baud 8N1  "
              f"{protocol.SENSOR_LEN} B out {tx * 1e3:.2f} ms, "
              f"{protocol.MOTOR_LEN} B back {rx * 1e3:.2f} ms, "
              f"round trip {budget * 1e3:.2f} ms")
        print(f"      control period {dt * 1e3:.2f} ms -> "
              + ("FITS" if budget <= dt else
                 f"DOES NOT FIT, ceiling is {1.0 / budget:.0f} Hz"))
    else:
        print("wire: unthrottled pty, no baud limit")
    print("acro mode. Ctrl+C to quit.\n")

    started = time.perf_counter()
    try:
        obs, _ = env.reset()
        for i in range(int(CTRL_HZ * args.seconds)):
            s = obs[0]
            gyro = Rotation.from_quat(s[3:7]).inv().apply(s[13:16])

            os.write(master, protocol.build_sensor(i, dt, gyro))
            spin(tx)

            frame = read_exact(master, protocol.MOTOR_LEN)
            if len(frame) != protocol.MOTOR_LEN:
                raise RuntimeError(
                    f"short read at step {i}: {len(frame)}B of "
                    f"{protocol.MOTOR_LEN} (controller exit {fc.poll()})")
            spin(rx)

            msg = protocol.parse_motor(frame)       # raises on a bad CRC
            if msg["seq"] != i:
                raise RuntimeError(f"sequence slip: got {msg['seq']}, expected {i}")

            action[0, :] = env.MAX_RPM * np.sqrt(np.clip(msg["m"], 0.0, 1.0))
            obs, _, _, _, _ = env.step(action)

            if i % CTRL_HZ == 0:
                roll, pitch, _ = obs[0][7:10]
                print(f"t={i * dt:5.1f}s {'ARM' if msg['armed'] else '---'} "
                      f"thr={msg['rc'][3]:.2f} z={obs[0][2]:6.2f} m  "
                      f"roll={np.degrees(roll):+6.1f} pitch={np.degrees(pitch):+6.1f} deg  "
                      f"m={['%.2f' % v for v in msg['m']]}")

        elapsed = time.perf_counter() - started
        rate = int(CTRL_HZ * args.seconds) / elapsed
        print(f"\n{int(CTRL_HZ * args.seconds)} exchanges in {elapsed:.2f} s "
              f"= {rate:.0f} Hz achieved, {CTRL_HZ} Hz wanted")
        if args.baud and budget > dt:
            print(f"the wire is the limit: {args.baud} baud tops out near "
                  f"{1.0 / budget:.0f} Hz")

    except KeyboardInterrupt:
        print("\nbye")
    finally:
        env.close()
        os.close(master)          # EOF: the controller's read loop ends
        try:
            fc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            fc.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
