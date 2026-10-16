#!/usr/bin/env python3
"""The plant from step 5, flying the Zephyr firmware instead of the host build.

    plant.py                                  build/fw/app/zephyr/zephyr.exe
    ---------------------------               ---------------------------
    pybullet world (CtrlAviary)               fw/app: same fc/core.hpp,
    reads the radio or a script               built for native_sim
    opens the pty zephyr.exe prints  --0xA5-->
                                     <--0x5A--

    python plant.py --no-gui --script arm-hover
    python plant.py --no-gui --script arm-hover --seconds 5 --record run.csv \\
        --compare simulator/golden/arm_hover.csv
    python plant.py --radio-index 0          # fly it with a radio

The sticks always come from this side: native_sim has no USB host, so the
firmware cannot open a radio itself. It gets the raw channels in the sensor
frame and makes the arming decision, the same as the host build's --input link.

The goldens under simulator/golden/ were recorded from the host build at
step 5. Same core, same frames, same lockstep, so the Zephyr build should
fly the same trajectory. compare.py says how close.

Linux only.
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))
import protocol  # noqa: E402
from channels import read_sticks  # noqa: E402
from flight_trace import TraceWriter  # noqa: E402
from scripts import SCRIPTS  # noqa: E402
from zephyr_sim import ZephyrSim, find_exe  # noqa: E402

CTRL_HZ = 240


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default=None, help="zephyr.exe to run "
                    "(default build/fw/app/zephyr/zephyr.exe)")
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--script", choices=sorted(SCRIPTS),
                    help="canned sticks instead of a radio")
    ap.add_argument("--radio-index", type=int, default=0)
    ap.add_argument("--ch-roll", type=int, default=0)
    ap.add_argument("--ch-pitch", type=int, default=1)
    ap.add_argument("--ch-throttle", type=int, default=2)
    ap.add_argument("--ch-yaw", type=int, default=3)
    ap.add_argument("--ch-arm", type=int, default=4)
    ap.add_argument("--record", default=None, help="write a per-step CSV trace")
    ap.add_argument("--compare", default=None,
                    help="golden trace to check --record against when the run ends")
    args = ap.parse_args()
    if args.compare and not args.record:
        ap.error("--compare needs --record")

    script = SCRIPTS[args.script] if args.script else None
    radio = None
    if script is None:
        from radio_hid import RadioError, open_radio
        try:
            radio = open_radio(args.radio_index)
        except RadioError as exc:
            raise SystemExit(f"radio: {exc}\n(or pass --script to fly without one)")
        print(f"radio: {radio.name}")

    from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
    from gym_pybullet_drones.utils.enums import DroneModel, Physics

    exe = Path(args.exe) if args.exe else find_exe("app")
    fw = ZephyrSim(exe)

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
    trace = TraceWriter(args.record) if args.record else None
    print(f"\nplant: CF2X at {CTRL_HZ} Hz")
    print(f"controller: {exe} on {fw.pty_path}")
    print(f"sticks: {'script ' + args.script if script else 'radio'}, sent over the wire")
    print("acro mode - it will NOT self-level. Ctrl+C to quit.\n")

    try:
        obs, _ = env.reset()
        for i in range(int(CTRL_HZ * args.seconds)):
            s = obs[0]
            gyro = Rotation.from_quat(s[3:7]).inv().apply(s[13:16])

            rc, arm = (None, 0.0)
            if script is not None:
                rc, arm = script(i * dt)
            elif radio is not None:
                sticks = read_sticks(radio, args)
                if sticks is None:
                    radio = None          # flags go clear, the failsafe disarms
                else:
                    rc, arm = sticks

            fw.write(protocol.build_sensor(i, dt, gyro, rc, arm))
            frame = fw.read_exact(protocol.MOTOR_LEN, timeout=2.0)
            if len(frame) != protocol.MOTOR_LEN:
                code = fw.proc.poll()
                raise RuntimeError(
                    f"no reply at step {i}: got {len(frame)}B of {protocol.MOTOR_LEN}"
                    + (f" (zephyr.exe exited, code {code})" if code is not None else ""))

            msg = protocol.parse_motor(frame)
            if msg["seq"] != i:
                raise RuntimeError(f"sequence slip: got {msg['seq']}, expected {i}")

            action[0, :] = env.MAX_RPM * np.sqrt(np.clip(msg["m"], 0.0, 1.0))
            obs, _, _, _, _ = env.step(action)
            if trace is not None:
                trace.row((i + 1) * dt, obs[0], msg)

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
        if radio is not None:
            radio.close()
        if trace is not None:
            trace.close()
        env.close()
        fw.close()

    if args.compare:
        print(f"\ncomparing against {args.compare}", flush=True)
        return subprocess.call([sys.executable,
                                str(Path(__file__).resolve().parents[1] / "common" / "compare.py"),
                                args.compare, args.record])
    return 0


if __name__ == "__main__":
    sys.exit(main())
