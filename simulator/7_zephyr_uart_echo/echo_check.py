#!/usr/bin/env python3
"""Talk to the fw/samples/2_uart_echo firmware over its pty. Linux only.

No physics and no frames yet. This answers one question: can a script reach
a Zephyr binary through the device path it prints, and how long does a round
trip take.

    python simulator/7_zephyr_uart_echo/echo_check.py
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))
from zephyr_sim import ZephyrSim, find_exe  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--exe", default=None, help="zephyr.exe to run")
    ap.add_argument("--rounds", type=int, default=200)
    args = ap.parse_args()

    exe = Path(args.exe) if args.exe else find_exe("2_uart_echo")
    with ZephyrSim(exe) as fw:
        print(f"{fw.uart} is {fw.pty_path}")

        # Every byte value, so anything a tty would translate shows up here.
        payload = bytes(range(256))
        fw.write(payload)
        back = fw.read_exact(len(payload))
        if back != payload:
            print(f"FAIL: sent 256 bytes, got {len(back)} back, "
                  f"first difference at {next(i for i, (a, b) in enumerate(zip(payload, back)) if a != b) if back else 0}")
            return 1
        print("all 256 byte values came back unchanged")

        # Round trip of a sensor-frame-sized message, the cost every
        # lockstep step will pay before any computing happens.
        msg = bytes(43)
        times = []
        for _ in range(args.rounds):
            t0 = time.perf_counter()
            fw.write(msg)
            fw.read_exact(len(msg))
            times.append(time.perf_counter() - t0)
        times.sort()
        print(f"43 B round trip over {args.rounds} rounds: "
              f"median {times[len(times) // 2] * 1e3:.2f} ms, "
              f"p95 {times[int(len(times) * 0.95)] * 1e3:.2f} ms")
    return 0


if __name__ == "__main__":
    sys.exit(main())
