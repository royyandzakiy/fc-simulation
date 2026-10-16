#!/usr/bin/env python3
"""Compare a flight trace against a golden one. Exit 0 when they match.

    python simulator/common/compare.py simulator/golden/arm_hover.csv run.csv

Position is compared in metres, attitude in degrees, motors as 0..1 demand.
The armed column must match exactly: arming a step late is a behaviour
change, not noise.
"""

import argparse
import sys

import numpy as np

from flight_trace import load


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("golden")
    ap.add_argument("run")
    ap.add_argument("--pos-tol", type=float, default=0.01, help="metres")
    ap.add_argument("--att-tol", type=float, default=0.5, help="degrees")
    ap.add_argument("--motor-tol", type=float, default=0.01, help="0..1 demand")
    args = ap.parse_args()

    g, r = load(args.golden), load(args.run)
    n = min(len(g["t"]), len(r["t"]))
    if len(g["t"]) != len(r["t"]):
        print(f"length differs: golden {len(g['t'])} rows, run {len(r['t'])}, "
              f"comparing the first {n}")

    groups = (
        (("x", "y", "z"), args.pos_tol, 1.0, "m"),
        (("roll", "pitch", "yaw"), args.att_tol, np.degrees(1.0), "deg"),
        (("m0", "m1", "m2", "m3"), args.motor_tol, 1.0, ""),
        (("armed",), 0.0, 1.0, ""),
    )

    ok = True
    for cols, tol, scale, unit in groups:
        for c in cols:
            err = np.abs(g[c][:n] - r[c][:n]) * scale
            worst = int(np.argmax(err))
            bad = np.nonzero(err > tol)[0]
            status = "ok  " if bad.size == 0 else "FAIL"
            line = (f"{status} {c:6s} max {err[worst]:.6f} {unit} "
                    f"at t={g['t'][worst]:.3f}s (tol {tol})")
            if bad.size:
                line += f", first over at t={g['t'][bad[0]]:.3f}s"
                ok = False
            print(line)

    print("MATCH" if ok else "MISMATCH")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
