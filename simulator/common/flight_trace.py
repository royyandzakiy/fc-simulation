#!/usr/bin/env python3
"""Per-step flight trace as CSV, written by the plants and read by compare.py.

One row per control step: time, position, attitude, motor demands and the
armed flag. Lockstep makes a run reproducible, so two traces from the same
script should match closely whichever controller binary produced them.
"""

import csv

import numpy as np

FIELDS = ("t", "x", "y", "z", "roll", "pitch", "yaw",
          "m0", "m1", "m2", "m3", "armed")


class TraceWriter:
    def __init__(self, path):
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(FIELDS)

    def row(self, t, state, msg):
        """state is CtrlAviary's obs[0], msg is protocol.parse_motor()'s dict."""
        x, y, z = state[0:3]
        roll, pitch, yaw = state[7:10]
        self._w.writerow([f"{t:.6f}"]
                         + [f"{v:.6f}" for v in (x, y, z, roll, pitch, yaw)]
                         + [f"{v:.6f}" for v in msg["m"]]
                         + [int(msg["armed"])])

    def close(self):
        self._f.close()


def load(path):
    """Trace -> dict of column name to numpy array."""
    data = np.genfromtxt(path, delimiter=",", names=True)
    return {name: data[name] for name in FIELDS}
