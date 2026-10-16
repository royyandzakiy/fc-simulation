#!/usr/bin/env python3
"""Hand-made frames against Zephyr firmware, no physics. Linux only.

Two levels, one per firmware:

    python link_check.py                 # fw/samples/3_link_echo: framing only
    python link_check.py --app app       # fw/app: framing, then flight behaviour

Framing: replies carry the right seq and CRC, garbage between frames is
skipped, a corrupted frame gets no reply. Behaviour (fw/app only): the arm
switch and its hold, the throttle gate, the rate loop's sign, and the RC-loss
failsafe. Each check prints PASS or FAIL; the exit code is the failure count.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))
import protocol  # noqa: E402
from zephyr_sim import ZephyrSim, find_exe  # noqa: E402

DT = 1.0 / 240.0
REPLY_TIMEOUT_S = 1.0


class Link:
    def __init__(self, fw):
        self.fw = fw
        self.seq = 0

    def send(self, gyro=(0.0, 0.0, 0.0), rc=None, arm=-1.0):
        """One lockstep exchange. Returns the parsed reply, or None on timeout."""
        self.seq += 1
        self.fw.write(protocol.build_sensor(self.seq, DT, gyro, rc, arm))
        frame = self.fw.read_exact(protocol.MOTOR_LEN, timeout=REPLY_TIMEOUT_S)
        if len(frame) != protocol.MOTOR_LEN:
            return None
        return protocol.parse_motor(frame)

    def run(self, seconds, **kw):
        """Repeats one frame for a span of simulated time, returns the last reply."""
        msg = None
        for _ in range(round(seconds / DT)):
            msg = self.send(**kw)
        return msg


class Checks:
    def __init__(self):
        self.failed = 0

    def check(self, name, ok, detail=""):
        print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""))
        self.failed += 0 if ok else 1


def framing(link, c):
    msg = link.send()
    c.check("reply to a frame", msg is not None)
    c.check("seq echoed", msg is not None and msg["seq"] == link.seq,
            f"sent {link.seq}, got {msg and msg['seq']}")

    # Bytes that are not a frame, including a stray sync byte mid-garbage.
    link.fw.write(bytes([0x00, 0x13, 0x11, 0x5A, 0xFF]))
    msg = link.send()
    c.check("garbage before a frame is skipped", msg is not None and msg["seq"] == link.seq)

    link.seq += 1
    bad = bytearray(protocol.build_sensor(link.seq, DT, (0.0, 0.0, 0.0)))
    bad[10] ^= 0xFF
    link.fw.write(bytes(bad))
    late = link.fw.read_exact(protocol.MOTOR_LEN, timeout=0.3)
    c.check("corrupted frame gets no reply", len(late) == 0, f"got {len(late)} B")

    msg = link.send()
    c.check("next good frame answered", msg is not None and msg["seq"] == link.seq)


def behaviour(link, c):
    idle = (0.0, 0.0, 0.0, 0.0)

    msg = link.run(0.5, rc=None, arm=1.0)
    c.check("no valid sticks: stays disarmed", msg and not msg["armed"])

    # Switch up from the first valid frame: must be seen down before it arms.
    msg = link.run(0.5, rc=idle, arm=1.0)
    c.check("switch already up at link-up: refused", msg and not msg["armed"])

    link.run(0.1, rc=idle, arm=-1.0)
    msg = link.run(0.1, rc=idle, arm=1.0)
    c.check("switch up 0.1 s: not yet armed (hold 0.2 s)", msg and not msg["armed"])
    msg = link.run(0.2, rc=idle, arm=1.0)
    c.check("switch up 0.3 s: armed", msg and msg["armed"])

    msg = link.run(0.1, rc=(0.0, 0.0, 0.0, 0.5), arm=1.0)
    spread = max(msg["m"]) - min(msg["m"]) if msg else 1.0
    c.check("level at half throttle: equal motors", msg and spread < 1e-3,
            f"m={msg and ['%.3f' % v for v in msg['m']]}")

    msg = link.send(gyro=(1.0, 0.0, 0.0), rc=(0.0, 0.0, 0.0, 0.5), arm=1.0)
    c.check("rolling right: left motors drop", msg and msg["m"][2] < msg["m"][0],
            f"m={msg and ['%.3f' % v for v in msg['m']]}")

    link.run(0.2, rc=(0.0, 0.0, 0.0, 0.5), arm=1.0)
    msg = link.run(0.3, rc=None, arm=1.0)
    c.check("sticks lost 0.3 s: still armed (timeout 0.5 s)", msg and msg["armed"])
    msg = link.run(0.3, rc=None, arm=1.0)
    c.check("sticks lost 0.6 s: failsafe disarm", msg and not msg["armed"] and max(msg["m"]) == 0.0)

    msg = link.run(0.5, rc=idle, arm=1.0)
    c.check("sticks back, switch still up: stays disarmed", msg and not msg["armed"])

    link.run(0.1, rc=(0.0, 0.0, 0.0, 0.5), arm=-1.0)
    msg = link.run(0.3, rc=(0.0, 0.0, 0.0, 0.5), arm=1.0)
    c.check("throttle up at arming: refused", msg and not msg["armed"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--app", default="3_link_echo",
                    help="build/fw/<app> to run: 3_link_echo or app")
    ap.add_argument("--exe", default=None, help="zephyr.exe to run instead")
    args = ap.parse_args()

    exe = Path(args.exe) if args.exe else find_exe(args.app)
    c = Checks()
    with ZephyrSim(exe, quiet=True) as fw:
        print(f"{exe.parent.parent.name}: {fw.uart} is {fw.pty_path}\n")
        link = Link(fw)
        framing(link, c)
        if args.app == "app":
            print()
            behaviour(link, c)

    print(f"\n{'all passed' if c.failed == 0 else f'{c.failed} failed'}")
    return c.failed


if __name__ == "__main__":
    sys.exit(main())
