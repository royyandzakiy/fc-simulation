#!/usr/bin/env python3
"""Flight controller, software in the loop. Knows nothing about physics.

Stands in for the C++ fc_simulation. It sees two things - gyro readings
arriving on stdin, and the gamepad it reads itself - and answers with a motor
demand per rotor. No pybullet, no world model, no idea it is being simulated.

    stdin   {"seq":41,"dt":0.004167,"gyro":[0.01,-0.02,0.00]}
    stdout  {"seq":41,"m":[0.51,0.49,0.52,0.48],"armed":1,"rc":[...]}

Acro (rate) mode. A gyro measures body rates, so a controller handed nothing
but rates can only close a rate loop. The sticks command a rotation rate, not
an angle: let go and the drone keeps whatever bank it had. Stabilised mode
needs attitude, which needs an accelerometer in the sensor line and an
estimator here.

Normally launched by plant.py rather than run directly.

    python fc_sitl.py --no-pad          # zero sticks, stays disarmed
    python fc_sitl.py --script arm-climb-roll

stdout is the data channel, so everything this prints for a human goes to
stderr.
"""

import argparse
import ctypes
import json
import sys
from ctypes import wintypes

import numpy as np

# --- stick feel -----------------------------------------------------------
MAX_RATE = np.radians(200.0)     # rad/s at full stick, both roll and pitch
MAX_YAW_RATE = np.radians(200.0)
ARM_THROTTLE = 0.05              # throttle must be below this to arm
IDLE_THRUST = 0.03               # thrust fraction while armed at zero throttle

# --- rate PID, in thrust fraction per rad/s -------------------------------
# Roll and pitch are driven by thrust differences across the arms. Yaw is
# driven by rotor drag, and on this airframe that is a much stronger effect
# than it looks - so yaw gets its own, far smaller gains. Sharing one set
# makes the yaw axis ring at the step rate.
KP, KI, KD = 0.055, 0.06, 0.0016
KP_YAW, KI_YAW, KD_YAW = 0.004, 0.004, 0.0

I_LIMIT = 0.3                    # same clamp as RatePid in src/app/fc_core.hpp

# Low-pass on the derivative, as a fraction of the new sample kept per step.
# Without this the D term amplifies step-to-step noise until the loop
# oscillates at the Nyquist frequency - which looks like a perfectly steady
# rate if you sample it slower than every step. Real firmware filters D too.
D_LPF = 0.08

# CF2X X-layout mixing, the table gym-pybullet-drones' own DSLPIDControl uses.
# Rows are motors 0-3, columns are [roll, pitch, yaw].
MIXER = np.array([[-0.5, -0.5, -1.0],
                  [-0.5, +0.5, +1.0],
                  [+0.5, +0.5, -1.0],
                  [+0.5, -0.5, +1.0]])

DEADZONE_L = 7849 / 32767.0
DEADZONE_R = 8689 / 32767.0
LB = 0x0100


def log(*a):
    """Human output goes to stderr. stdout carries the motor stream."""
    print(*a, file=sys.stderr, flush=True)


# --- gamepad --------------------------------------------------------------

class Gamepad(ctypes.Structure):
    _fields_ = [("wButtons", wintypes.WORD),
                ("bLeftTrigger", ctypes.c_ubyte),
                ("bRightTrigger", ctypes.c_ubyte),
                ("sThumbLX", ctypes.c_short),
                ("sThumbLY", ctypes.c_short),
                ("sThumbRX", ctypes.c_short),
                ("sThumbRY", ctypes.c_short)]


class State(ctypes.Structure):
    _fields_ = [("dwPacketNumber", wintypes.DWORD), ("Gamepad", Gamepad)]


def deadzone(v, dz):
    """Centre dead band, for sticks that mean 'nothing' when released."""
    if abs(v) < dz:
        return 0.0
    return (abs(v) - dz) / (1.0 - dz) * (1.0 if v > 0 else -1.0)


def throttle_from(v, edge=0.02):
    """Stick travel to 0..1: fully DOWN is zero, fully UP is full.

    No centre dead band - throttle is not a self-centring control. The small
    edges guarantee it bottoms out at exactly 0.0, which the arm gate needs.
    """
    t = (v + 1.0) / 2.0
    if t <= edge:
        return 0.0
    if t >= 1.0 - edge:
        return 1.0
    return (t - edge) / (1.0 - 2.0 * edge)


NEUTRAL = {"throttle": 0.0, "yaw": 0.0, "roll": 0.0, "pitch": 0.0, "lb": False}


def open_pad(slot):
    """Return a function giving the current stick positions."""
    for name in ("XInput1_4.dll", "XInput1_3.dll", "XInput9_1_0.dll"):
        try:
            xinput = ctypes.windll.LoadLibrary(name)
            break
        except OSError:
            continue
    else:
        sys.exit("no XInput DLL found - is this Windows?")

    state = State()

    def read():
        if xinput.XInputGetState(slot, ctypes.byref(state)) != 0:
            return dict(NEUTRAL)                  # unplugged: neutral sticks
        g = state.Gamepad
        return {"throttle": throttle_from(max(-1.0, g.sThumbLY / 32767.0)),
                "yaw": deadzone(max(-1.0, g.sThumbLX / 32767.0), DEADZONE_L),
                "roll": deadzone(max(-1.0, g.sThumbRX / 32767.0), DEADZONE_R),
                "pitch": deadzone(max(-1.0, g.sThumbRY / 32767.0), DEADZONE_R),
                "lb": bool(g.wButtons & LB)}

    if xinput.XInputGetState(slot, ctypes.byref(state)) != 0:
        sys.exit(f"no controller in slot {slot}.\n"
                 "  Check it with: python ..\\gamepad_test\\gamepad_test.py\n"
                 "  Or run the controller with --no-pad.")
    return read


def scripted_pad(name):
    """Stick input from a script, so the pair can be flown without hands."""
    def arm_climb_roll(t):
        if t < 0.5:
            return {**NEUTRAL, "throttle": 0.0}
        if t < 0.55:
            return {**NEUTRAL, "throttle": 0.0, "lb": True}   # arm
        if t < 3.0:
            return {**NEUTRAL, "throttle": 0.75}              # climb
        return {**NEUTRAL, "throttle": 0.75, "roll": 1.0}     # roll right

    scripts = {"arm-climb-roll": arm_climb_roll}
    if name not in scripts:
        sys.exit(f"unknown script {name!r}, have: {', '.join(scripts)}")

    clock = {"t": 0.0}
    fn = scripts[name]

    def read(dt):
        clock["t"] += dt
        return fn(clock["t"])
    return read


# --- the controller -------------------------------------------------------

class RatePid:
    """One axis. Integral clamped, like RatePid in src/app/fc_core.hpp.

    The derivative is taken on the *measurement*, not on the error. Both give
    the same answer while the setpoint is steady, but differentiating the error
    puts a spike of (stick_step / dt) through the D term the instant you move a
    stick - enough to slam all four motors to their rails on one sample. Real
    firmware avoids that the same way. Note src/app/fc_core.hpp still
    differentiates the error, so the two are not identical here.
    """

    def __init__(self, kp=KP, ki=KI, kd=KD, d_lpf=D_LPF):
        self.kp, self.ki, self.kd, self.d_lpf = kp, ki, kd, d_lpf
        self.integral = 0.0
        self.prev_meas = None
        self.d_filt = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_meas = None
        self.d_filt = 0.0

    def step(self, setpoint, meas, dt):
        err = setpoint - meas
        self.integral = float(np.clip(self.integral + err * dt, -I_LIMIT, I_LIMIT))
        if self.prev_meas is None or dt <= 1e-6:
            d = 0.0                       # nothing to differentiate yet
        else:
            d = -(meas - self.prev_meas) / dt
        self.prev_meas = meas
        self.d_filt += self.d_lpf * (d - self.d_filt)
        return self.kp * err + self.ki * self.integral + self.kd * self.d_filt


class Controller:
    """Sticks plus body rates in, four motor demands out. No state beyond arm."""

    def __init__(self):
        self.armed = False
        self.was_lb = False
        self.pids = (RatePid(), RatePid(),
                     RatePid(KP_YAW, KI_YAW, KD_YAW))

    def update_arming(self, pad):
        """LB toggles on the rising edge. Arming needs throttle at the bottom."""
        if pad["lb"] and not self.was_lb:
            if self.armed:
                self.armed = False
                log("DISARMED")
            elif pad["throttle"] <= ARM_THROTTLE:
                self.armed = True
                for p in self.pids:
                    p.reset()          # no stale integral from the last flight
                log("ARMED")
            else:
                log(f"arm refused - throttle is {pad['throttle'] * 100:.0f}%, "
                    "pull it all the way down")
        self.was_lb = pad["lb"]

    def motors(self, pad, gyro, dt):
        """Four thrust fractions, 0..1. Disarmed is always a hard zero."""
        if not self.armed:
            return [0.0, 0.0, 0.0, 0.0]

        # acro: the stick asks for a rotation RATE, not an angle
        sp = np.array([pad["roll"] * MAX_RATE,
                       pad["pitch"] * MAX_RATE,
                       -pad["yaw"] * MAX_YAW_RATE])
        meas = np.asarray(gyro, dtype=float)
        u = np.array([pid.step(s, m, dt)
                      for pid, s, m in zip(self.pids, sp, meas)])

        base = IDLE_THRUST + pad["throttle"] * (1.0 - IDLE_THRUST)
        return np.clip(base + MIXER @ u, 0.0, 1.0).tolist()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pad", type=int, default=0, help="controller slot, 0-3")
    ap.add_argument("--no-pad", action="store_true",
                    help="no gamepad: neutral sticks, never arms")
    ap.add_argument("--script", default=None,
                    help="fly from a named stick script instead of a gamepad")
    args = ap.parse_args()

    if args.script:
        scripted = scripted_pad(args.script)
        read_pad = None
        log(f"fc_sitl: stick script {args.script!r}")
    elif args.no_pad:
        scripted = None
        read_pad = lambda: dict(NEUTRAL)
        log("fc_sitl: --no-pad, sticks neutral")
    else:
        scripted = None
        read_pad = open_pad(args.pad)
        log(f"fc_sitl: gamepad in slot {args.pad}. LB arms with throttle down.")

    fc = Controller()

    # Binary streams with an explicit newline: Windows text mode would rewrite
    # \n as \r\n and corrupt the link.
    stdin, stdout = sys.stdin.buffer, sys.stdout.buffer

    for line in stdin:
        line = line.strip()
        if not line:
            continue
        msg = json.loads(line)
        dt = msg["dt"]

        pad = scripted(dt) if scripted else read_pad()
        fc.update_arming(pad)
        m = fc.motors(pad, msg["gyro"], dt)

        reply = {"seq": msg["seq"], "m": [round(v, 6) for v in m],
                 "armed": int(fc.armed),
                 "rc": [round(pad["roll"], 4), round(pad["pitch"], 4),
                        round(pad["yaw"], 4), round(pad["throttle"], 4)]}
        stdout.write(json.dumps(reply).encode() + b"\n")
        stdout.flush()          # lockstep deadlocks without this

    log("fc_sitl: stdin closed, exiting")
    return 0


if __name__ == "__main__":
    sys.exit(main())
