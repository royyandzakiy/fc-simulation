#!/usr/bin/env python3
"""Raw radio axes -> the stick values that go on the wire.

Kept byte-identical to Radio::poll in src/fc_sitl_cpp/main.cpp. If these two
ever disagree, the same radio flies differently depending on which side read
it, which is the one thing this split must not do. Shared by steps 5 and 6
rather than copied, for the same reason.

What is NOT here: any decision about arming. read_sticks returns the raw arm
channel and nothing more. Threshold, hysteresis, hold time and the throttle
gate all live in fc::ArmSwitch, because that is the code that eventually flies,
and a simulator has no business owning a safety interlock.
"""

from radio_hid import RadioError

# The radio has already calibrated and trimmed its own sticks, so this only
# has to kill the last of the jitter around centre.
STICK_DEADBAND = 0.03


def axis(axes, idx):
    """Raw axis as -1..1. A bad --ch-* reads as centred rather than crashing."""
    return axes[idx] if 0 <= idx < len(axes) else 0.0


def norm(v):
    """Self-centring stick axis, dead band removed and the rest rescaled."""
    if -STICK_DEADBAND < v < STICK_DEADBAND:
        return 0.0
    edge = STICK_DEADBAND if v > 0 else -STICK_DEADBAND
    return (v - edge) / (1.0 - STICK_DEADBAND)


def read_sticks(radio, args):
    """(rc, arm) for the sensor packet, or None if the radio has gone away.

    rc is (roll, pitch, yaw, throttle). args supplies ch_roll, ch_pitch,
    ch_yaw, ch_throttle and ch_arm.
    """
    try:
        axes, _ = radio.read()
    except RadioError as exc:
        print("radio: %s - failsafe, the controller will disarm" % exc)
        return None

    # A radio throttle does not self-centre, so it gets the full travel with no
    # centre dead band: stick down is zero, stick up is full. Getting this
    # wrong is what made the bottom two thirds of the throttle do nothing.
    throttle = min(1.0, max(0.0, (axis(axes, args.ch_throttle) + 1.0) * 0.5))

    rc = (norm(axis(axes, args.ch_roll)),
          norm(axis(axes, args.ch_pitch)),
          norm(axis(axes, args.ch_yaw)),
          throttle)
    return rc, axis(axes, args.ch_arm)
