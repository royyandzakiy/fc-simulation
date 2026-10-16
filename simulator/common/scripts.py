#!/usr/bin/env python3
"""Canned stick sequences the plant sends over the wire, for runs with no hands.

Each script maps simulated time to (rc, arm) exactly as a radio would deliver
them: rc is (roll, pitch, yaw, throttle), arm is the raw switch channel. The
controller still makes the arming decision, so a script exercises the same arm
switch, hold time and throttle gate a real radio does.

Timing against the controller's defaults (arm hold 0.2 s):
    0.0 s  switch down, throttle down   - the controller sees the switch low
    0.5 s  switch up                    - arms at 0.7 s, throttle still down
    0.8 s  throttle up
"""

ARM_AT_S = 0.5
THROTTLE_AT_S = 0.8


def _arm(t):
    return 1.0 if t >= ARM_AT_S else -1.0


def arm_hover(t):
    """Arm, then hold CF2X hover throttle."""
    thr = 0.62 if t >= THROTTLE_AT_S else 0.0
    return (0.0, 0.0, 0.0, thr), _arm(t)


def arm_climb_roll(t):
    """Arm, climb, then half right roll stick from 3 s."""
    thr = 0.75 if t >= THROTTLE_AT_S else 0.0
    roll = 0.5 if t >= 3.0 else 0.0
    return (roll, 0.0, 0.0, thr), _arm(t)


SCRIPTS = {
    "arm-hover": arm_hover,
    "arm-climb-roll": arm_climb_roll,
}
