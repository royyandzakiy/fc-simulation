# 3. The real plant, and flying it

Step 1 used a hand-made `quad.urdf` pushed around with invented forces. Here
the plant is gym-pybullet-drones' `CtrlAviary` flying a real Crazyflie 2.X, and
that changes the interface completely.

|  | step 1 | here |
| --- | --- | --- |
| input | force (N) + torque (N·m), applied by hand | `action` `(1,4)` of motor RPM, 0 to 21702 |
| output | three separate pybullet getters | one `obs` `(1,20)` array |
| vehicle | `quad.urdf`, 500 g, guessed inertia | CF2X, **27 g**, arm 39.7 mm |
| mixer | written by hand | inside the env (`KF`=3.16e-10, `KM`=7.94e-12) |

That input change is the whole point. `CtrlAviary` owns the mixer, so wiring up
the firmware later is one line, because thrust goes as rpm squared:

```python
action[0, :] = env.MAX_RPM * np.sqrt(np.clip(motor_demand, 0.0, 1.0))
```

Setup is in the [root README](../../README.md).

## plant_demo.py

Open loop, no controller at all. A fixed RPM schedule: 1.03x hover to climb,
then 1.00x, then 0.97x.

```bash
.venv\Scripts\python.exe simulator\3_ctrlaviary_check\plant_demo.py
```

```
CF2X: 27 g   arm 39.7 mm   hover 14468 rpm   max 21703 rpm
t= 2.0s  z= 1.257 m  gyro=[ -0.05  -0.05  -0.03] rad/s  rpm= 14469.9  (1.00x hover)
t= 4.0s  z= 3.044 m  gyro=[ -0.10  -0.10  -0.06] rad/s  rpm= 14035.8  (0.97x hover)
start z 0.113 m   peak z 3.182 m   final z 1.676 m   final tilt 26.9 deg
PASS: CtrlAviary stepped and the drone responded to RPM.
```

It climbs away and then tips over, and that is correct. Hover RPM holds
acceleration at zero, not altitude, so after the climb phase the upward
velocity is still there. And `IMBALANCE` puts 0.0001 extra on motor 0, because
no real quad is perfectly trimmed. On a 27 g airframe with J = 1.4e-5 that
integrates into 27 degrees of tilt over six seconds. Raise it to 1.0003 and it
flips a full 180.

`--perfect` removes the imbalance. The drone then stays exactly level and the
gyro reads zero for the whole run, which also means the body-rate conversion is
rotating zeros and proving nothing.

## pad_LB_arm_demo.py

Same plant, same fixed throttle. The new thing is that a human decides live
whether the motors are live at all. **The sticks do nothing yet.**

```bash
.venv\Scripts\python.exe simulator\3_ctrlaviary_check\pad_LB_arm_demo.py
```

Press LB to arm, press again to disarm. Disarmed the motors are hard zero.
Armed they spin up and it climbs.

The toggle fires on the rising edge, not the level. At 240 Hz a held button
would otherwise flip armed state 240 times a second and whether you ended up
armed would depend on how long you held it. Armed applies a gentle climb rather
than an idle, since there is no throttle stick wired up here yet and idling
would make arming invisible.

The pad is read through XInput via ctypes, same as
[step 2](../2_gamepad_test/README.md). If it ever seems dead, check it there
first.

## fly_demo.py

Both sticks, stabilised mode.

```bash
.venv\Scripts\python.exe simulator\3_ctrlaviary_check\fly_demo.py
```

| control | does |
| --- | --- |
| left stick up/down | throttle, fully down is zero |
| left stick left/right | yaw rate |
| right stick left/right | roll angle |
| right stick up/down | pitch angle |
| LB | arm / disarm |

Arming is refused unless the throttle stick is all the way down:

```
t=  2.3s  arm refused - throttle is 62%, pull it all the way down
```

**Stabilised, not acro.** The right stick commands an angle, 30 degrees at full
deflection, so centring it returns the drone to level on its own. In acro the
same stick commands a rate and centring only stops the rotation, leaving it
tilted wherever it was.

### Throttle

Linear across the whole stick travel: fully down is 0.00, centre 0.50, fully up
1.00. Unlike the other three axes it has no centre dead band, since a dead zone
belongs around the middle of a self-centring control and throttle is not one.
It has small dead zones at the ends instead, so the stick bottoms out at
exactly zero, which is what the arming check needs.

The stick commands **thrust, not RPM**. Thrust goes as rpm squared, so mapping
the stick straight to RPM wastes its whole lower half: half stick would be a
quarter of the thrust, and this airframe needs 44% of max thrust just to hover.
Taking the square root puts hover at about **43%**, near the middle of the
stick where it belongs.

| stick | linear RPM (wrong) | thrust-based (now) |
| --- | --- | --- |
| 0.30 | 8717 | 12296 |
| 0.50 | 12117 | 15575, flies |
| 0.70 | 15517, flies | 18274 |

Being at the bottom is required only to arm. Once armed, throttle is just
throttle. Note a gamepad stick springs back to centre, so letting go leaves you
at 50%, not idle. A real transmitter's throttle stays where you put it.

### What to expect

No altitude hold, this is stabilised and not position mode. Tilt costs lift, so
at hover throttle a 30 degree bank makes it sink and land. Feed in more throttle
when you lean it over.
