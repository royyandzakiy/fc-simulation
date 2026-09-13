# ctrlaviary_check

The real plant, flown open-loop. One rung between `scripts/pybullet_check`
(does 3D work at all) and `scripts/hil_bridge` (the full lockstep HIL bridge).

No controller, no gamepad, no subprocess, no packets, no rerun. The only new
thing here is the **environment layer**, so if this passes and the bridge later
fails, the failure is in the link, not in the physics.

## What changes from `pybullet_check/hover_demo.py`

|  | `hover_demo.py` | `plant_demo.py` |
| --- | --- | --- |
| Input | force (N) + torque (N·m), applied by hand | `action` `(1,4)` of **motor RPM**, 0…21702 |
| Output | three separate pybullet getters | one `obs` `(1,20)` array |
| Vehicle | hand-made `quad.urdf`, 500 g | real Crazyflie 2.X, **27 g**, arm 39.7 mm |
| Mixer | written by hand | inside the env (`KF`=3.16e-10, `KM`=7.94e-12) |
| Stabilisation | a Python PD controller | none, open loop |

That input change is the whole point. `CtrlAviary` owns the mixer, so
connecting the firmware later is one line — thrust goes as rpm², so a
normalised motor demand becomes RPM with a square root:

```python
action[0, :] = env.MAX_RPM * np.sqrt(np.clip(motor_demand, 0.0, 1.0))
```

Against `hover_demo.py`'s newton-based interface, that same connection would
mean hand-writing a mixer: four demands into collective thrust plus three
torques, applied at the right points.

## Setup

Needs CPython **3.11** (`py -3.11`), not a bare `python3` — on this machine
that resolves to the MSYS2 interpreter.

```bash
py -3.11 -m venv scripts\ctrlaviary_check\.venv
```

```bash
scripts\ctrlaviary_check\.venv\Scripts\python.exe -m pip install -r scripts\ctrlaviary_check\requirements.txt
```

```bash
scripts\ctrlaviary_check\.venv\Scripts\python.exe -m pip install --no-deps --ignore-requires-python -r scripts\ctrlaviary_check\requirements-gpd.txt
```

Two installs on purpose. `gym-pybullet-drones` is not on PyPI, and the
dependencies it declares pull `stable-baselines3` → PyTorch, several GB this
sample never imports since it only uses `CtrlAviary`. The second install takes
the package alone. `--ignore-requires-python` is needed because upstream
declares `>=3.12` while the package is pure Python and runs fine on 3.11.

Needs `git` on PATH for that second install.

## Three samples

| script | what is new | who decides the motors |
| --- | --- | --- |
| `plant_demo.py` | the environment layer | a fixed schedule in the code |
| `pad_demo.py` | the gamepad | you, arming and disarming live |
| `fly_demo.py` | the sticks, stabilised mode | you, flying it |

## plant_demo.py

```bash
scripts\ctrlaviary_check\.venv\Scripts\python.exe scripts\ctrlaviary_check\plant_demo.py
```

Flags: `--no-gui` (numbers only), `--seconds N`, `--perfect` (see below).

```
CF2X: 27 g   arm 39.7 mm   hover 14468 rpm   max 21703 rpm
t= 0.0s  z= 0.113 m  gyro=[ +0.00  +0.00  +0.00] rad/s  rpm= 14904.0  (1.03x hover)
t= 2.0s  z= 1.257 m  gyro=[ -0.05  -0.05  -0.03] rad/s  rpm= 14469.9  (1.00x hover)
t= 4.0s  z= 3.044 m  gyro=[ -0.10  -0.10  -0.06] rad/s  rpm= 14035.8  (0.97x hover)
start z 0.113 m   peak z 3.182 m   final z 1.676 m   final tilt 26.9 deg
PASS: CtrlAviary stepped and the drone responded to RPM.
      It tipped 27 deg, as expected with nothing controlling attitude.
```

Exit `0` on pass, `1` on fail.

## Reading the result

**It climbs away and then tips over. That is correct.** Two things are on
display, and both are arguments for the flight controller:

*Hover RPM holds acceleration at zero, not altitude.* After the 1.03× climb
phase the drone still has upward velocity, and 1.00× does nothing to remove it,
so it keeps rising. Open loop there is no altitude hold, only a thrust setting.

*A 0.01% motor imbalance is enough to tip it.* `IMBALANCE` in `plant_demo.py`
puts 0.0001 extra on motor 0 — no real quad is perfectly trimmed. On a 27 g
airframe with J = 1.4e-5 that integrates into 27° of tilt over six seconds.
Raise it to 1.0003 and it flips a full 180°. There is nothing to correct it.

The check the script asserts is therefore *"did the plant respond to RPM"*, not
*"did it hold altitude"* — open loop it cannot, and asserting otherwise would be
asserting something false.

`--perfect` removes the imbalance: the drone stays exactly level, tilt 0.0°, and
the gyro reads zero for the entire run. Useful for isolating the altitude
behaviour, but note that a perfectly symmetric run leaves the body-rate
conversion untested, since it is then rotating zeros.

## pad_demo.py

The gamepad in the loop, and nothing else. Still `CtrlAviary`, still a fixed
throttle number - the new thing is that a human decides live whether the motors
are live at all. **The sticks do nothing yet**; that is the next step.

```bash
scripts\ctrlaviary_check\.venv\Scripts\python.exe scripts\ctrlaviary_check\pad_demo.py
```

Press **LB** to arm, press it again to disarm. Disarmed, the motors are hard
zero and the drone sits on the ground or falls out of the sky. Armed, they spin
up and it climbs.

```
pad: XInput controller in slot 0
press LB to arm, press again to disarm. Ctrl+C to quit.
t=  0.0s  ---  z= 0.112 m  rpm=     0.0
t=  1.0s  ARMED   z=0.01 m
t=  2.0s  ARM  z= 0.211 m  rpm= 14757.8
t=  4.0s  DISARMED   z=1.69 m
t=  6.0s  ---  z= 0.013 m  rpm=     0.0
```

Flags: `--pad N` (controller slot 0-3), `--no-gui`, `--seconds N`.

The pad is read through **XInput via ctypes**, the same way
`scripts/gamepad_test` does - part of Windows, so there is nothing to install
for it. pygame was tried first and read nothing but zeros on this machine; if
the pad ever seems dead, check it with `scripts/gamepad_test` first.

Two details worth knowing:

*The toggle fires on the rising edge, not the level.* At 240 Hz a held button
would otherwise flip armed state 240 times a second, and whether you ended up
armed would depend on how long you held it.

*Armed applies a gentle climb (1.02x hover), not an idle.* A real flight
controller spins the motors at idle on arm and waits for throttle. There is no
throttle stick wired up yet, so idling would make arming invisible.

Windows-only, and XInput sees Xbox-style pads only - the same limits as
`scripts/gamepad_test`.

## fly_demo.py

Actually fly it. `pad_demo.py` could only arm and disarm; here the sticks mean
something.

```bash
scripts\ctrlaviary_check\.venv\Scripts\python.exe scripts\ctrlaviary_checkly_demo.py
```

| control | does |
| --- | --- |
| left stick up/down | throttle - **fully down is zero** |
| left stick left/right | yaw rate |
| right stick left/right | roll angle |
| right stick up/down | pitch angle |
| LB | arm / disarm |

**Arming is refused unless the throttle stick is all the way down**, so a stick
left half-up cannot spin the motors the instant you press LB:

```
t=  2.3s  arm refused - throttle is 62%, pull it all the way down
```

### Stabilised, not acro

The right stick commands an **angle**, not a rotation rate. Full deflection
asks for 30 degrees of bank; centre the stick and the drone returns to level by
itself. In acro the same stick would command a *rate*, and centring it would
only stop the rotation, leaving the drone tilted wherever it happened to be.

Throttle maps zero at the bottom of the stick travel to full at the top, so
hover sits around 64%. Note a gamepad stick springs back to **centre**, not to
the bottom - let go and you are at 50% throttle, not idle. A real transmitter
has a throttle stick that stays where you put it.

### What to expect

There is no altitude hold - this is stabilised, not position mode. Tilt costs
lift, so at hover throttle a 30 degree bank makes it sink and land. Feed in more
throttle when you lean it over. That is not a bug in the demo, it is what the
mode does, and it is the reason the next mode up exists.

## What this validates for `hil_bridge`

- `gym-pybullet-drones` imports and runs on 3.11 despite its `>=3.12` pin
- the `CtrlAviary(...)` keywords `bridge.py` passes all exist in the installed 2.2.0
- the `obs[0]` layout `bridge.py` indexes: `[0:3]` pos, `[3:7]` quat xyzw, `[13:16]` angular velocity
- `env.MAX_RPM` / `env.HOVER_RPM` / `env.M` / `env.L` are present and sane
- the world→body gyro rotation via `scipy`, which is why the imbalance matters:
  `obs` reports angular velocity in the **world** frame, but a real gyro measures
  **body** rates, so it must be rotated before the firmware sees it

## Not here

The CRC packet format, the subprocess link to `fc_simulation`, and rerun -
those are `scripts/hil_bridge`. The control law in `fly_demo.py` is Python
standing in for the firmware; wiring up the real thing means replacing that
block with packets, not rewriting the plant.
