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

## Run

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

## What this validates for `hil_bridge`

- `gym-pybullet-drones` imports and runs on 3.11 despite its `>=3.12` pin
- the `CtrlAviary(...)` keywords `bridge.py` passes all exist in the installed 2.2.0
- the `obs[0]` layout `bridge.py` indexes: `[0:3]` pos, `[3:7]` quat xyzw, `[13:16]` angular velocity
- `env.MAX_RPM` / `env.HOVER_RPM` / `env.M` / `env.L` are present and sane
- the world→body gyro rotation via `scipy`, which is why the imbalance matters:
  `obs` reports angular velocity in the **world** frame, but a real gyro measures
  **body** rates, so it must be rotated before the firmware sees it

## Not here

Controls of any kind, the CRC packet format, the subprocess link, rerun, the
gamepad. Those are `scripts/hil_bridge`.
