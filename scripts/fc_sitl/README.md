# fc_sitl

The flight controller in its own process. Two scripts talking over a pipe,
rehearsing the arrangement the C++ `fc_simulation` will slot into.

```
  plant.py                                fc_sitl.py
  --------------------------              --------------------------
  pybullet world (CtrlAviary)             rate PID + mixer
  the "IMU": derives body rates  --gyro-->
  m -> RPM, steps physics        <--m[4]-- reads the gamepad itself
```

`scripts/ctrlaviary_check/fly_demo.py` did all of this in one process. Here the
controller has no idea it is being simulated: it sees gyro readings arriving on
stdin and a gamepad it reads itself, and answers with a motor demand per rotor.
Stop the controller and the plant just falls out of the sky.

## Run

```bash
scripts\ctrlaviary_check\.venv\Scripts\python.exe scripts\fc_sitl\plant.py
```

`plant.py` launches the controller itself, so that is the only command. It uses
`sys.executable`, so the controller runs under the same interpreter whether or
not a venv is activated.

| flag | effect |
| --- | --- |
| `--no-gui` | no pybullet window |
| `--seconds N` | how long to run (default 120) |
| `--fc <path>` | use a different controller |
| `--fc-args=...` | pass arguments through to the controller |

Note `--fc-args` needs the `=` form when the value itself starts with a dash,
or argparse reads it as another flag:

```bash
scripts\ctrlaviary_check\.venv\Scripts\python.exe scripts\fc_sitl\plant.py --no-gui --fc-args=--no-pad
```

Controller flags: `--pad N`, `--no-pad` (neutral sticks, never arms - lets you
exercise the link with no controller plugged in), and
`--script arm-climb-roll` (flies a canned stick sequence, for testing without
hands).

## Setup

Needs the same packages as `scripts/ctrlaviary_check`. Simplest is to reuse
that venv, as the commands above do. To build a separate one:

```bash
py -3.11 -m venv scripts\fc_sitl\.venv
```

```bash
scripts\fc_sitl\.venv\Scripts\python.exe -m pip install -r scripts\fc_sitl\requirements.txt
```

```bash
scripts\fc_sitl\.venv\Scripts\python.exe -m pip install --no-deps --ignore-requires-python -r scripts\fc_sitl\requirements-gpd.txt
```

`fc_sitl.py` itself needs only numpy — no physics, and the gamepad comes from
XInput through ctypes, which is part of Windows.

## Protocol

One JSON object per line, each way. Deliberately the simplest thing that works;
the CRC framing of the real serial link is a later step.

```
plant -> fc    {"seq": 41, "dt": 0.004167, "gyro": [0.01, -0.02, 0.00]}
fc -> plant    {"seq": 41, "m": [0.51, 0.49, 0.52, 0.48], "armed": 1,
                "rc": [0.0, 0.0, 0.0, 0.43]}
```

- `gyro` — body rates in rad/s. `obs` reports angular velocity in the *world*
  frame, so the plant rotates it into the body frame first. That rotation is
  the IMU's whole job here.
- `m` — per-motor **thrust fraction**, 0..1, not RPM. The plant converts with
  `MAX_RPM * sqrt(m)`, because thrust goes as rpm squared. This is the same
  conversion `hil_bridge/bridge.py` applies to the C++ controller's output, so
  a motor demand already means the same thing on both sides.
- `seq` — echoed and checked. A mismatch stops the run rather than letting the
  two silently drift apart.

Strict lockstep: the plant writes one line, flushes, then blocks on the reply.
Nothing runs in real time, so runs are reproducible.

### Three things that will bite you

- **The controller must never print to stdout.** That is the data channel.
  `fc_sitl.py` has a `log()` helper that writes to stderr, which is inherited
  and lands on your terminal.
- **Binary streams, explicit `\n`.** Windows text mode rewrites `\n` as `\r\n`
  and corrupts the link.
- **Flush after every write, both sides.** Lockstep deadlocks on the first
  exchange otherwise, and it looks like a hang, not an error.

## Acro only

The sticks command a rotation **rate**, not an angle. Let go of the right stick
and the drone keeps whatever bank it had — it will not level itself. That is
correct, and it follows directly from the split: the controller is given body
rates and nothing else, and a rate is all you can close a loop on. Stabilised
mode needs attitude, which means adding an accelerometer to the sensor line and
an estimator in the controller.

Arming is unchanged from `fly_demo.py`: LB toggles, and it is refused unless the
throttle stick is all the way down.

## Tuning notes

Two things were wrong on the first attempt and are worth not rediscovering:

**Yaw needs its own gains.** Roll and pitch are driven by thrust differences
across the arms; yaw is driven by rotor drag, and on this airframe that is a
far stronger effect than it looks — open loop, a motor split of ±0.3 spins it
past 4000 deg/s. Sharing one gain set made yaw oscillate while roll and pitch
were fine.

**The D term must be filtered.** Unfiltered, the yaw rate flipped sign on
*every single step* — a limit cycle at the Nyquist frequency. The dangerous
part is how it looks: sampled every 60 steps it reads as a perfectly steady
-24.4 deg/s, so it passes any test that does not look at consecutive steps. If
you retune, measure the mean step-to-step change in the rate, not just the
average.

The derivative is also taken on the *measurement*, not the error. Both agree
while the setpoint is steady, but differentiating the error puts a spike of
`stick_step / dt` through D the instant you move a stick, which slams all four
motors to their rails. `src/app/fc_core.hpp` still differentiates the error, so
the two are not identical here.

## Not here

CRC framing and byte compatibility with `src/app/fc_core.hpp`, stabilised mode,
rerun telemetry, and the real serial link.
