# 4. Split the controller into its own process

Step 3 did everything in one process: the physics, the "IMU", the gamepad and
the control law. That is the opposite of the real arrangement, where the flight
controller is separate hardware that sees only sensor readings and stick input
and answers with motor demands.

```
  plant.py                              fc_sitl.py
  --------------------------            --------------------------
  pybullet world (CtrlAviary)           rate PID + mixer
  the "IMU": derives body rates --gyro-->
  m -> RPM, steps physics       <--m[4]-- reads the gamepad itself
```

Still Python on both sides. This is the rehearsal for
[step 5](../5_fc_sitl_cpp/README.md), which swaps the controller for the real
C++ binary.

## Run

```bash
.venv\Scripts\python.exe simulator\4_fc_sitl_py\plant.py
```

`plant.py` launches the controller itself, so that is the only command. It uses
`sys.executable`, so the controller runs under the same interpreter whether or
not the venv is activated.

Flags: `--no-gui`, `--seconds N`, `--fc <path>`, `--fc-args=...`. Controller
flags: `--pad N`, `--no-pad` (neutral sticks, never arms, so you can exercise
the link with nothing plugged in), `--script arm-climb-roll` (canned stick
sequence, for flying it without hands).

```bash
.venv\Scripts\python.exe simulator\4_fc_sitl_py\plant.py --no-gui --fc-args=--no-pad
```

`--fc-args` needs the `=` form when the value starts with a dash, or argparse
reads it as another flag.

## Protocol

One JSON object per line, each way. The simplest thing that works, the CRC
framing of the real link comes in step 5.

```
plant -> fc    {"seq":41,"dt":0.004167,"gyro":[0.01,-0.02,0.00]}
fc -> plant    {"seq":41,"m":[0.51,0.49,0.52,0.48],"armed":1,"rc":[...]}
```

`gyro` is body rates in rad/s. `obs` reports angular velocity in the world
frame, so the plant rotates it into the body frame first, and that rotation is
the IMU's whole job here. `m` is a per-motor thrust fraction, 0 to 1, not RPM.
`seq` is echoed and checked, a mismatch stops the run rather than letting the
two drift apart silently.

Strict lockstep: the plant writes one line, flushes, then blocks on the reply.
Nothing runs in real time, so runs reproduce exactly.

### Three things that will bite you

The controller must never print to stdout, that is the data channel, so
`fc_sitl.py` has a `log()` helper that writes to stderr. Open the pipes binary
and write an explicit `\n`, because Windows text mode rewrites `\n` as `\r\n`
and corrupts the link. And flush after every write on both sides, or lockstep
deadlocks on the first exchange and it looks like a hang rather than an error.

## Acro only

The sticks command a rotation rate, not an angle. Let go of the right stick and
the drone keeps whatever bank it had. That follows from the split: the
controller is given body rates and nothing else, and a rate is all you can close
a loop on. Stabilised mode needs attitude, which means an accelerometer in the
sensor line and an estimator in the controller.

Arming is unchanged from step 3: LB toggles, refused unless the throttle stick
is all the way down.

## Tuning notes

Two things I got wrong first time, worth not rediscovering.

Yaw needs its own gains. Roll and pitch are driven by thrust differences across
the arms, yaw is driven by rotor drag, and on this airframe that is far stronger
than it looks. Open loop, a motor split of ±0.3 spins it past 4000 deg/s.
Sharing one gain set made yaw oscillate while roll and pitch were fine.

The D term must be filtered. Unfiltered, the yaw rate flipped sign on every
single step, a limit cycle at the Nyquist frequency. The nasty part is how it
looks: sampled every 60 steps it reads as a perfectly steady -24.4 deg/s, so it
passes any test that does not compare consecutive samples. If you retune,
measure the mean step-to-step change in the rate, not just the average.

The derivative is also taken on the measurement, not the error. Both agree while
the setpoint is steady, but differentiating the error puts a spike of
`stick_step / dt` through D the instant you move a stick, which slams all four
motors to their rails.
