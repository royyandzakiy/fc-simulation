# sitl_cpp

Software in the loop with the **real C++ flight controller**. Same split as
`scripts/fc_sitl`, but the controller is `fc_simulation` instead of a Python
stand-in, and the link is the binary CRC framing declared in
`src/app/fc_core.hpp` instead of JSON.

```
  plant.py                                 fc_simulation.exe
  ---------------------------              ---------------------------
  pybullet world (CtrlAviary)              rate PID + mixer, acro
  the "IMU": derives body rates  --0xA5-->
  m -> RPM, steps physics        <--0x5A-- reads the gamepad over SDL3
```

The controller has no idea it is being simulated. Swap the pipe for a real
serial port with `--dev COM7` and the same binary talks to real hardware.

## Run

```bash
cmake --build build/clang-cl-debug
```

```bash
scripts\3_ctrlaviary_check\.venv\Scripts\python.exe scripts\sitl_cpp\plant.py
```

`plant.py` finds and launches the binary itself — the newest `fc_simulation`
under `bin/` or `build/`, so it picks up whichever toolchain you built last.
Hold **LB** with the throttle down to arm, **B** to disarm.

| flag | effect |
| --- | --- |
| `--no-gui` | no pybullet window |
| `--seconds N` | how long to run (default 120) |
| `--fc <path>` | use a specific binary |
| `--fc-args=...` | pass arguments to the controller |

Fly it with no hands, for testing:

```bash
scripts\3_ctrlaviary_check\.venv\Scripts\python.exe scripts\sitl_cpp\plant.py --no-gui --fc-args=--script=arm-hover
```

Controller flags worth knowing: `--script arm-hover` / `--script arm-climb-roll`
(canned stick sequences), `--no-pad` (neutral sticks, never arms), `--list`,
`--debug-pad`, `--dev <port>` (talk over a serial port instead of the pipe).

Note `--fc-args` needs the `=` form when the value starts with a dash, or
argparse reads it as another flag.

## Wire format

Exactly the structs in `src/app/fc_core.hpp`, which `static_assert` their own
sizes. [protocol.py](protocol.py) mirrors them with `struct` and a hand-rolled
CRC, so this folder installs nothing extra — no `construct`, no `crc` package.
The bytes are identical either way.

```
sensor  22 B  0xA5  seq u32, dt f32, gyro[3] f32, crc u8
motor   39 B  0x5A  seq u32, m[4] f32, rc[4] f32, armed u8, crc u8
```

CRC-8 over every preceding byte: poly `0xD5`, init 0, no reflection, no final
xor — the same polynomial CRSF uses. A frame that fails CRC is dropped by the
controller and raises on the plant side rather than being flown.

The controller resynchronises on the sync byte, so a partial or corrupt frame
costs one cycle rather than desyncing the link permanently.

`m` is a per-motor **thrust fraction**, 0..1, not RPM. The plant converts with
`MAX_RPM * sqrt(m)` because thrust goes as rpm squared.

Two mechanical details, both already handled but easy to break:

- **Binary stdio.** `plat::set_binary_stdio()` runs before any byte moves. On
  Windows the CRT would otherwise translate `0x0A` to `0x0D 0x0A` and corrupt
  every frame containing that byte — which floats produce constantly.
- **stdout is the data channel.** Everything the controller prints for a human
  goes to stderr.

## Acro only

The sticks command a rotation **rate**, not an angle. Centre the right stick
and the drone keeps whatever bank it had. That follows from the sensor packet:
the controller is sent body rates and nothing else, and a rate is all you can
close a loop on. Stabilised mode would need an accelerometer in the sensor
frame and an estimator in the controller.

## Changes this needed in `src/app`

- **`add_subdirectory(src/app)` was commented out** in the root `CMakeLists.txt`,
  so `fc_simulation` was not being built at all — `cmake --build` silently did
  nothing while the stale binary sat in `bin/`. Re-enabled.
- **`RatePid` now differentiates the measurement, not the error.** The two agree
  while the setpoint is steady, but differentiating the error puts a spike of
  `stick_step / dt` through the D term the instant a stick moves, which slams
  all four motors to their rails for a cycle. Betaflight does the same thing
  for the same reason.
- **The derivative is low-passed** (`kDLpf`). Unfiltered it amplifies
  step-to-step gyro noise until the loop oscillates at the sample rate. That
  failure is worth recognising: sampled slower than every step it reads as a
  perfectly *steady* rate, so it passes any test that does not look at
  consecutive samples.
- **`--no-pad` and `--script`** added to `main.cpp`, so the loop can be flown
  end to end with no gamepad. Script time comes from the sensor packet's `dt`,
  not a wall clock, so scripted runs stay in lockstep and reproduce exactly.

Gains were left alone: with the D term fixed, the existing
`0.10 / 0.20 / 0.0020` (roll, pitch) and `0.20 / 0.10 / 0` (yaw) track a
commanded 172 deg/s at 173.7 deg/s with 0.016 deg/s per step of ringing.

## Not here

Stabilised mode, rerun telemetry, and the real serial link — though `--dev`
already exists for that.
