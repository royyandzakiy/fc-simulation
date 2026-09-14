# 5. The real C++ controller in the loop

Same split as [step 4](../4_fc_sitl_py/README.md), except the controller is the
actual `fc_sitl_cpp` binary from `src/fc_sitl_cpp/`, and the link is the binary CRC
framing declared in `src/fc_sitl_cpp/fc_core.hpp` instead of JSON.

```
  plant.py                                 fc_sitl_cpp.exe
  ---------------------------              ---------------------------
  pybullet world (CtrlAviary)              rate PID + mixer, acro
  the "IMU": derives body rates  --0xA5-->
  m -> RPM, steps physics        <--0x5A-- reads the gamepad over SDL3
```

The controller has no idea it is being simulated. Swap the pipe for a serial
port with `--dev COM7` and the same binary talks to real hardware.

## Run

Build the firmware first, then:

```bash
.venv\Scripts\python.exe simulator\5_fc_sitl_cpp\plant.py
```

`plant.py` finds and launches the binary itself, taking the newest
`fc_sitl_cpp` under `bin/` or `build/`, so it picks up whichever toolchain you
built last. Press LB with the throttle down to arm, press again to disarm. B is
a hard disarm.

Flags: `--no-gui`, `--seconds N`, `--fc <path>`, `--fc-args=...`. Fly it with no
hands:

```bash
.venv\Scripts\python.exe simulator\5_fc_sitl_cpp\plant.py --no-gui --fc-args=--script=arm-hover
```

Controller flags worth knowing: `--script arm-hover` / `--script arm-climb-roll`
(canned stick sequences), `--no-pad` (neutral sticks, never arms), `--list`,
`--debug-pad`, `--dev <port>`.

## Wire format

Exactly the structs in `src/fc_sitl_cpp/fc_core.hpp`, which `static_assert` their own
sizes. [protocol.py](protocol.py) mirrors them with `struct` and a hand-rolled
CRC, so this step installs nothing extra. The bytes are identical either way.

```
sensor  22 B  0xA5  seq u32, dt f32, gyro[3] f32, crc u8
motor   39 B  0x5A  seq u32, m[4] f32, rc[4] f32, armed u8, crc u8
```

CRC-8 over every preceding byte: poly `0xD5`, init 0, no reflection, no final
xor, the same polynomial CRSF uses. A frame that fails CRC gets dropped by the
controller and raises on the plant side rather than being flown. The controller
resynchronises on the sync byte, so a corrupt frame costs one cycle instead of
desyncing the link permanently.

`m` is a per-motor thrust fraction, 0 to 1, not RPM. The plant converts with
`MAX_RPM * sqrt(m)`.

Two mechanical details, both handled but easy to break. `plat::set_binary_stdio()`
runs before any byte moves, because on Windows the CRT would otherwise translate
`0x0A` to `0x0D 0x0A` and corrupt every frame containing that byte, which floats
produce constantly. And stdout is the data channel, so everything the controller
prints for a human goes to stderr.

## Acro only

The sticks command a rate, not an angle, for the same reason as step 4: the
sensor frame carries body rates and nothing else.

## What this needed in src/fc_sitl_cpp

- **`add_subdirectory(src/fc_sitl_cpp)` was commented out** in the root `CMakeLists.txt`,
  so `fc_sitl_cpp` was not being built at all. `cmake --build` reported
  success while doing nothing and the stale binary sat in `bin/`.
- **Arming is a toggle now, not hold-to-arm.** `ArmingGate` used to disarm
  whenever the arm button was not held, so a tap armed on the press and
  disarmed again on the release, and staying armed meant holding LB for the
  whole flight. It toggles on the press edge now. A refused arm says so on
  stderr instead of failing silently.
- **`RatePid` differentiates the measurement, not the error**, and low-passes
  the derivative. Same two fixes as step 4, for the same reasons. Betaflight
  does both.
- **`--no-pad` and `--script`** added to `main.cpp` so the loop can be flown
  with no gamepad. Script time comes from the sensor packet's `dt` rather than a
  wall clock, so scripted runs stay in lockstep and reproduce exactly. The
  scripts tap the arm control rather than holding it, so they exercise the same
  path a human does.

Gains were left alone. With the D term fixed, the existing `0.10 / 0.20 / 0.0020`
(roll, pitch) and `0.20 / 0.10 / 0` (yaw) track a commanded 172 deg/s at
173.7 deg/s with 0.016 deg/s per step of ringing.
