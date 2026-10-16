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

## Who reads the radio

`--sticks fc` is the default and is what hardware does: the controller opens the
HID device itself over SDL3 and the plant never sees a stick position.

`--sticks plant` turns it around. This script reads the device and the channels
ride inside the sensor frame; the controller runs `--input link` and never
initialises SDL at all. That last part is the reason it exists: a Zephyr
`native_sim` build has no USB host, so sticks that do not arrive on the wire do
not arrive.

```bash
.venv@B@Scripts@B@python.exe simulator@B@5_fc_sitl_cpp@B@plant.py --sticks plant
```

**The arming decision stays in C++ either way.** The plant sends the raw arm
channel, never an armed flag. Threshold, Schmitt hysteresis, the hold time, the
throttle gate and the startup guard all live in `fc::ArmSwitch`, because that is
the code that eventually flies, and a simulator has no business owning a safety
interlock.

Channel numbers are set with `--ch-roll` / `--ch-pitch` / `--ch-throttle` /
`--ch-yaw` / `--ch-arm` on whichever side is doing the reading. To find them:

```bash
python simulator/common/radio_hid.py
```

They do **not** transfer between hosts. `/dev/input/jsN` on Linux reports the
radio's own axis order, the same one `src/samples/radio_test` prints.
`joyGetPosEx` on Windows reports a fixed X, Y, Z, R, U, V and has no way to
express a seventh axis, so an eight-channel radio loses the top two. Check them
per host.

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
`--debug-pad`, `--dev <port>`, `--input auto|gamepad|joystick|link`.

## Wire format

Exactly the structs in `src/fc_sitl_cpp/fc_core.hpp`, which `static_assert` their own
sizes. [protocol.py](protocol.py) mirrors them with `struct` and a hand-rolled
CRC, so this step installs nothing extra. The bytes are identical either way.

```
sensor  43 B  0xA5  seq u32, dt f32, gyro[3] f32,
                    rc[4] f32, arm f32, flags u8, crc u8
motor   39 B  0x5A  seq u32, m[4] f32, rc[4] f32, armed u8, crc u8
```

The sensor frame always carries the stick fields, whether or not anyone filled
them in. One framing is worth more than the bytes a second frame type would
save, and it means switching sides is a flag rather than a different
conversation on the wire. `flags` bit 0 says the fields mean something: without
it, "no radio here" and "sticks centred, arm switch down" are the same bytes and
the controller has nothing to say about why it will not arm.

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
