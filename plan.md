# Plan: from simulator to Zephyr app

The goal is a Zephyr flight controller app. The approach is simulation first:
prove the flight logic against a real plant on the host, then move the same
code onto Zephyr one rung at a time, keeping the simulator in the loop for as
long as possible.

Hardware: **ESP32-DevKitC** (`esp32_devkitc/esp32/procpu`) with an **MPU-6050**
on I2C. Zephyr **v4.4.2**.

Gap analysis written 2026-10-16. Phases 0-3 done 2026-10-17 on branch
`zephyr-port`.

## Where it stands

| area | state | notes |
| --- | --- | --- |
| plant (CtrlAviary, CF2X, 240 Hz) | done | steps 1-6 |
| lockstep link, binary frames + CRC-8 | done | `fc/link.hpp`, memcpy at fixed offsets, no packed structs |
| flight logic | done for acro | `fc/core.hpp`: rate PID x3, mixer, arming, RC-loss failsafe |
| unit tests | done | 27 GoogleTest cases on the host, 9 ztest cases on `native_sim` |
| golden traces | done | `simulator/golden/`, recorded from the host build |
| Zephyr app on `native_sim` | done | `fw/app`, flies both goldens with zero difference (step 9) |
| Zephyr samples ladder | done | `fw/samples/1-4`, paired with simulator steps 7-9 |
| ESP32 build | build-checked | app, samples and tests build; nothing flashed yet |
| CI | written, not run on GitHub | every step run by hand under WSL |
| `--sticks fc` on Linux | broken | listed in `DEVELOPMENT.md`; Zephyr only uses the link path |
| real hardware I/O (IMU, receiver, ESCs) | not started | MPU-6050 sample builds, untested |
| stabilised (angle) mode | not started | needs accel and an attitude estimate |

The project is now about 45% of the way to a Zephyr app flying hardware. All
the simulation-side work is done and the firmware is proven equivalent to the
host build. What is left is entirely on real hardware.

## Port blockers from the analysis

1. ~~Packed wire structs used by the flight logic~~. Fixed: `fc/link.hpp`
   decodes with `memcpy`. More important on the ESP32 than the analysis
   assumed: Xtensa has no unaligned loads at all. `1_hello_core` and the ztest
   suite decode from an unaligned buffer to keep it that way.
2. ~~Controller coupled to the wire format~~. Fixed: `Controller::step(gyro,
   sticks, dt)` returns four motor demands and knows nothing about frames.
3. ~~`LinkSticks` in `main.cpp`~~. Moved into the core, reports `refused()`
   and `rc_lost()` instead of printing.
4. ~~No RC-loss failsafe~~. `fc::RcWatchdog`, 0.5 s default. Short dropouts hold
   the last sticks, longer ones disarm, and the arm switch must be cycled after.
5. **The loop is clocked by incoming frames.** Still true, and still right for
   SITL and HIL. Becomes Phase 6's sensor-source work.
6. ~~Stale `fc_min.cpp` references~~. Fixed.

## Steps

### Phase 0 - housekeeping (done)

- [x] Commit the current work in progress (you had already)
- [x] Split the core into `fc/core.hpp` (logic) and `fc/link.hpp` (codec), header-only, in `src/fc_core/`
- [x] Move `LinkSticks` into the core, replace its prints with status flags
- [x] Add an RC-loss timeout to the core
- [x] Wire up `test/`: CRC check value, a frame byte-identical to Python's, unaligned decode, reader resync, mixer, PID, arming gate, arm switch, watchdog, failsafe
- [x] Record golden runs: `arm_hover.csv` and `arm_climb_roll.csv`, via step 5's new `--script` and `--record`
- [x] Fix the stale references

### Phase 1 - Zephyr `native_sim` app (done)

- [x] `fw/scripts/setup_zephyr_ws.sh`: workspace in `~/zephyr-ws`, only `hal_espressif`, ESP32 toolchain
- [x] `fw/app`: link UART, `LinkSticks`, controller; Kconfig for arm threshold, hold and RC timeout
- [x] Samples 1-3 building up to it: core under Zephyr, pty echo, frame echo
- [x] Builds for `native_sim/native/64`, so no 32-bit multilib is needed on the host

Two things the analysis did not predict, both now in the board configs:
`native_sim`'s 100 Hz tick made a round trip 20 ms (now 0.22 ms at 10 kHz), and
Zephyr 4.4 only applies board overlays named for the full target,
`native_sim_native_64.overlay`.

### Phase 2 - plant drives Zephyr (done)

- [x] `simulator/common/zephyr_sim.py`: launches `zephyr.exe`, finds and opens its pty
- [x] Step 7: byte echo and round-trip time
- [x] Step 8: framing and flight rules with hand-made frames, no physics
- [x] Step 9: the step 5 plant flying `fw/app`, scripted sticks sent from the plant
- [x] Both goldens match: zero difference between the Windows clang-cl host build and the Linux GCC Zephyr build

### Phase 3 - tests and CI on Zephyr (done)

- [x] `fw/tests/core`: ztest on `native_sim`
- [x] Twister over `fw/`: runs on `native_sim`, builds everything for the ESP32
- [x] `fw/samples/4_esp32_mpu6050`: gyro over the sensor API, build-only
- [x] `.github/workflows/ci.yml`: host tests without Conan, Twister, link checks, golden runs

Not verified: the workflow itself has not run on GitHub.

### Phase 4 - hardware decisions (partly made)

- [x] Board: ESP32-DevKitC
- [x] IMU: MPU-6050 on `i2c0`, GPIO21/22, address 0x68
- [ ] Receiver: CRSF over a UART (the CRC poly already matches). ESP32 `uart2` is free
- [ ] ESC protocol: PWM through LEDC first, DShot later
- [ ] Frame, motors and props, which decide the gains

### Phase 5 - HIL on the real board (about 1 week)

- [ ] Flash `fw/app`, run step 9 with the plant on the DevKitC's USB-UART at 460800
      (needs a serial-port mode in `zephyr_sim.py` alongside the pty one)
- [ ] Drain the boot ROM's 115200 output before the first frame
- [ ] Match the goldens on real silicon: same code, Xtensa float unit
- [ ] Measure loop time on target, stack sizes, FPU context switches

### Phase 6 - real I/O, props off (2-4 weeks)

- [ ] Sensor source: MPU-6050 data-ready interrupt clocks the loop, `dt` from the cycle counter
- [ ] Map the MPU-6050 axes to the ENU body frame the core uses, and calibrate gyro bias at boot
- [ ] CRSF parser in the core (pure logic, unit-tested), fed from `uart2`
- [ ] Motor output through LEDC PWM, motor idle and minimum throttle
- [ ] RC-loss failsafe tested by pulling the receiver
- [ ] Gyro filtering tuned for real noise
- [ ] Telemetry back over the USB-UART for logging

### Phase 7 - flight

- [ ] Re-tune PID gains for the real airframe. The current ones are for a 27 g Crazyflie
- [ ] Tethered hover, then free hover in acro
- [ ] Stabilised mode: accel in the sensor frame, attitude estimate in the core, angle loop on top. Simulator first, as before

## Open questions

- Whether the 240 Hz loop stays or moves to 1 kHz on hardware. The MPU-6050
  does up to 8 kHz gyro output; the plant runs at 240 Hz, so HIL would need the
  plant sped up or the loop decoupled
- Whether to keep the echoed `rc` floats in the frames. They cost 32 bytes per
  exchange and are only for plotting
- Whether `--sticks fc` on Linux is worth fixing, since Zephyr only uses the
  link path
