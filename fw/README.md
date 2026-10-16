# fw - Zephyr firmware

The flight controller as a Zephyr application, plus a ladder of samples that
builds up to it one piece at a time. Every sample and the app build for two
targets:

| target | what it is for |
| --- | --- |
| `native_sim/native/64` | runs on Linux as `zephyr.exe`, flown by the simulator |
| `esp32_devkitc/esp32/procpu` | ESP32-DevKitC, the hardware target (MPU-6050 IMU) |

The flight logic is not here. It is `src/fc_core/include/fc/`, shared
unchanged with the host build and the unit tests, and every app pulls it in
through [cmake/fc_core.cmake](cmake/fc_core.cmake).

## The ladder

Each rung adds one thing. Each one that has a simulator step pairs with it.

| | adds | simulator step | runs on |
| --- | --- | --- | --- |
| [samples/1_hello_core](samples/1_hello_core) | the core compiled under Zephyr, C++23, logging | none, prints its results | native_sim, ESP32 |
| [samples/2_uart_echo](samples/2_uart_echo) | the link UART: a pty on native_sim | [7](../simulator/7_zephyr_uart_echo/README.md) | native_sim, ESP32 |
| [samples/3_link_echo](samples/3_link_echo) | sensor and motor frames, no flight logic | [8](../simulator/8_zephyr_link_check/README.md) | native_sim, ESP32 |
| [app](app) | the controller: LinkSticks, rate PID, mixer | [8](../simulator/8_zephyr_link_check/README.md), [9](../simulator/9_fc_zephyr/README.md) | native_sim, ESP32 |
| [samples/4_esp32_mpu6050](samples/4_esp32_mpu6050) | the real gyro over I2C | none yet | ESP32 only |
| [tests/core](tests/core) | the core under ztest | none | native_sim |

## Setup

Linux or WSL only, since `native_sim` does not exist on Windows. One-off, about
3.5 GB: the workspace into `~/zephyr-ws`, the SDK into `~/zephyr-sdk-*`.

```bash
fw/scripts/setup_zephyr_ws.sh
```

That creates a Python venv, a Zephyr v4.4.2 workspace with only the
`hal_espressif` module, and the Zephyr SDK's ESP32 toolchain. Host GCC builds
`native_sim`, so nothing else is needed for it. Then, in each new shell:

```bash
. fw/scripts/env.sh
```

`env.sh` sets `ZEPHYR_BASE` explicitly, so a second Zephyr on the same machine
(an nRF Connect SDK install, say) cannot be picked up by mistake.

## Build and run

From the repo root. Build directories go under `build/fw/`, which is where the
simulator steps look for `zephyr.exe`.

```bash
west build -b native_sim/native/64 fw/samples/1_hello_core -d build/fw/1_hello_core
build/fw/1_hello_core/zephyr/zephyr.exe -stop_at=1
```

```
<inf> hello_core: crc8("123456789") = 0xBC, expect 0xBC
<inf> hello_core: mixer thr 0.5 roll +0.1 -> m = 400 400 600 600 (x1000), expect 400 400 600 600
<inf> hello_core: unaligned decode ok, seq 7 gyro.y -200 (x1000)
```

The others build the same way, with `-d build/fw/<name>`. The app's build
directory is `build/fw/app`.

For the ESP32:

```bash
west build -b esp32_devkitc/esp32/procpu fw/app -d build/fw/app_esp32
west flash -d build/fw/app_esp32
```

Everything is build-checked for the ESP32 but nothing has run on one yet.

## Tests

All of it at once, through Twister:

```bash
west twister -T fw -p native_sim/native/64 -p esp32_devkitc/esp32/procpu -O build/twister
```

That runs `tests/core` and checks `1_hello_core`'s output on `native_sim`, and
builds everything for both targets. CI runs the same command.

## Board notes

**native_sim.** The link is `uart1`, chosen as `fc,link-uart`, on its own pty.
`uart0` is disabled and the console and logs go to `zephyr.exe`'s stdout
instead, so there is one pty to find and the logs never mix with frames. The
tick is raised to 10 kHz; the default 100 Hz makes a single `k_usleep()` cost
10 ms. All of that is in each app's `boards/native_sim_native_64.*`. The files
must carry the full target name: Zephyr 4.4 does not apply a plain
`native_sim.overlay` to `native_sim/native/64`.

**ESP32.** The link is `uart0`, the DevKitC's USB-UART bridge, at 460800 baud:
82 bytes per exchange take 1.8 ms on the wire against the 4.17 ms period.
115200 does not fit, see [step 6](../simulator/6_fc_sitl_tty/README.md). Since
`uart0` carries frames, the app turns logging, printk and the console off on
this board. The MPU-6050 goes on `i2c0`: SDA GPIO21, SCL GPIO22, address 0x68.
