# 9. The Zephyr firmware in the loop

The plant from [step 5](../5_fc_sitl_cpp/README.md), unchanged, flying
[fw/app](../../fw/app) built for `native_sim` instead of the host build. Same
`fc/core.hpp`, same frames, same lockstep. Only the binary on the other end of
the link is different.

```
  plant.py                                  zephyr.exe (fw/app)
  ---------------------------               ---------------------------
  pybullet world (CtrlAviary)               LinkSticks -> Controller
  reads the radio or a script               rate PID + mixer, acro
  opens uart_1's pty             --0xA5-->
                                 <--0x5A--
```

The sticks always come from this side. `native_sim` has no USB host, so the
firmware cannot open a radio; it gets the raw channels in the sensor frame and
makes the arming decision itself, exactly as the host build does with
`--input link`.

Linux only.

## Run

Build the firmware from the repo root, then run the plant with the simulator's
Python environment:

```bash
. fw/scripts/env.sh
west build -b native_sim/native/64 fw/app -d build/fw/app
```

```bash
python simulator/9_fc_zephyr/plant.py --no-gui --script arm-hover --seconds 5 \
    --record build/arm_hover.csv --compare simulator/golden/arm_hover.csv
```

Fly it by hand with a radio instead of a script (`--ch-*` as at step 5):

```bash
python simulator/9_fc_zephyr/plant.py --radio-index 0
```

The firmware's log lines come through prefixed with `fw|`.

## The golden comparison

`simulator/golden/` holds traces recorded from the host build at step 5, on
Windows with clang-cl. The Zephyr build, compiled by GCC for Linux, flies both
of them to the last printed digit:

```
ok   z      max 0.000000 m at t=0.004s (tol 0.01)
ok   roll   max 0.000000 deg at t=0.004s (tol 0.5)
ok   m0     max 0.000000  at t=0.004s (tol 0.01)
ok   armed  max 0.000000  at t=0.004s (tol 0.0)
MATCH
```

That is the point of this step. The port is proven by a number rather than
by watching it fly: same inputs, same outputs, across compilers, operating
systems and an RTOS. Any change that moves the trajectory now shows up as a
`MISMATCH` here and in CI.

`arm-climb-roll` is the stronger of the two, because it puts a roll command
through the rate PID. `arm-hover` only checks arming and the mixer at zero
correction.

If a change is meant to alter the flight, rerecord the golden from the host
build with step 5's `--record`, and check the new one into the same commit.
