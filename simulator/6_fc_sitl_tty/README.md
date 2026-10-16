# 6. A pty instead of a pipe, and a real baud rate

Same controller and same wire format as [step 5](../5_fc_sitl_cpp/README.md).
The one thing that changes is how the two ends are connected: a pseudo-terminal
rather than a pipe, so both sides open a device path.

```
  plant.py                                  fc_sitl_cpp
  ---------------------------               ---------------------------
  pybullet world (CtrlAviary)               rate PID + mixer, acro
  pty master                     --0xA5-->  opens /dev/pts/N with --dev
                                 <--0x5A--
```

That is the point of the step. Zephyr will not hand you a pipe. On `native_sim`
the UART is a pty, and on real hardware it is a UART or a USB CDC port, so both
of those are a device path too. Moving to Zephyr becomes a change of path rather
than a change of plumbing.

Linux only: ptys are POSIX. Run it under WSL if you are on Windows.

## Run

```bash
python simulator/6_fc_sitl_tty/plant.py --no-gui --fc-args=--script=arm-hover
```

Flags are step 5's, plus `--baud`. `--sticks fc|plant` works here too and means
the same thing.

## Why --baud exists

A pty is a memory buffer with no wire in it. It will happily run thousands of
exchanges a second and tell you nothing about whether a real serial link could
keep up, which makes it a comfortable way to be wrong about throughput until
hardware arrives.

`--baud` emulates the serialisation time so the limit shows up here instead. At
8N1 each byte costs ten bit times, one start and one stop on top of the eight:

```
python simulator/6_fc_sitl_tty/plant.py --no-gui --baud 115200 --fc-args=--script=arm-hover

wire: 115200 baud 8N1  43 B out 3.73 ms, 39 B back 3.39 ms, round trip 7.12 ms
      control period 4.17 ms -> DOES NOT FIT, ceiling is 140 Hz

960 exchanges in 7.23 s = 133 Hz achieved, 240 Hz wanted
```

So 115200 cannot carry this loop at 240 Hz, and no amount of tuning changes
that: 82 bytes of framing at ten bit times each is 7.12 ms of serialisation
before anything computes. 230400 halves it to 3.56 ms, which fits inside the
4.17 ms period with very little room. 460800 is the first speed with real
headroom.

Three ways out, if you ever need them, none of them taken here: run the control
loop slower, make the frames smaller (the four echoed `rc` floats and the four
sent ones are for plotting and debugging, not for flying), or use a faster link.
USB CDC sidesteps the question entirely, since its "baud rate" is a fiction and
the real limit is the 1 ms USB frame.

`--baud 0` runs the pty unthrottled, which is the honest way to measure
everything that is not the wire.

## The busy-wait

`spin()` burns CPU rather than calling `time.sleep`. On Linux `sleep` rounds up
to roughly a millisecond, which is the same order as the 1.87 ms a sensor frame
takes at 230400, so sleeping would swamp the thing being measured. It costs a core
while the sim runs, which is the right trade for a measurement tool and would be
the wrong one for anything else.

## Mechanical details

`cfmakeraw`, `VMIN=1`, `VTIME=0` on the controller's side: a tty in canonical
mode would line-buffer, echo, and translate bytes, and the frames are full of
values that look like control characters. `0x11` and `0x13` are XON/XOFF, and
floats produce both constantly, so software flow control has to be off or the
link stalls on the first packet that happens to contain one.

The plant closes its copy of the slave fd as soon as the controller has its own.
Holding it open means EOF never arrives when the controller exits, and the read
blocks forever instead of reporting what happened.
