# 8. Frames and flight behaviour, no physics

Hand-made sensor frames sent to Zephyr firmware, with the replies checked one
by one. No pybullet, so a failure here is a link or firmware problem and cannot
be a physics one.

Two firmwares, two levels:

| firmware | what it does | what gets checked |
| --- | --- | --- |
| [3_link_echo](../../fw/samples/3_link_echo) | answers each sensor frame with zero motors | framing only |
| [app](../../fw/app) | the real controller | framing, then flight behaviour |

Framing: the reply carries the seq that was sent and a valid CRC, garbage
between frames is skipped, a corrupted frame gets no reply at all, and the next
good frame is answered.

Behaviour, `app` only, each one a rule from `fc/core.hpp`:

- no valid sticks, never arms
- arm switch already up when the sticks first arrive: refused until it is cycled
- the switch has to stay up 0.2 s before it arms
- level at half throttle gives four equal motors
- rolling right with the sticks centred brings the left motors down
- sticks lost for less than 0.5 s keeps flying, more than that disarms
- after a failsafe, the switch has to be cycled again
- throttle up at the moment of arming: refused

## Run

Build both, from the repo root:

```bash
. fw/scripts/env.sh
west build -b native_sim/native/64 fw/samples/3_link_echo -d build/fw/3_link_echo
west build -b native_sim/native/64 fw/app -d build/fw/app
```

```bash
python3 simulator/8_zephyr_link_check/link_check.py
python3 simulator/8_zephyr_link_check/link_check.py --app app
```

```
PASS  switch up 0.1 s: not yet armed (hold 0.2 s)
PASS  switch up 0.3 s: armed
PASS  level at half throttle: equal motors  (m=['0.500', '0.500', '0.500', '0.500'])
PASS  rolling right: left motors drop  (m=['0.639', '0.639', '0.361', '0.361'])
...
all passed
```

The exit code is the number of failures, so CI runs it as is.
