# 7. Reaching a Zephyr binary through its pty

The first step with Zephyr in it, and there is no flight controller yet. The
firmware is [fw/samples/2_uart_echo](../../fw/samples/2_uart_echo), which sends
back every byte it receives. The question is only whether a script can reach a
`native_sim` binary at all, and how fast.

```
  echo_check.py                             zephyr.exe (2_uart_echo)
  ---------------------------               ---------------------------
  reads zephyr.exe's stdout                 creates a pty for uart_1,
  for the pty path               <--------  prints its path
  opens it, sends bytes          -------->  uart_poll_in -> uart_poll_out
```

Step 6 had it the other way round: the plant made the pty and handed it to the
controller with `--dev`. A `native_sim` binary makes its own and prints
`uart_1 connected to pseudotty: /dev/pts/N`, so the script launches it, reads
that line, and opens the path. [zephyr_sim.py](../common/zephyr_sim.py) does
this for steps 7 to 9.

Linux only. Run it under WSL on Windows.

## Run

Build the sample once, from the repo root (see [fw/README.md](../../fw/README.md)
for the one-off workspace setup):

```bash
. fw/scripts/env.sh
west build -b native_sim/native/64 fw/samples/2_uart_echo -d build/fw/2_uart_echo
```

Then, with any Python 3 (this step imports nothing beyond the standard library):

```bash
python3 simulator/7_zephyr_uart_echo/echo_check.py
```

```
uart_1 is /dev/pts/5
all 256 byte values came back unchanged
43 B round trip over 200 rounds: median 0.22 ms, p95 0.51 ms
```

## What it shows

- **All 256 byte values survive.** Both ends are in raw mode. A cooked tty would
  translate `0x0A` and `0x0D` and swallow `0x11` / `0x13` as XON/XOFF, and float
  frames contain all of them.
- **The round trip fits in a 240 Hz period** with room to spare. It did not at
  first: `native_sim` defaults to a 100 Hz tick, so every `k_usleep(50)` in the
  firmware's poll loop rounded up to 10 ms and a round trip took 20 ms. The
  board config raises the tick to 10 kHz, see
  [native_sim_native_64.conf](../../fw/samples/2_uart_echo/boards/native_sim_native_64.conf).
