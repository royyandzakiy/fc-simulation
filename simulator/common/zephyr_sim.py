#!/usr/bin/env python3
"""Launch a Zephyr native_sim binary and open the pty its link UART sits on.

The reverse of step 6: there the plant created the pty and handed it to the
controller. A native_sim zephyr.exe creates its own and prints the path:

    uart_1 connected to pseudotty: /dev/pts/5

so this reads zephyr.exe's stdout until that line appears, then opens the
path. Every other stdout line (the firmware's logs) is echoed with a "fw|"
prefix so it stays readable next to the plant's own output. Linux only.
"""

import os
import queue
import re
import select
import subprocess
import sys
import threading
import time
import tty
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
_PTY_LINE = re.compile(r"(\S+) connected to pseudotty: (\S+)")


def find_exe(app):
    """build/fw/<app>/zephyr/zephyr.exe, the path the README's build commands use."""
    exe = REPO_ROOT / "build" / "fw" / app / "zephyr" / "zephyr.exe"
    if not exe.is_file():
        raise SystemExit(
            f"{exe} not found. Build it first, from WSL:\n"
            f"  . fw/scripts/env.sh\n"
            f"  west build -b native_sim/native/64 fw/... -d build/fw/{app}")
    return exe


class ZephyrSim:
    def __init__(self, exe, args=(), uart="uart_1", quiet=False, timeout=10.0):
        self.proc = subprocess.Popen(
            [str(exe), *args],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self._quiet = quiet
        self._uart = uart
        self._ptys = queue.Queue()
        self.lines = []
        threading.Thread(target=self._pump, daemon=True).start()

        try:
            self.uart, self.pty_path = self._ptys.get(timeout=timeout)
        except queue.Empty:
            self.close()
            raise SystemExit(f"{exe} printed no pseudotty line within {timeout:.0f} s")

        self.fd = os.open(self.pty_path, os.O_RDWR | os.O_NOCTTY)
        # Raw: the frames carry bytes that a cooked tty would translate or
        # treat as XON/XOFF, the same reason the firmware side is raw.
        tty.setraw(self.fd)

    def _pump(self):
        for line in self.proc.stdout:
            line = line.rstrip("\n")
            self.lines.append(line)
            m = _PTY_LINE.search(line)
            if m and m.group(1) == self._uart:
                self._ptys.put((m.group(1), m.group(2)))
            if not self._quiet:
                print(f"fw| {line}", file=sys.stderr)

    def write(self, data):
        view = memoryview(data)
        while view:
            n = os.write(self.fd, view)
            view = view[n:]

    def read_exact(self, n, timeout=None):
        """Waits for n bytes. Returns fewer if the pty closes or timeout
        (seconds, None for forever) runs out."""
        deadline = None if timeout is None else time.monotonic() + timeout
        buf = b""
        while len(buf) < n:
            if deadline is not None:
                left = deadline - time.monotonic()
                if left <= 0 or not select.select([self.fd], [], [], left)[0]:
                    break
            try:
                chunk = os.read(self.fd, n - len(buf))
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
        return buf

    def close(self):
        if getattr(self, "fd", None) is not None:
            os.close(self.fd)
            self.fd = None
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
