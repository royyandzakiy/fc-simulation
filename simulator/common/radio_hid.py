#!/usr/bin/env python3
"""Read an RC transmitter in USB HID joystick mode, from Python, with no deps.

This is the other half of --sticks plant: the plant reads the radio and ships
the channels down the wire, instead of the controller opening the device
itself. Worth having because a Zephyr native_sim build cannot reach a USB HID
device at all, so over the wire is the only route sticks have to it.

Two backends, chosen by platform, both through the standard library:

    Linux    /dev/input/jsN, the joydev event stream
    Windows  joyGetPosEx from winmm.dll, via ctypes

No pygame. Not out of principle: pygame is SDL underneath, and SDL is exactly
what read zeros here before.

    from radio_hid import open_radio
    radio = open_radio()          # raises RadioError if there is nothing there
    axes, buttons = radio.read()  # axes in -1..1, buttons as bools

Both backends normalise to -1..1 and are non-blocking: read() returns the
current position, never waits for movement.

CHANNEL NUMBERS DO NOT TRANSFER BETWEEN THE TWO. joydev reports the radio's
own axis order, which is what src/samples/radio_test prints. winmm reports a
fixed X, Y, Z, R, U, V and nothing past the sixth axis, so a radio with eight
channels loses the top two and the rest may well be in a different order. Check
--ch-* on each host separately rather than assuming.
"""

import ctypes
import os
import struct
import sys


class RadioError(RuntimeError):
    """No radio, or one that stopped answering."""


# =====================================================================
# Linux: /dev/input/jsN
# =====================================================================

# joydev event: u32 time (ms), i16 value, u8 type, u8 number.
_JS_EVENT = struct.Struct("<IhBB")
_JS_EVENT_BUTTON = 0x01
_JS_EVENT_AXIS = 0x02
_JS_EVENT_INIT = 0x80  # synthetic burst on open, reporting current state


class _LinuxRadio:
    def __init__(self, index=0):
        path = "/dev/input/js%d" % index
        try:
            self._fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            raise RadioError(
                "cannot open %s: %s\n"
                "  is the radio in USB Joystick mode?\n"
                "  under WSL it needs usbipd attach first, and "
                "modprobe joydev\n"
                "  check ls -l /dev/input/ and whether you can read it"
                % (path, exc)) from exc

        self.name = path
        self._axes = [0.0] * 32
        self._buttons = [False] * 32

        # The kernel replays current state as init events the moment we open,
        # so one drain gives a real starting position rather than a centred
        # guess. Matters for the arm switch: guessing "centred" on a switch
        # that is actually up would look like someone just flipped it.
        self.read()

    def read(self):
        while True:
            try:
                data = os.read(self._fd, _JS_EVENT.size)
            except BlockingIOError:
                break
            except OSError as exc:
                raise RadioError("radio disconnected: %s" % exc) from exc
            if not data or len(data) < _JS_EVENT.size:
                break

            _, value, ev_type, number = _JS_EVENT.unpack(data)
            kind = ev_type & ~_JS_EVENT_INIT
            if kind == _JS_EVENT_AXIS and number < len(self._axes):
                # i16, so the negative end reaches one count further than the
                # positive. Divide by 32767 and clamp, the same way the C++
                # side does, so full stick reads as exactly -1.0.
                self._axes[number] = max(-1.0, min(1.0, value / 32767.0))
            elif kind == _JS_EVENT_BUTTON and number < len(self._buttons):
                self._buttons[number] = bool(value)

        return list(self._axes), list(self._buttons)

    def close(self):
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None


# =====================================================================
# Windows: joyGetPosEx
# =====================================================================

_JOY_RETURNALL = 0x000000FF
_JOYERR_NOERROR = 0
_MAXPNAMELEN = 32
_MAX_JOYSTICKOEMVXDNAME = 260


class _JOYINFOEX(ctypes.Structure):
    _fields_ = [
        ("dwSize", ctypes.c_uint32),
        ("dwFlags", ctypes.c_uint32),
        ("dwXpos", ctypes.c_uint32),
        ("dwYpos", ctypes.c_uint32),
        ("dwZpos", ctypes.c_uint32),
        ("dwRpos", ctypes.c_uint32),
        ("dwUpos", ctypes.c_uint32),
        ("dwVpos", ctypes.c_uint32),
        ("dwButtons", ctypes.c_uint32),
        ("dwButtonNumber", ctypes.c_uint32),
        ("dwPOV", ctypes.c_uint32),
        ("dwReserved1", ctypes.c_uint32),
        ("dwReserved2", ctypes.c_uint32),
    ]


class _JOYCAPSW(ctypes.Structure):
    _fields_ = [
        ("wMid", ctypes.c_uint16),
        ("wPid", ctypes.c_uint16),
        ("szPname", ctypes.c_wchar * _MAXPNAMELEN),
        ("wXmin", ctypes.c_uint32), ("wXmax", ctypes.c_uint32),
        ("wYmin", ctypes.c_uint32), ("wYmax", ctypes.c_uint32),
        ("wZmin", ctypes.c_uint32), ("wZmax", ctypes.c_uint32),
        ("wNumButtons", ctypes.c_uint32),
        ("wPeriodMin", ctypes.c_uint32), ("wPeriodMax", ctypes.c_uint32),
        ("wRmin", ctypes.c_uint32), ("wRmax", ctypes.c_uint32),
        ("wUmin", ctypes.c_uint32), ("wUmax", ctypes.c_uint32),
        ("wVmin", ctypes.c_uint32), ("wVmax", ctypes.c_uint32),
        ("wCaps", ctypes.c_uint32),
        ("wMaxAxes", ctypes.c_uint32), ("wNumAxes", ctypes.c_uint32),
        ("wMaxButtons", ctypes.c_uint32),
        ("szRegKey", ctypes.c_wchar * _MAXPNAMELEN),
        ("szOEMVxD", ctypes.c_wchar * _MAX_JOYSTICKOEMVXDNAME),
    ]


class _WindowsRadio:
    # joyGetPosEx hands back six axes in this fixed order and has no way to
    # express a seventh. A radio with eight channels loses the top two here.
    _AXIS_FIELDS = ("dwXpos", "dwYpos", "dwZpos", "dwRpos", "dwUpos", "dwVpos")

    def __init__(self, index=0):
        self._winmm = ctypes.WinDLL("winmm")
        self._id = self._find(index)

        caps = _JOYCAPSW()
        if self._winmm.joyGetDevCapsW(self._id, ctypes.byref(caps),
                                      ctypes.sizeof(caps)) == _JOYERR_NOERROR:
            self.name = caps.szPname or "joystick %d" % self._id
            self._n_axes = min(int(caps.wNumAxes), len(self._AXIS_FIELDS))
            self._n_buttons = min(int(caps.wNumButtons), 32)
            # Per-axis ranges rather than a blanket 0..65535: the caps say what
            # this device actually reports, and honouring them is the
            # difference between a stick that reaches 1.00 and one that stops
            # short.
            self._ranges = [
                (int(getattr(caps, "w%smin" % ax)),
                 int(getattr(caps, "w%smax" % ax)))
                for ax in ("X", "Y", "Z", "R", "U", "V")]
        else:
            self.name = "joystick %d" % self._id
            self._n_axes = len(self._AXIS_FIELDS)
            self._n_buttons = 32
            self._ranges = [(0, 65535)] * len(self._AXIS_FIELDS)

    def _find(self, index):
        n = self._winmm.joyGetNumDevs()
        if n == 0:
            raise RadioError("this Windows build reports no joystick support")

        info = _JOYINFOEX()
        info.dwSize = ctypes.sizeof(info)
        info.dwFlags = _JOY_RETURNALL

        live = []
        for joy_id in range(n):
            if self._winmm.joyGetPosEx(joy_id,
                                       ctypes.byref(info)) == _JOYERR_NOERROR:
                live.append(joy_id)
        if not live:
            raise RadioError(
                "no joystick found.\n"
                "  is the radio in USB Joystick mode rather than storage "
                "mode?\n"
                "  it should appear under Set up USB game controllers")
        if index >= len(live):
            raise RadioError("no joystick at index %d, found %d"
                             % (index, len(live)))
        return live[index]

    def read(self):
        info = _JOYINFOEX()
        info.dwSize = ctypes.sizeof(info)
        info.dwFlags = _JOY_RETURNALL
        if self._winmm.joyGetPosEx(self._id,
                                   ctypes.byref(info)) != _JOYERR_NOERROR:
            raise RadioError("radio disconnected")

        axes = []
        for i in range(self._n_axes):
            raw = getattr(info, self._AXIS_FIELDS[i])
            lo, hi = self._ranges[i]
            span = hi - lo
            axes.append(0.0 if span <= 0
                        else max(-1.0, min(1.0, 2.0 * (raw - lo) / span - 1.0)))

        buttons = [bool(info.dwButtons & (1 << i))
                   for i in range(self._n_buttons)]
        return axes, buttons

    def close(self):
        pass


# =====================================================================

def open_radio(index=0):
    """Open the radio for this platform. Raises RadioError if there is none."""
    if sys.platform.startswith("linux"):
        return _LinuxRadio(index)
    if sys.platform == "win32":
        return _WindowsRadio(index)
    raise RadioError("no HID backend for %s. Let the controller read the "
                     "radio instead: --sticks fc" % sys.platform)


def main():
    """radio_hid.py on its own prints live channels, like radio_test does.

    The quickest way to find your --ch-* numbers on a host where you have not
    built the C++ samples.
    """
    import argparse
    import time

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--index", type=int, default=0)
    args = ap.parse_args()

    radio = open_radio(args.index)
    print("radio: %s" % radio.name)
    print("Move one stick or flip one switch at a time and watch which")
    print("number reacts. Ctrl+C quits.\n")
    try:
        while True:
            axes, buttons = radio.read()
            cells = " ".join("a%d=%+.2f" % (i, v) for i, v in enumerate(axes))
            down = " ".join("b%d" % i for i, b in enumerate(buttons) if b)
            print("\r%s  down: %s\x1b[K" % (cells, down or "-"), end="",
                  flush=True)
            time.sleep(0.016)
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        radio.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
