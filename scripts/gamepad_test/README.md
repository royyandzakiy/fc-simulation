# gamepad_test

Reads an Xbox gamepad and prints what it does. One file, no install, no venv.

```bash
python scripts\gamepad_test\gamepad_test.py
```

That is the whole setup. It uses `ctypes` and XInput, both already part of
Windows and Python, so any Python 3 works — including the MSYS2 one on your
PATH.

## Modes

**Default — events**, the same shape as `bin\clang-cl\gamepad_btn_test.exe`:

```
pad: XInput controller in slot 0  (via XInput1_4.dll)
press buttons / push sticks to max, Ctrl+C to quit

axis    left stick  Y  MAX - (-1.00)
button south (A)      DOWN
button south (A)      up
button dpad left      DOWN
button dpad left      up
trigger right      MAX (32767)
button right shoulder DOWN
button right shoulder up
```

Buttons print on press and release. Sticks and triggers print once when pushed
to an extreme and rearm only after returning near centre, so holding a stick
does not spam a line per poll.

**`--live`** — one continuously updating line. Best for watching stick drift or
finding where a deadzone ends:

```
L(+0.00,-0.98) R(+0.12,+0.03) LT 0.00 RT 1.00  south (A), right shoulder
```

**`--raw`** — exactly what XInput returns, uninterpreted. Use this when you want
to know what the hardware said rather than what this script made of it:

```
packet 319753     buttons 0x0000  LT   0 RT   0  LX     +0 LY     +0  RX     +0 RY     +0
```

**`--pad N`** selects a controller slot, 0–3. With nothing in the requested
slot it looks in the others and tells you which one to use.

## Matching the C++ tool

Two deliberate conversions, so output can be compared line for line:

- **Names are SDL's, not XInput's** — `south (A)`, `west (X)`,
  `right shoulder`, `dpad left`.
- **Stick Y is flipped.** XInput reports up as *positive*; SDL reports it as
  *negative*. Negating it keeps `axis left stick Y MAX - (-1.00)` meaning "up"
  in both tools. Triggers are scaled from XInput's 0–255 to SDL's 0–32767 for
  the same reason.

Everything else already agrees between the two APIs.

## Why XInput and not pygame

The pygame version in `scripts/ctrlaviary_check/` reads nothing — every button
stays 0 and every axis sits at its default, while the C++ tool reads the same
controller perfectly. Don't retry that path expecting a different result.

The pad itself is fine: XInput reports `XInputGetState(0) -> 0` (connected), and
its packet counter climbs steadily, which means Windows is actively updating the
device. The likely culprit is SDL2's RAWINPUT joystick backend, which needs a
real window to receive `WM_INPUT` messages — with no window the device
enumerates but its state never updates. XInput has no such requirement: it
polls, so it works from a bare console script.

## Limitations

- **Windows only.** XInput is a Windows API.
- **Xbox-style pads only.** A DirectInput-only joystick, a flight stick or an
  arcade pad will not appear here at all. The script says so when it finds
  nothing rather than failing silently.
- XInput exposes 4 slots, so at most 4 controllers.
