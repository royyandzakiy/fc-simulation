# 2. Read the gamepad

One file, no install, no venv. It uses `ctypes` and XInput, both already part
of Windows and Python, so any Python 3 runs it:

```bash
python simulator\2_gamepad_test\gamepad_test.py
```

## Modes

**Default, events**, the same shape as `gamepad_btn_test.exe`:

```
pad: XInput controller in slot 0  (via XInput1_4.dll)
press buttons / push sticks to max, Ctrl+C to quit

axis    left stick  Y  MAX - (-1.00)
button south (A)      DOWN
button south (A)      up
trigger right      MAX (32767)
button right shoulder DOWN
```

Buttons print on press and release. Sticks and triggers print once at an
extreme and rearm only after returning near centre, so holding a stick does not
spam a line per poll.

**`--live`** gives one continuously updated line, good for watching stick drift
or finding where a deadzone ends. **`--raw`** prints what XInput returns with no
interpretation. **`--pad N`** picks a slot, 0 to 3.

## Matching the C++ tool

Two conversions so the output compares line for line. Names are SDL's rather
than XInput's (`south (A)`, `west (X)`, `right shoulder`, `dpad left`), and
stick Y is flipped because XInput reports up as positive while SDL reports it
as negative. Triggers scale from XInput's 0-255 to SDL's 0-32767 for the same
reason.

## Why XInput and not pygame

I tried pygame first and it read nothing: every button stayed 0 and every axis
sat at its default, while the C++ tool read the same controller fine. The pad
was never the problem, XInput reports it connected and its packet counter
climbs. The likely cause is SDL2's RAWINPUT joystick backend, which needs a
real window to receive `WM_INPUT`, so with no window the device enumerates but
its state never updates. XInput just polls, so it works from a bare console
script.

Do not retry the pygame route expecting a different result.

## Limitations

Windows only, and XInput sees Xbox-style pads only. A DirectInput-only joystick
or flight stick will not appear here at all, and the script says so rather than
failing quietly.
