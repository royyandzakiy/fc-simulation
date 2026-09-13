# hil_bridge

Lockstep hardware-in-the-loop bridge: `gym-pybullet-drones` is the plant,
`fc_simulation` is the controller, a gamepad supplies the sticks.

Every physics step blocks on the controller's reply, so nothing runs in
realtime and runs are reproducible.

## Setup

```
py -3.11 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe -m pip install --no-deps --ignore-requires-python -r requirements-gpd.txt
```

Two steps on purpose. `gym-pybullet-drones` is not on PyPI, and the
dependencies it declares pull in `stable-baselines3` and PyTorch - several
GB that this bridge never imports, since it only uses `CtrlAviary`. The
second install takes the package alone; everything it genuinely needs at
runtime is already pinned in `requirements.txt`. `--ignore-requires-python`
is there because upstream declares `>=3.12` while the package is pure Python
and runs fine on 3.11.

Use **Python 3.11** (`py -3.11`), not a bare `python3` - on Windows that may
resolve to the MSYS2 interpreter, which cannot build `pybullet`.

The first install takes a few minutes: `pybullet` publishes no Windows
wheels, so pip compiles it from source with MSVC. Everything else is a wheel.

The rerun **viewer** is a separate binary (the SDK stopped bundling it in
0.37) and is not installed here. Without it `bridge.py` says so and carries
on with console output; grab it from
<https://github.com/rerun-io/rerun/releases> if you want the plots.

## Run

Build the controller first, then plug in the pad:

```
.venv\Scripts\python.exe bridge.py
```

| flag | effect |
| --- | --- |
| `--no-gui` | no pybullet window |
| `--no-rerun` | skip the rerun viewer, console output only |
| `--seconds N` | simulated seconds (default 60) |
| `--pad-throttle trigger` | throttle on the trigger instead of the stick |
| `--fc <path>` | use a specific controller binary |

`bridge.py` finds the binary itself by taking the most recently built
`fc_simulation[.exe]` under `bin/` or `build/`, so it picks up whichever
toolchain you built last.

## Wire format

Defined once per side and asserted on both:

- `src/app/fc_core.hpp` - `#pragma pack(1)` structs, `static_assert`ed sizes
- `bridge.py` - the same layout as `construct` structs, sizes asserted at import

Sensor packet is 22 bytes (`0xA5`), motor packet 39 (`0x5A`), each with a
trailing CRC-8 (poly `0xD5`, init 0, no reflection) over every preceding
byte. If you change one side, change the other - the asserts will catch a
size drift, but not a reordered field.
