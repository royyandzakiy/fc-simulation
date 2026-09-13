# pybullet_check

Standalone proof that 3D physics and the OpenGL window work on this machine.
No C++, no serial link, no gamepad, no `gym-pybullet-drones` — nothing here
imports anything else in this repo. If these two scripts pass, the environment
is fine and any later failure is in the bridge, not in PyBullet.

## Setup

Needs CPython **3.11** — `pybullet` publishes prebuilt `cp311-win_amd64`
wheels, so nothing compiles and no MSVC toolchain is required.

```bash
py -3.11 -m venv scripts\pybullet_check\.venv
```

```bash
scripts\pybullet_check\.venv\Scripts\python.exe -m pip install -r scripts\pybullet_check\requirements.txt
```

Activating the venv is optional. Every command below calls the venv's
`python.exe` by full path, which works from any directory and from both
`cmd.exe` and PowerShell.

> **Do not use `python3` on this machine.** It resolves to MSYS2's
> `C:\msys64\ucrt64\bin\python3.exe`, which has no PyBullet wheels — that is
> the `ModuleNotFoundError: No module named 'numpy'` you get otherwise.

## Run

**`smoke_test.py`** — drops a cube on a plane, checks it fell and came to rest.

```bash
scripts\pybullet_check\.venv\Scripts\python.exe scripts\pybullet_check\smoke_test.py
```

A window opens and a small cube falls from 2 m. Expect `final z 0.0250 m` and
`PASS`. Flags: `--headless` (no window), `--seconds N`, `--camera` (render a
frame offscreen; saves `smoke_test.png` if Pillow is installed).

**`hover_demo.py`** — a quadrotor takes off, hovers, and rides out two gusts.

```bash
scripts\pybullet_check\.venv\Scripts\python.exe scripts\pybullet_check\hover_demo.py
```

It climbs to 1 m and holds. At t=3 s and t=6 s a sideways gust blows it about
29 cm off station; it leans ~2° into the wind and recovers within a second,
while altitude stays within a centimetre.

```
t= 3.0s  <- gust
t= 4.0s  z= 0.99 m  drift=0.29 m  tilt= 2.0 deg
t= 5.0s  z= 1.00 m  drift=0.02 m  tilt= 1.9 deg
```

Flags: `--headless`, `--seconds N`.

Both scripts exit `0` on pass and `1` on fail, so they work as a gate in a
script or CI job.

## Files

| File | What it is |
|---|---|
| `smoke_test.py` | Does physics step and does a window open at all |
| `hover_demo.py` | A drone flying, with a ~8 line controller |
| `quad.urdf` | The aircraft. PyBullet ships no drone model, so this defines one |
| `requirements.txt` | `pybullet` and `numpy`, pinned |

`quad.urdf` is a body box plus four rotor discs in X layout, front pair
coloured orange so you can tell which way it faces. The props spin on velocity
motors purely for looks — lift comes from the controller's force, not from the
visual rotors.

In `hover_demo.py`, the flight controller is only the lines between the
`--- the entire flight controller ---` markers: an altitude PD, an outer loop
turning position error into a lean request, and an inner loop chasing that
attitude. That block is what gets replaced later by packets from the real
flight controller; everything around it is the plant and stays.

## If it fails

**`Cannot load URDF file` / `URDF file 'quad.urdf' not found`** — you are on an
old copy of `hover_demo.py`. It now resolves the URDF relative to the script,
so the working directory no longer matters.

**`could not connect to a PyBullet server`** in GUI mode — no OpenGL context.
Happens over Remote Desktop, inside WSL without a display, or with broken
drivers. Confirm the physics still works with `--headless`; if that passes, the
problem is the display, not PyBullet. A working machine prints its renderer at
startup, e.g. `Version = 4.6.0`, `Vendor = Intel`.

**`ModuleNotFoundError`** — you are running the system Python instead of the
venv's. Use the full `.venv\Scripts\python.exe` path as shown above.

**The drone flips over instead of hovering.** If you have been editing the
controller: `applyExternalForce` with `WORLD_FRAME` treats `posObj` as a point
in *world* coordinates, not an offset from the body, so passing `[0, 0, 0]`
applies the force at the world origin. The lever arm that creates will flip the
drone as soon as it drifts off the origin. Thrust uses `LINK_FRAME` (body axes,
applied at the centre of mass) for exactly this reason.
