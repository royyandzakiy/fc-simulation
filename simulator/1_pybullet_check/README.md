# 1. Does 3D work here at all

Before anything else, check that PyBullet runs and opens a window on this
machine. Nothing here imports the rest of the repo, so if these two pass, any
later failure is in the simulator code and not in the environment.

Setup is in the [root README](../../README.md). Both scripts exit `0` on pass
and `1` on fail, so you can use them as a gate.

## smoke_test.py

Drops a cube on a plane and checks it fell and came to rest.

```bash
.venv\Scripts\python.exe simulator\1_pybullet_check\smoke_test.py
```

```
python   3.11.0
pybullet 3.2.7
mode     GUI (window)

stepping 480 steps (2s at 240 Hz)...
  t= 0.00s  z= 2.000 m
  t= 0.50s  z= 0.771 m
  t= 1.00s  z= 0.025 m

final z  0.0250 m  (expected ~0.025)
PASS: physics stepped, cube fell and came to rest on the plane.
```

Flags: `--headless` (no window), `--seconds N`, `--camera` (render a frame
offscreen).

## hover_demo.py

A quadrotor takes off, holds 1 m, and rides out two gusts. About 90 lines, and
the flight controller is only the 8 lines between the `--- the entire flight
controller ---` markers.

```bash
.venv\Scripts\python.exe simulator\1_pybullet_check\hover_demo.py
```

```
t= 3.0s  <- gust
t= 4.0s  z= 0.99 m  drift=0.29 m  tilt= 2.0 deg
t= 5.0s  z= 1.00 m  drift=0.02 m  tilt= 1.9 deg
```

PyBullet ships no drone model, so `quad.urdf` defines one: a body box and four
rotor discs in X layout, front pair coloured orange so you can tell which way
it faces. The props spin on velocity motors purely for looks, the lift comes
from the controller's force.

## Worth remembering

`applyExternalForce` with `WORLD_FRAME` treats `posObj` as a point in *world*
coordinates, not an offset from the body. Passing `[0,0,0]` applies the force at
the world origin, which is fine while the drone hovers directly above it and
then flips it violently the moment it drifts. Thrust here uses `LINK_FRAME`
(body axes, at the centre of mass) for that reason.

## If the window does not open

`could not connect to a PyBullet server` in GUI mode means no OpenGL context,
which happens over Remote Desktop or inside WSL without a display. Confirm the
physics still works with `--headless`; if that passes, the problem is the
display and not PyBullet.
