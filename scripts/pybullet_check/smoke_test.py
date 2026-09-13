#!/usr/bin/env python3
"""Minimal PyBullet smoke test - does 3D physics work on this machine at all?

Deliberately standalone: no fc_simulation, no serial, no gamepad, no
gym-pybullet-drones. Drops a cube onto a plane and checks that it fell and
came to rest. If this passes, PyBullet and its renderer are fine and any
later failure is in the bridge, not the environment.

    python smoke_test.py              # opens a window, watch the cube drop
    python smoke_test.py --headless   # no window, physics only (CI-safe)
    python smoke_test.py --camera     # headless + render one frame to PNG

Exit code is 0 on pass, 1 on fail, so it is usable as a gate in a script.
"""

import argparse
import sys
import time

HZ = 240
START_HEIGHT = 2.0
# A settled cube rests with its centre one half-extent above the plane.
# pybullet's cube_small.urdf is 0.05 m per side.
RESTING_Z = 0.025
TOLERANCE = 0.02


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--headless", action="store_true",
                    help="run without a window (DIRECT mode)")
    ap.add_argument("--camera", action="store_true",
                    help="also render a frame to smoke_test.png")
    ap.add_argument("--seconds", type=float, default=3.0,
                    help="simulated seconds (default: 3)")
    args = ap.parse_args()

    try:
        import pybullet as p
        import pybullet_data
    except ImportError as exc:
        print(f"FAIL: {exc}\n  Install deps: pip install -r requirements.txt", file=sys.stderr)
        return 1

    print(f"python   {sys.version.split()[0]} ({sys.executable})")
    print(f"pybullet {getattr(p, '__version__', '3.2.7')}")

    # GUI opens a real OpenGL window; DIRECT is pure computation. GUI is the
    # part most likely to break (drivers, remote desktop, WSL), so it is the
    # default - a headless pass would not prove much about 3D.
    mode = p.DIRECT if args.headless else p.GUI
    print(f"mode     {'DIRECT (headless)' if args.headless else 'GUI (window)'}")

    client = p.connect(mode)
    if client < 0:
        print("FAIL: could not connect to a PyBullet server.\n"
              "  In GUI mode this usually means no OpenGL context "
              "(no display, or a driver problem). Retry with --headless.",
              file=sys.stderr)
        return 1

    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / HZ)

        p.loadURDF("plane.urdf")
        cube = p.loadURDF("cube_small.urdf", [0, 0, START_HEIGHT])

        if mode == p.GUI:
            p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50,
                                         cameraPitch=-30, cameraTargetPosition=[0, 0, 0.3])

        steps = int(HZ * args.seconds)
        print(f"\nstepping {steps} steps ({args.seconds:g}s at {HZ} Hz)...")

        for i in range(steps):
            p.stepSimulation()
            if mode == p.GUI:
                # Only in GUI: pace to wall clock so the drop is watchable.
                time.sleep(1.0 / HZ)
            if i % (HZ // 4) == 0:
                z = p.getBasePositionAndOrientation(cube)[0][2]
                print(f"  t={i / HZ:5.2f}s  z={z:6.3f} m")

        final_z = p.getBasePositionAndOrientation(cube)[0][2]

        if args.camera:
            w, h, px, _, _ = p.getCameraImage(320, 240)
            try:
                import numpy as np
                from PIL import Image
                Image.fromarray(np.reshape(px, (h, w, 4))[:, :, :3].astype("uint8")).save("smoke_test.png")
                print("wrote smoke_test.png")
            except ImportError:
                print(f"rendered {w}x{h} frame (install Pillow to save it as PNG)")

        print(f"\nfinal z  {final_z:.4f} m  (expected ~{RESTING_Z:.3f})")

        if final_z > START_HEIGHT - 0.1:
            print("FAIL: the cube never fell - gravity or stepping is broken.", file=sys.stderr)
            return 1
        if abs(final_z - RESTING_Z) > TOLERANCE:
            print(f"FAIL: cube settled at {final_z:.4f}, not ~{RESTING_Z:.3f} - "
                  "collision response looks wrong.", file=sys.stderr)
            return 1

        print("\nPASS: physics stepped, cube fell and came to rest on the plane.")
        if mode == p.GUI:
            print("      A window opened, so the renderer works too.")
        return 0

    finally:
        p.disconnect()


if __name__ == "__main__":
    sys.exit(main())
