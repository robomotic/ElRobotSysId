#!/usr/bin/env python3
"""Printable boom for the stall test (issue #2): screws onto the gripper's camera mount in place of the U20CAM.

WARNING (2026-11-23, user: "the camera base is quite fragile"): do NOT use it on the camera mount. The whole joint
torque (up to ~0.85 N m) would pass through the mount's thin post, snap fit and M2 self-tapping screws, which carry a
14 g camera (~0.01 N m). Use the fingertips on M6 instead (README, stall test). Kept as a geometry reference.

Geometry (plate frame: x = sideways along the gripper's jaw-closing axis, y = in the seat plane "up", z = out of the
seat; the seat faces forward and 24 deg up, so the camera looks forward/down):
- plate 44 x 44 x 4 mm with 4 diagonal slots for M2 screws (2.4 mm wide): fits square hole patterns of 26-35 mm.
  The norma-core CameraMount_square_27mm.stp has its 4 x 2.0 mm pilot holes 33.8 mm apart (M2 self-tapping).
- arm 100 mm sideways (+x), 14 mm tall (in the seat plane, which is the load direction) x 10 mm thick.
- leg at x = 100 mm going 55 mm down the seat plane, ending in a 5 mm-radius rounded nub that presses on the scale.

At the level pose the nub is ~139 mm above the table and ~99 mm (horizontally) from the M7 roll axis; lower M7 ticks
press it down (stall test: --stall 7 --into -1 --lever-mm <measured>). Measure the lever on the arm: horizontal
distance from the gripper's roll axis to the nub's contact point.

Print: PETG or PLA, plate flat on the bed (the leg then stands up), 100 % infill or 6 walls, no supports needed.
Load: ~10 N at the nub at the test's 17 % duty -> ~1 N m at the plate; the section takes it with a large margin.

Usage: python3 tools/make_stall_boom.py [--out hardware/stall_boom.stl]   (needs trimesh + manifold3d)
"""
import argparse
import os

import numpy as np
import trimesh

PLATE = 44.0
PLATE_T = 4.0
SLOT_W = 2.4
SLOT_SQUARE = (26.0, 35.0)  # hole-pattern sides the slots cover
ARM_L = 100.0
ARM_H = 14.0  # in the seat plane (y), the load direction
ARM_T = 10.0  # out of the seat (z)
LEG_L = 55.0
LEG_W = 12.0
NUB_R = 5.0  # = ARM_T / 2: flush with both faces, so the part prints flat


def box(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box(extents=(x1 - x0, y1 - y0, z1 - z0))
    b.apply_translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    return b


def slot(angle_deg, r0, r1, w, z0, z1):
    """A slot along a ray from the origin, from radius r0 to r1 (rounded ends)."""
    a = np.radians(angle_deg)
    d = np.array([np.cos(a), np.sin(a), 0.0])
    length = r1 - r0
    body = trimesh.creation.box(extents=(length, w, z1 - z0))
    body.apply_transform(trimesh.transformations.rotation_matrix(a, (0, 0, 1)))
    body.apply_translation(d * (r0 + r1) / 2 + (0, 0, (z0 + z1) / 2))
    ends = [trimesh.creation.cylinder(radius=w / 2, height=z1 - z0, sections=32) for _ in range(2)]
    for e, r in zip(ends, (r0, r1)):
        e.apply_translation(d * r + (0, 0, (z0 + z1) / 2))
    return trimesh.boolean.union([body] + ends)


def build():
    # Outward from the seat is +z here (the seat face is z = 0); the plate is z in [0, 4].
    plate = box(-PLATE / 2, PLATE / 2, -PLATE / 2, PLATE / 2, 0, PLATE_T)
    arm = box(0, ARM_L + LEG_W / 2, -ARM_H / 2, ARM_H / 2, 0, ARM_T)
    leg = box(ARM_L - LEG_W / 2, ARM_L + LEG_W / 2, -LEG_L, ARM_H / 2, 0, ARM_T)
    nub = trimesh.creation.icosphere(subdivisions=3, radius=NUB_R)
    nub.apply_translation((ARM_L, -LEG_L, ARM_T / 2))
    gusset = box(PLATE / 2 - 6, PLATE / 2 + 14, -ARM_H / 2 - 2, ARM_H / 2 + 2, 0, ARM_T)  # stiffer root, clear of screw heads
    solid = trimesh.boolean.union([plate, arm, leg, nub, gusset])
    r0, r1 = SLOT_SQUARE[0] / np.sqrt(2), SLOT_SQUARE[1] / np.sqrt(2)
    cuts = [slot(a, r0, r1, SLOT_W, -1, ARM_T + 1) for a in (45, 135, 225, 315)]
    return trimesh.boolean.difference([solid] + cuts)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "hardware", "stall_boom.stl"))
    a = ap.parse_args()
    m = build()
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    m.export(a.out)
    lo, hi = m.bounds
    print("written %s" % os.path.abspath(a.out))
    print("watertight %s, volume %.1f cm^3 (~%.0f g PLA at 100 %%), size %.0f x %.0f x %.0f mm" % (
        m.is_watertight, m.volume / 1000, m.volume / 1000 * 1.24, *(hi - lo)))


if __name__ == "__main__":
    main()
