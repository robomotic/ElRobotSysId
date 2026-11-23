#!/usr/bin/env python3
"""Printable boom for the stall test (issue #2), screwed onto the M8 (gripper) servo horn in place of the gripper gear.

Why there: the horn is made to carry the servo's full torque, so the test load goes straight into the servo and nothing
fragile sits in between. (A first design on the camera mount was withdrawn: the user pointed out the camera base is
fragile, and the whole joint torque would have gone through it.)

Taking the fingers off: unscrew the 4 M3 screws that hold the gripper gear to the M8 horn (the horn itself stays on the
servo spline); the jaws come off with the gear. MARK the gear's angle on the horn first, so it goes back at the same
angle and the station's gripper calibration stays valid.

Interface, copied from norma-core Gripper_Gear.stp / "Gripper Gear.stl":
- 4 x 3.3 mm holes on a 14 mm circle (0/90/180/270 deg), centre hole 7 mm;
- a 23.5 mm recess, 3.15 mm deep, that centres the part on the horn; 2.85 mm of material under the screw heads, so the
  gear's own M3 screws fit.

Geometry (part frame: z = the M8 axis, pointing away from the servo, i.e. forward along the fingers; z = 0 is the
horn face; x = sideways):
- hub: disc 34 mm, recess 23.6 x 3.15 on the horn side, 4 x 3.4 mm holes on 14 mm, centre 7 mm, flange 2.85 mm;
- standoff tube 34/22 mm up to z = 20 (the gripper base reaches 12 mm in front of the horn and +-40 mm sideways: the
  arm has to pass in front of it); the 22 mm bore leaves the 4 screws reachable with a screwdriver from the front;
- arm at z = 20..28 (8 mm), 14 mm tall, out to the nub at 100 mm from the axis;
- nub: R 5 mm on the arm's lower edge at x = 100 mm (the contact point on the scale).

At the level pose the M8 axis is ~129 mm above the table; the nub's lowest point ~117 mm. Lever = horizontal distance
from the M8 axis to the nub's contact point: 100 mm by design (measure it). The 4-hole pattern lets the arm point to
either side (90 deg steps); find the pressing direction on the Pose screen (M8, 1-5 tick steps).

Print: PETG or PLA, arm side (z = 28 face) down on the bed so the tube stands up, 100 % infill or 6 walls; the horn
recess is then on top (no supports). Load: <= ~0.85 N m (8.5 N at the nub) at the default --stall-max 20 even with
Kt*eta*N 1.7: small for this section.

Usage: python3 tools/make_stall_boom.py [--out hardware/stall_boom_m8.stl]   (needs trimesh + manifold3d)
"""
import argparse
import os

import numpy as np
import trimesh

HUB_D = 34.0
RECESS_D = 23.6
RECESS_DEPTH = 3.15
FLANGE = 2.85
HOLE_D = 3.4
HOLE_PCD = 14.0
CENTRE_D = 7.0
BORE_D = 22.0
STANDOFF = 20.0  # horn face to the arm's back face
ARM_T = 8.0  # along the axis
ARM_H = 14.0  # tangential: the load direction
LEVER = 100.0
NUB_R = 5.0


def cyl(d, z0, z1, x=0.0, y=0.0, sections=96):
    c = trimesh.creation.cylinder(radius=d / 2, height=z1 - z0, sections=sections)
    c.apply_translation((x, y, (z0 + z1) / 2))
    return c


def box(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box(extents=(x1 - x0, y1 - y0, z1 - z0))
    b.apply_translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))
    return b


def build():
    top = STANDOFF + ARM_T
    tube = cyl(HUB_D, 0, top)
    arm = box(0, LEVER, -ARM_H / 2, ARM_H / 2, STANDOFF, top)
    nub = trimesh.creation.icosphere(subdivisions=3, radius=NUB_R)
    nub.apply_translation((LEVER, -ARM_H / 2, STANDOFF + ARM_T / 2))
    root = box(HUB_D / 2 - 4, HUB_D / 2 + 12, -ARM_H / 2 - 3, ARM_H / 2 + 3, STANDOFF - 6, top)  # stiffer root
    solid = trimesh.boolean.union([tube, arm, nub, root])
    cuts = [
        cyl(RECESS_D, -1, RECESS_DEPTH),  # sits over the horn
        cyl(BORE_D, RECESS_DEPTH + FLANGE, top + 1),  # screwdriver access from the front
        cyl(CENTRE_D, -1, top + 1),
    ]
    for a in (0, 90, 180, 270):
        x, y = HOLE_PCD / 2 * np.cos(np.radians(a)), HOLE_PCD / 2 * np.sin(np.radians(a))
        cuts.append(cyl(HOLE_D, -1, top + 1, x, y, sections=32))
    return trimesh.boolean.difference([solid] + cuts)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "hardware", "stall_boom_m8.stl"))
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
