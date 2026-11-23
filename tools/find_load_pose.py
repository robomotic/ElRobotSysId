#!/usr/bin/env python3
"""Find a gravity-loaded pose for the torque fit (issue #2): a pose where M2 and M3 carry a real gravity load, well above
the table, inside the calibrated ranges, and reachable on a straight path (in ticks) from the present pose.

This is how the load pose M2 1750 / M3 2100 / M4 2450 (2026-11-23) was found: ~0.31/0.48/0.42 N m on M2/M3/M4, 91 mm
above the table, the straight path from the folded pose never below 51 mm. Reach it with the Pose screen `g` key:
`elrobot-sysid --target <ticks M1..M7>`. Moving joints one at a time can be much worse: from the folded pose, raising
M2 alone swings the gripper into the table.

    python3 tools/find_load_pose.py --present 2001,1042,2051,3160,1881,1869,2244 [--min-torque 0.3] [--min-clear 80]

Uses the norma-core URDF (gravity, joint positions), the tool spheres, table height, calibration and ranges of
sysid_safety.toml. Needs mujoco and numpy. The app re-checks every move with its own safety model (src/safety.rs).
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params_file  # noqa: E402
from elrobot_model import RANGE_MAX, RANGE_MIN, Arm  # noqa: E402

TOL = 5  # ticks of tolerance on the calibrated ranges (as the app)


class Model:
    def __init__(self):
        self.arm = Arm()
        safety = params_file.load(params_file.SAFETY)[""]
        self.spheres = [(np.array(c), r) for c, r in safety.get("tool_spheres", [])]
        self.table = float(safety.get("table_z_m", 0.0))
        m = self.arm.m
        self.gb = m.body("Gripper_Base_v1_1").id
        self.jids = [m.joint("rev_motor_0%d" % (i + 1)).id for i in range(7)]

    def state(self, ticks):
        """(lowest point above the table [m], gravity torques M1..M7 [N m], gripper position [m])."""
        self.arm.set_ticks(ticks)
        d = self.arm.d
        zs = [d.xanchor[j][2] for j in self.jids[1:]]
        r, p = d.xmat[self.gb].reshape(3, 3), d.xpos[self.gb]
        zs += [(p + r @ c)[2] - rad for c, rad in self.spheres]
        return min(zs) - self.table, d.qfrc_bias[self.arm.dadr].copy(), p.copy()

    def path_low(self, a, b, n=60):
        return min(self.state([a[i] + (b[i] - a[i]) * s for i in range(7)])[0] for s in np.linspace(0, 1, n))

    def sweep_low(self, pose, joint, amp):
        return min(self.state([pose[i] + (d if i == joint else 0) for i in range(7)])[0] for d in range(-amp, amp + 1, 5))


def in_range(j, t):
    return RANGE_MIN[j] - TOL <= t <= RANGE_MAX[j] + TOL


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--present", required=True, help="present pose, 7 ticks M1..M7 (e.g. from --show-clearance)")
    ap.add_argument("--min-torque", type=float, default=0.3, help="gravity torque each of M2 and M3 must carry, N m")
    ap.add_argument("--min-clear", type=float, default=80.0, help="lowest point above the table at the pose, mm")
    ap.add_argument("--amp", type=int, default=50, help="planned sysid amplitude: sweeps of M2-M4 checked, ticks")
    ap.add_argument("--step", type=int, default=50, help="grid step, ticks")
    ap.add_argument("--show", type=int, default=8)
    a = ap.parse_args()
    now = [int(x) for x in a.present.split(",")]
    if len(now) != 7:
        sys.exit("--present needs 7 ticks")
    mdl = Model()
    low, tau, grip = mdl.state(now)
    print("present: lowest point %.0f mm above the table; gravity M2 %.2f M3 %.2f M4 %.2f N m" % (
        low * 1e3, abs(tau[1]), abs(tau[2]), abs(tau[3])))
    found = []
    for t2 in range(RANGE_MIN[1] + 50, RANGE_MAX[1] - 50, a.step):
        for t3 in range(RANGE_MIN[2] + 50, RANGE_MAX[2] - 50, a.step):
            for t4 in range(RANGE_MIN[3] + 50, RANGE_MAX[3] - 50, 2 * a.step):
                pose = [now[0], t2, t3, t4] + now[4:]
                low, tau, grip = mdl.state(pose)
                if low * 1e3 < a.min_clear or abs(tau[1]) < a.min_torque or abs(tau[2]) < a.min_torque:
                    continue
                move = max(abs(pose[i] - now[i]) for i in range(7))
                found.append((move, pose, low, tau, grip))
    found.sort(key=lambda f: f[0])
    print("%d poses with M2, M3 >= %.2f N m and >= %.0f mm clear; smallest moves first:" % (len(found), a.min_torque, a.min_clear))
    shown = 0
    for move, pose, low, tau, grip in found:
        ok_range = all(in_range(j, pose[j] + s * a.amp) for j in (1, 2, 3) for s in (-1, 1))
        if not ok_range:
            continue
        path = mdl.path_low(now, pose)
        sweeps = min(mdl.sweep_low(pose, j, a.amp) for j in (1, 2, 3))
        if path * 1e3 < 30 or sweeps * 1e3 < 30:
            continue
        print("  --target %s | move %4d ticks | gravity M2 %.2f M3 %.2f M4 %.2f N m | %3.0f mm clear, sweeps +-%d %3.0f mm, "
              "straight path %3.0f mm | gripper %s mm" % (
                  ",".join(str(t) for t in pose), move, abs(tau[1]), abs(tau[2]), abs(tau[3]), low * 1e3, a.amp,
                  sweeps * 1e3, path * 1e3, np.round(grip * 1e3)))
        shown += 1
        if shown >= a.show:
            break


if __name__ == "__main__":
    main()
