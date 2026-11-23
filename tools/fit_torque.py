#!/usr/bin/env python3
"""Torque per amp (Kt*eta*N) and gearbox static friction per joint, from a gravity-loaded session (issue #2).

The ramp test ends each leg with a hold: `ramp_hold+` is reached moving up the tick scale, `ramp_hold-` moving down.
At standstill the motor holds gravity minus (or plus) the gearbox's static friction, depending on the side it came
from. With the gravity torque from MuJoCo (norma-core URDF) at the measured pose:

    Kt*eta*N * i_motor(+) = |tau_g| - tau_f        Kt*eta*N * i_motor(-) = |tau_g| + tau_f   (or swapped)
    => Kt*eta*N = |tau_g| / mean(i+, i-)           tau_f = Kt*eta*N * |i+ - i-| / 2

i_motor = reported current / |duty| (the ST3215 reports the supply current). Needs a pose where the joint carries a
real gravity load (>= 0.15 N m), e.g. M2 1750 / M3 2100 / M4 2450 (2026-11-23): results there were Kt*eta*N
1.69/1.62/1.84 N m/A and static friction 0.25/0.30/0.41 N m on M2/M3/M4.

Usage:  python3 tools/fit_torque.py sysid_logs/<stamp>_<payload> [--out torque.toml]
Needs mujoco and numpy (laptop); ELROBOT_URDF overrides the URDF path.
"""
import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params_file  # noqa: E402
from elrobot_model import Arm, read_meta_base  # noqa: E402

MIN_TAU = 0.15  # N m of gravity load for a usable estimate
MIN_DUTY = 0.02  # below: the motor current is taken as ~0 (gearbox holds)


def hold_side(rows, name):
    rr = [r for r in rows if r["segment"] == name]
    if not rr:
        return None
    duty = np.array([abs(float(r["load"])) / 1000 for r in rr])
    cur = np.array([float(r["current_ma"]) / 1000 for r in rr])
    im = np.where(duty > MIN_DUTY, cur / np.maximum(duty, MIN_DUTY), 0.0)
    pos = int(np.median([int(r["position"]) for r in rr]))
    return {"i_motor_a": float(np.mean(im)), "duty": float(np.mean(duty)), "position": pos, "samples": len(rr)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--out", help="output TOML (default <session>/torque.toml)")
    ap.add_argument("--write-params", action="store_true",
                    help="write kt_eta_n and static_friction_nm per joint to servo_params.toml (not lower bounds)")
    a = ap.parse_args()
    base = read_meta_base(a.session)
    arm = Arm()
    lines = ["# Torque fit (tools/fit_torque.py, issue #2) of %s" % os.path.basename(a.session.rstrip("/")),
             "# base_ticks = %s" % base, ""]
    print("%-4s %9s %9s %9s %11s %10s  %s" % ("", "gravity", "i+ A", "i- A", "Kt*eta*N", "friction", ""))
    for j in range(1, 8):
        path = os.path.join(a.session, "M%d.csv" % j)
        if not os.path.exists(path):
            continue
        rows = list(csv.DictReader(open(path)))
        up, down = hold_side(rows, "ramp_hold+"), hold_side(rows, "ramp_hold-")
        lines.append("[M%d]" % j)
        if not up or not down:
            print("M%-3d no ramp holds (run the ramp test)" % j)
            lines += ['note = "no ramp holds"', ""]
            continue
        taus = []
        for side in (up, down):
            t = list(base)
            t[j - 1] = side["position"]
            taus.append(abs(arm.gravity(t)[j - 1]))
        tau = float(np.mean(taus))
        im = (up["i_motor_a"] + down["i_motor_a"]) / 2
        ok = tau >= MIN_TAU and im > 0
        k = tau / im if ok else float("nan")
        fric = k * abs(up["i_motor_a"] - down["i_motor_a"]) / 2 if ok else float("nan")
        note = "" if ok else "gravity load %.2f N m < %.2f: use a loaded pose" % (tau, MIN_TAU)
        if ok and min(up["i_motor_a"], down["i_motor_a"]) < 0.01:
            # One side needs no current: friction >= gravity there, so Kt*eta*N is a lower bound, friction too.
            note = "one side holds with ~0 A (friction >= gravity): Kt*eta*N and friction are lower bounds"
        print("M%-3d %7.3f Nm %9.3f %9.3f %7.2f Nm/A %7.3f Nm  %s" % (j, tau, up["i_motor_a"], down["i_motor_a"], k, fric, note))
        lines += [
            "gravity_nm = %.4f" % tau,
            "i_motor_up_a = %.4f" % up["i_motor_a"],
            "i_motor_down_a = %.4f" % down["i_motor_a"],
            "duty_up = %.4f" % up["duty"],
            "duty_down = %.4f" % down["duty"],
            "kt_eta_n_nm_per_a = %s" % ("%.4f" % k if ok else "nan"),
            "static_friction_nm = %s" % ("%.4f" % fric if ok else "nan"),
        ]
        if note:
            lines.append('note = "%s"' % note)
        lines.append("")
        if a.write_params:
            if ok and "lower bound" not in note:
                vals = {"kt_eta_n": round(k, 3), "static_friction_nm": round(fric, 3)}
                params_file.update(j, vals, source="fit_torque.py %s" % os.path.basename(a.session.rstrip("/")))
                print("     written to servo_params.toml [servo.M%d]: %s" % (j, vals))
            else:
                print("     M%d not written (%s)" % (j, note or "no estimate"))
    out = a.out or os.path.join(a.session, "torque.toml")
    open(out, "w").write("\n".join(lines))
    print("written %s" % out)


if __name__ == "__main__":
    main()
