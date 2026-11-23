#!/usr/bin/env python3
"""Accuracy statistics of the MuJoCo servo model against a real run (issue #2), as system-ID practice reports them:

- in physical units: RMS, mean absolute, median absolute, 95th percentile, max, and bias (mean signed error,
  sim - real) of the joint position; ticks, degrees, and the resulting error at the fingertip (mm);
- normalised fit (MATLAB System Identification "compare"): fit % = 100 (1 - ||y - y_sim|| / ||y - mean(y)||);
- per test type (steps, reversals, ramp, chirp) as well as overall;
- step behaviour, real vs sim: dead time and t63 of each step;
- validation on HELD-OUT data when --fit-on names another run (parameters fitted there), else on the fitted run.

    python3 tools/replay_stats.py sysid_logs/<stamp>_<payload> --joint 7 [--fit-on <other session>] [--csv out.csv]

Resolution floor: 1 tick = 0.088 deg; the servos' dead zone is ~4 ticks.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from elrobot_model import Arm, read_meta_base  # noqa: E402
from make_replay_video import fit_params  # noqa: E402
from servo_sim import load_csv, replay  # noqa: E402

DEG = 360 / 4096
FINGER_TIP = np.array([0.0, 0.104, 0.0])  # in Gripper_Base_v1_1 (PingPongSimBot FINGER_TIP_Y)
CAMERA_LENS = np.array([0.0017, 0.0124, 0.0661])  # RoboSim innomaker_u20cam cameraPcbLensFaceCentreM


def tip_radius_m(session, joint):
    """Distance from the joint's axis to the fingertip at the run's base pose (m), and the point's name. For the
    roll joints the fingertip lies on the axis, so the camera lens (~66 mm off it) is used instead."""
    for name, point in (("fingertip", FINGER_TIP), ("camera lens", CAMERA_LENS)):
        r = _radius(session, joint, point)
        if r > 0.02:
            return r, name
    return r, name


def _radius(session, joint, point):
    arm = Arm()
    arm.set_ticks(read_meta_base(session))
    m, d = arm.m, arm.d
    gb = m.body("Gripper_Base_v1_1").id
    tip = d.xpos[gb] + d.xmat[gb].reshape(3, 3) @ point
    jid = m.joint("rev_motor_0%d" % joint).id
    r = tip - d.xanchor[jid]
    ax = d.xaxis[jid]
    return float(np.linalg.norm(r - ax * (r @ ax)))


def stats(err, real):
    a = np.abs(err)
    denom = np.linalg.norm(real - real.mean())
    return {
        "n": len(err),
        "rms": float(np.sqrt(np.mean(err ** 2))),
        "mae": float(a.mean()),
        "median": float(np.median(a)),
        "p95": float(np.percentile(a, 95)),
        "max": float(a.max()),
        "bias": float(err.mean()),
        "rms_unbiased": float(np.std(err)),
        # Undefined when the real joint barely moves (holds): the denominator is ~0 and the score explodes.
        "fit": float(100 * (1 - np.linalg.norm(err) / denom)) if np.ptp(real) >= 5 else float("nan"),
    }


def step_timing(t, goal, pos, i0, i1):
    """Dead time (first move >= 2 ticks) and t63 after a goal change at index i0 (segment up to i1), seconds."""
    start, target = pos[i0], goal[i0]
    span = target - start
    if abs(span) < 8:
        return float("nan"), float("nan")
    d = (pos[i0:i1] - start) * np.sign(span)
    tt = t[i0:i1] - t[i0]
    moved = np.where(d >= 2)[0]
    reach = np.where(d >= 0.63 * abs(span))[0]
    return (tt[moved[0]] if len(moved) else float("nan"), tt[reach[0]] if len(reach) else float("nan"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--joint", type=int, required=True)
    ap.add_argument("--fit-on")
    ap.add_argument("--csv")
    a = ap.parse_args()
    log = load_csv(a.session, a.joint)
    src = a.fit_on or a.session
    p = fit_params(src, a.joint, load_csv(src, a.joint) if a.fit_on else log)
    sim, _ = replay(a.session, a.joint, p, log)
    t, goal, real = log["t"], log["goal"], log["position"]
    seg = np.array(log["segment"])
    err = sim - real
    radius, point = tip_radius_m(a.session, a.joint)
    mm = radius * np.radians(DEG) * 1e3  # fingertip mm per tick
    kind = np.array([s.split("_")[0].rstrip("+-0123456789") if not s.startswith("step") else "steps" for s in seg])
    kind = np.where(np.char.startswith(seg.astype(str), "rev"), "reversals", kind)
    kind = np.where(np.char.startswith(seg.astype(str), "ramp"), "ramp", kind)
    kind = np.where(np.char.startswith(seg.astype(str), "chirp"), "chirp", kind)

    label = "HELD OUT (parameters from %s)" % os.path.basename(a.fit_on.rstrip("/")) if a.fit_on else "fitted on this run"
    print("M%d, %s, %s: kp %.4f, dead zone %.0f, friction %.2f; %s %.0f mm from the axis (1 tick = %.2f mm there)" % (
        a.joint, os.path.basename(a.session.rstrip("/")), label, p.kp_duty, p.dead_zone, p.static_friction, point,
        radius * 1e3, mm))
    rows = [("all", np.ones(len(err), bool))] + [(k, kind == k) for k in ("hold", "steps", "reversals", "ramp", "chirp") if (kind == k).any()]
    hdr = "| part | samples | RMS ticks | RMS deg | RMS %s mm | mean abs | median abs | 95 %% | max | bias (sim-real) | RMS without bias | fit %% |" % point
    print(hdr)
    print("|" + "---|" * (hdr.count("|") - 1))
    out = []
    for name, msk in rows:
        s = stats(err[msk], real[msk])
        out.append((name, s))
        print("| %s | %d | %.2f | %.2f | %.1f | %.2f | %.2f | %.1f | %.0f | %+.2f | %.2f | %.1f |" % (
            name, s["n"], s["rms"], s["rms"] * DEG, s["rms"] * mm, s["mae"], s["median"], s["p95"], s["max"], s["bias"],
            s["rms_unbiased"], s["fit"]))
    # step behaviour
    print("\n| step | dead real ms | dead sim ms | t63 real ms | t63 sim ms |")
    print("|---|---|---|---|---|")
    for name in dict.fromkeys(seg):
        if not str(name).startswith("step"):
            continue
        idx = np.where(seg == name)[0]
        i0, i1 = idx[0], idx[-1] + 1
        dr, tr = step_timing(t, goal, real, i0, i1)
        ds, ts = step_timing(t, goal, sim, i0, i1)
        if np.isnan(dr) and np.isnan(ds):
            continue
        print("| %s | %.0f | %.0f | %.0f | %.0f |" % (name, dr * 1e3, ds * 1e3, tr * 1e3, ts * 1e3))
    if a.csv:
        with open(a.csv, "w") as f:
            f.write("part,samples,rms_ticks,rms_deg,rms_tip_mm,mae,median,p95,max,bias,rms_unbiased,fit_pct\n")
            for name, s in out:
                f.write("%s,%d,%.3f,%.3f,%.2f,%.3f,%.3f,%.2f,%.1f,%.3f,%.3f,%.2f\n" % (
                    name, s["n"], s["rms"], s["rms"] * DEG, s["rms"] * mm, s["mae"], s["median"], s["p95"], s["max"], s["bias"],
                    s["rms_unbiased"], s["fit"]))


if __name__ == "__main__":
    main()
