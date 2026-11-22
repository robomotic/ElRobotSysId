#!/usr/bin/env python3
"""Fit the ST3215 electrical model per joint from an elrobot-sysid session (issue #2).

    V_m = duty * V_bus = R * i + (Ke*N) * omega

duty = Present Load / 1000, V_bus = voltage_dv / 10, omega = output speed in rad/s (4096 ticks per turn), i in A.
The ST3215 reports the current as a magnitude only (never negative on the real arm), so the fit is
|V_m - Ke*N*omega| = R*|i|: a 1-D search over Ke*N, with R by least squares at each value.

Also tried: the servo reports the PWM-averaged supply current, i_meas = |duty| * i. Whichever model explains V_m
better is reported as the preferred one. Temperature and voltage readings have short glitches (pairs of samples, e.g.
26 -> 99 degC during motion); they are median-filtered (51 samples).

Needs current: the unloaded wrist draws < 60 mA (< 10 steps of 6.5 mA), too little to identify R and Ke*N. Loaded
joints (M2-M4 holding against gravity) are needed; the fit is flagged when the current is too small.

Usage:  python3 tools/fit_electrical.py sysid_logs/<stamp>_<payload> [--eta 1.0] [--out electrical.toml]

Writes <session>/electrical.toml (or --out) with R, Ke*N, the fit quality, the servo's protection settings and a
MuJoCo actuator. Kt*eta (torque per amp) still needs the gravity torques at the hold poses from MuJoCo; until then
the actuator uses Kt = Ke and the --eta you give (gearbox efficiency, placeholder 1.0).

Needs numpy only (Python 3.8+).
"""
import argparse
import csv
import math
import re
import sys
from pathlib import Path

import numpy as np

TICKS_PER_REV = 4096.0
MIN_DUTY_B = 0.05  # model B divides by |duty|; skip samples below this
CURRENT_LSB_A = 0.0065
MIN_PEAK_A = 0.15  # below this the current has too few levels to fit
MEDIAN_WINDOW = 51  # samples (~25 ms at 2 kHz); glitches last 1-4 samples and come in clusters


def read_meta(path):
    """Integer keys of each [servo.Mn] section of meta.toml (a tiny parser: Python 3.8 has no tomllib)."""
    servos, cur = {}, None
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        m = re.fullmatch(r"\[servo\.M(\d+)\]", line)
        if m:
            cur = servos.setdefault(int(m.group(1)), {})
            continue
        if line.startswith("["):
            cur = None
            continue
        m = re.fullmatch(r"(\w+)\s*=\s*(0x[0-9a-fA-F]+|-?\d+)", line)
        if cur is not None and m:
            cur[m.group(1)] = int(m.group(2), 0)
    return servos


def read_csv(path):
    with path.open() as f:
        rows = list(csv.DictReader(f))
    col = lambda k: np.array([float(r[k]) for r in rows])
    return {
        "t": col("t"),
        "test": np.array([r["test"] for r in rows]),
        "duty": col("load") / 1000.0,
        "vbus": median_filter(col("voltage_dv")) / 10.0,
        "omega": col("speed") * 2.0 * math.pi / TICKS_PER_REV,
        "i": np.abs(col("current_ma")) / 1000.0,
        "temp": median_filter(col("temp_c")),
        "temp_raw": col("temp_c"),
    }


def median_filter(x, w=MEDIAN_WINDOW):
    if len(x) < w:
        return x
    pad = np.pad(x, (w // 2, w // 2), mode="edge")
    return np.median(np.lib.stride_tricks.sliding_window_view(pad, w), axis=1)


def fit_unsigned(v, omega, i_abs):
    """|v - k*omega| = R*|i|: search k, R by least squares at each k. Returns (R, k, r2, rms)."""
    def at(k):
        u = np.abs(v - k * omega)
        den = i_abs @ i_abs
        r = (u @ i_abs) / den if den > 0 else float("nan")
        res = u - r * i_abs
        return res @ res, r, u
    ks = np.linspace(-2.0, 10.0, 1201)
    costs = [at(k)[0] for k in ks]
    k0 = ks[int(np.argmin(costs))]
    fine = np.linspace(k0 - 0.02, k0 + 0.02, 81)
    k = fine[int(np.argmin([at(x)[0] for x in fine]))]
    sse, r, u = at(k)
    ss = ((u - u.mean()) ** 2).sum()
    return r, k, (1.0 - sse / ss if ss > 0 else float("nan")), float(np.sqrt(sse / max(len(u) - 2, 1)))


def fit_joint(d):
    v = d["duty"] * d["vbus"]
    # Same sample set for both models, so their R^2 compare fairly.
    m = np.abs(d["duty"]) >= MIN_DUTY_B
    out = {"samples": int(len(v)), "samples_fit": int(m.sum())}
    peak = float(d["i"].max())
    out["max_abs_current_a"] = peak
    out["current_levels"] = int(len(np.unique(np.round(d["i"][m] / CURRENT_LSB_A))))
    out["temp_c"] = [float(d["temp"].min()), float(d["temp"].max())]
    out["temp_glitch_samples"] = int((np.abs(d["temp_raw"] - d["temp"]) >= 3).sum())
    hold = d["test"] == "Hold"
    if hold.any():
        out["hold_current_a"] = float(np.mean(d["i"][hold]))
        out["hold_duty"] = float(np.mean(d["duty"][hold]))
    if m.sum() < 20 or np.ptp(d["omega"][m]) == 0:
        out["note"] = "not enough motion or duty to fit"
        return out
    if peak < MIN_PEAK_A:
        out["note"] = ("too little current to identify R and Ke*N (peak %.0f mA, %d levels of 6.5 mA); "
                       "needs a loaded joint" % (peak * 1e3, out["current_levels"]))
    for name, i in (("motor_current", d["i"][m]), ("supply_current", d["i"][m] / np.abs(d["duty"][m]))):
        r, ken, r2, rms = fit_unsigned(v[m], d["omega"][m], i)
        out[name] = {"r_ohm": r, "ke_n": ken, "r2": r2, "rms_v": rms}
    out["preferred"] = max(("motor_current", "supply_current"), key=lambda k: out[k]["r2"])
    p = out[out["preferred"]]
    vb = float(np.median(d["vbus"]))
    out["v_bus"] = vb
    out["stall_current_a"] = vb / p["r_ohm"] if p["r_ohm"] > 0 else float("nan")
    out["no_load_speed_rad_s"] = vb / p["ke_n"] if p["ke_n"] > 0 else float("nan")
    return out


def protection(meta):
    """The servo's own limits, in SI units where the unit is known (Feetech STS memory table)."""
    keys = {
        "max_temperature_c": 1.0, "max_voltage_dv": 0.1, "min_voltage_dv": 0.1, "max_torque": 0.1,
        "protection_current": 6.5, "protective_torque": 1.0, "protection_time": 10.0, "overload_torque": 1.0,
        "overcurrent_protection_time": 10.0, "unloading_conditions": 1.0,
    }
    names = {
        "max_voltage_dv": "max_voltage_v", "min_voltage_dv": "min_voltage_v", "max_torque": "max_duty_pct",
        "protection_current": "protection_current_ma", "protective_torque": "protective_torque_pct",
        "protection_time": "protection_time_ms", "overload_torque": "overload_torque_pct",
        "overcurrent_protection_time": "overcurrent_protection_time_ms",
    }
    return {names.get(k, k): meta[k] * f for k, f in keys.items() if k in meta}


def toml_value(x):
    if isinstance(x, str):
        return '"%s"' % x
    if isinstance(x, (list, tuple)):
        return "[" + ", ".join(toml_value(v) for v in x) + "]"
    if isinstance(x, (int, np.integer)):
        return str(int(x))
    return "nan" if not math.isfinite(x) else "%.6g" % x


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path, help="sysid_logs/<stamp>_<payload> folder (meta.toml + M<n>.csv)")
    ap.add_argument("--eta", type=float, default=1.0, help="gearbox efficiency for the MuJoCo gains (placeholder)")
    ap.add_argument("--out", type=Path, help="output TOML (default <session>/electrical.toml)")
    a = ap.parse_args()

    meta_path = a.session / "meta.toml"
    meta = read_meta(meta_path) if meta_path.exists() else {}
    if meta_path.exists() and "SIMULATED" in meta_path.read_text().splitlines()[0]:
        print("(simulated servos: expect R 4.4 ohm, Ke*N 2.5 V*s/rad)")
    lines = ["# Electrical fit (tools/fit_electrical.py, issue #2) of %s" % a.session.name,
             "# V_m = duty*V_bus = R*i + (Ke*N)*omega + offset; omega in rad/s at the output", ""]
    csvs = sorted(a.session.glob("M*.csv"), key=lambda p: int(p.stem[1:]))
    if not csvs:
        sys.exit("no M<n>.csv in %s" % a.session)
    print("%-4s %8s %9s %7s %8s %9s %7s %9s  %s" % ("", "R ohm", "Ke*N", "R^2", "rms V", "stall A", "peak A", "temp C", "model"))
    for p in csvs:
        jid = int(p.stem[1:])
        f = fit_joint(read_csv(p))
        lines.append("[M%d]" % jid)
        if "note" in f:
            lines.append('note = "%s"' % f["note"])
        if "preferred" not in f:
            print("M%-3d %s" % (jid, f["note"]))
            lines.append("")
            continue
        b = f[f["preferred"]]
        other = f["supply_current" if f["preferred"] == "motor_current" else "motor_current"]
        temp = "%.0f-%.0f" % tuple(f["temp_c"])
        print("M%-3d %8.3f %9.3f %7.4f %8.4f %9.2f %7.3f %9s  %s (other model R^2 %.4f)%s" % (
            jid, b["r_ohm"], b["ke_n"], b["r2"], b["rms_v"], f["stall_current_a"], f["max_abs_current_a"], temp,
            f["preferred"], other["r2"], "\n     UNRELIABLE: " + f["note"] if "note" in f else ""))
        for k in ("preferred", "samples", "samples_fit", "v_bus", "stall_current_a", "no_load_speed_rad_s",
                  "hold_current_a", "hold_duty", "max_abs_current_a", "current_levels", "temp_c", "temp_glitch_samples"):
            if k in f:
                lines.append("%s = %s" % (k, toml_value(f[k])))
        for model in ("motor_current", "supply_current"):
            lines.append("%s = { %s }" % (model, ", ".join("%s = %s" % (k, toml_value(v)) for k, v in f[model].items())))
        prot = protection(meta.get(jid, {}))
        if prot:
            lines.append("protection = { %s }" % ", ".join("%s = %s" % (k, toml_value(v)) for k, v in prot.items()))
        # MuJoCo: ctrl = V_m; tau = (eta*Kt*N/R)*V - (eta*Kt*N*Ke*N/R)*qdot, with Kt = Ke until the gravity fit.
        gain = a.eta * b["ke_n"] / b["r_ohm"]
        damp = a.eta * b["ke_n"] ** 2 / b["r_ohm"]
        lines.append('mjcf = \'<general joint="M%d" ctrlrange="%.1f %.1f" gaintype="fixed" gainprm="%.4g" '
                     'biastype="affine" biasprm="0 0 %.4g"/>\'  # eta %.2f, Kt = Ke (assumed)'
                     % (jid, -f["v_bus"], f["v_bus"], gain, -damp, a.eta))
        lines.append("")
    out = a.out or a.session / "electrical.toml"
    out.write_text("\n".join(lines))
    print("written %s" % out)


if __name__ == "__main__":
    main()
