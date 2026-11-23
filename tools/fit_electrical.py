#!/usr/bin/env python3
"""Fit the ST3215 electrical model per joint from an elrobot-sysid session (issue #2).

    V_m = s * duty * V_bus = R * i + (Ke*N) * omega

duty = Present Load / 1000, V_bus = voltage_dv / 10, omega = output speed in rad/s (4096 ticks per turn), i in A.
s = +1 or -1: on the real follower the load is reported with the opposite sign to the speed (s = -1); the fit finds s.

Two steps, because an unloaded joint draws almost no current while it moves (the drive voltage nearly cancels the
back-EMF; 5-9 mA mean on the real arm):
1. Start: V_m against omega on the samples where the joint moves (|omega| > 0.2 rad/s); gives s and a first Ke*N.
   On an unloaded joint this is already the answer; first real values 2.5-3.1 V*s/rad (datasheet ~2.5).
2. With enough current (>= 150 mA peak): |V_m - Ke*N*omega| = R*|i| (the ST3215 reports |i|), a 1-D search over Ke*N
   with R by least squares. With less, Ke*N stays the motion-only value and R is not identifiable. Flagged unreliable below 150 mA peak; the MuJoCo line then uses the datasheet
   R (4.4 ohm, 12 V / 2.7 A stall). Also tried for R: the servo reports the PWM-averaged supply current,
   i_meas = |duty| * i.

Temperature and voltage readings have short glitches (1-4 samples, e.g. 26 -> 99 degC during motion); they are
median-filtered (51 samples).

Usage:  python3 tools/fit_electrical.py sysid_logs/<stamp>_<payload> [--eta 1.0] [--out electrical.toml]

Writes <session>/electrical.toml (or --out) with Ke*N, R, the fit quality, the servo's protection settings and a
MuJoCo actuator. Kt*eta (torque per amp) is not measured yet: the actuator uses Kt = Ke and the --eta you give
(gearbox efficiency, placeholder 1.0).

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
MIN_PEAK_A = 0.15  # below this the current has too few levels to fit R
MIN_OMEGA = 0.2  # rad/s: 'moving' for the Ke*N fit
R_DATASHEET = 4.4  # ohm: 12 V / 2.7 A stall (ST3215 12 V datasheet)
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
    """|v - k*omega| = R*|i|: search k in [0, 10], R by least squares at each k. Returns (R, k, r2, rms)."""
    den = i_abs @ i_abs

    def at(k):
        u = np.abs(v - k * omega)
        r = (u @ i_abs) / den if den > 0 else float("nan")
        res = u - r * i_abs
        return res @ res, r, u
    ks = np.linspace(0.0, 10.0, 1001)
    k0 = ks[int(np.argmin([at(x)[0] for x in ks]))]
    fine = np.linspace(k0 - 0.02, k0 + 0.02, 81)
    k = fine[int(np.argmin([at(x)[0] for x in fine]))]
    sse, r, u = at(k)
    ss = ((u - u.mean()) ** 2).sum()
    return r, k, (1.0 - sse / ss if ss > 0 else float("nan")), float(np.sqrt(sse / max(len(u) - 2, 1)))


def fit_joint(d):
    v_raw = d["duty"] * d["vbus"]
    omega, i = d["omega"], d["i"]
    out = {"samples": int(len(v_raw))}
    peak = float(i.max())
    out["max_abs_current_a"] = peak
    out["temp_c"] = [float(d["temp"].min()), float(d["temp"].max())]
    out["temp_glitch_samples"] = int((np.abs(d["temp_raw"] - d["temp"]) >= 3).sum())
    out["v_bus"] = float(np.median(d["vbus"]))
    hold = d["test"] == "Hold"
    if hold.any():
        out["hold_current_a"] = float(np.mean(i[hold]))
        out["hold_duty"] = float(np.mean(d["duty"][hold]))
    # 1. Start from motion alone: V = k*omega + c on moving samples; the sign of k is the load/speed convention.
    mv = np.abs(omega) > MIN_OMEGA
    out["moving_samples"] = int(mv.sum())
    if mv.sum() < 200:
        out["note"] = "too little motion to fit Ke*N (%d moving samples)" % mv.sum()
        return out
    k, c = np.polyfit(omega[mv], v_raw[mv], 1)
    sgn = 1.0 if k >= 0 else -1.0
    v = sgn * v_raw
    k = abs(k)
    # 2. With enough current: |V - Ke*N*omega| = R*|i| (the ST3215 reports |i|), a 1-D search over Ke*N with R by
    #    least squares (exact on the simulator). With too little current R only absorbs noise and drags Ke*N, so
    #    Ke*N stays the motion-only value and R is not fitted.
    m = mv | (np.abs(d["duty"]) >= MIN_DUTY_B)
    out["current_levels"] = int(len(np.unique(np.round(i[m] / CURRENT_LSB_A))))
    enough = peak >= MIN_PEAK_A
    fits = {}
    for name, ii in (("motor_current", i[m]), ("supply_current", i[m] / np.maximum(np.abs(d["duty"][m]), MIN_DUTY_B))):
        if enough:
            r, kk, r2, rms = fit_unsigned(v[m], omega[m], ii)
        else:
            kk = k
            u = np.abs(v[m] - kk * omega[m])
            r = float((u @ ii) / (ii @ ii)) if ii @ ii > 0 else float("nan")
            e = u - r * ii
            ss = ((u - u.mean()) ** 2).sum()
            r2, rms = (float(1 - (e @ e) / ss) if ss > 0 else float("nan")), float(np.sqrt(np.mean(e ** 2)))
        fits[name] = {"ke_n": float(kk), "r_ohm": float(r), "r2": float(r2), "rms_v": float(rms)}
        out[name] = fits[name]
    best = max(fits, key=lambda n: fits[n]["r2"])
    out["ke_n"] = fits[best]["ke_n"]
    out["ke_n_source"] = "joint fit with current" if enough else "motion only (current ~0)"
    out["ke_n_corr"] = float(abs(np.corrcoef(omega[mv], v_raw[mv])[0, 1]))
    out["ke_n_rms_v"] = float(np.sqrt(np.mean((v[mv] - k * omega[mv] - sgn * c) ** 2)))
    out["offset_v"] = float(sgn * c)
    out["duty_sign"] = int(sgn)
    out["no_load_speed_rad_s"] = out["v_bus"] / out["ke_n"]
    out["preferred"] = best
    r = out[out["preferred"]]["r_ohm"]
    out["r_reliable"] = bool(peak >= MIN_PEAK_A and r > 0)
    if not out["r_reliable"]:
        out["note"] = ("R not identifiable: peak current %.0f mA (%d levels of 6.5 mA); needs load or acceleration. "
                       "MuJoCo line uses the datasheet R %.1f ohm" % (peak * 1e3, out["current_levels"], R_DATASHEET))
    out["r_ohm"] = r if out["r_reliable"] else R_DATASHEET
    out["stall_current_a"] = out["v_bus"] / out["r_ohm"]
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
             "# V_m = s*duty*V_bus = R*i + (Ke*N)*omega; omega in rad/s at the output", ""]
    csvs = sorted(a.session.glob("M*.csv"), key=lambda p: int(p.stem[1:]))
    if not csvs:
        sys.exit("no M<n>.csv in %s" % a.session)
    print("%-4s %7s %6s %7s %5s %8s %7s %7s %8s" % ("", "Ke*N", "corr", "moving", "sign", "R ohm", "R?", "peak A", "temp C"))
    for p in csvs:
        jid = int(p.stem[1:])
        f = fit_joint(read_csv(p))
        lines.append("[M%d]" % jid)
        if "note" in f:
            lines.append('note = "%s"' % f["note"])
        if "ke_n" not in f:
            print("M%-3d %s" % (jid, f["note"]))
            lines.append("")
            continue
        temp = "%.0f-%.0f" % tuple(f["temp_c"])
        rstr = "%.2f" % f[f["preferred"]]["r_ohm"] if f["r_reliable"] else "–"
        print("M%-3d %7.3f %6.3f %7d %+5d %8s %7s %7.3f %8s" % (
            jid, f["ke_n"], f["ke_n_corr"], f["moving_samples"], f["duty_sign"], rstr,
            "ok" if f["r_reliable"] else "NO", f["max_abs_current_a"], temp))
        for k in ("ke_n", "ke_n_source", "ke_n_corr", "ke_n_rms_v", "offset_v", "duty_sign", "moving_samples", "no_load_speed_rad_s",
                  "r_ohm", "r_reliable", "preferred", "v_bus", "stall_current_a", "hold_current_a", "hold_duty",
                  "max_abs_current_a", "current_levels", "temp_c", "temp_glitch_samples", "samples"):
            if k in f:
                val = f[k]
                lines.append("%s = %s" % (k, ("true" if val else "false") if isinstance(val, bool) else toml_value(val)))
        for model in ("motor_current", "supply_current"):
            lines.append("r_fit_%s = { %s }" % (model, ", ".join("%s = %s" % (k, toml_value(v)) for k, v in f[model].items())))
        prot = protection(meta.get(jid, {}))
        if prot:
            lines.append("protection = { %s }" % ", ".join("%s = %s" % (k, toml_value(v)) for k, v in prot.items()))
        # MuJoCo: ctrl = V_m; tau = (eta*Kt*N/R)*V - (eta*Kt*N*Ke*N/R)*qdot, with Kt = Ke (not measured yet).
        gain = a.eta * f["ke_n"] / f["r_ohm"]
        damp = a.eta * f["ke_n"] ** 2 / f["r_ohm"]
        lines.append('mjcf = \'<general joint="M%d" ctrlrange="%.1f %.1f" gaintype="fixed" gainprm="%.4g" '
                     'biastype="affine" biasprm="0 0 %.4g"/>\'  # eta %.2f, Kt = Ke assumed, R %s'
                     % (jid, -f["v_bus"], f["v_bus"], gain, -damp, a.eta, "fitted" if f["r_reliable"] else "datasheet"))
        lines.append("")
        if "note" in f:
            print("     " + f["note"])
    out = a.out or a.session / "electrical.toml"
    out.write_text("\n".join(lines))
    print("written %s" % out)


if __name__ == "__main__":
    main()
