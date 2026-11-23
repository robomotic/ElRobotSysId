#!/usr/bin/env python3
"""Fit the ST3215 electrical model per joint from an elrobot-sysid session (issue #2).

    V_m = s * duty * V_bus = R * i + (Ke*N) * omega

duty = Present Load / 1000, V_bus = voltage_dv / 10, omega = output speed in rad/s (4096 ticks per turn), i in A.
s = +1 or -1: on the real follower the load is reported with the opposite sign to the speed (s = -1); the fit finds s.

The ST3215 reports the SUPPLY current, i = |duty| * i_motor, as a magnitude (found 2026-11-23: at standstill under
load R comes out 3.9-4.2 ohm this way, 25-36 ohm if read as motor current). So:
1. R at standstill under load (no back-EMF): |V_m| = R * i / |duty|. Runs without a loaded standstill use 4.1 ohm.
2. Ke*N with R fixed: |V_m - Ke*N*omega| = R * i / |duty| on moving samples, a 1-D search; the absolute values make the
   unknown current sign (lifting vs lowering) irrelevant. Real arm: 1.6-2.0 V*s/rad on every joint, loaded or not.
Torque per amp (Kt*eta*N ~ 1.6-1.8 N m/A) and the gearbox's static friction (0.25-0.41 N m) came from the two-sided
holds of the loaded run with MuJoCo gravity torques (not in this script yet).

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
R_DEFAULT = 4.1  # ohm: measured at standstill on M2-M4, 2026-11-23 (3.9-4.2; datasheet 12 V / 2.7 A = 4.4)
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


def fit_ke_n(v, omega, i_motor, r):
    """|v - k*omega| = R*|i_motor| with R known: 1-D search over k in [0, 8] V*s/rad. The absolute values make the
    unknown current sign (lifting vs lowering, motoring vs braking) irrelevant. Returns (k, rms)."""
    ks = np.linspace(0.0, 8.0, 1601)
    cost = [np.sum((np.abs(v - k * omega) - r * i_motor) ** 2) for k in ks]
    k = ks[int(np.argmin(cost))]
    return float(k), float(np.sqrt(min(cost) / max(len(v), 1)))


def fit_joint(d):
    v_raw = d["duty"] * d["vbus"]
    omega, i = d["omega"], d["i"]
    duty = np.abs(d["duty"])
    out = {"samples": int(len(v_raw))}
    out["max_reported_current_a"] = float(i.max())
    out["temp_c"] = [float(d["temp"].min()), float(d["temp"].max())]
    out["temp_glitch_samples"] = int((np.abs(d["temp_raw"] - d["temp"]) >= 3).sum())
    out["v_bus"] = float(np.median(d["vbus"]))
    hold = d["test"] == "Hold"
    if hold.any():
        out["hold_current_a"] = float(np.mean(i[hold]))
        out["hold_duty"] = float(np.mean(d["duty"][hold]))
    # Sign convention: V against omega while moving (on the follower the load has the opposite sign to the speed).
    mv = np.abs(omega) > MIN_OMEGA
    out["moving_samples"] = int(mv.sum())
    if mv.sum() < 200:
        out["note"] = "too little motion to fit Ke*N (%d moving samples)" % mv.sum()
        return out
    k_motion, c = np.polyfit(omega[mv], v_raw[mv], 1)
    sgn = 1.0 if k_motion >= 0 else -1.0
    v = sgn * v_raw
    out["duty_sign"] = int(sgn)
    out["ke_n_corr"] = float(abs(np.corrcoef(omega[mv], v_raw[mv])[0, 1]))
    # The ST3215 reports the SUPPLY current: i = |duty| * i_motor.
    # 1. R at standstill under load: no back-EMF, |V| = R * i_motor.
    st = (omega == 0) & (duty >= 0.10) & (i > 0)
    out["standstill_samples"] = int(st.sum())
    if st.sum() >= 2000:
        rs = np.abs(v[st]) * duty[st] / i[st]
        out["r_ohm"] = float(np.median(rs))
        out["r_iqr"] = [float(x) for x in np.percentile(rs, [25, 75])]
        out["r_source"] = "standstill under load"
    else:
        out["r_ohm"] = R_DEFAULT
        out["r_source"] = "default (no loaded standstill in this run)"
    # 2. Ke*N with R fixed, on moving samples with some duty (i_motor = i / |duty| needs |duty| > 0).
    m = mv & (duty >= MIN_DUTY_B)
    if m.sum() < 200:
        out["note"] = "too few moving samples with duty >= %.2f to fit Ke*N" % MIN_DUTY_B
        return out
    out["ke_n"], out["ke_n_rms_v"] = fit_ke_n(v[m], omega[m], i[m] / duty[m], out["r_ohm"])
    out["ke_n_samples"] = int(m.sum())
    out["no_load_speed_rad_s"] = out["v_bus"] / out["ke_n"] if out["ke_n"] > 0 else float("nan")
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
    print("%-4s %7s %6s %7s %5s %7s  %-28s %8s" % ("", "Ke*N", "corr", "samples", "sign", "R ohm", "R from", "temp C"))
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
        print("M%-3d %7.3f %6.3f %7d %+5d %7.2f  %-28s %8s" % (
            jid, f["ke_n"], f["ke_n_corr"], f["ke_n_samples"], f["duty_sign"], f["r_ohm"], f["r_source"][:28], temp))
        for k in ("ke_n", "ke_n_rms_v", "ke_n_samples", "ke_n_corr", "duty_sign", "moving_samples", "no_load_speed_rad_s",
                  "r_ohm", "r_iqr", "r_source", "standstill_samples", "v_bus", "stall_current_a", "hold_current_a", "hold_duty",
                  "max_reported_current_a", "temp_c", "temp_glitch_samples", "samples"):
            if k in f:
                lines.append("%s = %s" % (k, toml_value(f[k])))
        prot = protection(meta.get(jid, {}))
        if prot:
            lines.append("protection = { %s }" % ", ".join("%s = %s" % (k, toml_value(v)) for k, v in prot.items()))
        # MuJoCo: ctrl = V_m; tau = (eta*Kt*N/R)*V - (eta*Kt*N*Ke*N/R)*qdot, with Kt = Ke (not measured yet).
        gain = a.eta * f["ke_n"] / f["r_ohm"]
        damp = a.eta * f["ke_n"] ** 2 / f["r_ohm"]
        lines.append('mjcf = \'<general joint="M%d" ctrlrange="%.1f %.1f" gaintype="fixed" gainprm="%.4g" '
                     'biastype="affine" biasprm="0 0 %.4g"/>\'  # eta %.2f, Kt = Ke assumed'
                     % (jid, -f["v_bus"], f["v_bus"], gain, -damp, a.eta))
        lines.append("")
    out = a.out or a.session / "electrical.toml"
    out.write_text("\n".join(lines))
    print("written %s" % out)


if __name__ == "__main__":
    main()
