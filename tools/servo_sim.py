#!/usr/bin/env python3
"""ST3215 servos on the ElRobot in MuJoCo: position loop, DC motor, gearbox friction, overload protection, heating
(issue #2). Every joint M1..M7 is one servo; MuJoCo gets the motor torques through qfrc_applied.

Per servo, every physics step (1 ms):
    goal      arrives `delay_s` late (transport + firmware; ~80 ms measured)
    duty      = clamp(kp_duty * (goal - pos) [ticks], +-duty_cap), 0 inside the dead zone
    V         = duty * V_bus
    i_motor   = (V - Ke*N * omega) / R(T)                  R(T) = R_25 * (1 + 0.0039 (T_w - 25))
    tau       = Kt*eta*N * i_motor   (+ MuJoCo: frictionloss = gearbox static friction, damping, armature)
    reported  current = |duty| * |i_motor|  (the ST3215 reports the supply current), load = duty * 1000
    protection: |i_motor| > protection current, or |duty| > overload, for longer than protection time
                -> duty capped at the protective torque until the cause has been gone for the same time
    heating   C_w dT_w/dt = i^2 R(T) - (T_w - T_c)/R_wc ;  C_c dT_c/dt = (T_w - T_c)/R_wc - (T_c - T_amb)/R_ca
              the servo's temperature reading is T_c (case/board).

Parameters measured on the real arm (2026-11-21..23, issue #2): R 4.0-4.2 ohm, Ke*N 1.5-2.0 V s/rad, Kt*eta*N
1.6-1.8 N m/A, static friction 0.25-0.41 N m (M2-M4), duty cap 50 %, delay ~80 ms, protection 3.25 A / 25 % for 2 s
-> 20 %. kp_duty is fitted from the log being replayed. NOT measured yet: the thermal constants (placeholders, no
thermal test so far), armature and viscous damping (chosen by the replay fit).

Usage:
    python3 tools/servo_sim.py replay sysid_logs/<stamp>_<payload> --joint 3 [--plot out.png]
        Replays the logged goals of one joint (all others hold at base) and compares position and current.
    python3 tools/servo_sim.py heat --joint 3 --pose 2001,1750,2100,2450,1881,1869,2244 --minutes 10
        Holds a pose against gravity with the measured parameters: current, protection and temperature over time.

Needs mujoco and numpy (laptop); matplotlib only for --plot.
"""
import argparse
import collections
import csv
import os
import sys
from dataclasses import dataclass, field

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params_file  # noqa: E402
from elrobot_model import RAD_PER_TICK, Arm, q_to_ticks, read_meta_base  # noqa: E402

DT = 0.001


@dataclass
class ServoParams:
    r_ohm: float = 4.1
    ke_n: float = 1.8  # V s/rad at the output
    kt_eta_n: float = 1.7  # N m/A at the output
    v_bus: float = 12.1
    kp_duty: float = 0.0042  # duty per tick of error (0.42 %/tick; firmware P 16)
    dead_zone: float = 4.0  # ticks of |error| with no drive
    duty_cap: float = 0.5  # Max Torque 500
    delay_s: float = 0.08
    static_friction: float = 0.3  # N m (MuJoCo frictionloss)
    damping: float = 0.05  # N m s/rad (viscous, beyond back-EMF)
    armature: float = 0.01  # kg m^2 (rotor inertia * N^2)
    # protection (meta.toml: protection_current 500 = 3.25 A, overload 25 %, protection time 2 s, protective 20 %)
    i_prot: float = 3.25
    overload: float = 0.25
    prot_time: float = 2.0
    protective: float = 0.20
    # thermal: PLACEHOLDERS until a thermal test (issue #2)
    c_w: float = 3.0  # J/K winding
    c_c: float = 40.0  # J/K case
    r_wc: float = 3.0  # K/W winding -> case
    r_ca: float = 15.0  # K/W case -> ambient
    t_amb: float = 25.0
    t_max: float = 70.0  # max_temperature_c
    copper_coef: float = 0.0039  # 1/K

    @classmethod
    def from_file(cls, joint, path=None, **overrides):
        """[default] + [servo.M<joint>] of servo_params.toml (issue #5); missing keys keep the defaults above, then
        `overrides` (None values are ignored)."""
        vals = params_file.joint_params(joint, path or params_file.PARAMS)
        kw = {attr: float(vals[key]) for key, attr in FILE_KEYS.items() if key in vals}
        kw.update({k: v for k, v in overrides.items() if v is not None})
        return cls(**kw)

    def to_file_values(self, attrs):
        inv = {v: k for k, v in FILE_KEYS.items()}
        return {inv[a]: getattr(self, a) for a in attrs}


# servo_params.toml key -> ServoParams attribute (the same keys as unity/St3215Servo.cs St3215Params.Load).
FILE_KEYS = {
    "resistance_ohm": "r_ohm", "ke_n": "ke_n", "kt_eta_n": "kt_eta_n", "bus_voltage": "v_bus",
    "copper_temp_coef": "copper_coef", "duty_per_tick": "kp_duty", "dead_zone_ticks": "dead_zone",
    "duty_cap": "duty_cap", "delay_s": "delay_s", "protection_current_a": "i_prot", "overload_duty": "overload",
    "protection_time_s": "prot_time", "protective_duty": "protective", "static_friction_nm": "static_friction",
    "damping_nm_s_per_rad": "damping", "armature_kg_m2": "armature", "winding_heat_capacity": "c_w",
    "case_heat_capacity": "c_c", "winding_to_case_k_per_w": "r_wc", "case_to_ambient_k_per_w": "r_ca",
    "ambient_c": "t_amb", "max_temperature_c": "t_max",
}


@dataclass
class ServoState:
    goal_q: collections.deque = field(default_factory=collections.deque)
    goal: float = 0.0
    duty: float = 0.0
    i_motor: float = 0.0
    over_t: float = 0.0
    ok_t: float = 0.0
    tripped: bool = False
    t_w: float = 25.0
    t_c: float = 25.0


class ArmSim:
    def __init__(self, params, start_ticks):
        self.arm = Arm()
        self.m, self.d = self.arm.m, self.arm.d
        self.m.opt.timestep = DT
        self.m.opt.noslip_iterations = 10
        self.p = params if isinstance(params, list) else [params] * 7
        for j, p in enumerate(self.p):
            dof = self.arm.dadr[j]
            self.m.dof_frictionloss[dof] = p.static_friction
            self.m.dof_damping[dof] = p.damping
            self.m.dof_armature[dof] = p.armature
            # MuJoCo's frictionloss is soft by default: a sustained load creeps through it (the arm sagged 20+
            # ticks/s under 0.48 N m against 0.8 N m of friction). Stiff constraint + noslip = gearbox stiction.
            self.m.dof_solref[dof] = (0.002, 1.0)
            self.m.dof_solimp[dof] = (0.99, 0.999, 0.001, 0.5, 2.0)
        self.arm.set_ticks(start_ticks)
        self.s = [ServoState(goal=float(start_ticks[j]), t_w=self.p[j].t_amb, t_c=self.p[j].t_amb) for j in range(7)]
        self.t = 0.0

    def ticks(self):
        return q_to_ticks(self.d.qpos[self.arm.qadr])

    def command(self, j, goal_ticks):
        self.s[j].goal_q.append((self.t + self.p[j].delay_s, float(goal_ticks)))

    def step(self):
        pos = self.ticks()
        omega = self.d.qvel[self.arm.dadr]
        tau = np.zeros(7)
        for j in range(7):
            p, s = self.p[j], self.s[j]
            while s.goal_q and s.goal_q[0][0] <= self.t:
                s.goal = s.goal_q.popleft()[1]
            err = s.goal - pos[j]
            cap = p.protective if s.tripped else p.duty_cap
            s.duty = 0.0 if abs(err) <= p.dead_zone else float(np.clip(p.kp_duty * err, -cap, cap))
            r = p.r_ohm * (1 + p.copper_coef * (s.t_w - 25.0))
            s.i_motor = (s.duty * p.v_bus - p.ke_n * omega[j]) / r
            tau[j] = p.kt_eta_n * s.i_motor
            # protection
            over = abs(s.i_motor) > p.i_prot or abs(s.duty) > p.overload
            if over:
                s.over_t += DT
                s.ok_t = 0.0
                if s.over_t > p.prot_time:
                    s.tripped = True
            else:
                s.over_t = 0.0
                s.ok_t += DT
                if s.tripped and s.ok_t > p.prot_time:
                    s.tripped = False
            # heating
            pw = s.i_motor ** 2 * r
            q_wc = (s.t_w - s.t_c) / p.r_wc
            s.t_w += DT * (pw - q_wc) / p.c_w
            s.t_c += DT * (q_wc - (s.t_c - p.t_amb) / p.r_ca) / p.c_c
        self.d.qfrc_applied[:] = 0
        self.d.qfrc_applied[self.arm.dadr] = tau
        mujoco.mj_step(self.m, self.d)
        self.t += DT

    def reading(self, j):
        """What the servo would report: position, speed (ticks/s), load (0.1 %), supply current (mA), temperature."""
        s = self.s[j]
        return {
            "position": self.ticks()[j],
            "speed": self.d.qvel[self.arm.dadr[j]] / RAD_PER_TICK,
            "load": s.duty * 1000,
            "current_ma": abs(s.duty) * abs(s.i_motor) * 1000,
            "i_motor_a": s.i_motor,
            "temp_c": s.t_c,
            "winding_c": s.t_w,
            "tripped": s.tripped,
        }


def load_csv(session, joint):
    rows = list(csv.DictReader(open(os.path.join(session, "M%d.csv" % joint))))
    f = lambda k: np.array([float(r[k]) for r in rows])  # noqa: E731
    return {"t": f("t"), "goal": f("goal"), "position": f("position"), "load": f("load"), "current_ma": f("current_ma"),
            "segment": [r["segment"] for r in rows]}


def fit_kp(log):
    """Firmware P as duty per tick of error, from the log (|load| against |goal - position| while the error is big)."""
    e = log["goal"] - log["position"]
    m = np.abs(e) > 5
    if m.sum() < 50:
        return ServoParams.kp_duty
    return float(np.sum(np.abs(log["load"][m]) / 1000 * np.abs(e[m])) / np.sum(e[m] ** 2))


def replay(session, joint, params, log=None):
    log = log or load_csv(session, joint)
    base = read_meta_base(session)
    j = joint - 1
    # Initial condition from the data: the tested joint starts where the real one was (friction holds it a few ticks
    # off its commanded base; starting at the base gave M3 a constant +3.9-tick bias), the servo's goal is the base.
    start = list(base)
    start[j] = int(round(log["position"][0]))
    # The tested joint gets `params`; the others hold with their own values from servo_params.toml.
    sim = ArmSim([params if k == j else ServoParams.from_file(k + 1) for k in range(7)], start)
    sim.s[j].goal = float(base[j])
    t_log, goal = log["t"], log["goal"]
    out = np.zeros(len(t_log))
    cur = np.zeros(len(t_log))
    gi = 0
    k = 0
    t_end = t_log[-1]
    while sim.t <= t_end and k < len(t_log):
        while gi < len(t_log) and t_log[gi] <= sim.t:
            if gi == 0 or goal[gi] != goal[gi - 1]:
                sim.command(j, goal[gi])
            gi += 1
        sim.step()
        while k < len(t_log) and t_log[k] <= sim.t:
            r = sim.reading(j)
            out[k], cur[k] = r["position"], r["current_ma"]
            k += 1
    return out, cur


def cmd_replay(a):
    log = load_csv(a.session, a.joint)
    kp = fit_kp(log)
    best = None
    # The stick-slip staircase of the real joints is set by the dead zone and the static friction; armature and
    # damping matter much less. --fit searches the first two.
    grid = ([(dz, fr) for dz in (4.0, 8.0, 12.0, 16.0) for fr in (0.02, 0.05, 0.15, 0.3, 0.5)] if a.fit else [(a.dead_zone, a.friction)])
    for dz, fr in grid:
        p = ServoParams.from_file(a.joint, kp_duty=kp, armature=a.armature, damping=a.damping, static_friction=fr,
                                  dead_zone=dz, delay_s=a.delay, ke_n=a.ke_n, kt_eta_n=a.kt, r_ohm=a.r)
        pos, cur = replay(a.session, a.joint, p, log)
        rms = float(np.sqrt(np.mean((pos - log["position"]) ** 2)))
        print("  dead zone %4.1f ticks  friction %.2f N m  ->  position RMS %.2f ticks" % (dz, fr, rms))
        if best is None or rms < best[0]:
            best = (rms, p, pos, cur)
    rms, p, pos, cur = best
    print("M%d: kp_duty %.4f/tick (from the log), friction %.2f N m, delay %.0f ms, dead zone %.0f ticks" % (
        a.joint, p.kp_duty, p.static_friction, p.delay_s * 1e3, p.dead_zone))
    print("M%d: best dead zone %.1f ticks, friction %.2f N m: position RMS %.2f ticks (measured range %d ticks); "
          "reported current mean sim %.0f / real %.0f mA" % (a.joint, p.dead_zone, p.static_friction, rms,
                                                              np.ptp(log["position"]), cur.mean(), log["current_ma"].mean()))
    if a.write_params:
        if not a.fit:
            print("--write-params needs --fit: nothing written")
        else:
            vals = p.to_file_values(["kp_duty", "dead_zone", "static_friction"])
            params_file.update(a.joint, vals, source="servo_sim.py replay --fit %s" % os.path.basename(a.session.rstrip("/")))
            print("written to %s [servo.M%d]: %s" % (params_file.PARAMS, a.joint, vals))
    seg = np.array(log["segment"])
    for name in ("step+50", "step-50", "ramp_across", "chirp"):
        m = seg == name
        if m.any():
            print("   %-12s RMS %5.2f ticks" % (name, np.sqrt(np.mean((pos[m] - log["position"][m]) ** 2))))
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
        ax[0].plot(log["t"], log["goal"], lw=0.8, label="goal")
        ax[0].plot(log["t"], log["position"], lw=0.8, label="real")
        ax[0].plot(log["t"], pos, lw=0.8, label="MuJoCo")
        ax[0].set_ylabel("ticks")
        ax[0].legend()
        ax[1].plot(log["t"], log["current_ma"], lw=0.5, label="real (supply)")
        ax[1].plot(log["t"], cur, lw=0.5, label="MuJoCo (supply)")
        ax[1].set_ylabel("mA")
        ax[1].set_xlabel("s")
        ax[1].legend()
        fig.suptitle("M%d replay of %s" % (a.joint, os.path.basename(a.session.rstrip("/"))))
        fig.tight_layout()
        fig.savefig(a.plot, dpi=110)
        print("plot: %s" % a.plot)


def cmd_heat(a):
    pose = [int(x) for x in a.pose.split(",")]
    p = ServoParams.from_file(a.joint, ke_n=a.ke_n, kt_eta_n=a.kt, r_ohm=a.r, static_friction=a.friction)
    sim = ArmSim(p, pose)
    j = a.joint - 1
    # Approach the pose from below so the motor (not friction) carries the load: command it 30 ticks lower, then up.
    sim.command(j, pose[j] - 30)
    for _ in range(int(1.0 / DT)):
        sim.step()
    sim.command(j, pose[j])
    report = 0.0
    print("   t      pos  load  i_motor  reported  winding  case  protection")
    while sim.t < a.minutes * 60:
        sim.step()
        if sim.t >= report:
            r = sim.reading(j)
            print("%5.0fs %7.0f %5.0f %7.3fA %7.0fmA %7.1fC %5.1fC  %s" % (
                sim.t, r["position"], r["load"], r["i_motor_a"], r["current_ma"], r["winding_c"], r["temp_c"],
                "TRIPPED" if r["tripped"] else "-"))
            report += a.every
    print("(thermal constants are placeholders until a thermal test, issue #2)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("replay", "heat"):
        s = sub.add_parser(name)
        s.add_argument("--joint", type=int, required=True)
        # Unset options come from servo_params.toml ([default] + [servo.M<joint>]).
        s.add_argument("--ke-n", type=float)
        s.add_argument("--kt", type=float, help="Kt*eta*N, N m/A")
        s.add_argument("--r", type=float)
        s.add_argument("--friction", type=float, help="static friction, N m")
    r = sub.choices["replay"]
    r.add_argument("session")
    r.add_argument("--delay", type=float)
    r.add_argument("--dead-zone", type=float)
    r.add_argument("--armature", type=float)
    r.add_argument("--damping", type=float)
    r.add_argument("--fit", action="store_true", help="small grid over dead zone and static friction")
    r.add_argument("--write-params", action="store_true",
                   help="with --fit: write duty_per_tick, dead_zone_ticks, static_friction_nm to servo_params.toml")
    r.add_argument("--plot")
    h = sub.choices["heat"]
    h.add_argument("--pose", required=True, help="7 ticks M1..M7")
    h.add_argument("--minutes", type=float, default=5)
    h.add_argument("--every", type=float, default=30, help="report interval, s")
    a = ap.parse_args()
    {"replay": cmd_replay, "heat": cmd_heat}[a.cmd](a)


if __name__ == "__main__":
    main()
