#!/usr/bin/env python3
"""Checks for tools/servo_sim.py (needs mujoco): python3 tools/test_servo_sim.py"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_sim import DT, ArmSim, ServoParams  # noqa: E402

LOAD_POSE = [2001, 1750, 2100, 2450, 1881, 1869, 2244]


def test_stiction_holds_the_arm():
    # Friction above the gravity load (M3 0.48 N m): nothing creeps (MuJoCo's soft frictionloss did, 20+ ticks/s).
    sim = ArmSim(ServoParams(static_friction=0.6), LOAD_POSE)
    for _ in range(1000):
        sim.step()
    drift = np.abs(sim.ticks() - LOAD_POSE).max()
    assert drift < 0.5, drift


def test_overload_protection_trips_and_caps_the_duty():
    # A blocked joint (huge friction) commanded 200 ticks away: duty saturates at 50 % > overload 25 %;
    # after 2 s the servo protects itself and caps the duty at 20 %.
    sim = ArmSim(ServoParams(static_friction=50.0), LOAD_POSE)
    sim.command(6, LOAD_POSE[6] + 200)
    for _ in range(int(1.5 / DT)):
        sim.step()
    r = sim.reading(6)
    assert not r["tripped"] and abs(r["load"]) > 450, r
    for _ in range(int(1.0 / DT)):
        sim.step()
    r = sim.reading(6)
    assert r["tripped"] and abs(r["load"]) <= 200 + 1e-6, r


def test_supply_current_is_duty_times_motor_current():
    sim = ArmSim(ServoParams(), LOAD_POSE)
    sim.command(2, LOAD_POSE[2] + 50)
    for _ in range(300):
        sim.step()
    r = sim.reading(2)
    assert abs(r["current_ma"] - abs(r["load"]) / 1000 * abs(r["i_motor_a"]) * 1000) < 1e-6


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok  %s" % name)
