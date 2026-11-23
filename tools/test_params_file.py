#!/usr/bin/env python3
"""Checks for servo_params.toml handling (issue #5): python3 tools/test_params_file.py (servo_sim needs mujoco)."""
import os
import shutil
import sys
import tempfile
from dataclasses import fields

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params_file as pf  # noqa: E402


def test_default_section_reproduces_servoparams_defaults():
    from servo_sim import FILE_KEYS, ServoParams
    default = pf.load(pf.PARAMS)["default"]
    sp = ServoParams()
    missing = [k for k in FILE_KEYS if k not in default]
    assert not missing, missing
    diff = {k: (default[k], getattr(sp, a)) for k, a in FILE_KEYS.items() if abs(float(default[k]) - getattr(sp, a)) > 1e-12}
    assert not diff, diff
    # every numeric ServoParams field is in the file
    assert {f.name for f in fields(ServoParams)} == set(FILE_KEYS.values())


def test_joint_merges_default_and_section():
    from servo_sim import ServoParams
    d = pf.load(pf.PARAMS)
    m3 = ServoParams.from_file(3)
    assert m3.r_ohm == d["servo.M3"]["resistance_ohm"]
    assert m3.duty_cap == d["default"]["duty_cap"]
    assert ServoParams.from_file(3, r_ohm=9.0).r_ohm == 9.0, "explicit overrides win"
    assert ServoParams.from_file(3, r_ohm=None).r_ohm == m3.r_ohm, "None overrides are ignored"


def test_update_keeps_comments_and_layout():
    tmp = os.path.join(tempfile.mkdtemp(), "p.toml")
    shutil.copy(pf.PARAMS, tmp)
    before = open(tmp).read()
    pf.update(3, {"static_friction_nm": 0.5, "dead_zone_ticks": 5.0}, source="test", path=tmp)
    pf.update(8, {"ke_n": 1.0}, path=tmp)
    after = open(tmp).read()
    assert after.count("#") == before.count("#")
    assert pf.joint_params(3, tmp)["static_friction_nm"] == 0.5
    assert pf.joint_params(3, tmp)["dead_zone_ticks"] == 5.0 and isinstance(pf.joint_params(3, tmp)["dead_zone_ticks"], float)
    assert pf.joint_params(8, tmp)["ke_n"] == 1.0
    assert pf.load(tmp)[""]["source"] == "test"


def test_calibration_both_layouts():
    cal = pf.calibration()
    assert cal["zero"][2] == 2051.3 and cal["range_min"][7] == 1237
    pretty = "table_z_m = 0.0\ntool_spheres = [\n    [\n    [\n    0.0,\n    0.045,\n],\n    0.04,\n],\n]\nzero = [\n    2050.3,\n    2053.0,\n]\n"
    assert pf.parse(pretty)[""]["zero"] == [2050.3, 2053.0]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok  %s" % name)
