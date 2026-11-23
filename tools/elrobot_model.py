"""Shared MuJoCo helpers for the ElRobot follower (issue #2): load the norma-core URDF, convert servo ticks to joint
angles with the same mapping as the safety model (src/safety.rs), gravity torques.

Needs `mujoco` (3.x) and numpy. Runs on the laptop (MuJoCo is not installed on the Pi).
"""
import os
import tempfile

import mujoco
import numpy as np

URDF = os.path.expanduser(os.environ.get(
    "ELROBOT_URDF", "~/DevOps/norma-core/hardware/elrobot/simulation/elrobot_follower.urdf"))
TICKS_PER_REV = 4096.0
RAD_PER_TICK = 2 * np.pi / TICKS_PER_REV
# Same as SafetyConfig::default(): sign +1, zeros from the station viewer mapping matched at level (2026-11-21).
ZERO = [2050.3, 2053.0, 2051.3, 2053.0, 1842.7, 1924.5, 2198.6]
RANGE_MIN = [1087, 1004, 935, 900, 141, 1040, 134, 1237]
RANGE_MAX = [3009, 3092, 3161, 3196, 3955, 3056, 3962, 2859]
JOINTS = ["rev_motor_0%d" % (i + 1) for i in range(7)]


def load_model(urdf=URDF):
    """The URDF as a MuJoCo model (meshes from its assets folder)."""
    u = open(urdf).read().replace('filename="assets/', 'filename="')
    u = u.replace(
        '<robot name="Elrobot" >',
        '<robot name="Elrobot"><mujoco><compiler meshdir="%s" balanceinertia="true" boundmass="0.001" '
        'boundinertia="1e-8" fusestatic="false"/></mujoco>' % os.path.join(os.path.dirname(urdf), "assets"),
        1,
    )
    f = tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False)
    try:
        f.write(u)
        f.close()
        return mujoco.MjModel.from_xml_path(f.name)
    finally:
        os.unlink(f.name)


def ticks_to_q(ticks):
    return np.array([(ticks[i] - ZERO[i]) * RAD_PER_TICK for i in range(7)])


def q_to_ticks(q):
    return np.array([q[i] / RAD_PER_TICK + ZERO[i] for i in range(7)])


class Arm:
    """Model + data with joint addresses."""

    def __init__(self, urdf=URDF):
        self.m = load_model(urdf)
        # The URDF's meshes overlap (base <-> Joint_01 by 28 mm, gripper parts): contacts would push the joints apart
        # (a 22-tick jump at t = 0). Self-collision is not part of the servo model, so contacts are off.
        self.m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
        self.d = mujoco.MjData(self.m)
        self.qadr = np.array([self.m.joint(n).qposadr[0] for n in JOINTS])
        self.dadr = np.array([self.m.joint(n).dofadr[0] for n in JOINTS])

    def set_ticks(self, ticks):
        self.d.qpos[:] = 0
        self.d.qvel[:] = 0
        self.d.qpos[self.qadr] = ticks_to_q(ticks)
        mujoco.mj_forward(self.m, self.d)

    def gravity(self, ticks):
        """Gravity torque (N m) on M1..M7 at a pose in ticks: what the motors must hold (sign: MuJoCo's bias force)."""
        self.set_ticks(ticks)
        return self.d.qfrc_bias[self.dadr].copy()


def read_meta_base(session):
    """base_ticks from a session's meta.toml (M1..M7)."""
    for line in open(os.path.join(session, "meta.toml")):
        if line.startswith("base_ticks"):
            return [int(x) for x in line.split("[", 1)[1].split("]", 1)[0].split(",")]
    raise ValueError("no base_ticks in %s/meta.toml" % session)
