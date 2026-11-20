//! Table safety area (issue #1): never drive the arm, gripper or paddle into the table.
//!
//! Forward kinematics with positions from `hardware/elrobot/simulation/elrobot_follower.urdf` (norma-core): every joint
//! has rpy = 0, so each link frame is the parent frame translated by the joint origin and rotated about the joint axis.
//! The arm's link frames sit at the joints; the gripper and paddle are modelled as spheres around points in the
//! gripper-base frame (configurable). Clearance = lowest sphere bottom (or joint point) above the table plane.

use nalgebra::{Rotation3, Unit, Vector3};
use serde::{Deserialize, Serialize};

/// (origin of the fixed link before the joint, origin of the revolute joint, axis), metres, base frame at zero pose.
/// Fixed joints "Rigid N" precede each revolute joint.
const CHAIN: [([f64; 3], [f64; 3], [f64; 3]); 7] = [
    ([0.0, 0.007585, 0.0315], [0.0, 0.034904, 0.017505], [0.0, -0.008727, 0.999962]),
    ([0.000625, -5.8e-05, 0.0067], [-0.020293, -0.000308, 0.035233], [-0.999962, -7.6e-05, 0.008726]),
    ([0.019863, -0.000487, 0.055827], [-0.019985, -0.000309, 0.035408], [-0.999848, -0.000152, 0.017452]),
    ([0.021263, 0.001933, 0.036403], [-0.02029, 0.035229, 0.000662], [-0.99981, 0.008574, 0.017527]),
    ([0.020391, 0.065663, -0.010035], [0.000323, 0.0172, 0.010051], [-0.008573, -0.999963, 0.000151]),
    ([6.5e-05, 0.0075, -1e-06], [-0.019988, 0.035407, 0.000351], [-0.999697, 0.0173, 0.017525]),
    ([0.020478, 0.038023, -0.010611], [0.000473, 0.017284, 0.009899], [-0.017144, -0.999812, 0.009029]),
];

/// Gripper motor (M8) relative to the gripper base ("Rigid 17").
const GRIPPER_MOTOR: [f64; 3] = [0.000776, 0.045242, -0.000409];

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
pub struct SafetyConfig {
    /// Height of the table plane in the robot base frame (m). Must be taught (`teach-table`) or measured:
    /// runs on the arm are refused while it is None.
    pub table_z_m: Option<f64>,
    /// Minimum allowed clearance (m) between any modelled point/sphere and the table.
    pub margin_m: f64,
    /// Spheres (centre in the gripper-base frame, metres; radius) enclosing the gripper and the paddle.
    pub tool_spheres: Vec<([f64; 3], f64)>,
    /// rad = sign * (tick - zero) * 2pi/4096 for M1..M7 (same convention as PingPongSimBot).
    pub sign: [f64; 7],
    pub zero: [f64; 7],
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            table_z_m: None,
            margin_m: 0.03,
            // Conservative: the jaws (~60 mm past the gripper motor) and the Ø150 mm paddle beyond them.
            // The paddle direction relative to the gripper frame is not measured, so its sphere is large.
            tool_spheres: vec![([0.0, 0.045, 0.0], 0.04), ([0.0, 0.10, 0.0], 0.05), ([0.0, 0.17, 0.0], 0.11)],
            sign: [-1.0; 7],
            zero: [2048.0; 7],
        }
    }
}

impl SafetyConfig {
    pub fn tick_to_rad(&self, j: usize, tick: f64) -> f64 {
        self.sign[j] * (tick - self.zero[j]) * std::f64::consts::TAU / 4096.0
    }

    /// Joint angles of M1..M7 from ticks.
    pub fn angles(&self, ticks: &[u16]) -> [f64; 7] {
        let mut q = [0.0; 7];
        for j in 0..7 {
            q[j] = self.tick_to_rad(j, ticks[j] as f64);
        }
        q
    }
}

/// Positions of the 7 joint frames, plus the gripper-base pose (rotation, position).
pub fn fk(q: &[f64; 7]) -> ([Vector3<f64>; 7], Rotation3<f64>, Vector3<f64>) {
    let mut r = Rotation3::identity();
    let mut p = Vector3::zeros();
    let mut joints = [Vector3::zeros(); 7];
    for (j, (fixed, origin, axis)) in CHAIN.iter().enumerate() {
        p += r * Vector3::from(*fixed);
        p += r * Vector3::from(*origin);
        joints[j] = p;
        r *= Rotation3::from_axis_angle(&Unit::new_normalize(Vector3::from(*axis)), q[j]);
    }
    (joints, r, p)
}

/// Lowest point of the modelled arm + tool (m, base frame) for joint angles q.
pub fn lowest_z(cfg: &SafetyConfig, q: &[f64; 7]) -> f64 {
    let (joints, r, p) = fk(q);
    let mut z = joints[1..].iter().map(|j| j.z).fold(f64::INFINITY, f64::min);
    z = z.min((p + r * Vector3::from(GRIPPER_MOTOR)).z);
    for (c, rad) in &cfg.tool_spheres {
        z = z.min((p + r * Vector3::from(*c)).z - rad);
    }
    z
}

/// Clearance above the table (m) for a pose in ticks; None if the table height is not known.
pub fn clearance(cfg: &SafetyConfig, ticks: &[u16]) -> Option<f64> {
    cfg.table_z_m.map(|t| lowest_z(cfg, &cfg.angles(ticks)) - t)
}

/// Smallest clearance while joint `j` sweeps base ± amp (every 2 ticks), all other joints at base.
pub fn sweep_clearance(cfg: &SafetyConfig, base: &[u16; 7], j: usize, amp: f64) -> Option<f64> {
    let t = cfg.table_z_m?;
    let mut worst = f64::INFINITY;
    let mut off = -amp;
    while off <= amp + 1e-9 {
        let mut pose = *base;
        pose[j] = (base[j] as f64 + off).round().clamp(0.0, 4095.0) as u16;
        worst = worst.min(lowest_z(cfg, &cfg.angles(&pose)) - t);
        off += 2.0;
    }
    Some(worst)
}

/// Largest amplitude (ticks, ≤ `wanted`) whose sweep keeps the margin; 0 if even the base pose doesn't.
pub fn safe_amp(cfg: &SafetyConfig, base: &[u16; 7], j: usize, wanted: f64) -> f64 {
    let ok = |a: f64| sweep_clearance(cfg, base, j, a).is_some_and(|c| c >= cfg.margin_m);
    if ok(wanted) {
        return wanted;
    }
    let (mut lo, mut hi) = (0.0, wanted);
    if !ok(0.0) {
        return 0.0;
    }
    while hi - lo > 1.0 {
        let mid = (lo + hi) / 2.0;
        if ok(mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo.floor()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_pose_positions_are_the_summed_urdf_origins() {
        let (joints, _, p) = fk(&[0.0; 7]);
        let mut s = Vector3::zeros();
        for (f, o, _) in CHAIN.iter() {
            s += Vector3::from(*f) + Vector3::from(*o);
        }
        assert!((p - s).norm() < 1e-12);
        assert!((joints[0] - Vector3::new(0.0, 0.007585 + 0.034904, 0.0315 + 0.017505)).norm() < 1e-12);
    }

    #[test]
    fn base_rotation_keeps_heights() {
        let mut q = [0.2, 0.3, -0.5, 0.4, 0.1, 0.2, -0.3];
        let z0 = lowest_z(&SafetyConfig::default(), &q);
        q[0] += 1.0; // M1 turns about (almost) vertical: heights barely change
        let z1 = lowest_z(&SafetyConfig::default(), &q);
        assert!((z0 - z1).abs() < 0.005, "{z0} vs {z1}");
    }

    #[test]
    fn a_plan_that_reaches_the_table_is_cut_down() {
        let mut cfg = SafetyConfig::default();
        let base = [2003u16, 1002, 1942, 3185, 1864, 1892, 2226];
        let q = cfg.angles(&base);
        // Put the table 5 cm below the lowest point at the base pose.
        cfg.table_z_m = Some(lowest_z(&cfg, &q) - 0.05);
        // The shoulder (M2) sweeping ±200 ticks moves the tool a lot: some direction must come closer than 5 cm.
        let c = sweep_clearance(&cfg, &base, 1, 200.0).unwrap();
        assert!(c < 0.05, "clearance {c}");
        let a = safe_amp(&cfg, &base, 1, 200.0);
        assert!(a < 200.0, "amp {a}");
        assert!(sweep_clearance(&cfg, &base, 1, a).unwrap() >= cfg.margin_m - 1e-6);
        // No table known: no clearance, so no permission.
        cfg.table_z_m = None;
        assert!(clearance(&cfg, &[0; 7]).is_none());
    }
}
