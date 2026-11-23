//! Stall test against a kitchen scale (issue #2): torque per amp without the URDF masses.
//!
//! A lever of known length on the joint rests on a scale. The goal is stepped INTO the scale (the joint is blocked, so
//! the error becomes duty) at increasing then decreasing offsets; at each level the tool holds, measures duty and the
//! reported (supply) current, and asks for the scale reading. Torque = (grams - tare) / 1000 * g * lever. The steps up
//! and down approach each level from both sides, so the gearbox's static friction cancels in the fit and shows up as
//! their difference.
//!
//! Safety: the whole sweep (base ± largest offset) is checked against the table and the calibrated range first; the
//! duty stays under ~17 % (0.42 %/tick × 40 ticks), below the servo's 25 % overload threshold; it aborts and returns to
//! base when the duty passes `MAX_DUTY` or the joint moves more than `SLIP_TICKS` past its goal (the lever slipped).

use crate::io::Io;
use crate::safety::{self, SafetyConfig};
use anyhow::{bail, Result};
use std::time::{Duration, Instant};

pub const G: f64 = 9.81;
/// Abort above this duty (servo overload protection starts at 25 % for 2 s).
pub const MAX_DUTY: f64 = 0.30;
/// The joint moved this far beyond its goal into the scale: the lever slipped off.
pub const SLIP_TICKS: i32 = 15;
/// Up to the largest offset and back down: each level (except the top) is approached from both sides.
pub const OFFSETS: [i32; 11] = [0, 5, 10, 20, 30, 40, 30, 20, 10, 5, 0];

#[derive(Debug, Clone)]
pub struct StallOpts {
    pub joint: u8,
    /// +1 or -1: which way (in ticks) presses the lever into the scale.
    pub into: i32,
    pub lever_m: f64,
    /// Scale of the offsets (1.0 = OFFSETS as is).
    pub max_offset: i32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct StallPoint {
    pub offset: i32,
    /// true on the way up (increasing offset), false on the way down.
    pub up: bool,
    pub goal: u16,
    pub position: f64,
    pub duty: f64,
    pub current_ma: f64,
    /// Supply current / |duty| (the ST3215 reports the supply current).
    pub i_motor_a: f64,
    pub grams: f64,
    pub torque_nm: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct StallFit {
    /// Slope of torque against motor current (N m/A) with both approach directions pooled.
    pub kt_eta_n: f64,
    pub intercept_nm: f64,
    /// Half the mean torque difference between approaching up and down at the same offset (N m).
    pub friction_nm: f64,
    pub r2: f64,
    pub points: usize,
}

pub fn offsets(max: i32) -> Vec<i32> {
    OFFSETS.iter().map(|&o| ((o as f64) * max as f64 / 40.0).round() as i32).collect()
}

/// Torque from the scale: grams above the tare, times g, times the lever.
pub fn torque(grams: f64, tare: f64, lever_m: f64) -> f64 {
    (grams - tare) / 1000.0 * G * lever_m
}

/// Least squares of the scale torque against motor current with a friction term:
/// torque = Kt*eta*N * i + b - f (going up: friction eats part of the motor torque) or + f (coming down).
pub fn fit(points: &[StallPoint]) -> Option<StallFit> {
    let p: Vec<&StallPoint> = points.iter().filter(|p| p.offset != 0 && p.i_motor_a > 0.0).collect();
    if p.len() < 4 || p.iter().all(|q| q.up) || p.iter().all(|q| !q.up) {
        return None;
    }
    // Normal equations for x = [k, b, f], rows [i, 1, s] with s = -1 up, +1 down.
    let mut a = [[0.0f64; 3]; 3];
    let mut y = [0.0f64; 3];
    for q in &p {
        let r = [q.i_motor_a, 1.0, if q.up { -1.0 } else { 1.0 }];
        for i in 0..3 {
            for j in 0..3 {
                a[i][j] += r[i] * r[j];
            }
            y[i] += r[i] * q.torque_nm;
        }
    }
    let x = solve3(a, y)?;
    let mean = p.iter().map(|q| q.torque_nm).sum::<f64>() / p.len() as f64;
    let (mut sse, mut sst) = (0.0, 0.0);
    for q in &p {
        let pred = x[0] * q.i_motor_a + x[1] + x[2] * if q.up { -1.0 } else { 1.0 };
        sse += (q.torque_nm - pred).powi(2);
        sst += (q.torque_nm - mean).powi(2);
    }
    Some(StallFit { kt_eta_n: x[0], intercept_nm: x[1], friction_nm: x[2], r2: if sst > 0.0 { 1.0 - sse / sst } else { f64::NAN }, points: p.len() })
}

fn solve3(mut a: [[f64; 3]; 3], mut y: [f64; 3]) -> Option<[f64; 3]> {
    for c in 0..3 {
        let piv = (c..3).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if a[piv][c].abs() < 1e-12 {
            return None;
        }
        a.swap(c, piv);
        y.swap(c, piv);
        for r in 0..3 {
            if r != c {
                let f = a[r][c] / a[c][c];
                for k in 0..3 {
                    a[r][k] -= f * a[c][k];
                }
                y[r] -= f * y[c];
            }
        }
    }
    Some([y[0] / a[0][0], y[1] / a[1][1], y[2] / a[2][2]])
}

/// Checks the sweep before anything moves: table clearance and calibrated range for base ± the largest offset.
pub fn check(sc: &SafetyConfig, base: &[u16; 7], m8: u16, o: &StallOpts) -> Result<()> {
    if !(1..=8).contains(&o.joint) {
        bail!("stall test: joint must be 1-8");
    }
    if o.into != 1 && o.into != -1 {
        bail!("--into must be +1 or -1 (which way presses the lever into the scale)");
    }
    if !(5..=60).contains(&o.max_offset) {
        bail!("--stall-max must be 5-60 ticks");
    }
    let j = o.joint as usize - 1;
    let reach = o.max_offset as f64 + SLIP_TICKS as f64;
    let here = if j == 7 { m8 } else { base[j] };
    if sc.range_room(j, here) < reach {
        bail!("M{}: {} ticks to the end of its calibrated range, the test needs {reach:.0}", o.joint, sc.range_room(j, here));
    }
    if j == 7 {
        // M8 (the boom on the horn, fingers off) does not move the arm: no table sweep, but the arm must be clear now.
        return match safety::clearance(sc, base) {
            None => bail!("table height not set"),
            Some(c) if c < sc.margin_m => bail!("the arm is {:.0} mm above the table, margin {:.0} mm", c * 1e3, sc.margin_m * 1e3),
            _ => Ok(()),
        };
    }
    match safety::sweep_clearance(sc, base, j, reach) {
        None => bail!("table height not set"),
        Some(c) if c < sc.margin_m => bail!("M{} sweep ±{reach:.0} comes within {:.0} mm of the table", o.joint, c * 1e3),
        _ => Ok(()),
    }
}

/// Runs the test. `ask` gets each measured point and returns the scale reading in grams (None = abort).
pub async fn run<I: Io>(
    io: &mut I,
    sc: &SafetyConfig,
    o: &StallOpts,
    mut ask: impl FnMut(&StallPoint) -> Option<f64>,
    log: &mut impl FnMut(String),
) -> Result<Vec<StallPoint>> {
    for id in 1..=8u8 {
        if !io.ping(id).await {
            bail!("servo M{id} does not answer (station still running?)");
        }
    }
    let present = io.read_positions(&[1, 2, 3, 4, 5, 6, 7, 8]).await?;
    let mut base = [0u16; 7];
    base.copy_from_slice(&present[..7]);
    check(sc, &base, present[7], o)?;
    io.hold_present(None).await?;
    let id = o.joint;
    let start = present[id as usize - 1];
    let b = start as i32;
    let mut points = Vec::new();
    let mut tare = None;
    let mut prev = 0;
    let result: Result<()> = async {
        for &off in &offsets(o.max_offset) {
            let up = off > prev || (off == 0 && points.is_empty());
            prev = off;
            let goal = (b + o.into * off) as u16;
            io.write_goals(&[(id, goal)]).await?;
            // Settle 0.8 s, watching for a slip or runaway duty.
            let t0 = Instant::now();
            while t0.elapsed() < Duration::from_millis(800) {
                let s = io.read_state(id).await?;
                guard(&s, goal, o.into)?;
            }
            // Measure 0.5 s.
            let (mut n, mut duty, mut cur, mut pos) = (0.0, 0.0, 0.0, 0.0);
            let t1 = Instant::now();
            while t1.elapsed() < Duration::from_millis(500) {
                let s = io.read_state(id).await?;
                guard(&s, goal, o.into)?;
                n += 1.0;
                duty += (s.load as f64 / 1000.0).abs();
                cur += s.current_ma.abs();
                pos += s.position as f64;
            }
            let (duty, cur, pos) = (duty / n, cur / n, pos / n);
            let mut p = StallPoint {
                offset: off,
                up,
                goal,
                position: pos,
                duty,
                current_ma: cur,
                i_motor_a: if duty > 0.01 { cur / 1000.0 / duty } else { 0.0 },
                ..Default::default()
            };
            log(format!(
                "M{id} offset {off:>3} ({}): goal {goal}, position {pos:.0}, duty {:.1} %, current {cur:.0} mA -> motor {:.3} A",
                if up { "up" } else { "down" },
                duty * 100.0,
                p.i_motor_a
            ));
            let Some(g) = ask(&p) else { bail!("aborted") };
            p.grams = g;
            if off == 0 && tare.is_none() {
                tare = Some(g);
            }
            p.torque_nm = torque(g, tare.unwrap_or(0.0), o.lever_m);
            points.push(p);
        }
        Ok(())
    }
    .await;
    // Always back to the start position (releases the scale).
    io.write_goals(&[(id, start)]).await?;
    tokio::time::sleep(Duration::from_millis(800)).await;
    result?;
    Ok(points)
}

fn guard(s: &crate::bus::MotorState, goal: u16, into: i32) -> Result<()> {
    let duty = (s.load as f64 / 1000.0).abs();
    if duty > MAX_DUTY {
        bail!("duty {:.0} % above {:.0} %: stopping", duty * 100.0, MAX_DUTY * 100.0);
    }
    let past = (s.position as i32 - goal as i32) * into;
    if past > SLIP_TICKS {
        bail!("the joint moved {past} ticks past its goal: the lever slipped off the scale?");
    }
    Ok(())
}

pub fn csv(points: &[StallPoint]) -> String {
    let mut s = String::from("offset,approach,goal,position,duty,current_ma,i_motor_a,grams,torque_nm\n");
    for p in points {
        s += &format!(
            "{},{},{},{:.1},{:.4},{:.1},{:.4},{:.1},{:.4}\n",
            p.offset,
            if p.up { "up" } else { "down" },
            p.goal,
            p.position,
            p.duty,
            p.current_ma,
            p.i_motor_a,
            p.grams,
            p.torque_nm
        );
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn offsets_go_up_and_back_down() {
        assert_eq!(offsets(40), OFFSETS.to_vec());
        assert_eq!(offsets(20), vec![0, 3, 5, 10, 15, 20, 15, 10, 5, 3, 0]);
    }

    #[test]
    fn fit_recovers_kt_and_friction() {
        // Motor torque k*i; going up the scale sees k*i - f, coming down k*i + f (friction holds part of the force).
        let (k, f) = (1.1, 0.05);
        let mut pts = Vec::new();
        for (i, &(off, up)) in [(5, true), (10, true), (20, true), (40, true), (20, false), (10, false), (5, false)].iter().enumerate() {
            let im = 0.1 + off as f64 * 0.02 + i as f64 * 1e-4;
            let t = k * im - if up { f } else { -f };
            pts.push(StallPoint { offset: off, up, i_motor_a: im, torque_nm: t, ..Default::default() });
        }
        let r = fit(&pts).unwrap();
        assert!((r.kt_eta_n - k).abs() < 0.05, "{r:?}");
        assert!((r.friction_nm - f).abs() < 0.01, "{r:?}");
    }

    #[test]
    fn torque_from_grams() {
        // 200 g above a 50 g tare on a 100 mm lever: 0.15 kg * 9.81 * 0.1 m.
        assert!((torque(200.0, 50.0, 0.1) - 0.14715).abs() < 1e-9);
    }

    #[test]
    fn check_refuses_without_room() {
        let sc = SafetyConfig { table_z_m: Some(0.0), ..Default::default() };
        let base = [2001, 1042, 2051, 3160, 1881, 1869, 2244];
        let o = |joint, max| StallOpts { joint, into: 1, lever_m: 0.1, max_offset: max };
        assert!(check(&sc, &base, 2018, &o(7, 40)).is_ok());
        assert!(check(&sc, &base, 2018, &o(4, 40)).unwrap_err().to_string().contains("calibrated range"), "M4 at 3160");
        assert!(check(&sc, &base, 2018, &o(7, 80)).is_err(), "offset cap");
        assert!(check(&sc, &base, 2018, &o(8, 20)).is_ok(), "M8 with the boom on the horn");
        assert!(check(&sc, &base, 1240, &o(8, 20)).unwrap_err().to_string().contains("calibrated range"), "M8 near 1237");
    }
}
