//! A system-identification session: test plan, per-joint runs, statistics, and the safety checks around them.
//! Used by both the command line and the full-screen app; works on the real bus or simulated servos ([`Io`]).

use crate::bus::MotorState;
use crate::io::Io;
use crate::safety::{self, SafetyConfig};
use anyhow::{bail, Context, Result};
use clap::ValueEnum;
use std::fmt::Write as _;
use std::io::Write as _;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU8, Ordering};

/// Run control shared with the UI (an `AtomicU8`).
pub mod ctl {
    /// Keep going.
    pub const RUN: u8 = 0;
    /// SAFETY STOP: freeze the arm where it is (goal = present position, torque stays on) and wait for a decision.
    pub const FREEZE: u8 = 1;
    /// Stop gently: the joint returns to base, then the arm to its start pose.
    pub const RETURN: u8 = 2;
    /// After a freeze: leave the arm where it is and end the run.
    pub const LEAVE: u8 = 3;
}
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum Test {
    Hold,
    Steps,
    Reversals,
    Ramp,
    Chirp,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
pub enum Base {
    /// Where the arm is now (no move before the tests).
    Current,
    /// The `level` pose from the paddle.toml (min-jerk move there first).
    Level,
}


/// Minimum-jerk interpolation factor for tau in [0, 1].
pub fn min_jerk(tau: f64) -> f64 {
    let t = tau.clamp(0.0, 1.0);
    t * t * t * (10.0 - 15.0 * t + 6.0 * t * t)
}

// ---------------------------------------------------------------------------------------------------------------
// Test signals: offsets (ticks) from the base position as a function of time within a segment.

#[derive(Debug, Clone)]
pub enum Shape {
    Hold { at: f64 },
    /// Jump to `to` at once and stay.
    Step { to: f64 },
    Ramp { from: f64, to: f64 },
    /// Log sweep f0 -> f1 Hz with a 1 s cosine fade in and out.
    Chirp { amp: f64, f0: f64, f1: f64 },
}

#[derive(Debug, Clone)]
pub struct Segment {
    pub name: String,
    pub test: Test,
    pub dur: f64,
    pub shape: Shape,
}

impl Segment {
    pub fn offset(&self, t: f64) -> f64 {
        let t = t.clamp(0.0, self.dur);
        match self.shape {
            Shape::Hold { at } => at,
            Shape::Step { to } => to,
            Shape::Ramp { from, to } => from + (to - from) * t / self.dur,
            Shape::Chirp { amp, f0, f1 } => {
                let k = f1 / f0;
                let phase = 2.0 * std::f64::consts::PI * f0 * self.dur / k.ln() * (k.powf(t / self.dur) - 1.0);
                let fade = |x: f64| if x >= 1.0 { 1.0 } else { 0.5 - 0.5 * (std::f64::consts::PI * x).cos() };
                amp * phase.sin() * fade(t) * fade(self.dur - t)
            }
        }
    }
}

pub fn plan(tests: &[Test], max_amp: f64) -> Vec<Segment> {
    let mut s = Vec::new();
    let mut push = |name: String, test: Test, dur: f64, shape: Shape| s.push(Segment { name, test, dur, shape });
    for &test in tests {
        match test {
            Test::Hold => push("hold".into(), test, 2.0, Shape::Hold { at: 0.0 }),
            Test::Steps => {
                for a in [10.0, 25.0, 50.0, 100.0f64] {
                    if a > max_amp {
                        continue;
                    }
                    push(format!("step+{a}"), test, 1.0, Shape::Step { to: a });
                    push(format!("step+{a}_back"), test, 1.0, Shape::Step { to: 0.0 });
                    push(format!("step-{a}"), test, 1.0, Shape::Step { to: -a });
                    push(format!("step-{a}_back"), test, 1.0, Shape::Step { to: 0.0 });
                }
            }
            Test::Reversals => {
                for a in [5.0, 10.0, 20.0f64] {
                    if a > max_amp {
                        continue;
                    }
                    push(format!("rev{a}_up"), test, 1.0, Shape::Ramp { from: 0.0, to: a });
                    push(format!("rev{a}_down"), test, 2.0, Shape::Ramp { from: a, to: -a });
                    push(format!("rev{a}_back"), test, 1.0, Shape::Ramp { from: -a, to: 0.0 });
                    push(format!("rev{a}_rest"), test, 0.5, Shape::Hold { at: 0.0 });
                }
            }
            Test::Ramp => {
                let a = (max_amp / 2.0).min(50.0);
                let d = a / 15.0;
                push("ramp_out+".into(), test, d, Shape::Ramp { from: 0.0, to: a });
                push("ramp_hold+".into(), test, 0.5, Shape::Hold { at: a });
                push("ramp_across".into(), test, 2.0 * d, Shape::Ramp { from: a, to: -a });
                push("ramp_hold-".into(), test, 0.5, Shape::Hold { at: -a });
                push("ramp_back".into(), test, d, Shape::Ramp { from: -a, to: 0.0 });
            }
            Test::Chirp => {
                push("chirp".into(), test, 25.0, Shape::Chirp { amp: 20.0f64.min(max_amp), f0: 0.2, f1: 4.0 });
                push("chirp_rest".into(), test, 1.0, Shape::Hold { at: 0.0 });
            }
        }
    }
    s
}

// ---------------------------------------------------------------------------------------------------------------
// Logging and on-line summary.

pub struct Sample {
    pub t: f64,
    pub seg: usize,
    pub goal: u16,
    pub st: MotorState,
}

#[derive(Debug, Default, Clone)]
pub struct StepStats {
    pub amp: f64,
    pub dead_ms: f64,
    pub t63_ms: f64,
    pub overshoot_pct: f64,
    pub steady_err: f64,
}

/// Dead time (first 3-tick move), 63% time, overshoot and final error for one step segment.
pub fn step_stats(samples: &[Sample], from: f64, to: f64) -> Option<StepStats> {
    let first = samples.first()?;
    let (t0, p0) = (first.t, first.st.position as f64);
    let amp = to - from;
    if amp.abs() < 1.0 {
        return None;
    }
    let s = amp.signum();
    let moved = |p: f64| (p - p0) * s;
    let dead = samples.iter().find(|x| moved(x.st.position as f64) >= 3.0).map(|x| x.t - t0)?;
    let t63 = samples.iter().find(|x| moved(x.st.position as f64) >= 0.63 * amp.abs()).map(|x| x.t - t0).unwrap_or(f64::NAN);
    let peak = samples.iter().map(|x| moved(x.st.position as f64)).fold(f64::MIN, f64::max);
    let tail: Vec<f64> = samples.iter().filter(|x| x.t - t0 > 0.7).map(|x| x.goal as f64 - x.st.position as f64).collect();
    let steady = if tail.is_empty() { f64::NAN } else { tail.iter().sum::<f64>() / tail.len() as f64 };
    Some(StepStats {
        amp,
        dead_ms: dead * 1e3,
        t63_ms: t63 * 1e3,
        overshoot_pct: ((peak - amp.abs()) / amp.abs() * 100.0).max(0.0),
        steady_err: steady,
    })
}

/// Slop estimate at a reversal: goal travel (ticks) after the turn before the position moves 2 ticks the new way.
pub fn reversal_slop(samples: &[Sample], dir: f64) -> Option<f64> {
    let first = samples.first()?;
    let g0 = first.goal as f64;
    // Furthest the position got in the old direction (it may keep going briefly after the goal turns, from lag).
    let p_extreme = samples.iter().map(|x| x.st.position as f64 * -dir).fold(f64::MIN, f64::max) * -dir;
    samples
        .iter()
        .find(|x| (x.st.position as f64 - p_extreme) * dir >= 2.0)
        .map(|x| ((x.goal as f64 - g0) * dir).max(0.0))
}

// ---------------------------------------------------------------------------------------------------------------
// Servo settings for the model.

pub struct ServoInfo {
    eeprom: Vec<u8>,
    ram: Vec<u8>,
}

impl ServoInfo {
    /// The duty the servo can reach, in 0.1 %: the smaller of the EEPROM Max Torque and the RAM Torque Limit.
    /// The ElRobot follower is set to 500 (50 %), so a blocked joint saturates there, never at 100 %.
    pub fn duty_cap(&self) -> i32 {
        let ram = u16::from_le_bytes([self.ram[8], self.ram[9]]) as i32;
        let cap = (self.u16(0x10) as i32).min(if ram > 0 { ram } else { 1000 });
        if (1..=1000).contains(&cap) { cap } else { 1000 }
    }

    fn u16(&self, a: usize) -> u16 {
        u16::from_le_bytes([self.eeprom[a], self.eeprom[a + 1]])
    }

    pub fn describe(&self, out: &mut String, id: u8) {
        let e = &self.eeprom;
        let r = &self.ram; // from 0x28
        let _ = writeln!(out, "[servo.M{id}]");
        let _ = writeln!(out, "model = {}", self.u16(0x00));
        let _ = writeln!(out, "firmware = {}", e[0x02]);
        let _ = writeln!(out, "min_angle_limit = {}", self.u16(0x09));
        let _ = writeln!(out, "max_angle_limit = {}", self.u16(0x0B));
        // Limits and protection (Feetech STS memory table), for the electrical/thermal model (#2).
        let _ = writeln!(out, "max_temperature_c = {}", e[0x0D]);
        let _ = writeln!(out, "max_voltage_dv = {}", e[0x0E]);
        let _ = writeln!(out, "min_voltage_dv = {}", e[0x0F]);
        let _ = writeln!(out, "max_torque = {}  # 0.1% units", self.u16(0x10));
        let _ = writeln!(out, "unloading_conditions = 0x{:02x}  # bits: 0 voltage, 2 temperature, 3 current, 5 overload", e[0x13]);
        let _ = writeln!(out, "p_coef = {}", e[0x15]);
        let _ = writeln!(out, "d_coef = {}", e[0x16]);
        let _ = writeln!(out, "i_coef = {}", e[0x17]);
        let _ = writeln!(out, "min_startup_force = {}", self.u16(0x18));
        let _ = writeln!(out, "cw_dead_zone = {}", e[0x1A]);
        let _ = writeln!(out, "ccw_dead_zone = {}", e[0x1B]);
        let _ = writeln!(out, "protection_current = {}  # 6.5 mA units = {:.0} mA", self.u16(0x1C), self.u16(0x1C) as f64 * 6.5);
        let _ = writeln!(out, "offset = {}", self.u16(0x1F));
        let _ = writeln!(out, "mode = {}", e[0x21]);
        let _ = writeln!(out, "protective_torque = {}  # % of max, applied after a protection trips", e[0x22]);
        let _ = writeln!(out, "protection_time = {}  # 10 ms units, overload before tripping", e[0x23]);
        let _ = writeln!(out, "overload_torque = {}  # % of max that counts as overload", e[0x24]);
        let _ = writeln!(out, "overcurrent_protection_time = {}  # 10 ms units", e[0x26]);
        let _ = writeln!(out, "torque_enable = {}", r[0]);
        let _ = writeln!(out, "acc = {}", r[1]);
        let _ = writeln!(out, "goal_speed = {}", u16::from_le_bytes([r[6], r[7]]));
        let _ = writeln!(out, "torque_limit = {}", u16::from_le_bytes([r[8], r[9]]));
        let hex = |b: &[u8]| b.iter().map(|x| format!("{x:02x}")).collect::<String>();
        let _ = writeln!(out, "eeprom_hex = \"{}\"", hex(e));
        let _ = writeln!(out, "ram_0x28_hex = \"{}\"\n", hex(r));
    }
}

pub async fn read_servo_info(io: &mut impl Io, id: u8) -> Result<ServoInfo> {
    let eeprom = io.read_bytes(id, 0x00, 0x28).await?;
    let ram = io.read_bytes(id, 0x28, 10).await?;
    Ok(ServoInfo { eeprom, ram })
}

// ---------------------------------------------------------------------------------------------------------------

pub async fn move_all(io: &mut impl Io, from: [u16; 7], to: [u16; 7], secs: f64) -> Result<()> {
    let t0 = Instant::now();
    let mut tick = tokio::time::interval(Duration::from_millis(20));
    loop {
        tick.tick().await;
        let s = min_jerk(t0.elapsed().as_secs_f64() / secs.max(0.05));
        let goals: Vec<(u8, u16)> = (0..7)
            .map(|j| (j as u8 + 1, (from[j] as f64 + (to[j] as f64 - from[j] as f64) * s).round() as u16))
            .collect();
        io.write_goals(&goals).await?;
        if s >= 1.0 {
            return Ok(());
        }
    }
}

/// Smoothly bring one joint from `from` to `to`.
pub async fn move_joint(io: &mut impl Io, id: u8, from: u16, to: u16, secs: f64) -> Result<()> {
    let t0 = Instant::now();
    let mut tick = tokio::time::interval(Duration::from_millis(20));
    loop {
        tick.tick().await;
        let s = min_jerk(t0.elapsed().as_secs_f64() / secs.max(0.05));
        let g = (from as f64 + (to as f64 - from as f64) * s).round() as u16;
        io.write_goals(&[(id, g)]).await?;
        if s >= 1.0 {
            return Ok(());
        }
    }
}

/// Runs the plan on one joint. Returns the samples, and Some(reason) if it stopped early.
#[allow(clippy::too_many_arguments)]
pub async fn run_joint(
    io: &mut impl Io,
    id: u8,
    base: u16,
    base7: &[u16; 7],
    sc: &safety::SafetyConfig,
    segs: &[Segment],
    stop: &AtomicU8,
    csv: &mut impl std::io::Write,
    emit: &mut impl FnMut(Event),
    total_s: f64,
    blocked_load: i32,
) -> Result<(Vec<Sample>, Option<String>)> {
    writeln!(csv, "t,segment,test,goal,position,speed,load,current_ma,voltage_dv,temp_c")?;
    let mut samples = Vec::new();
    let t_start = Instant::now();
    let mut last_goal = base;
    let mut overload_since: Option<f64> = None;
    let mut read_errors = 0;
    for (si, seg) in segs.iter().enumerate() {
        let t0 = Instant::now();
        loop {
            match stop.load(Ordering::Relaxed) {
                ctl::RUN => {}
                ctl::RETURN => return Ok((samples, Some("stopped by the user".into()))),
                _ => {
                    // SAFETY STOP: hold exactly where the joint is now.
                    let here = io.read_state(id).await.map(|s| s.position).unwrap_or(last_goal);
                    io.write_goals(&[(id, here)]).await?;
                    return Ok((samples, Some(format!("SAFETY STOP at {here} in {}", seg.name))));
                }
            }
            let ts = t0.elapsed().as_secs_f64();
            if ts >= seg.dur {
                break;
            }
            let goal = (base as f64 + seg.offset(ts)).round().clamp(0.0, 4095.0) as u16;
            if goal != last_goal {
                io.write_goals(&[(id, goal)]).await?;
                last_goal = goal;
            }
            let st = match io.read_state(id).await {
                Ok(s) => {
                    read_errors = 0;
                    s
                }
                Err(e) => {
                    read_errors += 1;
                    if read_errors >= 5 {
                        return Ok((samples, Some(format!("5 failed reads in a row: {e:#}"))));
                    }
                    continue;
                }
            };
            let t = t_start.elapsed().as_secs_f64();
            if let Some(prev) = samples.last().map(|s: &Sample| s.t) {
                if t - prev > 0.25 {
                    emit(Event::Status(format!("M{id}: no samples for {:.2} s (in {}): the loop was held up", t - prev, seg.name)));
                }
            }
            writeln!(
                csv,
                "{t:.5},{},{:?},{goal},{},{},{},{:.1},{},{}",
                seg.name, seg.test, st.position, st.speed, st.load, st.current_ma, st.voltage_dv, st.temperature_c
            )?;
            // Safety: sustained high load (blocked joint) or the joint far from its goal (collision, slip). The load
            // threshold is 90 % of the servo's duty cap: with the follower's 50 % cap a stall saturates at 500.
            if (st.load as i32).abs() >= blocked_load {
                let since = *overload_since.get_or_insert(t);
                if t - since > 0.3 {
                    return Ok((samples, Some(format!("load {} for 0.3 s in {}", st.load, seg.name))));
                }
            } else {
                overload_since = None;
            }
            // Safety area: the measured pose must stay clear of the table.
            let mut pose = *base7;
            pose[id as usize - 1] = st.position;
            if let Some(c) = safety::clearance(sc, &pose) {
                if c < sc.margin_m {
                    return Ok((samples, Some(format!("clearance {:.0} mm below the {:.0} mm margin in {}", c * 1e3, sc.margin_m * 1e3, seg.name))));
                }
            }
            if (st.position as i32 - goal as i32).abs() > 300 {
                return Ok((samples, Some(format!("position {} is >300 ticks from goal {goal} in {}", st.position, seg.name))));
            }
            emit(Event::Sample { id, t, seg: seg.name.clone(), goal, pos: st.position, current: st.current_ma, load: st.load, progress: (t / total_s).min(1.0) });
            samples.push(Sample { t, seg: si, goal, st });
        }
    }
    Ok((samples, None))
}


// ---------------------------------------------------------------------------------------------------------------
// Summary, safety report, the whole session.

/// Largest |offset| (ticks) the plan reaches.
pub fn plan_reach(segs: &[Segment]) -> f64 {
    segs.iter().map(|s| (0..=200).map(|i| s.offset(s.dur * i as f64 / 200.0).abs()).fold(0.0, f64::max)).fold(0.0, f64::max)
}

#[derive(Debug, Clone)]
pub struct JointClearance {
    pub id: u8,
    /// Lowest clearance over the planned sweep (m); None when the table height is unknown.
    pub clearance: Option<f64>,
    pub ok: bool,
    pub safe_amp: f64,
    /// Ticks from the base to the nearer end of the joint's calibrated range.
    pub range_room: f64,
}

pub fn clearance_rows(sc: &SafetyConfig, base: &[u16; 7], joints: &[u8], reach: f64) -> Vec<JointClearance> {
    joints
        .iter()
        .map(|&id| {
            let j = id as usize - 1;
            let c = safety::sweep_clearance(sc, base, j, reach);
            let room = sc.range_room(j, base[j]);
            let ok = c.is_some_and(|c| c >= sc.margin_m) && reach <= room;
            let safe = if ok { reach } else { safety::safe_amp(sc, base, j, reach).min(room.floor()) };
            JointClearance { id, clearance: c, ok, safe_amp: safe, range_room: room }
        })
        .collect()
}

pub fn clearance_lines(sc: &SafetyConfig, base: &[u16; 7], rows: &[JointClearance], reach: f64) -> Vec<String> {
    let Some(t) = sc.table_z_m else {
        return vec!["safety: table height NOT SET (sysid_safety.toml): runs on the arm are refused. Measure it and use --set-table-z-mm.".into()];
    };
    let mut v = vec![format!(
        "safety: table at {:.0} mm, margin {:.0} mm; clearance at the base pose {:.0} mm",
        t * 1e3,
        sc.margin_m * 1e3,
        safety::clearance(sc, base).unwrap_or(f64::NAN) * 1e3
    )];
    for r in rows {
        v.push(format!(
            "  M{}: sweeping ±{reach:.0} ticks, lowest clearance {:.0} mm  {}",
            r.id,
            r.clearance.unwrap_or(f64::NAN) * 1e3,
            if r.ok {
                "ok".to_string()
            } else if reach > r.range_room {
                format!("PAST THE END OF ITS RANGE ({:.0} ticks to spare): largest safe --max-amp {:.0}", r.range_room, r.safe_amp)
            } else {
                format!("TOO CLOSE: largest safe --max-amp {:.0}", r.safe_amp)
            }
        ));
    }
    v
}

#[derive(Debug, Clone, Default)]
pub struct JointSummary {
    pub id: u8,
    pub samples: usize,
    pub rate_hz: f64,
    pub hold_load: Option<f64>,
    pub hold_current_ma: Option<f64>,
    pub hold_err: Option<f64>,
    pub steps: Vec<(String, StepStats)>,
    pub slops: Vec<f64>,
    pub ramp_peak_ma: Option<f64>,
    pub ramp_max_err: Option<f64>,
    pub stopped: Option<String>,
}

impl JointSummary {
    pub fn slop_mean(&self) -> Option<f64> {
        (!self.slops.is_empty()).then(|| self.slops.iter().sum::<f64>() / self.slops.len() as f64)
    }

    /// Human-readable lines (CLI output).
    pub fn lines(&self) -> Vec<String> {
        let mut v = vec![format!("M{}: {} samples, {:.0} Hz", self.id, self.samples, self.rate_hz)];
        if let (Some(l), Some(c), Some(e)) = (self.hold_load, self.hold_current_ma, self.hold_err) {
            v.push(format!("  hold: load {l:.0} (0.1%), current {c:.0} mA, goal-present {e:+.1} ticks"));
        }
        for (name, st) in &self.steps {
            v.push(format!(
                "  {:<14} dead {:>5.0} ms  t63 {:>5.0} ms  overshoot {:>4.0}%  steady err {:+5.1} ticks",
                name, st.dead_ms, st.t63_ms, st.overshoot_pct, st.steady_err
            ));
        }
        if let Some(m) = self.slop_mean() {
            v.push(format!("  slop {:?} ticks, mean {m:.1}", self.slops.iter().map(|s| (s * 10.0).round() / 10.0).collect::<Vec<_>>()));
        }
        if let (Some(p), Some(e)) = (self.ramp_peak_ma, self.ramp_max_err) {
            v.push(format!("  ramp: peak current {p:.0} mA, max |goal-present| {e:.0} ticks"));
        }
        if let Some(r) = &self.stopped {
            v.push(format!("  stopped early: {r}"));
        }
        v
    }

    /// TOML for meta.toml.
    pub fn toml(&self) -> String {
        let mut out = String::new();
        let id = self.id;
        let _ = writeln!(out, "[summary.M{id}]\nsamples = {}\nrate_hz = {:.0}", self.samples, self.rate_hz);
        if let (Some(l), Some(c), Some(e)) = (self.hold_load, self.hold_current_ma, self.hold_err) {
            let _ = writeln!(out, "hold_load = {l:.1}\nhold_current_ma = {c:.1}\nhold_goal_minus_present = {e:.2}");
        }
        if !self.steps.is_empty() {
            let rows: Vec<String> = self
                .steps
                .iter()
                .map(|(n, st)| {
                    format!(
                        "{{ segment = \"{n}\", amp = {}, dead_ms = {:.1}, t63_ms = {:.1}, overshoot_pct = {:.1}, steady_err = {:.2} }}",
                        st.amp, st.dead_ms, st.t63_ms, st.overshoot_pct, st.steady_err
                    )
                })
                .collect();
            let _ = writeln!(out, "steps = [\n  {}\n]", rows.join(",\n  "));
        }
        if let Some(m) = self.slop_mean() {
            let _ = writeln!(out, "reversal_slop_ticks = {:?}\nreversal_slop_mean = {m:.2}", self.slops);
        }
        if let (Some(p), Some(e)) = (self.ramp_peak_ma, self.ramp_max_err) {
            let _ = writeln!(out, "ramp_peak_current_ma = {p:.1}\nramp_max_goal_minus_present = {e:.1}");
        }
        if let Some(r) = &self.stopped {
            let _ = writeln!(out, "stopped = \"{}\"", r.replace('"', "'"));
        }
        out
    }
}

pub fn summarize(id: u8, segs: &[Segment], samples: &[Sample]) -> JointSummary {
    let dur = samples.last().map_or(0.0, |s| s.t) - samples.first().map_or(0.0, |s| s.t);
    let mut js = JointSummary { id, samples: samples.len(), rate_hz: samples.len() as f64 / dur.max(1e-9), ..Default::default() };
    let of = |si: usize| -> Vec<Sample> { samples.iter().filter(|s| s.seg == si).map(|x| Sample { t: x.t, seg: x.seg, goal: x.goal, st: x.st }).collect() };
    if let Some(hi) = segs.iter().position(|s| s.test == Test::Hold) {
        let h = of(hi);
        if !h.is_empty() {
            let n = h.len() as f64;
            js.hold_load = Some(h.iter().map(|s| s.st.load as f64).sum::<f64>() / n);
            js.hold_current_ma = Some(h.iter().map(|s| s.st.current_ma.abs()).sum::<f64>() / n);
            js.hold_err = Some(h.iter().map(|s| s.goal as f64 - s.st.position as f64).sum::<f64>() / n);
        }
    }
    let mut prev = 0.0;
    for (si, seg) in segs.iter().enumerate() {
        if let Shape::Step { to } = seg.shape {
            if let Some(st) = step_stats(&of(si), prev, to) {
                js.steps.push((seg.name.clone(), st));
            }
            prev = to;
        }
    }
    for (si, seg) in segs.iter().enumerate() {
        if seg.test != Test::Reversals {
            continue;
        }
        let dir = match seg.shape {
            Shape::Ramp { from, to } if from > 0.0 && to < 0.0 => -1.0,
            Shape::Ramp { from, to } if from < 0.0 && to >= 0.0 => 1.0,
            _ => continue,
        };
        if let Some(sl) = reversal_slop(&of(si), dir) {
            js.slops.push(sl);
        }
    }
    let ramp: Vec<&Sample> = samples.iter().filter(|s| segs[s.seg].test == Test::Ramp).collect();
    if !ramp.is_empty() {
        js.ramp_peak_ma = Some(ramp.iter().map(|s| s.st.current_ma.abs()).fold(0.0, f64::max));
        js.ramp_max_err = Some(ramp.iter().map(|s| (s.goal as f64 - s.st.position as f64).abs()).fold(0.0, f64::max));
    }
    js
}

/// Events from a running session.
#[derive(Debug, Clone)]
pub enum Event {
    Status(String),
    JointStart { id: u8, index: usize, of: usize, base: u16 },
    Sample { id: u8, t: f64, seg: String, goal: u16, pos: u16, current: f64, load: i16, progress: f64 },
    JointDone(JointSummary),
    /// SAFETY STOP: joint `id` is frozen in place; waiting for RETURN or LEAVE.
    Frozen { id: u8 },
    Finished { dir: Option<PathBuf>, error: Option<String> },
}

#[derive(Debug, Clone)]
pub struct SessionOpts {
    pub joints: Vec<u8>,
    pub tests: Vec<Test>,
    pub max_amp: f64,
    pub acc: u8,
    pub base: Base,
    pub payload: String,
    pub out: PathBuf,
    pub grip: Option<u16>,
    pub level: Option<[u16; 8]>,
    pub port: String,
    /// A pose (M1-M7 ticks) the Pose screen can go to with `g` (--target).
    pub target: Option<[u16; 7]>,
}

impl SessionOpts {
    /// Safety config for this run: the paddle spheres count only when the payload says the paddle is mounted.
    pub fn safety(&self, sc: &SafetyConfig) -> SafetyConfig {
        let mut s = sc.clone();
        s.with_paddle = self.payload.contains("paddle");
        s
    }

    pub fn level7(&self) -> Option<[u16; 7]> {
        self.level.map(|l| {
            let mut b = [0u16; 7];
            b.copy_from_slice(&l[..7]);
            b
        })
    }
}

/// Runs a whole session: checks (servos answer, safety area along every sweep and the way to the base pose), hold,
/// optional move to the base pose, `confirm`, each joint's tests, back to the start pose, `meta.toml`.
/// Nothing is written to the servos before the checks pass. Returns the output folder.
pub async fn run_session<I: Io>(
    io: &mut I,
    o: &SessionOpts,
    sc: &SafetyConfig,
    stop: &AtomicU8,
    confirm: impl FnOnce(&[u16], &[u16; 7]) -> bool,
    emit: &mut impl FnMut(Event),
) -> Result<Option<PathBuf>> {
    let sc = &o.safety(sc);
    for &j in &o.joints {
        if !(1..=7).contains(&j) {
            bail!("joints must be 1-7 (M8, the gripper, is never excited)");
        }
    }
    if sc.table_z_m.is_none() {
        bail!("table height not set: measure it and set it first (--set-table-z-mm; see README, safety area)");
    }
    let segs = plan(&o.tests, o.max_amp);
    let reach = plan_reach(&segs);
    let per_joint: f64 = segs.iter().map(|s| s.dur).sum::<f64>();

    let mut missing = Vec::new();
    for id in 1..=8u8 {
        if !io.ping(id).await {
            missing.push(id);
        }
    }
    if !missing.is_empty() {
        bail!("servo(s) {missing:?} do not answer on {}: wrong port, or arm not powered", o.port);
    }
    // Read-only first: check the safety area before anything is written to the servos.
    let peek = io.read_positions(&[1, 2, 3, 4, 5, 6, 7]).await?;
    let mut peek7 = [0u16; 7];
    peek7.copy_from_slice(&peek);
    let base7 = match (o.base, o.level7()) {
        (Base::Level, Some(l)) => l,
        (Base::Level, None) => bail!("no level pose in the config"),
        _ => peek7,
    };
    let rows = clearance_rows(sc, &base7, &o.joints, reach);
    for l in clearance_lines(sc, &base7, &rows, reach) {
        emit(Event::Status(l));
    }
    let bad: Vec<&JointClearance> = rows.iter().filter(|r| !r.ok).collect();
    if !bad.is_empty() {
        bail!(
            "refusing to move: {} would come within {:.0} mm of the table or leave its calibrated range. Use a smaller amplitude, other joints, or another base pose.",
            bad.iter().map(|r| format!("M{} (safe ≤ {:.0} ticks)", r.id, r.safe_amp)).collect::<Vec<_>>().join(", "),
            sc.margin_m * 1e3
        );
    }
    if base7 != peek7 {
        let worst = safety::path_clearance(sc, &peek7, &base7).unwrap_or(f64::NEG_INFINITY);
        if worst < sc.margin_m {
            bail!("refusing to move: the way to the base pose comes within {:.0} mm of the table", worst * 1e3);
        }
    }
    if !confirm(&peek, &base7) {
        emit(Event::Status("cancelled".into()));
        return Ok(None);
    }
    let present = io.hold_present(o.grip).await?;
    let mut now7 = [0u16; 7];
    now7.copy_from_slice(&present[..7]);
    if base7 != now7 {
        emit(Event::Status("moving to the base pose...".into()));
        move_all(io, now7, base7, 3.0).await?;
    }

    let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)?.as_secs();
    let tag = if io.kind() == "sim" { format!("{}_sim", o.payload) } else { o.payload.clone() };
    let dir = o.out.join(format!("{stamp}_{}", tag.replace(['/', ' '], "_")));
    std::fs::create_dir_all(&dir).with_context(|| format!("creating {}", dir.display()))?;
    let mut meta = String::new();
    let _ = writeln!(meta, "# elrobot-sysid run{}\nstamp = {stamp}\npayload = \"{}\"\nport = \"{}\"", if io.kind() == "sim" { " (SIMULATED servos)" } else { "" }, o.payload, o.port);
    let _ = writeln!(meta, "base_ticks = {:?}\npresent_at_start = {:?}\nmax_amp_ticks = {}\nacc = {}", base7, present, o.max_amp, o.acc);
    let _ = writeln!(meta, "joints = {:?}\ntests = {:?}\ngrip = {:?}\n", o.joints, o.tests, o.grip);
    let _ = writeln!(
        meta,
        "[safety]\ntable_z_m = {:?}\nmargin_m = {}\nwith_paddle = {}\nspheres = {:?}\n",
        sc.table_z_m,
        sc.margin_m,
        sc.with_paddle,
        sc.spheres().collect::<Vec<_>>()
    );
    let mut blocked_load = [900i32; 8];
    for id in 1..=8u8 {
        match read_servo_info(io, id).await {
            Ok(info) => {
                info.describe(&mut meta, id);
                blocked_load[id as usize - 1] = info.duty_cap() * 9 / 10;
            }
            Err(e) => emit(Event::Status(format!("M{id}: could not read settings: {e:#}"))),
        }
    }

    let mut abort: Option<String> = None;
    for (index, &id) in o.joints.iter().enumerate() {
        let j = id as usize - 1;
        let base = base7[j];
        let old_acc = io.read_bytes(id, 0x29, 1).await?.first().copied().unwrap_or(0);
        io.write_acc(id, o.acc).await?;
        emit(Event::JointStart { id, index, of: o.joints.len(), base });
        let mut csv = std::io::BufWriter::new(std::fs::File::create(dir.join(format!("M{id}.csv")))?);
        emit(Event::Status(format!("M{id}: blocked-joint stop at |load| >= {} (0.1 %) for 0.3 s", blocked_load[j])));
        let (samples, reason) =
            run_joint(io, id, base, &base7, sc, &segs, stop, &mut csv, emit, per_joint, blocked_load[j]).await?;
        csv.flush()?;
        if stop.load(Ordering::Relaxed) == ctl::FREEZE {
            emit(Event::Frozen { id });
            // Nothing moves until the user decides: return slowly, or leave the arm here.
            while stop.load(Ordering::Relaxed) == ctl::FREEZE {
                tokio::time::sleep(Duration::from_millis(50)).await;
            }
        }
        let leave = stop.load(Ordering::Relaxed) == ctl::LEAVE;
        if !leave {
            let at = io.read_state(id).await.map(|s| s.position).unwrap_or(base);
            move_joint(io, id, at, base, if reason.is_some() { 2.0 } else { 0.8 }).await?;
        }
        io.write_acc(id, old_acc).await?;
        let mut js = summarize(id, &segs, &samples);
        js.stopped = reason.clone();
        meta.push_str(&js.toml());
        meta.push('\n');
        emit(Event::JointDone(js));
        if let Some(r) = reason {
            abort = Some(r);
            break;
        }
    }
    if base7 != now7 && stop.load(Ordering::Relaxed) != ctl::LEAVE {
        emit(Event::Status("returning to the start pose...".into()));
        move_all(io, base7, now7, 3.0).await?;
    }
    std::fs::write(dir.join("meta.toml"), &meta)?;
    if let Some(r) = abort {
        bail!("stopped early: {r} (logs: {})", dir.display());
    }
    Ok(Some(dir))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn duty_cap_is_the_smaller_of_max_torque_and_torque_limit() {
        let info = |max: u16, limit: u16| {
            let mut eeprom = vec![0u8; 0x28];
            eeprom[0x10..0x12].copy_from_slice(&max.to_le_bytes());
            let mut ram = vec![0u8; 10];
            ram[8..10].copy_from_slice(&limit.to_le_bytes());
            ServoInfo { eeprom, ram }
        };
        assert_eq!(info(500, 500).duty_cap(), 500, "the follower: a stall saturates at 50 %");
        assert_eq!(info(1000, 300).duty_cap(), 300);
        assert_eq!(info(1000, 0).duty_cap(), 1000, "no RAM limit read: the EEPROM value");
        assert_eq!(info(0, 0).duty_cap(), 1000, "nonsense: fall back to full scale");
    }
    use crate::bus::MotorState;

    /// SAFETY STOP on simulated servos: the joint freezes where it is and stays there until the user decides;
    /// "return" then brings it back to base.
    #[test]
    fn safety_stop_freezes_then_returns() {
        use crate::io::{Io, SimIo};
        use std::sync::Arc;
        let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
        rt.block_on(async {
            let level = [2003u16, 1002, 1942, 3185, 1864, 1892, 2226, 1436];
            let mut sc = SafetyConfig::default();
            let l7 = [level[0], level[1], level[2], level[3], level[4], level[5], level[6]];
            sc.table_z_m = Some(safety::lowest_z(&sc, &sc.angles(&l7)) - 0.3);
            let out = std::env::temp_dir().join(format!("sysid-stop-test-{}", std::process::id()));
            let o = SessionOpts {
                joints: vec![6],
                tests: vec![Test::Steps],
                max_amp: 100.0,
                acc: 0,
                base: Base::Level,
                payload: "none".into(),
                out: out.clone(),
                grip: None,
                level: Some(level),
                port: "sim".into(),
                target: None,
            };
            let stop = Arc::new(AtomicU8::new(ctl::RUN));
            let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
            let (stop2, o2, sc2) = (stop.clone(), o.clone(), sc.clone());
            let task = tokio::spawn(async move {
                let mut io = SimIo::new(level);
                let mut emit = |e: Event| {
                    let _ = tx.send(e);
                };
                let r = run_session(&mut io, &o2, &sc2, &stop2, |_, _| true, &mut emit).await;
                (r.map_err(|e| e.to_string()), io.read_state(6).await.unwrap().position)
            });
            // Let it run into the first steps, then press SAFETY STOP.
            tokio::time::sleep(Duration::from_millis(2600)).await;
            stop.store(ctl::FREEZE, Ordering::Relaxed);
            let mut frozen = false;
            let t0 = Instant::now();
            while t0.elapsed() < Duration::from_millis(1500) {
                while let Ok(e) = rx.try_recv() {
                    match e {
                        Event::Frozen { id } => {
                            assert_eq!(id, 6);
                            frozen = true;
                        }
                        Event::Sample { .. } => assert!(!frozen, "no test samples after the freeze"),
                        _ => {}
                    }
                }
                tokio::time::sleep(Duration::from_millis(20)).await;
            }
            assert!(frozen, "the session reported the freeze");
            // While frozen, nothing moves: the session is waiting, not driving the joint.
            assert!(!task.is_finished(), "waits for the user's decision");
            stop.store(ctl::RETURN, Ordering::Relaxed);
            let (res, end) = task.await.unwrap();
            assert!(res.unwrap_err().contains("SAFETY STOP"), "the run ends as stopped");
            assert!((end as i32 - level[5] as i32).abs() <= 12, "back near base after 'return': {end}");
            let _ = std::fs::remove_dir_all(out);
        });
    }

    #[test]
    fn safety_stop_holds_position_while_frozen() {
        use crate::io::{Io, SimIo};
        let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
        rt.block_on(async {
            let mut io = SimIo::new([2048; 8]);
            io.write_goals(&[(6, 2148)]).await.unwrap();
            tokio::time::sleep(Duration::from_millis(80)).await;
            // What the freeze does: goal := present.
            let here = io.read_state(6).await.unwrap().position;
            io.write_goals(&[(6, here)]).await.unwrap();
            // A moving joint coasts for one transport delay (~30 ms) and settles at the edge of its dead zone
            // around the freeze point (observed: 2096 -> 2110 -> 2106 for M6). After that it must not move at all.
            tokio::time::sleep(Duration::from_millis(400)).await;
            let a = io.read_state(6).await.unwrap().position as i32;
            tokio::time::sleep(Duration::from_millis(800)).await;
            let b = io.read_state(6).await.unwrap().position as i32;
            assert!((a - b).abs() <= 1, "frozen joint does not keep moving: {a} -> {b}");
            assert!((b - here as i32).abs() <= 20, "coast + dead zone stay small: {here} -> {b}");
        });
    }

    #[test]
    fn plan_respects_max_amp() {
        let segs = plan(&[Test::Steps, Test::Reversals, Test::Ramp, Test::Chirp], 30.0);
        for s in &segs {
            for i in 0..=100 {
                let o = s.offset(s.dur * i as f64 / 100.0);
                assert!(o.abs() <= 30.0 + 1e-9, "{} gives {o}", s.name);
            }
        }
        assert!(!segs.iter().any(|s| s.name.contains("50")));
    }

    #[test]
    fn chirp_fades_in_and_out() {
        let s = Segment { name: "c".into(), test: Test::Chirp, dur: 25.0, shape: Shape::Chirp { amp: 20.0, f0: 0.2, f1: 4.0 } };
        assert!(s.offset(0.0).abs() < 1e-9);
        assert!(s.offset(25.0).abs() < 1e-9);
        let peak = (0..2500).map(|i| s.offset(i as f64 / 100.0).abs()).fold(0.0, f64::max);
        assert!(peak > 19.0 && peak <= 20.0);
    }

    fn mk(points: &[(f64, u16, u16)]) -> Vec<Sample> {
        points
            .iter()
            .map(|&(t, goal, pos)| Sample { t, seg: 0, goal, st: MotorState { position: pos, ..Default::default() } })
            .collect()
    }

    #[test]
    fn step_stats_on_a_first_order_response() {
        // 50-tick step, 40 ms dead time, then tau = 100 ms.
        let pts: Vec<(f64, u16, u16)> = (0..200)
            .map(|i| {
                let t = i as f64 * 0.005;
                let x = if t < 0.04 { 0.0 } else { 50.0 * (1.0 - (-(t - 0.04) / 0.1).exp()) };
                (t, 2050, 2000 + x.round() as u16)
            })
            .collect();
        let s = step_stats(&mk(&pts), 0.0, 50.0).unwrap();
        assert!((s.dead_ms - 45.0).abs() < 10.0, "{s:?}");
        assert!((s.t63_ms - 140.0).abs() < 15.0, "{s:?}");
        assert!(s.overshoot_pct < 1.0);
    }

    #[test]
    fn reversal_slop_detects_dead_zone() {
        // Goal ramps down from 2020 at 1 tick per sample; position stays at 2020 for 8 ticks of goal travel.
        let pts: Vec<(f64, u16, u16)> = (0..30)
            .map(|i| {
                let goal = 2020 - i as u16;
                let pos = if i < 8 { 2020 } else { 2020 - (i as u16 - 8) };
                (i as f64 * 0.01, goal, pos)
            })
            .collect();
        let slop = reversal_slop(&mk(&pts), -1.0).unwrap();
        assert!((slop - 10.0).abs() <= 1.0, "{slop}");
    }
}
