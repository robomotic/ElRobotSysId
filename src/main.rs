//! elrobot-sysid: per-joint system identification of the ElRobot follower's ST3215 servos, for the MuJoCo model
//! (RoboSim #69: sim2real2sim).
//!
//! For each chosen joint, around a base pose, it plays a fixed set of test signals on that one joint while every
//! other joint holds, and logs goal / position / speed / load / current at the full bus rate (several hundred Hz,
//! direct port access; stop the station first):
//!
//! | test       | signal                                               | identifies                                   |
//! |------------|------------------------------------------------------|----------------------------------------------|
//! | hold       | 2 s still                                            | static load / current at the pose (gravity)  |
//! | steps      | +a, 0, -a, 0 for a = 10, 25, 50, 100 ticks           | dead time, lag, kp, damping, overshoot       |
//! | reversals  | slow triangles of +-5, 10, 20 ticks                  | slop (backlash / dead zone)                  |
//! | ramp       | slow ramp out and back at 15 ticks/s                 | friction (breakaway current), compliance     |
//! | chirp      | 20 ticks, log sweep 0.2 -> 4 Hz over 25 s            | frequency response (bandwidth, phase lag)    |
//!
//! Output: `<out>/<stamp>_<payload>/` with `meta.toml` (servo EEPROM + RAM settings per joint, base pose, arguments,
//! summary) and one `M<n>.csv` per joint. Ctrl-C (or a safety stop) returns the joint to the base pose.
//!
//!   sudo systemctl stop normacore-station
//!   elrobot-sysid --joints 5,6,7 --payload paddle --dry-run     # show the plan, no motion
//!   elrobot-sysid --joints 5,6,7 --payload paddle               # asks for confirmation, then runs

use anyhow::{bail, Context, Result};
use clap::{Parser, ValueEnum};
mod bus;
mod settings;

use bus::{Bus, MotorState};
use settings::Settings;
use st3215::protocol::RamRegister;
use std::fmt::Write as _;
use std::io::Write as _;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum Test {
    Hold,
    Steps,
    Reversals,
    Ramp,
    Chirp,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum Base {
    /// Where the arm is now (no move before the tests).
    Current,
    /// The `level` pose from the paddle.toml (min-jerk move there first).
    Level,
}

#[derive(Parser, Debug)]
#[command(version, about = "Per-joint ST3215 system identification for the ElRobot follower")]
struct Cli {
    /// Optional paddle-balance `paddle.toml` (PingPongSimBot): port, level pose and grip are taken from it.
    #[arg(short, long, default_value = "paddle.toml")]
    config: PathBuf,
    /// Serial port of the follower bus (overrides the config).
    #[arg(long)]
    port: Option<String>,
    /// Joints to identify (1-7). M8 (gripper) is never excited.
    #[arg(long, value_delimiter = ',', default_values_t = [5u8, 6, 7])]
    joints: Vec<u8>,
    #[arg(long, value_enum, default_value_t = Base::Current)]
    base: Base,
    /// Largest excursion from the base pose, ticks (4096 per turn). Hard cap 200.
    #[arg(long, default_value_t = 100)]
    max_amp: u16,
    /// Servo acceleration register during the tests (0 = servo maximum). Restored afterwards.
    #[arg(long, default_value_t = 0)]
    acc: u8,
    /// Label for what the arm carries, e.g. paddle, none, paddle+ball. Goes into the folder name and meta.
    #[arg(long, default_value = "paddle")]
    payload: String,
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = [Test::Hold, Test::Steps, Test::Reversals, Test::Ramp, Test::Chirp])]
    tests: Vec<Test>,
    #[arg(long, default_value = "sysid_logs")]
    out: PathBuf,
    /// Print the plan and its duration, move nothing.
    #[arg(long)]
    dry_run: bool,
    /// Skip the confirmation prompt.
    #[arg(long)]
    yes: bool,
}

/// Minimum-jerk interpolation factor for tau in [0, 1].
fn min_jerk(tau: f64) -> f64 {
    let t = tau.clamp(0.0, 1.0);
    t * t * t * (10.0 - 15.0 * t + 6.0 * t * t)
}

// ---------------------------------------------------------------------------------------------------------------
// Test signals: offsets (ticks) from the base position as a function of time within a segment.

#[derive(Debug, Clone)]
enum Shape {
    Hold { at: f64 },
    /// Jump to `to` at once and stay.
    Step { to: f64 },
    Ramp { from: f64, to: f64 },
    /// Log sweep f0 -> f1 Hz with a 1 s cosine fade in and out.
    Chirp { amp: f64, f0: f64, f1: f64 },
}

#[derive(Debug, Clone)]
struct Segment {
    name: String,
    test: Test,
    dur: f64,
    shape: Shape,
}

impl Segment {
    fn offset(&self, t: f64) -> f64 {
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

fn plan(tests: &[Test], max_amp: f64) -> Vec<Segment> {
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

struct Sample {
    t: f64,
    seg: usize,
    goal: u16,
    st: MotorState,
}

#[derive(Debug, Default)]
struct StepStats {
    amp: f64,
    dead_ms: f64,
    t63_ms: f64,
    overshoot_pct: f64,
    steady_err: f64,
}

/// Dead time (first 3-tick move), 63% time, overshoot and final error for one step segment.
fn step_stats(samples: &[Sample], from: f64, to: f64) -> Option<StepStats> {
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
fn reversal_slop(samples: &[Sample], dir: f64) -> Option<f64> {
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

struct ServoInfo {
    eeprom: Vec<u8>,
    ram: Vec<u8>,
}

impl ServoInfo {
    fn u16(&self, a: usize) -> u16 {
        u16::from_le_bytes([self.eeprom[a], self.eeprom[a + 1]])
    }

    fn describe(&self, out: &mut String, id: u8) {
        let e = &self.eeprom;
        let r = &self.ram; // from 0x28
        let _ = writeln!(out, "[servo.M{id}]");
        let _ = writeln!(out, "model = {}", self.u16(0x00));
        let _ = writeln!(out, "firmware = {}", e[0x02]);
        let _ = writeln!(out, "min_angle_limit = {}", self.u16(0x09));
        let _ = writeln!(out, "max_angle_limit = {}", self.u16(0x0B));
        let _ = writeln!(out, "max_torque = {}  # 0.1% units", self.u16(0x10));
        let _ = writeln!(out, "p_coef = {}", e[0x15]);
        let _ = writeln!(out, "d_coef = {}", e[0x16]);
        let _ = writeln!(out, "i_coef = {}", e[0x17]);
        let _ = writeln!(out, "min_startup_force = {}", self.u16(0x18));
        let _ = writeln!(out, "cw_dead_zone = {}", e[0x1A]);
        let _ = writeln!(out, "ccw_dead_zone = {}", e[0x1B]);
        let _ = writeln!(out, "protection_current = {}", self.u16(0x1C));
        let _ = writeln!(out, "offset = {}", self.u16(0x1F));
        let _ = writeln!(out, "mode = {}", e[0x21]);
        let _ = writeln!(out, "overload_torque = {}", e[0x24]);
        let _ = writeln!(out, "torque_enable = {}", r[0]);
        let _ = writeln!(out, "acc = {}", r[1]);
        let _ = writeln!(out, "goal_speed = {}", u16::from_le_bytes([r[6], r[7]]));
        let _ = writeln!(out, "torque_limit = {}", u16::from_le_bytes([r[8], r[9]]));
        let hex = |b: &[u8]| b.iter().map(|x| format!("{x:02x}")).collect::<String>();
        let _ = writeln!(out, "eeprom_hex = \"{}\"", hex(e));
        let _ = writeln!(out, "ram_0x28_hex = \"{}\"\n", hex(r));
    }
}

async fn read_servo_info(bus: &mut Bus, id: u8) -> Result<ServoInfo> {
    let eeprom = bus.read_bytes(id, 0x00, 0x28).await?.to_vec();
    let ram = bus.read_bytes(id, 0x28, 10).await?.to_vec();
    Ok(ServoInfo { eeprom, ram })
}

// ---------------------------------------------------------------------------------------------------------------

async fn move_all(bus: &mut Bus, from: [u16; 7], to: [u16; 7], secs: f64) -> Result<()> {
    let t0 = Instant::now();
    let mut tick = tokio::time::interval(Duration::from_millis(20));
    loop {
        tick.tick().await;
        let s = min_jerk(t0.elapsed().as_secs_f64() / secs.max(0.05));
        let goals: Vec<(u8, u16)> = (0..7)
            .map(|j| (j as u8 + 1, (from[j] as f64 + (to[j] as f64 - from[j] as f64) * s).round() as u16))
            .collect();
        bus.sync_write_goals(&goals).await?;
        if s >= 1.0 {
            return Ok(());
        }
    }
}

/// Smoothly bring one joint from `from` to `to`.
async fn move_joint(bus: &mut Bus, id: u8, from: u16, to: u16, secs: f64) -> Result<()> {
    let t0 = Instant::now();
    let mut tick = tokio::time::interval(Duration::from_millis(20));
    loop {
        tick.tick().await;
        let s = min_jerk(t0.elapsed().as_secs_f64() / secs.max(0.05));
        let g = (from as f64 + (to as f64 - from as f64) * s).round() as u16;
        bus.sync_write_goals(&[(id, g)]).await?;
        if s >= 1.0 {
            return Ok(());
        }
    }
}

/// Runs the plan on one joint. Returns the samples, and Some(reason) if it stopped early.
async fn run_joint(
    bus: &mut Bus,
    id: u8,
    base: u16,
    segs: &[Segment],
    stop: &AtomicBool,
    csv: &mut impl std::io::Write,
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
            if stop.load(Ordering::Relaxed) {
                return Ok((samples, Some("Ctrl-C".into())));
            }
            let ts = t0.elapsed().as_secs_f64();
            if ts >= seg.dur {
                break;
            }
            let goal = (base as f64 + seg.offset(ts)).round().clamp(0.0, 4095.0) as u16;
            if goal != last_goal {
                bus.sync_write_goals(&[(id, goal)]).await?;
                last_goal = goal;
            }
            let st = match bus.read_state(id).await {
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
            writeln!(
                csv,
                "{t:.5},{},{:?},{goal},{},{},{},{:.1},{},{}",
                seg.name, seg.test, st.position, st.speed, st.load, st.current_ma, st.voltage_dv, st.temperature_c
            )?;
            // Safety: sustained high load (blocked joint) or the joint far from its goal (collision, slip).
            if (st.load as i32).abs() >= 900 {
                let since = *overload_since.get_or_insert(t);
                if t - since > 0.3 {
                    return Ok((samples, Some(format!("load {} for 0.3 s in {}", st.load, seg.name))));
                }
            } else {
                overload_since = None;
            }
            if (st.position as i32 - goal as i32).abs() > 300 {
                return Ok((samples, Some(format!("position {} is >300 ticks from goal {goal} in {}", st.position, seg.name))));
            }
            samples.push(Sample { t, seg: si, goal, st });
        }
    }
    Ok((samples, None))
}

fn summarize(id: u8, segs: &[Segment], samples: &[Sample], out: &mut String) {
    let dur = samples.last().map_or(0.0, |s| s.t) - samples.first().map_or(0.0, |s| s.t);
    let rate = samples.len() as f64 / dur.max(1e-9);
    let _ = writeln!(out, "[summary.M{id}]\nsamples = {}\nrate_hz = {rate:.0}", samples.len());
    println!("\nM{id}: {} samples, {rate:.0} Hz", samples.len());

    let of = |si: usize| -> Vec<&Sample> { samples.iter().filter(|s| s.seg == si).collect() };
    // Hold: static load.
    if let Some(hi) = segs.iter().position(|s| s.test == Test::Hold) {
        let h = of(hi);
        if !h.is_empty() {
            let n = h.len() as f64;
            let load = h.iter().map(|s| s.st.load as f64).sum::<f64>() / n;
            let cur = h.iter().map(|s| s.st.current_ma).sum::<f64>() / n;
            let err = h.iter().map(|s| s.goal as f64 - s.st.position as f64).sum::<f64>() / n;
            println!("  hold: load {load:.0} (0.1%), current {cur:.0} mA, goal-present {err:+.1} ticks");
            let _ = writeln!(out, "hold_load = {load:.1}\nhold_current_ma = {cur:.1}\nhold_goal_minus_present = {err:.2}");
        }
    }
    // Steps.
    let mut rows = Vec::new();
    let mut prev_target = 0.0;
    for (si, seg) in segs.iter().enumerate() {
        if let Shape::Step { to } = seg.shape {
            let s: Vec<Sample> = of(si).into_iter().map(|x| Sample { t: x.t, seg: x.seg, goal: x.goal, st: x.st }).collect();
            if let Some(st) = step_stats(&s, prev_target, to) {
                println!(
                    "  {:<14} dead {:>5.0} ms  t63 {:>5.0} ms  overshoot {:>4.0}%  steady err {:+5.1} ticks",
                    seg.name, st.dead_ms, st.t63_ms, st.overshoot_pct, st.steady_err
                );
                rows.push(format!(
                    "{{ segment = \"{}\", amp = {}, dead_ms = {:.1}, t63_ms = {:.1}, overshoot_pct = {:.1}, steady_err = {:.2} }}",
                    seg.name, st.amp, st.dead_ms, st.t63_ms, st.overshoot_pct, st.steady_err
                ));
            }
            prev_target = to;
        }
    }
    if !rows.is_empty() {
        let _ = writeln!(out, "steps = [\n  {}\n]", rows.join(",\n  "));
    }
    // Reversals: slop at the top (turning down) and at the bottom (turning up).
    let mut slops = Vec::new();
    for (si, seg) in segs.iter().enumerate() {
        if seg.test != Test::Reversals {
            continue;
        }
        let dir = match seg.shape {
            Shape::Ramp { from, to } if from > 0.0 && to < 0.0 => -1.0,
            Shape::Ramp { from, to } if from < 0.0 && to >= 0.0 => 1.0,
            _ => continue,
        };
        let s: Vec<Sample> = of(si).into_iter().map(|x| Sample { t: x.t, seg: x.seg, goal: x.goal, st: x.st }).collect();
        if let Some(sl) = reversal_slop(&s, dir) {
            slops.push(sl);
            println!("  {:<14} slop {:>4.1} ticks", seg.name, sl);
        }
    }
    if !slops.is_empty() {
        let mean = slops.iter().sum::<f64>() / slops.len() as f64;
        println!("  slop mean {mean:.1} ticks");
        let _ = writeln!(out, "reversal_slop_ticks = {slops:?}\nreversal_slop_mean = {mean:.2}");
    }
    // Ramp: peak current while creeping (friction + gravity).
    let ramp: Vec<&Sample> = samples.iter().filter(|s| segs[s.seg].test == Test::Ramp).collect();
    if !ramp.is_empty() {
        let peak = ramp.iter().map(|s| s.st.current_ma).fold(0.0, f64::max);
        let lag = ramp.iter().map(|s| (s.goal as f64 - s.st.position as f64).abs()).fold(0.0, f64::max);
        println!("  ramp: peak current {peak:.0} mA, max |goal-present| {lag:.0} ticks");
        let _ = writeln!(out, "ramp_peak_current_ma = {peak:.1}\nramp_max_goal_minus_present = {lag:.1}");
    }
    let _ = writeln!(out);
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let cli = Cli::parse();
    let cfg = Settings::load(Some(&cli.config), cli.port.clone())?;
    for &j in &cli.joints {
        if !(1..=7).contains(&j) {
            bail!("joints must be 1-7 (M8, the gripper, is never excited)");
        }
    }
    let max_amp = cli.max_amp.min(200) as f64;
    let segs = plan(&cli.tests, max_amp);
    let per_joint: f64 = segs.iter().map(|s| s.dur).sum::<f64>() + 2.0;
    println!(
        "plan: {} segments, {:.0} s per joint, {} joint(s) (M{}) -> about {:.1} min; max excursion {} ticks ({:.1} deg); acc {}; payload '{}'",
        segs.len(),
        per_joint,
        cli.joints.len(),
        cli.joints.iter().map(|j| j.to_string()).collect::<Vec<_>>().join(", M"),
        per_joint * cli.joints.len() as f64 / 60.0,
        max_amp,
        max_amp * 360.0 / 4096.0,
        cli.acc,
        cli.payload
    );
    for s in &segs {
        println!("  {:<10?} {:<16} {:>5.1} s", s.test, s.name, s.dur);
    }
    if cli.dry_run {
        return Ok(());
    }

    let mut bus = Bus::open(&cfg.port, cfg.timeout_ms)?;
    let missing: Vec<u8> = {
        let mut m = Vec::new();
        for id in 1..=8u8 {
            if !bus.ping(id).await {
                m.push(id);
            }
        }
        m
    };
    if !missing.is_empty() {
        bail!("servo(s) {missing:?} do not answer on {}: wrong port, or arm not powered", cfg.port);
    }
    let present = bus.hold_present(cfg.grip).await?;
    let mut now7 = [0u16; 7];
    now7.copy_from_slice(&present[..7]);
    let base7 = match cli.base {
        Base::Current => now7,
        Base::Level => {
            let l = cfg.level()?;
            let mut b = [0u16; 7];
            b.copy_from_slice(&l[..7]);
            b
        }
    };
    println!("present {:?}\nbase    {:?}", &present[..7], base7);
    if !cli.yes {
        print!("The listed joints will move up to {max_amp} ticks around the base, one at a time. Start? [y/N] ");
        std::io::stdout().flush()?;
        let mut l = String::new();
        std::io::stdin().read_line(&mut l)?;
        if !l.trim().eq_ignore_ascii_case("y") {
            println!("aborted");
            return Ok(());
        }
    }
    if base7 != now7 {
        println!("moving to base...");
        move_all(&mut bus, now7, base7, 3.0).await?;
    }

    let stop = Arc::new(AtomicBool::new(false));
    {
        let stop = stop.clone();
        tokio::spawn(async move {
            if tokio::signal::ctrl_c().await.is_ok() {
                stop.store(true, Ordering::Relaxed);
            }
        });
    }

    let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)?.as_secs();
    let dir = cli.out.join(format!("{stamp}_{}", cli.payload.replace(['/', ' '], "_")));
    std::fs::create_dir_all(&dir).with_context(|| format!("creating {}", dir.display()))?;
    let mut meta = String::new();
    let _ = writeln!(meta, "# elrobot-sysid run\nstamp = {stamp}\npayload = \"{}\"\nport = \"{}\"", cli.payload, cfg.port);
    let _ = writeln!(meta, "base_ticks = {:?}\npresent_at_start = {:?}\nmax_amp_ticks = {max_amp}\nacc = {}", base7, present, cli.acc);
    let _ = writeln!(meta, "joints = {:?}\ntests = {:?}\ngrip = {:?}\n", cli.joints, cli.tests, cfg.grip);

    // Settings of every servo (the model needs the neighbours too).
    for id in 1..=8u8 {
        match read_servo_info(&mut bus, id).await {
            Ok(info) => info.describe(&mut meta, id),
            Err(e) => log::warn!("M{id}: could not read settings: {e:#}"),
        }
    }

    let mut abort: Option<String> = None;
    for &id in &cli.joints {
        let j = id as usize - 1;
        let base = base7[j];
        let old_acc = bus.read_bytes(id, RamRegister::Acc.address(), 1).await?[0];
        bus.sync_write_u8(RamRegister::Acc, &[(id, cli.acc)]).await?;
        println!("\nM{id}: base {base}, running {:.0} s...", per_joint);
        let path = dir.join(format!("M{id}.csv"));
        let mut csv = std::io::BufWriter::new(std::fs::File::create(&path)?);
        let (samples, reason) = run_joint(&mut bus, id, base, &segs, &stop, &mut csv).await?;
        csv.flush()?;
        // Back to base, gently, whatever happened.
        let at = bus.read_state(id).await.map(|s| s.position).unwrap_or(base);
        move_joint(&mut bus, id, at, base, 0.8).await?;
        bus.sync_write_u8(RamRegister::Acc, &[(id, old_acc)]).await?;
        summarize(id, &segs, &samples, &mut meta);
        if let Some(r) = reason {
            println!("M{id} stopped early: {r}");
            let _ = writeln!(meta, "[stopped.M{id}]\nreason = \"{}\"\n", r.replace('"', "'"));
            abort = Some(r);
            break;
        }
    }

    if base7 != now7 {
        println!("returning to the start pose...");
        move_all(&mut bus, base7, now7, 3.0).await?;
    }
    std::fs::write(dir.join("meta.toml"), &meta)?;
    println!("\nlogs: {}", dir.display());
    if let Some(r) = abort {
        bail!("stopped early: {r}");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

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
