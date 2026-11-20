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
//!   elrobot-sysid                      # full-screen app (real arm); `elrobot-sysid --sim` without the arm
//!   elrobot-sysid --dry-run            # plan + safety report, no motion
//!   elrobot-sysid --joints 5,6,7 --payload paddle --cli    # line-based run (asks for confirmation)

mod bus;
mod io;
mod safety;
mod session;
mod settings;
mod tui;

use anyhow::{Context, Result};
use clap::Parser;
use session::{Base, Event, SessionOpts, Test};
use settings::Settings;
use std::io::Write as _;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;

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
    /// What the arm carries: none (normal for system ID), paddle, paddle+ball. Goes into the folder name and meta;
    /// "paddle…" also adds the paddle to the table safety model.
    #[arg(long, default_value = "none")]
    payload: String,
    #[arg(long, value_enum, value_delimiter = ',', default_values_t = [Test::Hold, Test::Steps, Test::Reversals, Test::Ramp, Test::Chirp])]
    tests: Vec<Test>,
    #[arg(long, default_value = "sysid_logs")]
    out: PathBuf,
    /// Print the plan and the safety report, move nothing.
    #[arg(long)]
    dry_run: bool,
    /// Run line-based on the real arm instead of opening the full-screen app.
    #[arg(long)]
    cli: bool,
    /// Full-screen app with SIMULATED servos: nothing moves, no hardware needed.
    #[arg(long)]
    sim: bool,
    /// Skip the confirmation prompt (line-based run).
    #[arg(long)]
    yes: bool,
    /// Table safety area (issue #1): table height, margin, tool spheres. Created by --set-table-z-mm.
    #[arg(long, default_value = "sysid_safety.toml")]
    safety: PathBuf,
    /// Record the MEASURED height of the table plane in the robot base frame (mm; about 0 if the base stands
    /// directly on the table, negative if it stands on a plate), then exit. Runs are refused until this is set.
    #[arg(long, allow_hyphen_values = true)]
    set_table_z_mm: Option<f64>,
    /// Read-only: print the modelled height of every joint and tool sphere at the current pose, and the clearance.
    /// Compare with a ruler to check the model. Nothing is written to the servos.
    #[arg(long)]
    show_clearance: bool,
}

pub fn load_safety(path: &std::path::Path) -> Result<safety::SafetyConfig> {
    if !path.exists() {
        return Ok(safety::SafetyConfig::default());
    }
    let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    toml::from_str(&text).with_context(|| format!("parsing {}", path.display()))
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    let cli = Cli::parse();
    let cfg = Settings::load(Some(&cli.config), cli.port.clone())?;
    let mut sc = load_safety(&cli.safety)?;
    let opts = SessionOpts {
        joints: cli.joints.clone(),
        tests: cli.tests.clone(),
        max_amp: cli.max_amp.min(200) as f64,
        acc: cli.acc,
        base: cli.base,
        payload: cli.payload.clone(),
        out: cli.out.clone(),
        grip: cfg.grip,
        level: cfg.level,
        port: cfg.port.clone(),
    };
    let line_mode = cli.cli || cli.dry_run || cli.set_table_z_mm.is_some() || cli.show_clearance;
    if !line_mode {
        // Full-screen app (default). Log lines would tear it, so no logger here.
        return tui::run(cfg, sc, cli.safety.clone(), opts, cli.sim).await;
    }
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    if let Some(z) = cli.set_table_z_mm {
        sc.table_z_m = Some(z / 1e3);
        std::fs::write(&cli.safety, toml::to_string_pretty(&sc)?)?;
        println!("table height {z:.0} mm saved to {} (margin {:.0} mm)", cli.safety.display(), sc.margin_m * 1e3);
        return Ok(());
    }
    let sc_run = opts.safety(&sc);
    if cli.show_clearance {
        let sc = &sc_run;
        let mut bus = bus::Bus::open(&cfg.port, cfg.timeout_ms)?;
        let p = bus.read_positions(&[1, 2, 3, 4, 5, 6, 7]).await?;
        let q = sc.angles(&p);
        let (joints, r, g) = safety::fk(&q);
        println!("present {p:?}");
        for (j, pt) in joints.iter().enumerate() {
            println!("  M{} joint  z {:>6.0} mm", j + 1, pt.z * 1e3);
        }
        println!("  (payload '{}': paddle {}in the model)", opts.payload, if sc.with_paddle { "" } else { "not " });
        for (c, rad) in sc.spheres() {
            let pt = g + r * nalgebra::Vector3::from(*c);
            println!("  tool sphere r {:>3.0} mm: centre z {:>6.0} mm, bottom {:>6.0} mm", rad * 1e3, pt.z * 1e3, (pt.z - rad) * 1e3);
        }
        println!("  lowest modelled point {:.0} mm", safety::lowest_z(sc, &q) * 1e3);
        match safety::clearance(sc, &p) {
            Some(c) => println!("  clearance above the table {:.0} mm (margin {:.0} mm)", c * 1e3, sc.margin_m * 1e3),
            None => println!("  table height not set (--set-table-z-mm)"),
        }
        return Ok(());
    }

    let segs = session::plan(&opts.tests, opts.max_amp);
    let per_joint: f64 = segs.iter().map(|s| s.dur).sum::<f64>() + 2.0;
    println!(
        "plan: {} segments, {:.0} s per joint, {} joint(s) (M{}) -> about {:.1} min; max excursion {} ticks ({:.1} deg); acc {}; payload '{}'",
        segs.len(),
        per_joint,
        opts.joints.len(),
        opts.joints.iter().map(|j| j.to_string()).collect::<Vec<_>>().join(", M"),
        per_joint * opts.joints.len() as f64 / 60.0,
        opts.max_amp,
        opts.max_amp * 360.0 / 4096.0,
        opts.acc,
        opts.payload
    );
    for s in &segs {
        println!("  {:<10?} {:<16} {:>5.1} s", s.test, s.name, s.dur);
    }
    let reach = session::plan_reach(&segs);
    if cli.dry_run {
        match opts.level7() {
            Some(b) => {
                if opts.base == Base::Current {
                    println!("(dry run: clearance shown for the level pose; the real run checks the actual pose)");
                }
                let rows = session::clearance_rows(&sc_run, &b, &opts.joints, reach);
                for l in session::clearance_lines(&sc_run, &b, &rows, reach) {
                    println!("{l}");
                }
            }
            None => println!("(dry run: no level pose in the config, clearance is checked at the start of the real run)"),
        }
        return Ok(());
    }

    let mut bus = bus::Bus::open(&cfg.port, cfg.timeout_ms)?;
    // Ctrl-C: stop and return gently to base. A second Ctrl-C skips the moves back: the arm stays where it is.
    let stop = Arc::new(AtomicU8::new(session::ctl::RUN));
    {
        let stop = stop.clone();
        tokio::spawn(async move {
            if tokio::signal::ctrl_c().await.is_ok() {
                stop.store(session::ctl::RETURN, Ordering::Relaxed);
                println!("\nstopping: returning to base (Ctrl-C again: stay where it is)");
                if tokio::signal::ctrl_c().await.is_ok() {
                    stop.store(session::ctl::LEAVE, Ordering::Relaxed);
                    println!("\nthe arm stays where it is");
                }
            }
        });
    }
    let yes = cli.yes;
    let max_amp = opts.max_amp;
    let confirm = |present: &[u16], base: &[u16; 7]| {
        println!("present {:?}\nbase    {:?}", &present[..7], base);
        if yes {
            return true;
        }
        print!("The listed joints will move up to {max_amp} ticks around the base, one at a time. Start? [y/N] ");
        let _ = std::io::stdout().flush();
        let mut l = String::new();
        let _ = std::io::stdin().read_line(&mut l);
        l.trim().eq_ignore_ascii_case("y")
    };
    let mut emit = |e: Event| match e {
        Event::Status(s) => println!("{s}"),
        Event::JointStart { id, base, .. } => println!("\nM{id}: base {base}, running {per_joint:.0} s..."),
        Event::JointDone(js) => js.lines().iter().for_each(|l| println!("{l}")),
        Event::Frozen { id } => println!("M{id} frozen in place"),
        _ => {}
    };
    match session::run_session(&mut bus, &opts, &sc, &stop, confirm, &mut emit).await? {
        Some(dir) => println!("\nlogs: {}", dir.display()),
        None => println!("aborted"),
    }
    Ok(())
}
