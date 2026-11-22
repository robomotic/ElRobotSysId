//! Full-screen app: choose joints and tests, see the safety report, run, watch goal vs position live, read the
//! summary. Same keys as PingPongSimBot: ↑↓ select · Space toggle · ←→ change · Enter start · a ARM · ? help · Esc/q.
//! The Pose screen (p) moves the arm by hand without the station: one joint at a time, or smoothly to level.

use crate::bus::Bus;
use crate::io::SimIo;
use crate::jog::{self, JogCmd, JogMsg, JogState};
use crate::safety::{self, SafetyConfig};
use crate::session::{self, ctl, Base, Event, JointSummary, SessionOpts, Test};
use crate::settings::Settings;
use anyhow::Result;
use crossterm::event::{self, Event as TermEvent, KeyCode, KeyEvent, KeyEventKind, KeyModifiers};
use ratatui::layout::{Constraint, Flex, Layout, Rect};
use ratatui::style::{Color, Style, Stylize};
use ratatui::symbols::Marker;
use ratatui::text::{Line, Span};
use ratatui::widgets::{Axis, Block, Cell, Chart, Clear, Dataset, Gauge, GraphType, Paragraph, Row, Table, Wrap};
use ratatui::Frame;
use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::mpsc::{unbounded_channel, UnboundedReceiver, UnboundedSender};

const NAMES: [&str; 7] = ["base", "shoulder", "elbow", "forearm", "wrist roll", "wrist pitch", "wrist roll"];
const TESTS: [(Test, &str, &str); 5] = [
    (Test::Hold, "hold", "2 s still: gravity load"),
    (Test::Steps, "steps", "±10…100 ticks: delay, lag, stiffness"),
    (Test::Reversals, "reversals", "slow triangles: slop"),
    (Test::Ramp, "ramp", "15 ticks/s: friction"),
    (Test::Chirp, "chirp", "0.2→4 Hz sweep: bandwidth"),
];
const AMPS: [u16; 6] = [25, 50, 75, 100, 150, 200];
const ACCS: [u8; 5] = [0, 50, 100, 150, 254];
const PAYLOADS: [&str; 3] = ["none", "paddle", "paddle+ball"];
const ARM_TIMEOUT: Duration = Duration::from_secs(30);
const POSE_NAMES: [&str; 8] = ["base", "shoulder", "elbow", "forearm", "wrist roll", "wrist pitch", "wrist roll", "gripper"];
const POSE_STEPS: [i32; 5] = [1, 5, 10, 25, 50];
const POSE_HELP: [&str; 13] = [
    "↑ ↓        choose a joint (M1 base … M7 wrist roll, M8 gripper)",
    "← →        move it by one step (real arm: needs ARMED, a)",
    "+ / -      bigger / smaller step: 1 · 5 · 10 · 25 · 50 ticks",
    "l          go smoothly to the level pose from paddle.toml (asks first)",
    "Space      FREEZE: every joint holds where it is",
    "Esc / q    back to Setup (the arm keeps holding; the bus is released for the run)",
    "",
    "Every step is checked first: inside the joint's calibrated range, and the whole way",
    "keeps the table margin (needs the measured table height). While moving, the measured",
    "pose is checked every 30 ms: below the margin, everything freezes.",
    "4096 ticks = one turn (1 tick ≈ 0.09°). The arm is folded at level: raising M2 alone",
    "swings the gripper DOWN. Watch the arm while moving it.",
    "",
];
const HELP: [&str; 16] = [
    "↑ ↓        move between joints, tests and settings",
    "Space      tick / untick the joint or test",
    "← →        change the setting (amplitude, acceleration, base pose, payload)",
    "Enter      start (real arm: needs ARMED and a measured table height; asks first)",
    "a          ARM / disarm the real arm (switches itself off after 30 s without a key)",
    "p          Pose screen: move the arm by hand (no station needed)",
    "Space      while running: SAFETY STOP, freezes the arm where it is (then r = return, q = leave)",
    "Esc        while running: stop gently, the joint returns to base · otherwise quit",
    "Ctrl-C     emergency: freeze where it is, leave it there, quit",
    "",
    "Joints are tested one at a time; all others hold. M8 (gripper) is never moved.",
    "Safety: every joint's whole sweep must stay ≥ margin above the table, or it can't start.",
    "'safe ≤ N' is the largest amplitude that keeps the margin for that joint.",
    "Results go to sysid_logs/<stamp>_<payload>/ (meta.toml + one CSV per joint).",
    "In simulation the servos are modelled (delay, lag, dead zone); runs are tagged _sim.",
    "",
];

enum Mode {
    Setup,
    Confirm,
    Running { rx: UnboundedReceiver<Event>, stop: Arc<AtomicU8>, started: Instant, frozen: Option<u8> },
    Done,
    Pose { tx: UnboundedSender<JogCmd>, rx: UnboundedReceiver<JogMsg>, closing: bool },
}

struct App {
    cfg: Settings,
    sc: SafetyConfig,
    sim_table: bool,
    opts: SessionOpts,
    sim: bool,
    joints: [bool; 7],
    tests: [bool; 5],
    amp_i: usize,
    acc_i: usize,
    payload_i: usize,
    cursor: usize,
    armed_until: Option<Instant>,
    mode: Mode,
    help: bool,
    quit: bool,
    // live
    joint: Option<(u8, usize, usize)>,
    seg: String,
    progress: f64,
    chart: VecDeque<(f64, f64, f64)>,
    last: Option<(u16, u16, f64, i16)>,
    rate: VecDeque<Instant>,
    summaries: Vec<JointSummary>,
    log: VecDeque<String>,
    result: Option<(Option<PathBuf>, Option<String>)>,
    // pose screen
    pose_sel: usize,
    pose_step_i: usize,
    pose: Option<JogState>,
    pose_confirm: bool,
}

const ROWS: usize = 7 + 5 + 4;

/// Simulated servos: the arm starts at the level pose and, if no table height is set, gets a table well below it
/// (clearly labelled). Returns true when that made-up table is used.
pub fn sim_setup(opts: &mut SessionOpts, sc: &mut SafetyConfig) -> bool {
    opts.level = opts.level.or(Some([2003, 1002, 1942, 3185, 1864, 1892, 2226, 1436]));
    opts.base = Base::Level;
    if sc.table_z_m.is_some() {
        return false;
    }
    let l = opts.level7().unwrap();
    sc.table_z_m = Some(safety::lowest_z(sc, &sc.angles(&l)) - 0.15);
    true
}

pub async fn run(cfg: Settings, sc: SafetyConfig, _safety_path: PathBuf, opts: SessionOpts, sim: bool) -> Result<()> {
    let mut opts = opts;
    let mut sc = sc;
    let sim_table = sim && sim_setup(&mut opts, &mut sc);
    let mut joints = [false; 7];
    for &j in &opts.joints {
        if (1..=7).contains(&j) {
            joints[j as usize - 1] = true;
        }
    }
    let mut tests = [false; 5];
    for (i, (t, _, _)) in TESTS.iter().enumerate() {
        tests[i] = opts.tests.contains(t);
    }
    let amp_i = AMPS.iter().position(|&a| a as f64 >= opts.max_amp).unwrap_or(3);
    let acc_i = ACCS.iter().position(|&a| a >= opts.acc).unwrap_or(0);
    let payload_i = PAYLOADS.iter().position(|&p| p == opts.payload).unwrap_or(0);
    let mut app = App {
        cfg,
        sc,
        sim_table,
        opts,
        sim,
        joints,
        tests,
        amp_i,
        acc_i,
        payload_i,
        cursor: 0,
        armed_until: None,
        mode: Mode::Setup,
        help: false,
        quit: false,
        joint: None,
        seg: String::new(),
        progress: 0.0,
        chart: VecDeque::new(),
        last: None,
        rate: VecDeque::new(),
        summaries: Vec::new(),
        log: VecDeque::new(),
        result: None,
        pose_sel: 1,
        pose_step_i: 1,
        pose: None,
        pose_confirm: false,
    };
    let mut terminal = ratatui::init();
    let res = async {
        let mut tick = tokio::time::interval(Duration::from_millis(50));
        loop {
            tick.tick().await;
            app.pump();
            while event::poll(Duration::ZERO)? {
                if let TermEvent::Key(k) = event::read()? {
                    if k.kind == KeyEventKind::Press {
                        app.key(k);
                    }
                }
            }
            terminal.draw(|f| app.draw(f))?;
            if app.quit && !matches!(app.mode, Mode::Running { .. } | Mode::Pose { .. }) {
                return Ok::<(), anyhow::Error>(());
            }
        }
    }
    .await;
    ratatui::restore();
    res
}

impl App {
    fn armed(&self) -> bool {
        self.sim || self.armed_until.is_some_and(|t| Instant::now() < t)
    }

    fn current_opts(&self) -> SessionOpts {
        let mut o = self.opts.clone();
        o.joints = (0..7).filter(|&j| self.joints[j]).map(|j| j as u8 + 1).collect();
        o.tests = (0..5).filter(|&i| self.tests[i]).map(|i| TESTS[i].0).collect();
        o.max_amp = AMPS[self.amp_i] as f64;
        o.acc = ACCS[self.acc_i];
        o.payload = PAYLOADS[self.payload_i].to_string();
        o
    }

    fn msg(&mut self, m: impl Into<String>) {
        self.log.push_back(m.into());
        while self.log.len() > 50 {
            self.log.pop_front();
        }
    }

    fn start(&mut self) {
        let o = self.current_opts();
        let (tx, rx) = unbounded_channel();
        let stop = Arc::new(AtomicU8::new(ctl::RUN));
        let (sc, sim, cfg, stop2) = (self.sc.clone(), self.sim, self.cfg.clone(), stop.clone());
        // On its own OS thread: if the terminal blocks (SSH, text selection), only the drawing stops, never the run
        // and its safety checks (a shared thread stalled a real run for up to 97 s, 2026-11-22).
        spawn_worker(async move {
            let etx = tx.clone();
            let mut emit = move |e: Event| {
                let _ = etx.send(e);
            };
            let res = if sim {
                let mut io = SimIo::new(o.level.unwrap_or([2048; 8]));
                session::run_session(&mut io, &o, &sc, &stop2, |_, _| true, &mut emit).await
            } else {
                match Bus::open(&cfg.port, cfg.timeout_ms) {
                    Ok(mut bus) => session::run_session(&mut bus, &o, &sc, &stop2, |_, _| true, &mut emit).await,
                    Err(e) => Err(e),
                }
            };
            let _ = tx.send(match res {
                Ok(dir) => Event::Finished { dir, error: None },
                Err(e) => Event::Finished { dir: None, error: Some(format!("{e:#}")) },
            });
        });
        self.summaries.clear();
        self.chart.clear();
        self.log.clear();
        self.result = None;
        self.joint = None;
        self.armed_until = None;
        self.mode = Mode::Running { rx, stop, started: Instant::now(), frozen: None };
        self.msg(if self.sim { "simulated servos: nothing moves" } else { "checking the servos and the safety area..." });
    }

    fn open_pose(&mut self) {
        let (ctx, crx) = unbounded_channel();
        let (mtx, mrx) = unbounded_channel();
        let (sc, sim, cfg) = (self.sc.clone(), self.sim, self.cfg.clone());
        let start = self.opts.level.unwrap_or([2048; 8]);
        spawn_worker(async move {
            let res = if sim {
                let mut io = SimIo::new(start);
                jog::run(&mut io, sc, crx, mtx.clone()).await
            } else {
                match Bus::open(&cfg.port, cfg.timeout_ms) {
                    Ok(mut bus) => jog::run(&mut bus, sc, crx, mtx.clone()).await,
                    Err(e) => Err(e),
                }
            };
            // The bus is dropped (released) here, before Closed is sent.
            let _ = mtx.send(JogMsg::Closed(res.err().map(|e| format!("{e:#}"))));
        });
        self.log.clear();
        self.pose = None;
        self.pose_confirm = false;
        self.mode = Mode::Pose { tx: ctx, rx: mrx, closing: false };
        self.msg(if self.sim { "simulated servos: nothing moves" } else { "opening the bus..." });
    }

    fn close_pose(&mut self) {
        if let Mode::Pose { tx, closing, .. } = &mut self.mode {
            if !*closing {
                let _ = tx.send(JogCmd::Close);
                *closing = true;
            }
        }
    }

    fn pose_send(&mut self, cmd: JogCmd) {
        if let Mode::Pose { tx, .. } = &self.mode {
            let _ = tx.send(cmd);
        }
    }

    fn pump_pose(&mut self) {
        let mut msgs = Vec::new();
        if let Mode::Pose { rx, .. } = &mut self.mode {
            while let Ok(m) = rx.try_recv() {
                msgs.push(m);
            }
        }
        for m in msgs {
            match m {
                JogMsg::State(st) => self.pose = Some(st),
                JogMsg::Info(t) => self.msg(t),
                JogMsg::Warn(t) => self.msg(format!("⚠ {t}")),
                JogMsg::Closed(err) => {
                    if let Some(e) = err {
                        self.msg(format!("✗ {e}"));
                    }
                    self.mode = Mode::Setup;
                }
            }
        }
    }

    fn pump(&mut self) {
        if matches!(self.mode, Mode::Pose { .. }) {
            self.pump_pose();
            return;
        }
        let mut events = Vec::new();
        if let Mode::Running { rx, .. } = &mut self.mode {
            while let Ok(e) = rx.try_recv() {
                events.push(e);
            }
        }
        for e in events {
            match e {
                Event::Status(s) => self.msg(s),
                Event::JointStart { id, index, of, base } => {
                    self.joint = Some((id, index, of));
                    self.chart.clear();
                    self.msg(format!("M{id} ({}): base {base}", NAMES[id as usize - 1]));
                }
                Event::Sample { id, t, seg, goal, pos, current, load, progress } => {
                    if self.joint.is_some_and(|(j, _, _)| j != id) {
                        continue;
                    }
                    self.chart.push_back((t, goal as f64, pos as f64));
                    while self.chart.front().is_some_and(|c| t - c.0 > 4.0) {
                        self.chart.pop_front();
                    }
                    self.seg = seg;
                    self.progress = progress;
                    self.last = Some((goal, pos, current, load));
                    self.rate.push_back(Instant::now());
                    while self.rate.front().is_some_and(|r| r.elapsed() > Duration::from_secs(1)) {
                        self.rate.pop_front();
                    }
                }
                Event::Frozen { id } => {
                    if let Mode::Running { frozen, .. } = &mut self.mode {
                        *frozen = Some(id);
                    }
                    self.msg(format!("SAFETY STOP: M{id} frozen in place. r = return slowly to base, q = leave it here"));
                }
                Event::JointDone(js) => {
                    self.msg(format!("M{} done{}", js.id, js.stopped.as_ref().map_or(String::new(), |r| format!(": stopped, {r}"))));
                    self.summaries.push(js);
                }
                Event::Finished { dir, error } => {
                    if let Some(e) = &error {
                        self.msg(format!("✗ {e}"));
                    }
                    self.result = Some((dir, error));
                    self.mode = Mode::Done;
                }
            }
        }
    }

    fn key(&mut self, k: KeyEvent) {
        if self.armed_until.is_some_and(|t| Instant::now() < t) {
            self.armed_until = Some(Instant::now() + ARM_TIMEOUT);
        }
        if k.code == KeyCode::Char('c') && k.modifiers.contains(KeyModifiers::CONTROL) {
            // Emergency: freeze where it is, leave it there, quit.
            if let Mode::Running { stop, .. } = &self.mode {
                stop.store(ctl::LEAVE, Ordering::Relaxed);
            }
            if matches!(self.mode, Mode::Pose { .. }) {
                self.pose_send(JogCmd::Freeze);
                self.close_pose();
            }
            self.quit = true;
            return;
        }
        // SAFETY STOP has priority over everything (also over the help overlay).
        if let Mode::Running { stop, frozen, .. } = &self.mode {
            if k.code == KeyCode::Char(' ') && frozen.is_none() && stop.load(Ordering::Relaxed) == ctl::RUN {
                stop.store(ctl::FREEZE, Ordering::Relaxed);
                self.help = false;
                self.msg("SAFETY STOP pressed: freezing the arm where it is");
                return;
            }
        }
        // FREEZE on the Pose screen has priority too.
        if matches!(self.mode, Mode::Pose { .. }) && k.code == KeyCode::Char(' ') {
            self.pose_confirm = false;
            self.help = false;
            self.pose_send(JogCmd::Freeze);
            return;
        }
        if self.help {
            self.help = false;
            return;
        }
        if k.code == KeyCode::Char('?') {
            self.help = true;
            return;
        }
        if matches!(self.mode, Mode::Pose { .. }) {
            self.pose_key(k);
            return;
        }
        match &self.mode {
            Mode::Pose { .. } => {}
            Mode::Running { stop, frozen, .. } => {
                let frozen = frozen.is_some() || stop.load(Ordering::Relaxed) == ctl::FREEZE;
                match (frozen, k.code) {
                    (true, KeyCode::Char('r')) => {
                        stop.store(ctl::RETURN, Ordering::Relaxed);
                        self.msg("returning slowly to base");
                    }
                    (true, KeyCode::Char('q')) => {
                        stop.store(ctl::LEAVE, Ordering::Relaxed);
                        self.msg("leaving the arm where it is");
                    }
                    (false, KeyCode::Esc | KeyCode::Char('q')) => {
                        stop.store(ctl::RETURN, Ordering::Relaxed);
                        self.msg("stopping: the joint returns to its base position");
                    }
                    _ => {}
                }
            }
            Mode::Confirm => {
                if k.code == KeyCode::Char('y') {
                    self.start();
                } else {
                    self.mode = Mode::Setup;
                }
            }
            Mode::Done => match k.code {
                KeyCode::Enter | KeyCode::Char(' ') => self.mode = Mode::Setup,
                KeyCode::Esc | KeyCode::Char('q') => self.quit = true,
                _ => {}
            },
            Mode::Setup => match k.code {
                KeyCode::Up => self.cursor = self.cursor.saturating_sub(1),
                KeyCode::Down => self.cursor = (self.cursor + 1).min(ROWS - 1),
                KeyCode::Char(' ') => match self.cursor {
                    c @ 0..=6 => self.joints[c] = !self.joints[c],
                    c @ 7..=11 => self.tests[c - 7] = !self.tests[c - 7],
                    _ => {}
                },
                KeyCode::Left | KeyCode::Right => {
                    let d: isize = if k.code == KeyCode::Right { 1 } else { -1 };
                    let step = |i: usize, n: usize| (i as isize + d).clamp(0, n as isize - 1) as usize;
                    match self.cursor {
                        12 => self.amp_i = step(self.amp_i, AMPS.len()),
                        13 => self.acc_i = step(self.acc_i, ACCS.len()),
                        14 if !self.sim => {
                            self.opts.base = if self.opts.base == Base::Current { Base::Level } else { Base::Current }
                        }
                        15 => self.payload_i = step(self.payload_i, PAYLOADS.len()),
                        _ => {}
                    }
                }
                KeyCode::Char('a') if !self.sim => {
                    self.armed_until = if self.armed() { None } else { Some(Instant::now() + ARM_TIMEOUT) };
                }
                KeyCode::Char('p') => self.open_pose(),
                KeyCode::Enter => {
                    let o = self.current_opts();
                    if o.joints.is_empty() || o.tests.is_empty() {
                        self.msg("tick at least one joint and one test");
                    } else if !self.armed() {
                        self.msg("press a to ARM first: this moves the arm");
                    } else if self.sc.table_z_m.is_none() {
                        self.msg("table height not set: measure it, then elrobot-sysid --set-table-z-mm <mm>");
                    } else {
                        self.mode = Mode::Confirm;
                    }
                }
                KeyCode::Esc | KeyCode::Char('q') => self.quit = true,
                _ => {}
            },
        }
    }

    fn pose_key(&mut self, k: KeyEvent) {
        if self.pose_confirm {
            self.pose_confirm = false;
            if k.code == KeyCode::Char('y') {
                match self.opts.level7() {
                    Some(to) => {
                        let from = self.pose.map(|p| p.present).unwrap_or_default();
                        let dist = (0..7).map(|j| (from[j] as i32 - to[j] as i32).abs()).max().unwrap_or(0) as f64;
                        // ~200 ticks/s (18°/s), at least 2 s.
                        self.pose_send(JogCmd::GoTo { to, secs: (dist / 200.0).max(2.0), name: "level".into() });
                    }
                    None => self.msg("no level pose: start with --config ~/paddle-ws/paddle.toml"),
                }
            } else {
                self.msg("cancelled");
            }
            return;
        }
        match k.code {
            KeyCode::Up => self.pose_sel = self.pose_sel.saturating_sub(1),
            KeyCode::Down => self.pose_sel = (self.pose_sel + 1).min(7),
            KeyCode::Char('+') | KeyCode::Char('=') => self.pose_step_i = (self.pose_step_i + 1).min(POSE_STEPS.len() - 1),
            KeyCode::Char('-') | KeyCode::Char('_') => self.pose_step_i = self.pose_step_i.saturating_sub(1),
            KeyCode::Left | KeyCode::Right => {
                if !self.armed() {
                    self.msg("press a to ARM first: this moves the arm");
                } else if self.pose.is_none() {
                    self.msg("waiting for the servos");
                } else {
                    let d = POSE_STEPS[self.pose_step_i] * if k.code == KeyCode::Right { 1 } else { -1 };
                    self.pose_send(JogCmd::Nudge { id: self.pose_sel as u8 + 1, delta: d });
                }
            }
            KeyCode::Char('a') if !self.sim => {
                self.armed_until = if self.armed() { None } else { Some(Instant::now() + ARM_TIMEOUT) };
            }
            KeyCode::Char('l') => {
                if !self.armed() {
                    self.msg("press a to ARM first: this moves the arm");
                } else {
                    self.pose_confirm = true;
                }
            }
            KeyCode::Char('?') => self.help = true,
            KeyCode::Esc | KeyCode::Char('q') => {
                self.close_pose();
                self.msg("releasing the bus...");
            }
            _ => {}
        }
    }

    fn draw_pose(&self, f: &mut Frame, area: Rect) {
        let [banner, tbl, info] = Layout::vertical([Constraint::Length(3), Constraint::Length(12), Constraint::Min(5)]).areas(area);
        let clear = self.pose.and_then(|p| p.clearance);
        let (ctext, ccol) = match (clear, self.sc.table_z_m) {
            (_, None) => ("table height NOT SET: only the gripper can move".to_string(), Color::Red),
            (Some(c), _) if c < self.sc.margin_m + 0.01 => (format!("{:.0} mm above the table (margin {:.0})", c * 1e3, self.sc.margin_m * 1e3), Color::Red),
            (Some(c), _) if c < self.sc.margin_m + 0.03 => (format!("{:.0} mm above the table (margin {:.0})", c * 1e3, self.sc.margin_m * 1e3), Color::Yellow),
            (Some(c), _) => (format!("{:.0} mm above the table (margin {:.0})", c * 1e3, self.sc.margin_m * 1e3), Color::Green),
            (None, _) => ("…".into(), Color::Gray),
        };
        let moving = self.pose.is_some_and(|p| p.moving);
        f.render_widget(
            Paragraph::new(Line::from(vec![
                Span::styled(" SPACE = FREEZE ", Style::new().fg(Color::Red).bold()),
                Span::raw(format!(" · step {} ticks ({:.1}°) · lowest point ", POSE_STEPS[self.pose_step_i], POSE_STEPS[self.pose_step_i] as f64 * 360.0 / 4096.0)),
                Span::styled(ctext, Style::new().fg(ccol).bold()),
                Span::raw(if moving { "  · moving" } else { "" }),
            ]))
            .block(Block::bordered().border_style(Style::new().fg(Color::Red))),
            banner,
        );
        let rows: Vec<Row> = (0..8)
            .map(|j| {
                let (lo, hi) = (self.sc.range_min[j] as i32, self.sc.range_max[j] as i32);
                let cells = match self.pose {
                    Some(p) => {
                        let pr = p.present[j] as i32;
                        let room = (pr - lo).min(hi - pr);
                        let pct = 100.0 * (pr - lo) as f64 / (hi - lo).max(1) as f64;
                        let room_col = if room < 30 { Color::Red } else if room < 100 { Color::Yellow } else { Color::Reset };
                        let deg = if j < 7 { format!("{:+.1}°", self.sc.tick_to_rad(j, pr as f64).to_degrees()) } else { "".into() };
                        vec![
                            Cell::from(format!("{pr}")),
                            Cell::from(format!("{}", p.goal[j])),
                            Cell::from(deg),
                            Cell::from(format!("{lo}–{hi}")),
                            Cell::from(format!("{pct:.0}%")),
                            Cell::from(Span::styled(format!("{room}"), Style::new().fg(room_col))),
                            Cell::from(format!("{}", p.load[j])),
                            Cell::from(format!("{:.0}", p.current_ma[j])),
                            Cell::from(format!("{}", p.temp_c[j])),
                        ]
                    }
                    None => vec![Cell::from("…")],
                };
                let mut v = vec![Cell::from(format!("{}M{} {}", if j == self.pose_sel { "▶ " } else { "  " }, j + 1, POSE_NAMES[j]))];
                v.extend(cells);
                let r = Row::new(v);
                if j == self.pose_sel { r.style(Style::new().fg(Color::Black).bg(Color::Cyan)) } else { r }
            })
            .collect();
        f.render_widget(
            Table::new(
                rows,
                [
                    Constraint::Length(18),
                    Constraint::Length(8),
                    Constraint::Length(8),
                    Constraint::Length(9),
                    Constraint::Length(11),
                    Constraint::Length(6),
                    Constraint::Length(7),
                    Constraint::Length(6),
                    Constraint::Length(6),
                    Constraint::Length(5),
                ],
            )
            .header(Row::new(vec!["joint", "present", "goal", "angle", "range", "%", "room", "load", "mA", "°C"]).style(Style::new().fg(Color::Cyan)))
            .block(Block::bordered().title(" pose (ticks; room = ticks to the nearer end of the calibrated range) ")),
            tbl,
        );
        let log: Vec<Line> = self.log.iter().rev().take(8).rev().map(|m| {
            let st = if m.starts_with('⚠') || m.starts_with('✗') { Style::new().fg(Color::Yellow) } else { Style::new() };
            Line::from(Span::styled(format!(" {m}"), st))
        }).collect();
        f.render_widget(Paragraph::new(log).block(Block::bordered().title(" status ")), info);
    }

    // -------------------------------------------------------------------------------------------------------------

    fn draw(&self, f: &mut Frame) {
        let [head, body, foot] = Layout::vertical([Constraint::Length(1), Constraint::Min(0), Constraint::Length(1)]).areas(f.area());
        let (title, keys): (&str, Vec<(&str, &str)>) = match self.mode {
            Mode::Setup => {
                let mut k = vec![("↑↓", "select"), ("Space", "tick"), ("←→", "change"), ("Enter", "start"), ("p", "pose")];
                if !self.sim {
                    k.insert(0, ("a", "arm"));
                }
                k.extend([("?", "help"), ("q", "quit")]);
                ("SYSTEM ID", k)
            }
            Mode::Confirm => ("SYSTEM ID", vec![("y", "start"), ("any key", "cancel")]),
            Mode::Running { frozen: Some(_), .. } => ("FROZEN", vec![("r", "return slowly to base"), ("q", "leave it here")]),
            Mode::Running { .. } => ("RUNNING", vec![("Space", "SAFETY STOP (freeze)"), ("Esc", "stop & return to base"), ("?", "help")]),
            Mode::Done => ("RESULTS", vec![("Enter", "new run"), ("q", "quit"), ("?", "help")]),
            Mode::Pose { .. } => {
                let mut k = vec![("↑↓", "joint"), ("←→", "move"), ("+/-", "step"), ("l", "level"), ("Space", "FREEZE"), ("Esc", "back")];
                if !self.sim {
                    k.insert(0, ("a", "arm"));
                }
                k.push(("?", "help"));
                ("POSE", k)
            }
        };
        let mut hs = vec![Span::styled(format!(" {title} "), Style::new().fg(Color::Black).bg(Color::Cyan).bold()), Span::raw("  ")];
        if self.sim {
            hs.push(Span::styled(" SIM ", Style::new().fg(Color::White).bg(Color::Blue).bold()));
        } else if self.armed() {
            let left = self.armed_until.map_or(0, |t| t.saturating_duration_since(Instant::now()).as_secs());
            hs.push(Span::styled(format!(" ⚠ ARMED {left:>2}s "), Style::new().fg(Color::White).bg(Color::Red).bold()));
        } else {
            hs.push(Span::styled(" disarmed (a to arm) ", Style::new().fg(Color::DarkGray)));
        }
        if let Mode::Running { started, .. } = &self.mode {
            hs.push(Span::raw(format!("  {:.0} s", started.elapsed().as_secs_f64())));
        }
        f.render_widget(Paragraph::new(Line::from(hs)), head);
        match &self.mode {
            Mode::Setup | Mode::Confirm => self.draw_setup(f, body),
            Mode::Running { .. } => self.draw_running(f, body),
            Mode::Done => self.draw_done(f, body),
            Mode::Pose { .. } => self.draw_pose(f, body),
        }
        let mut ks = Vec::new();
        for (k, d) in keys {
            ks.push(Span::styled(format!(" {k} "), Style::new().fg(Color::Black).bg(Color::Gray)));
            ks.push(Span::raw(format!(" {d}  ")));
        }
        f.render_widget(Paragraph::new(Line::from(ks)), foot);
        if matches!(self.mode, Mode::Confirm) {
            let o = self.current_opts();
            let segs = session::plan(&o.tests, o.max_amp);
            let total: f64 = (segs.iter().map(|s| s.dur).sum::<f64>() + 2.0) * o.joints.len() as f64;
            popup(
                f,
                &format!(
                    "Run {} on M{} (one at a time), up to {} ticks ({:.1}°) around the {} pose, about {:.1} min. {} (y/n)",
                    o.tests.iter().map(|t| format!("{t:?}").to_lowercase()).collect::<Vec<_>>().join(", "),
                    o.joints.iter().map(|j| j.to_string()).collect::<Vec<_>>().join(", M"),
                    o.max_amp,
                    o.max_amp * 360.0 / 4096.0,
                    if o.base == Base::Level { "level" } else { "current" },
                    total / 60.0,
                    if self.sim { "Simulated servos." } else { "THE ARM WILL MOVE. Keep the area clear." }
                ),
                Color::Yellow,
            );
        }
        if self.pose_confirm {
            let to = self.opts.level7().map_or("–".to_string(), |l| format!("{l:?}"));
            popup(f, &format!("Move smoothly to the level pose {to}? THE ARM WILL MOVE. (y/n)"), Color::Yellow);
        }
        if self.help {
            let help: &[&str] = if matches!(self.mode, Mode::Pose { .. }) { &POSE_HELP } else { &HELP };
            let w = help.iter().map(|l| l.chars().count()).max().unwrap_or(20) as u16 + 6;
            let [a] = Layout::horizontal([Constraint::Length(w)]).flex(Flex::Center).areas(f.area());
            let [a] = Layout::vertical([Constraint::Length(help.len() as u16 + 3)]).flex(Flex::Center).areas(a);
            f.render_widget(Clear, a);
            f.render_widget(
                Paragraph::new(help.iter().map(|l| Line::from(format!(" {l}"))).collect::<Vec<_>>())
                    .block(Block::bordered().title(" keys (any key closes) ").border_style(Style::new().fg(Color::Cyan))),
                a,
            );
        }
    }

    fn draw_setup(&self, f: &mut Frame, area: Rect) {
        let o = self.current_opts();
        let segs = session::plan(&o.tests, o.max_amp);
        let reach = session::plan_reach(&segs);
        let per_joint: f64 = segs.iter().map(|s| s.dur).sum::<f64>() + 2.0;
        // The pose the run will start from: level, or the current pose (known once the Pose screen has read it;
        // the run itself always reads it again and checks before anything moves).
        let base = if o.base == Base::Level || self.sim {
            o.level7()
        } else {
            self.pose.map(|p| {
                let mut b = [0u16; 7];
                b.copy_from_slice(&p.present[..7]);
                b
            })
        };
        let sc = o.safety(&self.sc);
        let safety_rows: Vec<Option<session::JointClearance>> = (1..=7u8)
            .map(|id| base.map(|b| session::clearance_rows(&sc, &b, &[id], reach).remove(0)))
            .collect();
        let [left, right] = Layout::horizontal([Constraint::Min(62), Constraint::Length(48)]).areas(area);
        let sel = |i: usize| i == self.cursor;
        let style = |i: usize| if sel(i) { Style::new().fg(Color::Black).bg(Color::Cyan).bold() } else { Style::new() };
        let mut lines = vec![Line::from(Span::styled(" JOINTS                         safety (whole sweep)", Style::new().fg(Color::DarkGray)))];
        for j in 0..7 {
            let check = if self.joints[j] { "[x]" } else { "[ ]" };
            let safety = match &safety_rows[j] {
                Some(r) if r.clearance.is_some() => {
                    let c = r.clearance.unwrap() * 1e3;
                    if r.ok {
                        Span::styled(format!("✓ {c:>4.0} mm clear"), Style::new().fg(Color::Green))
                    } else {
                        Span::styled(format!("✗ {c:>4.0} mm · safe ≤ {:.0}", r.safe_amp), Style::new().fg(Color::Red))
                    }
                }
                _ => Span::styled("table height not set", Style::new().fg(Color::Red)),
            };
            lines.push(Line::from(vec![
                Span::raw(if sel(j) { " ▸ " } else { "   " }),
                Span::styled(format!("{check} M{} {:<12}", j + 1, NAMES[j]), style(j)),
                Span::raw("  "),
                safety,
            ]));
        }
        lines.push(Line::from(""));
        lines.push(Line::from(Span::styled(" TESTS", Style::new().fg(Color::DarkGray))));
        for (i, (_, name, what)) in TESTS.iter().enumerate() {
            let r = 7 + i;
            lines.push(Line::from(vec![
                Span::raw(if sel(r) { " ▸ " } else { "   " }),
                Span::styled(format!("{} {name:<10}", if self.tests[i] { "[x]" } else { "[ ]" }), style(r)),
                Span::styled(format!("  {what}"), Style::new().fg(Color::DarkGray)),
            ]));
        }
        lines.push(Line::from(""));
        lines.push(Line::from(Span::styled(" SETTINGS", Style::new().fg(Color::DarkGray))));
        let settings = [
            ("amplitude", format!("{} ticks ({:.1}°)", AMPS[self.amp_i], AMPS[self.amp_i] as f64 * 360.0 / 4096.0)),
            ("acceleration", if ACCS[self.acc_i] == 0 { "servo max (0)".into() } else { ACCS[self.acc_i].to_string() }),
            ("base pose", if o.base == Base::Level { "level (from paddle.toml)".into() } else { "current (where the arm is)".into() }),
            ("payload", format!("{}{}", PAYLOADS[self.payload_i], if PAYLOADS[self.payload_i].contains("paddle") { "  (adds the paddle to the safety model)" } else { "" })),
        ];
        for (i, (name, val)) in settings.iter().enumerate() {
            let r = 12 + i;
            lines.push(Line::from(vec![
                Span::raw(if sel(r) { " ▸ " } else { "   " }),
                Span::styled(format!("{name:<14}"), style(r)),
                Span::raw(format!("  {val}")),
                Span::raw(if sel(r) { "  ◂ ▸" } else { "" }),
            ]));
        }
        f.render_widget(Paragraph::new(lines).block(Block::bordered().title(" what to identify ")), left);

        let dim = Style::new().fg(Color::DarkGray);
        let mut r = vec![
            Line::from(format!(" {} joint(s) × {:.0} s ≈ {:.1} min", o.joints.len(), per_joint, per_joint * o.joints.len() as f64 / 60.0)),
            Line::from(format!(" {} test segments, reach ±{:.0} ticks", segs.len(), reach)),
            Line::from(""),
        ];
        match self.sc.table_z_m {
            Some(t) => {
                r.push(Line::from(format!(" table {:.0} mm{}", t * 1e3, if self.sim_table { "  (simulated)" } else { "" })));
                r.push(Line::from(format!(" margin {:.0} mm", self.sc.margin_m * 1e3)));
                match base {
                    Some(b) => r.push(Line::from(format!(
                        " clearance at base {:.0} mm ({})",
                        safety::clearance(&sc, &b).unwrap_or(f64::NAN) * 1e3,
                        if o.base == Base::Level || self.sim { "level" } else { "current, as last read" }
                    ))),
                    None => {
                        r.push(Line::from(Span::styled(" current pose not read yet: press p", Style::new().fg(Color::Yellow))));
                        r.push(Line::from(" (the run checks it before moving)"));
                    }
                }
                r.push(Line::from(format!(" model: gripper{}", if sc.with_paddle { " + paddle" } else { " (no paddle)" })));
            }
            None => {
                r.push(Line::from(Span::styled(" table height NOT SET", Style::new().fg(Color::Red).bold())));
                r.push(Line::from(" runs on the arm are refused until it is"));
                r.push(Line::from(" measured: --set-table-z-mm <mm>"));
            }
        }
        if !self.sim {
            r.push(Line::from(""));
            r.push(Line::from(Span::styled(" safety model checked against the arm", dim)));
            r.push(Line::from(Span::styled(" (ruler, viewer) on 2026-11-21, issue #1", dim)));
        }
        r.push(Line::from(""));
        r.push(Line::from(Span::styled(format!(" port {}", self.cfg.port), dim)));
        r.push(Line::from(Span::styled(format!(" output {}/", self.opts.out.display()), dim)));
        for m in self.log.iter().rev().take(3).rev() {
            r.push(Line::from(Span::styled(format!(" {m}"), Style::new().fg(Color::Yellow))));
        }
        f.render_widget(Paragraph::new(r).wrap(Wrap { trim: false }).block(Block::bordered().title(" plan & safety ")), right);
    }

    fn draw_running(&self, f: &mut Frame, area: Rect) {
        let frozen = matches!(self.mode, Mode::Running { frozen: Some(_), .. });
        let [banner, area] = Layout::vertical([Constraint::Length(if frozen { 5 } else { 3 }), Constraint::Min(0)]).areas(area);
        let (text, style) = if frozen {
            (
                vec![
                    Line::from(Span::styled(" ■ SAFETY STOP: the arm is frozen where it is (torque on, nothing moves)", Style::new().fg(Color::White).bold())),
                    Line::from(""),
                    Line::from(Span::styled(" r  return slowly to base        q  leave it here and end the run", Style::new().fg(Color::White))),
                ],
                Style::new().bg(Color::Red),
            )
        } else {
            (
                vec![Line::from(Span::styled(" SPACE = SAFETY STOP: freezes the arm where it is · Esc = stop and return to base", Style::new().fg(Color::Red).bold()))],
                Style::new(),
            )
        };
        f.render_widget(Paragraph::new(text).style(style).block(Block::bordered().border_style(Style::new().fg(Color::Red))), banner);
        let [top, mid, bottom] = Layout::vertical([Constraint::Length(3), Constraint::Min(12), Constraint::Length(9)]).areas(area);
        let label = match self.joint {
            Some((id, i, n)) => format!("M{id} {} · joint {} of {} · {}", NAMES[id as usize - 1], i + 1, n, self.seg),
            None => "starting...".into(),
        };
        f.render_widget(Gauge::default().block(Block::bordered().title(" progress ")).gauge_style(Style::new().fg(Color::Cyan)).ratio(self.progress.clamp(0.0, 1.0)).label(label), top);
        let [chart_a, nums] = Layout::horizontal([Constraint::Min(40), Constraint::Length(30)]).areas(mid);
        let goal: Vec<(f64, f64)> = self.chart.iter().map(|c| (c.0, c.1)).collect();
        let pos: Vec<(f64, f64)> = self.chart.iter().map(|c| (c.0, c.2)).collect();
        let t1 = self.chart.back().map_or(4.0, |c| c.0);
        let (lo, hi) = self.chart.iter().fold((f64::MAX, f64::MIN), |(l, h), c| (l.min(c.1).min(c.2), h.max(c.1).max(c.2)));
        let (lo, hi) = if lo > hi { (2000.0, 2100.0) } else { (lo - 5.0, hi + 5.0) };
        let chart = Chart::new(vec![
            Dataset::default().name("goal").marker(Marker::Braille).graph_type(GraphType::Line).style(Style::new().fg(Color::Yellow)).data(&goal),
            Dataset::default().name("position").marker(Marker::Braille).graph_type(GraphType::Line).style(Style::new().fg(Color::Cyan)).data(&pos),
        ])
        .block(Block::bordered().title(" goal vs position (last 4 s, ticks) "))
        .x_axis(Axis::default().bounds([t1 - 4.0, t1]).labels(vec![Line::from(format!("{:.0}s", t1 - 4.0)), Line::from(format!("{t1:.0}s"))]))
        .y_axis(Axis::default().bounds([lo, hi]).labels(vec![Line::from(format!("{lo:.0}")), Line::from(format!("{hi:.0}"))]));
        f.render_widget(chart, chart_a);
        let mut n = Vec::new();
        if let Some((g, p, c, l)) = self.last {
            n.push(Line::from(format!(" goal      {g}")));
            n.push(Line::from(format!(" position  {p}")));
            n.push(Line::from(format!(" miss      {:+}", g as i32 - p as i32)));
            n.push(Line::from(format!(" current   {c:.0} mA")));
            n.push(Line::from(format!(" load      {l}")));
            n.push(Line::from(format!(" samples   {} /s", self.rate.len())));
        }
        n.push(Line::from(""));
        for s in &self.summaries {
            n.push(Line::from(Span::styled(format!(" ✓ M{} done", s.id), Style::new().fg(Color::Green))));
        }
        f.render_widget(Paragraph::new(n).block(Block::bordered().title(" now ")), nums);
        let log: Vec<Line> = self.log.iter().rev().take(7).rev().map(|m| Line::from(format!(" {m}"))).collect();
        f.render_widget(Paragraph::new(log).block(Block::bordered().title(" status ")), bottom);
    }

    fn draw_done(&self, f: &mut Frame, area: Rect) {
        let [tbl, info] = Layout::vertical([Constraint::Min(6), Constraint::Length(7)]).areas(area);
        let num = |v: Option<f64>, d: usize| v.filter(|x| x.is_finite()).map_or("–".to_string(), |x| format!("{x:.d$}"));
        let rows: Vec<Row> = self
            .summaries
            .iter()
            .map(|s| {
                let step = s.steps.iter().find(|(n, _)| n == "step+50").or(s.steps.first()).map(|(_, st)| st.clone());
                Row::new(vec![
                    Cell::from(format!("M{} {}", s.id, NAMES[s.id as usize - 1])),
                    Cell::from(format!("{:.0}", s.rate_hz)),
                    Cell::from(num(s.hold_load, 0)),
                    Cell::from(num(s.hold_current_ma, 0)),
                    Cell::from(num(step.as_ref().map(|x| x.dead_ms), 0)),
                    Cell::from(num(step.as_ref().map(|x| x.t63_ms), 0)),
                    Cell::from(num(step.as_ref().map(|x| x.overshoot_pct), 0)),
                    Cell::from(num(step.as_ref().map(|x| x.steady_err), 1)),
                    Cell::from(num(s.slop_mean(), 1)),
                    Cell::from(num(s.ramp_peak_ma, 0)),
                    Cell::from(if s.stopped.is_some() { Span::styled("stopped", Style::new().fg(Color::Red)) } else { Span::styled("ok", Style::new().fg(Color::Green)) }),
                ])
            })
            .collect();
        f.render_widget(
            Table::new(
                rows,
                [
                    Constraint::Length(16),
                    Constraint::Length(6),
                    Constraint::Length(7),
                    Constraint::Length(8),
                    Constraint::Length(8),
                    Constraint::Length(8),
                    Constraint::Length(10),
                    Constraint::Length(9),
                    Constraint::Length(7),
                    Constraint::Length(9),
                    Constraint::Length(8),
                ],
            )
            .header(
                Row::new(vec!["joint", "Hz", "hold ‰", "hold mA", "dead ms", "t63 ms", "overshoot%", "miss tk", "slop", "ramp mA", ""])
                    .style(Style::new().fg(Color::Cyan)),
            )
            .block(Block::bordered().title(" summary per joint (step columns: +50-tick step) ")),
            tbl,
        );
        let mut l = Vec::new();
        match &self.result {
            Some((Some(dir), None)) => l.push(Line::from(Span::styled(format!(" ✓ finished · logs in {}/", dir.display()), Style::new().fg(Color::Green)))),
            Some((_, Some(e))) => l.push(Line::from(Span::styled(format!(" ✗ {e}"), Style::new().fg(Color::Red)))),
            _ => l.push(Line::from(" cancelled")),
        }
        l.push(Line::from(Span::styled(
            " dead = time before it starts moving · t63 = time to 63% · miss = steady goal − position · slop = reversal dead zone",
            Style::new().fg(Color::DarkGray),
        )));
        for m in self.log.iter().rev().take(3).rev() {
            l.push(Line::from(format!(" {m}")));
        }
        f.render_widget(Paragraph::new(l).wrap(Wrap { trim: false }).block(Block::bordered().title(" result ")), info);
    }
}

/// Runs `fut` on a dedicated OS thread with its own runtime, so the bus loop never waits for the terminal.
fn spawn_worker(fut: impl std::future::Future<Output = ()> + Send + 'static) {
    std::thread::spawn(move || match tokio::runtime::Builder::new_current_thread().enable_all().build() {
        Ok(rt) => rt.block_on(fut),
        Err(e) => eprintln!("worker runtime: {e}"),
    });
}

fn popup(f: &mut Frame, text: &str, color: Color) {
    let w = (text.chars().count() as u16 / 2 + 8).clamp(40, f.area().width.saturating_sub(4));
    let [a] = Layout::horizontal([Constraint::Length(w)]).flex(Flex::Center).areas(f.area());
    let [a] = Layout::vertical([Constraint::Length(6)]).flex(Flex::Center).areas(a);
    f.render_widget(Clear, a);
    f.render_widget(
        Paragraph::new(vec![Line::from(""), Line::from(format!(" {text}"))]).wrap(Wrap { trim: false }).block(Block::bordered().title(" confirm ").border_style(Style::new().fg(color))),
        a,
    );
}
