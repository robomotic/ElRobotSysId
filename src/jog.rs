//! Moving the arm by hand from the app, without the station: one joint at a time in small steps, or a smooth move to
//! the level pose. A task owns the bus while the Pose screen is open; the screen sends commands and gets states back.
//!
//! Safety, checked here (not in the screen) before every goal is written:
//! - each joint stays inside its calibrated range (`SafetyConfig::range_min/max`);
//! - for M1–M7, the whole way to the new goal keeps the table margin (`safety::path_clearance`); refused without a
//!   measured table height;
//! - while anything moves, the measured pose is checked every cycle: below the margin, everything freezes in place.

use crate::io::Io;
use crate::safety::{self, SafetyConfig};
use crate::session::min_jerk;
use anyhow::Result;
use std::time::{Duration, Instant};
use tokio::sync::mpsc::{UnboundedReceiver, UnboundedSender};

/// Largest single nudge (ticks).
pub const MAX_STEP: i32 = 50;
/// Cycle: read all 8 servos, advance a smooth move.
const CYCLE: Duration = Duration::from_millis(30);

#[derive(Debug, Clone)]
pub enum JogCmd {
    /// Move joint `id` (1–8) by `delta` ticks from its current goal.
    Nudge { id: u8, delta: i32 },
    /// Smooth (min-jerk) move of M1–M7 to `to` over `secs`.
    GoTo { to: [u16; 7], secs: f64, name: String },
    /// Goal = present on every joint, torque stays on. Cancels a smooth move.
    Freeze,
    /// Release the bus (torque stays on, holding where it is).
    Close,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct JogState {
    pub present: [u16; 8],
    pub goal: [u16; 8],
    pub load: [i16; 8],
    pub current_ma: [f64; 8],
    pub temp_c: [u8; 8],
    /// Clearance of the measured pose above the table (m), if the table is known.
    pub clearance: Option<f64>,
    pub moving: bool,
}

#[derive(Debug, Clone)]
pub enum JogMsg {
    State(JogState),
    Info(String),
    /// A refused command or a safety freeze.
    Warn(String),
    Closed(Option<String>),
}

struct Move {
    from: [u16; 7],
    to: [u16; 7],
    t0: Instant,
    secs: f64,
}

fn seven(p: &[u16; 8]) -> [u16; 7] {
    let mut s = [0u16; 7];
    s.copy_from_slice(&p[..7]);
    s
}

/// Check a nudge: returns the new goal, or why not.
pub fn check_nudge(sc: &SafetyConfig, goal: &[u16; 8], id: u8, delta: i32) -> Result<u16, String> {
    if !(1..=8).contains(&id) {
        return Err(format!("no joint M{id}"));
    }
    if delta.abs() > MAX_STEP {
        return Err(format!("steps are at most {MAX_STEP} ticks"));
    }
    let j = id as usize - 1;
    let new = goal[j] as i32 + delta;
    if !sc.in_range(j, new as f64) {
        return Err(format!(
            "M{id}: {new} is outside its calibrated range {}–{}",
            sc.range_min[j], sc.range_max[j]
        ));
    }
    let new = new as u16;
    if j < 7 {
        let from = seven(goal);
        let mut to = from;
        to[j] = new;
        match safety::path_clearance(sc, &from, &to) {
            None => return Err("table height not set: only the gripper (M8) can be moved".into()),
            Some(c) if c < sc.margin_m => {
                return Err(format!("M{id} to {new}: {:.0} mm above the table, margin {:.0} mm", c * 1e3, sc.margin_m * 1e3))
            }
            _ => {}
        }
    }
    Ok(new)
}

/// Check a smooth move from `present` to `to`.
pub fn check_goto(sc: &SafetyConfig, present: &[u16; 7], to: &[u16; 7]) -> Result<(), String> {
    for j in 0..7 {
        if !sc.in_range(j, to[j] as f64) {
            return Err(format!("M{}: {} is outside its calibrated range {}–{}", j + 1, to[j], sc.range_min[j], sc.range_max[j]));
        }
    }
    match safety::path_clearance(sc, present, to) {
        None => Err("table height not set".into()),
        Some(c) if c < sc.margin_m => Err(format!("the way there comes within {:.0} mm of the table", c * 1e3)),
        _ => Ok(()),
    }
}

/// Owns the bus until `Close` (or the screen goes away). Holds every joint where it is on start.
pub async fn run<I: Io>(io: &mut I, sc: SafetyConfig, mut rx: UnboundedReceiver<JogCmd>, tx: UnboundedSender<JogMsg>) -> Result<()> {
    let mut missing = Vec::new();
    for id in 1..=8u8 {
        if !io.ping(id).await {
            missing.push(id);
        }
    }
    if !missing.is_empty() {
        anyhow::bail!("servo(s) {missing:?} do not answer: wrong port, arm not powered, or the station still running");
    }
    let held = io.hold_present(None).await?;
    let mut goal = [0u16; 8];
    goal.copy_from_slice(&held[..8]);
    let _ = tx.send(JogMsg::Info("holding every joint where it is (torque on)".into()));
    let mut st = JogState { goal, ..Default::default() };
    let mut mv: Option<Move> = None;
    let mut tick = tokio::time::interval(CYCLE);
    loop {
        tokio::select! {
            cmd = rx.recv() => match cmd {
                None | Some(JogCmd::Close) => return Ok(()),
                Some(JogCmd::Freeze) => {
                    mv = None;
                    let p = io.hold_present(None).await?;
                    goal.copy_from_slice(&p[..8]);
                    let _ = tx.send(JogMsg::Info("frozen: every joint holds where it is".into()));
                }
                Some(JogCmd::Nudge { id, delta }) => {
                    if mv.is_some() {
                        let _ = tx.send(JogMsg::Warn("wait for the move to finish (Space freezes)".into()));
                        continue;
                    }
                    match check_nudge(&sc, &goal, id, delta) {
                        Ok(new) => {
                            io.write_goals(&[(id, new)]).await?;
                            goal[id as usize - 1] = new;
                        }
                        Err(e) => { let _ = tx.send(JogMsg::Warn(format!("refused: {e}"))); }
                    }
                }
                Some(JogCmd::GoTo { to, secs, name }) => {
                    let from = seven(&st.present);
                    match check_goto(&sc, &from, &to) {
                        Ok(()) => {
                            let _ = tx.send(JogMsg::Info(format!("moving to {name} ({secs:.0} s)")));
                            mv = Some(Move { from, to, t0: Instant::now(), secs });
                        }
                        Err(e) => { let _ = tx.send(JogMsg::Warn(format!("refused: {name}: {e}"))); }
                    }
                }
            },
            _ = tick.tick() => {
                if let Some(m) = &mv {
                    let s = min_jerk((m.t0.elapsed().as_secs_f64() / m.secs).min(1.0));
                    let mut goals = Vec::with_capacity(7);
                    for j in 0..7 {
                        let g = (m.from[j] as f64 + (m.to[j] as f64 - m.from[j] as f64) * s).round() as u16;
                        goal[j] = g;
                        goals.push((j as u8 + 1, g));
                    }
                    io.write_goals(&goals).await?;
                    if s >= 1.0 {
                        mv = None;
                        let _ = tx.send(JogMsg::Info("arrived".into()));
                    }
                }
                for id in 1..=8u8 {
                    if let Ok(s) = io.read_state(id).await {
                        let j = id as usize - 1;
                        st.present[j] = s.position;
                        st.load[j] = s.load;
                        st.current_ma[j] = s.current_ma;
                        st.temp_c[j] = s.temperature_c;
                    }
                }
                st.goal = goal;
                st.moving = mv.is_some() || (0..7).any(|j| (st.present[j] as i32 - goal[j] as i32).abs() > 3);
                st.clearance = safety::clearance(&sc, &st.present[..7]);
                // Runtime check: the measured pose below the margin while moving -> freeze in place.
                if st.moving && st.clearance.is_some_and(|c| c < sc.margin_m) {
                    mv = None;
                    let p = io.hold_present(None).await?;
                    goal.copy_from_slice(&p[..8]);
                    let _ = tx.send(JogMsg::Warn(format!(
                        "SAFETY: {:.0} mm above the table (margin {:.0} mm): frozen in place",
                        st.clearance.unwrap_or(0.0) * 1e3, sc.margin_m * 1e3
                    )));
                }
                let _ = tx.send(JogMsg::State(st));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sc() -> SafetyConfig {
        SafetyConfig { table_z_m: Some(0.0), ..Default::default() }
    }

    const NOW: [u16; 8] = [2002, 1020, 2051, 3160, 1881, 1869, 2244, 2018];

    #[test]
    fn nudges_stay_inside_the_calibrated_range() {
        assert!(check_nudge(&sc(), &NOW, 2, -20).is_ok(), "1000: within the 5-tick tolerance below 1004");
        assert!(check_nudge(&sc(), &NOW, 2, -25).unwrap_err().contains("calibrated range"), "995: 9 below 1004");
        assert!(check_nudge(&sc(), &NOW, 4, 45).unwrap_err().contains("calibrated range"), "M4 3205: above 3196 + 5");
        assert!(check_nudge(&sc(), &NOW, 8, 20).is_ok(), "gripper inside 1237-2859");
        assert!(check_nudge(&sc(), &NOW, 3, 51).unwrap_err().contains("at most"));
    }

    #[test]
    fn nudges_need_the_table_and_keep_the_margin() {
        let no_table = SafetyConfig::default();
        assert!(check_nudge(&no_table, &NOW, 3, 5).unwrap_err().contains("table height"));
        assert!(check_nudge(&no_table, &NOW, 8, 5).is_ok(), "the gripper needs no table");
        // A table just under the current pose: lowering the arm is refused.
        let mut high = sc();
        high.table_z_m = Some(safety::lowest_z(&high, &high.angles(&NOW[..7])) - high.margin_m - 0.002);
        let refused = [-50, 50].iter().any(|&d| check_nudge(&high, &NOW, 2, d).is_err() || check_nudge(&high, &NOW, 3, d).is_err());
        assert!(refused, "some 50-tick move of M2 or M3 must come closer to a table 2 mm below the margin");
    }

    #[test]
    fn goto_checks_range_and_path() {
        let from = seven(&NOW);
        let mut to = from;
        to[1] = 995;
        assert!(check_goto(&sc(), &from, &to).unwrap_err().contains("calibrated range"));
        let level = [2003, 1002, 1942, 3185, 1864, 1892, 2226];
        assert!(check_goto(&sc(), &from, &level).is_ok(), "the taught level pose (M2 2 ticks below its range)");
        to[1] = 1045;
        let r = check_goto(&sc(), &from, &to);
        assert!(r.is_ok(), "{r:?}");
        // The arm is folded: raising M2 alone swings the gripper down; at 1200 it is ~15 mm above the table.
        to[1] = 1200;
        assert!(check_goto(&sc(), &from, &to).unwrap_err().contains("mm of the table"));
    }
}
