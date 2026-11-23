# CLAUDE.md: ElRobotSysId

Context for continuing this work in a new session. README.md has the tests, options, output format and the MuJoCo plan.

## What this is
`elrobot-sysid`: a Rust CLI that identifies the ElRobot follower's ST3215 servos one joint at a time (hold, steps, reversals,
ramp, chirp) at the full bus rate, for a MuJoCo model (RoboSim #69 Phase 1, sim2real2sim). Sibling repo:
robomotic/PingPongSimBot (ball balancing). Its findings motivated this tool: ~150 ms servo lag and ~20 ticks of tracking error
on M6 cause a 1 Hz limit cycle.

## Full-screen app (2026-11-20)
`elrobot-sysid` (no args) = TUI, `--sim` = simulated servos (src/io.rs SimIo), `--cli` = line-based run, `--dry-run`.
Code: src/session.rs (plan, run_session on any `Io`, summaries, clearance rows, events), src/io.rs (`Io` trait: Bus +
SimIo), src/tui.rs (Setup with live safety per joint, Running with goal-vs-position chart, Results table).
Screenshots in docs/screenshots (regenerate with PingPongSimBot/tools/tui_drive.py `p:` steps). SimIo timing is set
from the real wrist run (80 ms delay, tau 80 ms); the screenshots predate that (they show dead 32 ms).

**Safety additions (user, 2026-11-20):** the paddle is NOT on the arm during sysid (payload defaults to `none`; the
paddle spheres are added only for payload "paddle…"). The wrist camera IS always on: it's in `tool_spheres` (42 mm
sphere on top of the gripper, fitted to RoboSim's CAD-registered U20CAM boxes; 27 mm mount, 24×35×67 mm body).
**Pose screen (2026-11-22, user: "I hate restarting the norma station for just moving the arm"):** `p` in Setup;
src/jog.rs owns the bus in a task (JogCmd Nudge/GoTo/Freeze/Close → JogMsg State/Info/Warn/Closed). Checks every step:
calibrated range ± 5 ticks (`SafetyConfig::range_min/max`, `RANGE_TOL_TICKS`; the taught level has M2 at 1002 < 1004)
and `safety::path_clearance`; runtime freeze below the margin. Runs also refuse sweeps past the range. The arm is
folded at level: raising M2 alone swings the gripper DOWN (M2 1020 → 57 mm, 1100 → 38, 1200 → 15 mm).
SAFETY STOP: Space in the app freezes (goal = present, torque on), then r = return slowly, q = leave; `session::ctl`
RUN/FREEZE/RETURN/LEAVE. A moving joint coasts ~1° (one 30 ms command delay) and settles in its dead zone.

## Status (2026-11-21)
- **First real run: wrist M5–M7, ±50 ticks, nothing attached** (`sysid_logs/1795303354_none` on the Pi; copy in the
  session scratchpad only). ~2 kHz sampling, but 79 % of samples repeat (servo refreshes every ~1–2 ms). Dead time 70–100 ms
  outward, 80–145 ms back to base; t63 110–210 ms; slop M5 9, M6 17, M7 9 ticks; M6 stops ~10 ticks short going negative.
  Servo registers: Max Torque 500 (duty cap 50 %), P 16, protection current 500 (3.25 A), overload 25 % for 2 s → 20 %.
- **Second real run (2026-11-22): M2–M4, ±25 ticks, from M2 1042** (`sysid_logs/1795347460_none`). dead 78/100/108 ms,
  slop 3.8/17/14.5 ticks, M4 steady miss 20. **Holds read load 0 and 0 mA** on M2–M4: the gearbox holds the static gravity
  load by friction, so holds cannot give Kt·η (use up/down ramp asymmetry instead). Duty ≈ 0.4 % per tick of error (P 16),
  so ±25-tick tests reach only ~12 % duty and < 75 mA: too little for the electrical fit; it needs steps of ~100+ ticks
  (e.g. on a wrist roll joint, where the sweep stays clear). The app stalled 26/77/97 s (UI and run shared one thread; a
  blocked SSH terminal stops both): fixed, runs and the Pose task now have their own thread (stall test: 9.5 s gap → 4 ms).
- **Third real run (2026-11-23): M7 alone, amplitude 150** (steps to ±100, `sysid_logs/1795366217_none`): dead 78 ms,
  t63 171 ms, slop 4.8; |load| ≤ 17.6 %, current ≤ 46 mA (mean 6 mA while moving). **Ke·N from duty·V_bus vs speed:**
  M7 2.54/2.53 (two runs), M2 2.95, M3 2.80, M5 3.08, M6 2.93 V·s/rad (corr ~0.95); load sign is opposite to speed.
  R still unidentified (needs ≥ 150 mA): fitter falls back to the datasheet 4.4 Ω.
- **Fourth run (2026-11-23): M2–M4 ±50 at the load pose** (`sysid_logs/1795395590_none`, base 2000 1753 2103 2482).
  **The reported current is the SUPPLY current, |duty|·i_motor**: standstill R = 4.17/4.04/4.17 Ω (median; IQR ~3.8–4.4).
  With R fixed, Ke·N 1.6–2.0 V·s/rad on all runs (the earlier 2.5–3.1 "motion only" values were biased: the motor
  current is ~10× the reported one). Two-sided holds + MuJoCo gravity: Kt·η·N 1.69/1.62/1.84 N·m/A, static friction
  0.25/0.30/0.41 N·m (M2/M3/M4). Open: these imply ~5 N·m at 12 V vs the datasheet's 2.94 (URDF masses? datasheet?).
  SimIo reports supply current too. The Kt script was ad hoc (scratchpad loadpose/kt.py), not in the repo yet.
- **Load pose for R (2026-11-23, MuJoCo search on the norma-core URDF, arm 0.62 kg):** M2 1750, M3 2100, M4 2450
  (others as now): arm half upright, gripper forward ~200 mm, 91 mm above the table; gravity M2/M3/M4 0.31/0.48/0.42 N m.
  Reach it with the Pose screen `g` (`--target 2001,1750,2100,2450,1881,1869,2244`): straight path never below 51 mm,
  back to level ≥ 85 mm. Moving M2 alone first would hit the table (the Pose screen refuses it). Search scripts were
  in the session scratchpad (loadpose/search*.py), not in the repo.
- **MuJoCo servo step loop (2026-11-23):** tools/servo_sim.py (+ elrobot_model.py, fit_torque.py, test_servo_sim.py;
  run with ~/.pyenv/versions/antifake/bin/python3, mujoco 3.2.3; not on the Pi). Pitfalls found: URDF meshes overlap
  (contacts off), MuJoCo frictionloss creeps under sustained load (dof_solref 0.002, solimp 0.99/0.999, noslip 10).
  Replay fits: M2 4.2, M3 4.5, M4 2.9, M7 4.9 ticks RMS; kp_duty from the log (0.0042/tick M2–M4, 0.0021 M7).
  Friction is load-dependent (0.3 loaded, 0.05 unloaded wrist): next model step. User (2026-11-23): printed parts make
  the URDF masses uncertain → add a stall test against a kitchen scale (torque without masses).
- **Stall test (2026-11-23):** src/stall.rs + `--stall J --into ±1 --lever-mm L [--stall-max 40]` (line-based, types
  grams per level; tare at offset 0; fit torque = k·i + b ∓ f by 3-parameter least squares). Not run on the arm yet.
  SimIo duty slope set to the real 0.42 %/tick (was 2.5).
- **Stall boom (2026-11-23):** first design on the camera mount withdrawn (user: the camera base is fragile; ~0.85 N·m
  would have gone through it). **Now (user's idea): on the M8 horn, fingers off** — hardware/stall_boom_m8.stl from
  tools/make_stall_boom.py: interface copied from norma-core Gripper_Gear (4 × M3 on Ø14, Ø7 centre, Ø23.5 × 3.15 recess,
  2.85 mm flange: the gear's screws fit), 20 mm standoff tube (gripper base reaches y 57 mm, ±40 mm sideways; horn face
  y ≈ 45), arm 100 mm. M8 axis = gripper y; at level 129 mm above the table. `boom_spheres` model it (payload "boom";
  `--stall 8` adds them). Mark the gear's angle before removing it (gripper calibration). Not printed/run yet.
  Alternative without printing: fingertips on M6 (--into 1, lever ~160 mm). --stall-max default 20.
- **Electrical/thermal model (issue #2):** meta.toml decodes the protection registers; `--sim` servos have a DC motor
  model (R 4.4 Ω, Ke·N 2.5 V·s/rad); `--sim --cli` runs headless; `tools/fit_electrical.py` fits
  `|duty·V_bus − Ke·N·ω| = R·|i|` (the ST3215 current is a magnitude: never negative on the arm) and the supply-current
  alternative. Temperature/voltage glitch for 1–4 samples (26 → 99 °C): 51-sample median. Wrist currents < 60 mA are too
  small to fit (flagged); needs M2–M4. Open: Kt·η from gravity torques at holds, thermal test, MuJoCo step loop.
- **BAM calibration (issue #3, 2026-11-23):** a bench pendulum test of one spare ST3215 (D=0 and I=0 in RAM, P sweep, 12 V)
  for Kt/R/Ke/armature without the URDF masses; friction is refitted per joint on the arm logs. The user is still printing
  the pendulum: don't write the bench mode until they say so.
- Elrobot-sysid on the Pi: `~/.local/bin/elrobot-sysid` links to the release build; run it from `~/ElRobotSysId` (it reads
  `sysid_safety.toml` and writes `sysid_logs/` in the current folder).
- Next: a Python fitter (per joint: dead time + first-order lag + kp/damping/frictionloss,
  then a check against the chirp), then the MuJoCo actuator parameters.

## Requirement: safety area (table) before any further run (user, 2026-11-20)
The arm sits on a table and **must never hit it**. The per-joint excursion cap (`--max-amp`) is not enough: a shoulder or
elbow move of a few degrees swings the paddle or gripper a long way. Before sysid runs on the arm (and on M1–M4 in
particular), add a workspace safety check (GitHub issue #1):
- full forward kinematics with positions (URDF joint origins, not only the axes) for the gripper, the paddle rim and the
  elbow/wrist links;
- a configurable keep-out region: table plane height plus margin (and optionally a box around the base or other obstacles);
- **before moving:** check the base pose and every extreme of the planned signal for each joint; refuse joints/amplitudes
  that would enter the keep-out region, and say by how much;
- **while running:** stop and return to base if the measured pose comes within the margin;
- the table height must be measured or taught (e.g. jog the gripper down to just above the table and record it), not guessed.

**Progress on the safety area (2026-11-20):** `src/safety.rs` (FK with URDF positions, tool spheres, sweep clearance,
safe amplitude) is wired into the CLI (`--set-table-z-mm`, `--show-clearance`, dry-run report, refusal before any servo
write, path check to the base pose, runtime stop). 3 unit tests.
**Calibrated 2026-11-21 with the user:** sign +1, zeros from the station viewer mapping (URDF limits + follower ranges in
PingPongSimBot tools/throw_sim.py), matched at level; the viewer matches the real arm at level. Table at 0 mm (base clamped
on the table; the base mesh starts at z = 0; the table edge runs along the base, the model's infinite plane is
conservative). Camera sphere from RoboSim's CAD-registered U20CAM boxes (on top of the gripper, +z of Gripper_Base_v1_1).
Ruler: gripper motor centre ~114 mm vs model 108; lowest gripper point ~80 mm vs spheres 56 mm. Same values are the code
defaults and in the repo's commented `sysid_safety.toml` (committed 2026-11-23 for other users; table_z_m = 0). Sign check confirmed by the user
(2026-11-22): the viewer arm turns the same way as the real one for M2–M4. Next: M2–M4 run at `--max-amp 30` (dry-run
first), when the user is back.

## Hardware and access
- Pi "NormaStation" (Pi 4, Debian 13): `ssh robomotic@normastation.local`. If mDNS fails (Wi-Fi drops), use the LAN IP
  (192.168.68.56 on 2026-11-19; may change). Passwordless sudo works.
- Stop the station before running: `sudo systemctl stop normacore-station`; start it again afterwards. Servo torque stays on
  when it stops.
- Follower bus: CH343 serial `5B14033257` (`/dev/ttyACM1`, use the by-id path; that's the default). Leader: `5B14112350`.
- Build dependencies on the Pi: rustup toolchain, `protobuf-compiler`, `libudev-dev`.
- `--config` can point at PingPongSimBot's `paddle.toml` (on the Pi: `~/paddle-ws/paddle.toml`) for the level pose
  `2003 1002 1942 3185 1864 1892 2226 1440` and grip 1396.

## Working agreements with the user
- **Never move the arm without the user's explicit OK** at that moment. Say which joints move and by how much.
  `--dry-run` is always fine.
- The user runs interactive commands in their own SSH terminal and pastes the output. This tool asks [y/N] unless `--yes`.
- For M1–M4, take the paddle off first and use a smaller `--max-amp` (e.g. 50): those joints carry the whole arm.
- Long builds over SSH: `nohup … &` plus an exit marker in a log. Don't use `pgrep -f` over SSH (it matches itself).

## Code notes
- `src/bus.rs` is a copy of PingPongSimBot's bus module (`read_torque_enabled` dropped). Keep fixes in sync, or extract a shared
  crate if more tools appear.
- `src/settings.rs` reads only `[bus].port`, `[bus].timeout_ms`, `[poses].level` and `[poses].grip` from a paddle.toml;
  everything else in that file is ignored.
- Test signals: `plan()` builds `Segment`s; `Segment::offset(t)` gives ticks from base. Summaries: `step_stats`,
  `reversal_slop`.
- Feetech encodings: speed and current are sign-magnitude in bit 15; load has its sign in bit 10.

## Releases
All built on the Pi, stripped, `.sha256` attached (asset `elrobot-sysid-aarch64-linux`), tag at the built commit;
the binary needs glibc ≥ 2.39.
- v0.1.0 (2026-11-20, `3036410`): line-based CLI with the table safety area.
- v0.2.0 (2026-11-20, `effa4e0`): full-screen app (Setup/Running/Results), `--sim` simulated servos, `--cli`.
- v0.2.1 (2026-11-20, `5a35680`, latest): SAFETY STOP (Space), wrist camera always in the safety model, payload
  defaults to `none`.
To repeat: on the Pi `bash ~/pp-build.sh` (pulls and builds both repos, log in ~/pp-build.log), then `strip` a copy, run
`sha256sum`, scp it down, and `gh release create vX.Y.Z --target <full sha> --notes-file ... <binary> <binary>.sha256`.
Check the minimum glibc with `objdump -T <bin> | grep -o 'GLIBC_[0-9.]*' | sort -V | tail -1`.
