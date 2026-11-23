# CLAUDE.md: ElRobotSysId

Context for continuing this work in a new session. README.md has the overview, options and tools;
docs/SERVO_MODEL.md has the registers, equations, parameters, validation and the MuJoCo/Unity implementations;
data/sysid_runs/README.md lists the real runs.

## What this is
System identification of the ElRobot follower's ST3215 servos (RoboSim #69, sim2real2sim) and a servo model for MuJoCo
(Python, `tools/`) and Unity (C#, `unity/`). Motivated by PingPongSimBot (~150 ms servo lag, 1 Hz limit cycle on M6).
- `src/` (Rust, runs on the Pi): `elrobot-sysid`. tui.rs (Setup/Running/Results, Pose screen), session.rs (plan,
  run_session on any `Io`, summaries, clearance rows), io.rs (`Io`: Bus + SimIo), bus.rs (serial), safety.rs (FK,
  spheres, clearance, ranges), jog.rs (Pose screen bus task), stall.rs (stall test), settings.rs (paddle.toml subset).
- `tools/` (Python 3.8+, numpy; MuJoCo ones need `~/.pyenv/versions/antifake/bin/python3`, mujoco 3.2.3; not on the
  Pi): fit_electrical, fit_torque, find_load_pose, servo_sim (step loop, replay, heat), replay_stats,
  make_replay_video, make_stall_boom, params_file (TOML), tests.
- `unity/`: St3215Servo.cs (model + TOML loading), ElRobotServoDriver.cs (MjScene glue like RoboSim's
  BipedServoDriver; compile in Unity), St3215Servo.Check (`dotnet run --project unity/St3215Servo.Check`, 19 checks).
- Config: `sysid_safety.toml` (calibrated safety set-up, committed for other users), `servo_params.toml` (fitted
  parameters, `[default]` + `[servo.Mn]`, written by the fitters with `--write-params`).

## State (2026-11-23)
- Five real runs in `data/sysid_runs/` (wrist ±50; M2–M4 ±25 folded; M7 ±150; M2–M4 ±50 and ±75 at the load pose
  M2 1750 / M3 2100 / M4 2450). The ±25 run had UI stalls (fixed: runs and the Pose task have their own thread).
- Measured: R 4.04–4.17 Ω (standstill under load), Ke·N 1.5–2.0 V·s/rad (R fixed), Kt·η·N 1.6–1.8 N·m/A and static
  friction 0.26/0.31/0.41 N·m on M2/M3/M4 (two-sided ramp holds + MuJoCo gravity; M4's are lower bounds), duty
  0.42 %/tick (P 16), duty cap 50 %, dead zone ~4 ticks, apparent dead time 65–110 ms.
- **The current register is the SUPPLY current** |duty|·|i_motor|, magnitude only. Load = duty, sign opposite to speed.
  Gravity-loaded holds read zero load: the gearbox carries the arm by stiction.
- MuJoCo model validated on held-out runs: 0.30–0.37° RMS on M2/M3/M4/M7. Weak spots: ~3-tick gravity offset on M3,
  the step rise (the firmware's duty grows like √error: target rate limiting, not modelled), one friction per joint
  (it is load-dependent: ~0.3 loaded, 0.02–0.05 unloaded wrist). Videos in docs/videos, tables in docs/stats.
- MuJoCo pitfalls: the URDF meshes overlap by up to 28 mm (contacts off); default frictionloss creeps under load
  (dof_solref 0.002, solimp 0.99/0.999/0.001, noslip 10).
- Open: stall test (code done, not run: boom on the M8 horn `hardware/stall_boom_m8.stl` or fingertips on M6),
  12 V torque mismatch (fitted values imply ~5 N·m vs datasheet 2.94), thermal test, load-dependent friction (#2).
- **BAM calibration (#3, #4):** bench pendulum test of one spare ST3215 (D=0 and I=0 in RAM, P sweep, 12 V) for
  Kt/R/Ke/armature without the URDF masses; the leader's ST-3215-C046 (7.4 V, 1:147) too. **The user is still printing
  the pendulum: don't write the bench mode until they say so.** Upstream BAM's STS3215 profile is 7.4 V/1:345 with a
  5 ms command delay: our "dead time" is probably mostly stiction + target rate limiting.
- The withdrawn camera-mount boom: the camera base is fragile (user); never load it.
- Prebuilt release v0.2.1 predates the calibration, Pose screen, stall test and thread fix.

## Safety facts (issue #1, closed; all enforced in src/safety.rs, jog.rs, session.rs)
- Tick → angle: sign +1, zeros 2050.3 2053.0 2051.3 2053.0 1842.7 1924.5 2198.6 (station viewer mapping, checked with
  the viewer, a ruler and the joint directions). Not `zero 2048, sign −1`.
- Calibrated ranges ± 5 ticks (min 1087 1004 935 900 141 1040 134 1237, max 3009 3092 3161 3196 3955 3056 3962
  2859); the servos' own angle limits are 0–4095. The taught level pose has M2 at 1002 (2 ticks below its range).
- Table at 0 mm (base clamped on the table; the table edge runs along the base, the infinite plane is conservative),
  margin 30 mm. Spheres: gripper motor, jaws, wrist camera (42 mm, on top of the gripper, from RoboSim's CAD), paddle
  (payload paddle…), stall boom (payload boom, `--stall 8`).
- The arm is folded at level: raising M2 alone swings the gripper DOWN (M2 1020 → 57 mm, 1100 → 38, 1200 → 15 mm).
  Use `--target` + `g` (straight path checked) or `l`.
- Blocked-joint stop at 90 % of the duty cap (450 here; the old 900 could never fire). Freeze = goal ← present, torque
  on; a moving joint coasts ~1° (servo response delay ~80 ms).
- The app reads `sysid_safety.toml` from the current folder: run it from `~/ElRobotSysId`.

## Hardware and access
- Pi "NormaStation" (Pi 4, Debian 13): `ssh robomotic@normastation.local`; if mDNS fails, the LAN IP (192.168.68.56
  on 2026-11-19; may change). Passwordless sudo. `~/.local/bin/elrobot-sysid` links to the release build.
- Stop the station before running (`sudo systemctl stop normacore-station`; torque stays on), start it afterwards.
- Follower bus: CH343 `5B14033257` (by-id path is the default). Leader: `5B14112350`.
- Build on the Pi: `bash ~/pp-build.sh` (pulls and builds PingPongSimBot and this repo; log ~/pp-build.log); needs
  rustup, `protobuf-compiler`, `libudev-dev`. Long builds over SSH: `nohup … &` with an exit marker; no `pgrep -f`.
- `--config ~/paddle-ws/paddle.toml` (PingPongSimBot) gives the level pose `2003 1002 1942 3185 1864 1892 2226 1440`
  and grip 1396.
- MuJoCo tools need the norma-core URDF at `~/DevOps/norma-core/hardware/elrobot/simulation/` (`ELROBOT_URDF`).

## Working agreements with the user
- **Never move the arm without the user's explicit OK** at that moment. Say which joints move, by how much, and why
  it's safe. `--dry-run` and read-only reads are fine.
- The user runs interactive commands in their own SSH terminal and pastes the output.
- Nothing attached to the gripper unless the user says so (the wrist camera is always on). For M1–M4 use a small
  `--max-amp` (25–75 so far) and check clearance first.
- Commit/push when asked.

## Code notes
- `src/bus.rs` is a copy of PingPongSimBot's bus module (`read_torque_enabled` dropped): keep fixes in sync.
- `src/settings.rs` reads only `[bus].port`, `[bus].timeout_ms`, `[poses].level` and `[poses].grip` from paddle.toml.
- Test signals: `plan()` builds `Segment`s; `Segment::offset(t)` gives ticks from base. Summaries: `step_stats`,
  `reversal_slop`.
- Feetech encodings: speed sign-magnitude in bit 15; load sign in bit 10; current is a magnitude (6.5 mA units).
  The `st3215` crate labels EEPROM 0x12/0x13 one off (0x13 = unload condition, 44).
- Screenshots: regenerate with PingPongSimBot/tools/tui_drive.py `p:` steps (app 130×40, dry runs 120×53, `--sim`
  with the repo's sysid_safety.toml).

## Releases
Built on the Pi, stripped, `.sha256` attached (asset `elrobot-sysid-aarch64-linux`), tag at the built commit; needs
glibc ≥ 2.39. v0.1.0 (`3036410`, line-based CLI + table safety), v0.2.0 (`effa4e0`, full-screen app, `--sim`),
v0.2.1 (`5a35680`, SAFETY STOP, camera in the safety model). To repeat: `bash ~/pp-build.sh` on the Pi, `strip` a
copy, `sha256sum`, scp it down, `gh release create vX.Y.Z --target <full sha> --notes-file … <bin> <bin>.sha256`;
check the minimum glibc with `objdump -T <bin> | grep -o 'GLIBC_[0-9.]*' | sort -V | tail -1`.
