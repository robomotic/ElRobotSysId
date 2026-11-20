# CLAUDE.md: ElRobotSysId

Context for continuing this work in a new session. README.md has the tests, options, output format and the MuJoCo plan.

## What this is
`elrobot-sysid`: a Rust CLI that identifies the ElRobot follower's ST3215 servos one joint at a time (hold, steps, reversals,
ramp, chirp) at the full bus rate, for a MuJoCo model (RoboSim #69 Phase 1, sim2real2sim). Sibling repo:
robomotic/PingPongSimBot (ball balancing). Its findings motivated this tool: ~150 ms servo lag and ~20 ticks of tracking error
on M6 cause a 1 Hz limit cycle.

## Status (2026-11-20)
- Written and unit-tested. Built on the Pi earlier (inside `~/paddle-ws`, as a second binary of paddle-balance).
  **Not yet run on the arm.** The user will do the system ID later.
- No analysis or fitting scripts yet. Next: a Python fitter (per joint: dead time + first-order lag + kp/damping/frictionloss,
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
write, path check to the base pose, runtime stop). 3 unit tests. **Open:** the tick→angle convention is unverified. With
sign −1 / zero 2048 the level pose comes out below the base, so the model refuses everything. Next: derive per-joint
sign/zero from the station viewer mapping (URDF limits + each motor's calibrated min/max), and check with
`--show-clearance` and a ruler with the user present.

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
v0.1.0 (2026-11-20): aarch64 binary built on the Pi, stripped, `.sha256` attached, tag at the built commit.
To repeat: on the Pi `bash ~/pp-build.sh` (pulls and builds both repos, log in ~/pp-build.log), then `strip` a copy, run
`sha256sum`, scp it down, and `gh release create vX.Y.Z --target <full sha> --notes-file ... <binary> <binary>.sha256`.
Check the minimum glibc with `objdump -T <bin> | grep -o 'GLIBC_[0-9.]*' | sort -V | tail -1`.
