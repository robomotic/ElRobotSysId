# Real-arm sysid runs

The five system-identification runs on the ElRobot follower (bus `5B14033257`), copied unchanged from the Pi's
`~/ElRobotSysId/sysid_logs/`. Every number in [docs/SERVO_MODEL.md](../../docs/SERVO_MODEL.md),
[servo_params.toml](../../servo_params.toml) and `docs/stats/` comes from these files. Each folder has `meta.toml`
(arguments, base pose, the EEPROM/RAM of all 8 servos, the per-joint summary) and one `M<n>.csv` per joint
(`t, segment, test, goal, position, speed, load, current_ma, voltage_dv, temp_c`; ~2 kHz). Nothing was attached to the
gripper (payload `none`); the wrist camera was on. All five tests (hold, steps, reversals, ramp, chirp) in every run.

| Run | Date (UTC) | Joints | Amplitude | Pose | Used for | Notes |
|---|---|---|---|---|---|---|
| `1795303354_none` | 2026-11-21 23:22 | M5, M6, M7 | ±50 | folded, near level | first run; wrist timing and slop; M5–M7 `duty_per_tick`/dead zone/friction in `servo_params.toml`; M7 parameters for the held-out test | line-based run |
| `1795347460_none` | 2026-11-22 11:37 | M2, M3, M4 | ±25 | folded (M2 1042, moved off its range end) | holds read zero load → gearbox stiction; Ke·N on M2/M3 | **stalls of 26/77/97 s** in M3/M4 (UI and run on one thread, fixed in c2f734c): M3/M4 chirps cut short |
| `1795366217_none` | 2026-11-22 16:50 | M7 | ±150 (steps to ±100) | folded | Ke·N from motion; **held-out** test of the M7 model (0.36° RMS) | |
| `1795395590_none` | 2026-11-23 00:59 | M2, M3, M4 | ±50 | **gravity-loaded** (M2 1750, M3 2100, M4 2450, reached with the Pose screen `g`) | R at standstill (supply current), Ke·N with R fixed, Kt·η·N and static friction (`fit_torque.py`); fitted replays; `servo_params.toml` M2–M4 | |
| `1795433098_none` | 2026-11-23 11:24 | M2, M3, M4 | ±75 | gravity-loaded (same pose) | **held-out** test of M2/M3/M4 with parameters from `1795395590` (0.30–0.37° RMS) | |

Re-run the analysis, for example:

```bash
python3 tools/fit_electrical.py data/sysid_runs/1795395590_none
python3 tools/fit_torque.py data/sysid_runs/1795395590_none          # needs mujoco
python3 tools/replay_stats.py data/sysid_runs/1795433098_none --joint 3 --fit-on data/sysid_runs/1795395590_none
```

New runs land in `sysid_logs/` (ignored by git); copy the ones worth keeping here.
