# The ST3215 servo model: registers, equations, system identification, MuJoCo and Unity

This document collects what the ElRobot sysid work (issues [#1](https://github.com/robomotic/ElRobotSysId/issues/1),
[#2](https://github.com/robomotic/ElRobotSysId/issues/2)) established about the Feetech/Waveshare **ST3215** servos of
the ElRobot follower (12 V version), and how the model is implemented in MuJoCo (Python, `tools/`) and in C# for
Unity (`unity/`). All numbers were measured on one follower (bus `5B14033257`) between 2026-11-21 and 2026-11-23,
unless marked *datasheet* or *placeholder*.

Contents:
1. [Units and conventions](#1-units-and-conventions)
2. [Registers the servo can read and write](#2-registers)
3. [The model: equations](#3-the-model-equations)
4. [Measured parameters](#4-measured-parameters)
5. [How the system identification works](#5-how-the-system-identification-works)
6. [How well the model matches the real arm](#6-how-well-the-model-matches-the-real-arm)
7. [MuJoCo implementation (Python)](#7-mujoco-implementation-python)
8. [Unity / C# implementation](#8-unity--c-implementation)
9. [Open points](#9-open-points)

---

## 1. Units and conventions

| Quantity | Unit on the bus | Conversion |
|---|---|---|
| position | ticks | 4096 per output turn: 1 tick = 0.0879° = 1.534 mrad |
| speed | ticks/s, sign-magnitude in bit 15 | ω [rad/s] = ticks/s × 2π/4096 |
| load | 0.1 % of full PWM, sign in bit 10 | **duty** = load / 1000 (it is the PWM duty, not a torque) |
| current | 6.5 mA per unit | **supply** current, magnitude only (see §3.4) |
| voltage | 0.1 V | V_bus = value / 10 |
| temperature | °C | case/board sensor; 1 °C steps, short glitches (§3.6) |

- **Joint angle**: `q [rad] = sign × (ticks − zero) × 2π/4096`, with `sign = +1` and per-joint `zero` from the station
  viewer's mapping (each motor's calibrated tick range spread over the URDF joint limits, matched at the level pose).
  Values in [`sysid_safety.toml`](../sysid_safety.toml): zeros 2050.3, 2053.0, 2051.3, 2053.0, 1842.7, 1924.5, 2198.6
  for M1…M7. Not `zero 2048, sign −1`: that is only right for small relative moves.
- **Sign of the load**: on the follower the reported load has the **opposite sign to the speed** (driving towards higher
  ticks reads as negative load). The fitters detect it; the models below use the physical sign (positive duty drives
  towards increasing ticks).
- **Output side**: R, Ke·N, Kt·η·N, friction and inertia are all expressed at the joint (after the gearbox of ratio N),
  so N never has to be known separately.

---

## 2. Registers

Memory map of the ST3215 (Feetech STS protocol; addresses as in the `st3215` crate of norma-core, `protocol/memory.rs`),
with the values read from the follower's servos (all 8 identical except ID and offset). EEPROM writes are persistent
and change the station's behaviour too: none are made by this project.

### EEPROM (0x00–0x27), read by `meta.toml` at every run

| Addr | Size | Name | Follower value | Meaning / use in the model |
|---|---|---|---|---|
| 0x00 | 2 | Model number | 2563 (0x0A03) | |
| 0x02 | 1 | Firmware version | 0 | (0x03–0x04 read 9, 3: not decoded) |
| 0x05 | 1 | ID | 1–8 | |
| 0x06 | 1 | Baud rate | 0 | 1 Mbaud |
| 0x07 | 1 | Return delay | 0 | |
| 0x08 | 1 | Response status level | 1 | |
| 0x09 | 2 | Min angle limit | 0 | no servo-side angle limit: the calibrated range (station) is the only end stop → enforced in software (`range_min/max`) |
| 0x0B | 2 | Max angle limit | 4095 | |
| 0x0D | 1 | Max temperature | 70 °C | servo unloads above it (`MaxTemperatureC`) |
| 0x0E | 1 | Max input voltage | 140 (14.0 V) | |
| 0x0F | 1 | Min input voltage | 40 (4.0 V) | |
| 0x10 | 2 | Max torque | **500** (50 %) | **duty cap** 0.5 |
| 0x12 | 1 | (phase / setting byte) | 12 (0x0C) | the `st3215` crate labels this "Unload condition": **wrong by one**, see 0x13 |
| 0x13 | 1 | **Unload condition** | 44 (0x2C) | bits 2 temperature, 3 current, 5 overload: what makes the servo cut/limit torque (Feetech default 44; the crate calls this "LED alarm") |
| 0x14 | 1 | LED alarm condition | 47 (0x2F) | Feetech default 47 |
| 0x15 | 1 | P coefficient | **16** | position loop gain (Feetech default 32): measured as **0.42 % duty per tick** of error |
| 0x16 | 1 | D coefficient | 0 | |
| 0x17 | 1 | I coefficient | 0 | |
| 0x18 | 2 | Min startup force | 16 | |
| 0x1A | 1 | CW dead zone | 1 | the effective dead zone is larger: ~4 ticks, plus gearbox slop (§4) |
| 0x1B | 1 | CCW dead zone | 1 | |
| 0x1C | 2 | Protection current | **500** (× 6.5 mA = 3.25 A) | protection threshold (motor current) |
| 0x1E | 1 | Angular resolution | 1 | |
| 0x1F | 2 | Position offset | e.g. 0xF8A8 | calibration offset (Feetech: bit 11 is the sign) |
| 0x21 | 1 | Mode | 0 | position mode |
| 0x22 | 1 | Protective torque | **20** % | duty cap after a protection trips |
| 0x23 | 1 | Protection time | **200** (× 10 ms = 2 s) | overload must last this long to trip |
| 0x24 | 1 | Overload torque | **25** % | duty that counts as overload |
| 0x25 | 1 | Speed closed-loop P | 10 | |
| 0x26 | 1 | Over-current protection time | 200 (2 s) | |
| 0x27 | 1 | Velocity closed-loop I | 200 | |

### RAM (0x28–0x46)

| Addr | Size | Name | R/W | Follower / use |
|---|---|---|---|---|
| 0x28 | 1 | Torque enable | RW | 1; the station leaves torque on when it stops |
| 0x29 | 1 | Acceleration | RW | 0 = servo maximum (sysid `--acc`, restored after a run) |
| 0x2A | 2 | **Goal position** | RW | the command (sync-write to all joints) |
| 0x2C | 2 | Goal time | RW | 0 |
| 0x2E | 2 | Goal speed | RW | 0 = maximum |
| 0x30 | 2 | Torque limit | RW | **500** (50 %): the duty cap is min(Max torque, Torque limit) |
| 0x37 | 1 | Lock (EEPROM) | RW | |
| 0x38 | 2 | **Present position** | R | ticks |
| 0x3A | 2 | **Present speed** | R | ticks/s, sign-magnitude bit 15 |
| 0x3C | 2 | **Present load** | R | duty × 1000, sign bit 10 (opposite to speed on this arm) |
| 0x3E | 1 | **Present voltage** | R | 0.1 V (12.1–12.3 V; dips are mostly read glitches) |
| 0x3F | 1 | **Present temperature** | R | °C (case/board) |
| 0x41 | 1 | Status | R | error flags |
| 0x42 | 1 | Moving | R | |
| 0x45 | 2 | **Present current** | R | × 6.5 mA, **supply current, magnitude** (never negative on the arm) |

The sysid tool reads 0x38–0x46 in one bulk read per sample: **~2 kHz** per joint, though ~79 % of samples repeat the
previous state (the servo refreshes its registers every ~1–2 ms).

---

## 3. The model: equations

Per servo, at the joint output. Symbols: e = goal − position (ticks), D = duty, V = motor voltage, i = motor current,
ω = joint speed (rad/s, in the direction of increasing ticks), τ = motor torque at the joint.

### 3.1 Firmware position loop

```
goal_active(t) = goal(t − d)                           transport + firmware delay, d ≈ 80 ms
e = goal_active − position                             ticks
D = 0                                    if |e| ≤ e_dz            dead zone, e_dz ≈ 4 ticks
D = clamp(k_d · e, −D_cap, +D_cap)       otherwise                k_d ≈ 0.0042 per tick, D_cap = 0.5
D_cap = min(Max torque, Torque limit)/1000;  0.20 after a protection trip;  0 above the max temperature
```

Observed but **not in the model yet**: for steps of 10–100 ticks the peak duty grows with the square root of the error
(≈ 17 % × √(e/100)), which looks like the firmware's acceleration-limited motion profile. The linear law makes the model
start ~15–25 ms late and rise faster on big steps (§6).

### 3.2 Motor (electrical)

```
V = D · V_bus
V = R(T) · i + L di/dt + Ke·N · ω          L/R ≪ 1 ms, so L is neglected:
i = (D · V_bus − Ke·N · ω) / R(T)
R(T) = R_25 · (1 + 0.0039 · (T_w − 25 °C))   copper
```

### 3.3 Torque and friction

```
τ_motor = Kt·η·N · i                         (Kt = Ke in SI for an ideal motor; η = gearbox efficiency)
J_total q̈ = τ_motor + τ_gravity − τ_static_friction − b q̇        (rigid body: MuJoCo)
```

The gearbox's **static friction** is large: about as large as the gravity load on a loaded joint, which is why a
gravity-loaded hold reads **zero** load and current (the gearbox carries the arm). It depends on the load: ~0.3 N·m on
the loaded shoulder/elbow, ~0.02–0.05 N·m on the unloaded wrist roll.

### 3.4 What the servo reports

```
Present Current = |D| · |i| / 6.5 mA          the SUPPLY current (PWM-averaged), magnitude only
Present Load    = −D · 1000 (follower sign)   the duty
```

This is why the current looked tiny: at 10–15 % duty a 0.5 A motor current reads 50–75 mA. Evidence: at standstill
under load (no back-EMF) `R = |V|·|D|/I_reported` gives 4.04–4.17 Ω on three joints (datasheet 4.4 Ω); read as motor
current it would be an implausible 25–36 Ω.

### 3.5 Protection (servo firmware)

```
overload = |i| > I_prot (3.25 A)  or  |D| > D_overload (0.25)
overload for > t_prot (2 s)        →  tripped: D_cap = D_protective (0.20)
no overload for > t_prot           →  released
case temperature > T_max (70 °C)   →  unloaded (D = 0)
```

### 3.6 Heating (two nodes)

```
C_w dT_w/dt = i² R(T_w) − (T_w − T_c)/R_wc        winding
C_c dT_c/dt = (T_w − T_c)/R_wc − (T_c − T_amb)/R_ca   case/board = what the servo reports
```

Constants are **placeholders** (C_w 3 J/K, C_c 40 J/K, R_wc 3 K/W, R_ca 15 K/W) until a thermal test. The temperature
register glitches for 1–4 samples in clusters (e.g. 26 → 99 °C during motion): use a ≥ 25 ms median filter.

### 3.7 MJCF-only alternative

Without a step loop, the motor alone is an affine MuJoCo actuator with `ctrl = V`:

```xml
<!-- τ = (Kt·η·N / R) · V − (Kt·η·N · Ke·N / R) · q̇ -->
<general joint="rev_motor_03" ctrlrange="-6.05 6.05" gaintype="fixed" gainprm="0.415"
         biastype="affine" biasprm="0 0 -0.746"/>
```

(values for Kt·η·N 1.7, Ke·N 1.8, R 4.1, cap 0.5 × 12.1 V). The dead zone, delay, protection, heating and the reported
current need the step loop (§7, §8).

---

## 4. Measured parameters

**The machine-readable source of these numbers is [`servo_params.toml`](../servo_params.toml)** (issue #5): `[default]`
for every joint, `[servo.Mn]` for what was measured on joint n. The fitters write it (`--write-params` on
`tools/fit_electrical.py`, `tools/fit_torque.py`, `tools/servo_sim.py replay --fit`); `tools/servo_sim.py` and
`unity/St3215Servo.cs` (`St3215Params.Load`) read it. The calibration (zero, sign, ranges) stays in
`sysid_safety.toml`. The table below summarises it.

| Parameter | Value | Source (run) | Notes |
|---|---|---|---|
| R | 4.04 / 4.17 / 4.17 Ω (M3/M2/M4) | standstill under load, `fit_electrical.py` (2026-11-23 load pose) | datasheet 4.4 Ω |
| Ke·N | 1.5–2.0 V·s/rad | all runs, R fixed, `fit_electrical.py` | consistent loaded and unloaded |
| Kt·η·N | 1.71 / 1.62 / 1.84 N·m/A (M2/M3/M4) | two-sided holds + MuJoCo gravity, `fit_torque.py` | relies on URDF masses (printed parts): to confirm with the stall test |
| static friction | 0.26 / 0.31 / 0.41 N·m (M2/M3/M4, loaded); 0.02–0.05 (M7, unloaded) | holds and replay fits | load dependent |
| k_d (duty per tick) | 0.0042 (M2–M4), 0.0021–0.0038 (wrist M5–M7, unloaded) | `servo_sim.py fit_kp` | firmware P 16; the same 0.0042 in `[default]` |
| dead zone | ~4 ticks | replay fits | EEPROM says 1 |
| delay | 65–110 ms outward, up to 145 ms back to base | step dead times (sysid summary) | model: 80 ms |
| reversal slop | M2 4–10, M3 17–18, M4 0–15, M5 9, M6 17, M7 5–9 ticks | reversals test | gearbox backlash + dead zone |
| duty cap | 50 % | EEPROM 0x10, RAM 0x30 | |
| protection | 3.25 A / 25 % for 2 s → 20 % | EEPROM | |
| thermal | placeholders | none yet | needs a thermal test |

Implied torque at full 12 V stall: Kt·η·N × 12 V / R ≈ 5 N·m, against the datasheet's 2.94 N·m. One of the two (or
the URDF masses behind Kt·η·N) is off: the **stall test** (`--stall`, §5.4) measures torque against a kitchen scale,
without masses.

---

## 5. How the system identification works

### 5.1 The tests (per joint, one joint at a time, all others holding)

| Test | Signal | What it identifies |
|---|---|---|
| hold | 2 s still | static load and current (zero under gearbox stiction!) |
| steps | ±10, ±25, ±50, ±100 ticks and back | delay, t63, overshoot, steady miss |
| reversals | slow triangles 5/10/20 ticks | slop (backlash + dead zone) |
| ramp | 15 ticks/s out, across, back, with holds | friction; the holds at each end (`ramp_hold+/−`), approached from both sides, give Kt·η·N and static friction |
| chirp | 0.2 → 4 Hz sweep | bandwidth, phase lag |

Amplitude is capped (`--max-amp`, hard cap 200 ticks) and by the joint's calibrated range ± 5 ticks.

### 5.2 Safety (issue #1)

Forward kinematics on the URDF joint origins plus spheres around the gripper, the wrist camera (fitted to RoboSim's
CAD-registered U20CAM) and optional payloads (paddle, stall boom). Before anything moves: every sweep and the path to
the base pose must stay ≥ 30 mm above the table and inside the calibrated ranges. While running: stop below the
margin, on a blocked joint (90 % of the duty cap for 0.3 s), or > 300 ticks from the goal; Space freezes. The Pose
screen moves the arm without the station with the same checks.

### 5.3 Pipeline

1. `elrobot-sysid` (app or `--cli`) → `sysid_logs/<stamp>_<payload>/meta.toml` + `M<n>.csv` (the five real runs
   behind this document are in [`data/sysid_runs/`](../data/sysid_runs); t, segment, test, goal,
   position, speed, load, current_ma, voltage_dv, temp_c).
2. `tools/fit_electrical.py <session>`: sign convention, R at standstill under load, Ke·N with R fixed (§3.2, §3.4).
3. `tools/fit_torque.py <session>`: Kt·η·N and static friction from the two-sided holds and MuJoCo gravity
   (needs a gravity-loaded pose, e.g. M2 1750 / M3 2100 / M4 2450, found with `tools/find_load_pose.py` and reached
   with the Pose screen `g` and `--target`).
4. `tools/servo_sim.py replay <session> --joint N --fit`: the full step loop in MuJoCo driven by the logged goals;
   fits dead zone and static friction; `heat` simulates a hold with protection and heating.
5. `tools/replay_stats.py` and `tools/make_replay_video.py`: validation statistics and real-vs-sim videos (§6).

### 5.4 Stall test

`elrobot-sysid --stall <joint> --into ±1 --lever-mm <mm>`: presses a lever into a kitchen scale at 0, 3, 5, 10, 15,
20, …, 0 ticks; at each level it holds, measures duty and current and asks for the reading in grams. Fit:
`τ_scale = Kt·η·N · i + b ∓ friction` (approaching up / down). Set-ups: the printed boom on the M8 horn
(`hardware/stall_boom_m8.stl`, fingers off) or the closed fingertips on M6. Not run yet.

---

## 6. How well the model matches the real arm

Validated with the recommended practice: on **held-out** runs (parameters fitted on another run), errors in physical
units, bias reported separately, MATLAB-style normalised fit `100 (1 − ‖y − ŷ‖/‖y − ȳ‖)`, per test type.

| Joint | Held-out run | RMS | at the tool | median | 95 % | bias | fit % |
|---|---|---|---|---|---|---|---|
| M2 | ±75 at the load pose (params from ±50) | 4.3 ticks = 0.37° | 1.7 mm (fingertip) | 3.1 | 8.3 | +1.5 | 67.7 |
| M3 | ±75 at the load pose (params from ±50) | 4.0 ticks = 0.35° | 1.8 mm | 3.8 | 6.4 | +2.9 | 63.4 |
| M4 | ±75 at the load pose (params from ±50) | 3.4 ticks = 0.30° | 1.5 mm | 2.0 | 4.6 | +1.1 | 71.4 |
| M7 | ±150 (params from the ±50 run) | 4.0 ticks = 0.36° | 0.4 mm (camera lens) | 1.3 | 8.5 | +0.3 | 82.8 |

Held-out errors equal the in-sample ones (M2 4.26/4.26, M3 4.00/3.94, M4 3.40/3.42, M7 4.04/4.12): the parameters
transfer. Slow motions (ramps, reversals) are within ~1.4–3.5 ticks, near the resolution and dead zone. Weak spots: a
constant offset of ~3 ticks on M3 (the gravity/friction/torque balance: masses or Kt·η·N), the step rise (§3.1), and
one-friction-for-all-joints. Per-part tables: `docs/stats/*.csv`; videos: `docs/videos/` (real arm blue, MuJoCo ghost
orange, motion ×5).

---

## 7. MuJoCo implementation (Python)

`tools/servo_sim.py` (`ArmSim`) with `tools/elrobot_model.py`:

- **Model**: the norma-core URDF (`elrobot_follower.urdf`) loaded directly; ticks ↔ angle as §1.
- **Torque**: the servo step loop (§3.1–3.5) runs in Python every 1 ms step and writes the motor torque to
  `data.qfrc_applied` of each joint dof (no MuJoCo actuators needed).
- **Stiction**: `model.dof_frictionloss = static friction`, made stiff: `dof_solref = (0.002, 1)`,
  `dof_solimp = (0.99, 0.999, 0.001, 0.5, 2)`, `opt.noslip_iterations = 10`. With MuJoCo's defaults a held arm crept
  20+ ticks/s under 0.48 N·m against 0.8 N·m of "friction".
- **Contacts off** (`mjDSBL_CONTACT`): the URDF meshes overlap by up to 28 mm (base ↔ first link, gripper parts) and the
  contact solver kicked the joints apart. Collisions are not part of the servo model.
- `dof_damping` 0.05 N·m·s/rad and `dof_armature` 0.01 kg·m² (small effect; not identified).
- Readings: position, speed, load = duty × 1000, current = |D|·|i| (supply), temperature = T_c.
- Validation: `replay` replays logged goals (initial joint position from the log), `--fit` grids dead zone × friction.

---

## 8. Unity / C# implementation

Files in [`unity/`](../unity):

| File | What |
|---|---|
| `St3215Servo.cs` | **the servo model in plain C#** (no Unity/MuJoCo types): `St3215Params` (all §4 numbers as defaults; `Load(servo_params.toml, joint)`, `LoadCalibration(sysid_safety.toml, joint)`, and a minimal TOML reader `St3215Toml`) and `St3215Servo` (`Command(goalTicks)`, `Step(dt, angleRad, velocityRadS) → torque`, readings: `Position`, `Speed`, `Load`, `ReportedCurrentMilliamp`, `TemperatureC`, internals `Duty`, `MotorCurrent`, `WindingC`, `Tripped`) |
| `ElRobotServoDriver.cs` | Unity/MuJoCo glue in the style of RoboSim's `BipedServoDriver`: finds each joint's `MjHingeJoint` (qpos/dof address) and `MjActuator` by name in `postInitEvent`, sets the stiff frictionloss and noslip on the compiled model, and in every `preUpdateEvent` writes `data->ctrl[actuator] = servo.Step(dt, qpos, qvel)` |
| `St3215Servo.Check/` | `dotnet run --project unity/St3215Servo.Check`: 19 checks of the C# model (electrical law, supply current, delay, protection trip and cap, thermal steady state, closed-loop step, sign) and of loading `servo_params.toml` / `sysid_safety.toml` (`[default]` reproduces `St3215Params.Default()`) |

Use in a scene, with both TOML files imported as text assets (`servoParams`, `sysidSafety`):

```csharp
using ElRobotSysId;

var names = new[] { "rev_motor_01", "rev_motor_02", "rev_motor_03", "rev_motor_04",
                    "rev_motor_05", "rev_motor_06", "rev_motor_07" };
var level = new[] { 2003, 1002, 1942, 3185, 1864, 1892, 2226 };
var servos = new St3215Servo[7];
for (int i = 0; i < 7; i++)
{
    var p = St3215Params.Load(servoParams.text, i + 1)        // [default] + [servo.M<i+1>]
                        .LoadCalibration(sysidSafety.text, i + 1); // zero, sign
    servos[i] = new St3215Servo(p, level[i]);
}
// friction, damping and armature come from each servo's parameters
var driver = new ElRobotServoDriver(MjScene.Instance, robotRoot, names, servos);

// whenever the controller sends a goal (like a bus write):
driver.Command(2, 2100);
// readings, e.g. for a UI or for checking a controller against the real servo:
double mA = driver.Servos[2].ReportedCurrentMilliamp, temp = driver.Servos[2].TemperatureC;
```

Scene requirements: one torque motor per joint (`MjActuator`, gear 1, no gain/bias, ctrlrange ≥ the servo's peak
torque, named like the joint), no MuJoCo position servo on those joints, timestep 1 ms, unsafe code enabled. The
joint's sign must match the MuJoCo axis (`St3215Params.Sign`; +1 for the norma-core URDF).

---

## 9. Open points

- Stall test (torque without URDF masses) and the 12 V torque mismatch (§4).
- Load-dependent static friction (one value per joint today).
- The square-root duty law of the firmware for large steps (§3.1).
- Thermal test: sustained current (a hold barely heats: the gearbox carries the load at ~0.1 A).
- Coordinated multi-joint motions: every validation so far moves one joint at a time.
