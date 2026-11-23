// ST3215 servo model for MuJoCo in Unity: position loop, DC motor (electrical), gearbox friction hand-off, overload
// protection, heating, and the readings the real servo reports. Plain C#, no Unity or MuJoCo types: the driver
// (ElRobotServoDriver.cs) feeds it the joint angle and speed every physics step and writes the torque back.
//
// Same equations and numbers as tools/servo_sim.py (ElRobotSysId, issue #2); see docs/SERVO_MODEL.md. Measured on the
// ElRobot follower 2026-11-21..23: R 4.0-4.2 ohm, Ke*N 1.5-2.0 V s/rad, Kt*eta*N 1.6-1.8 N m/A, static friction
// 0.25-0.41 N m loaded (0.02-0.05 unloaded wrist), duty 0.42 %/tick, delay ~80 ms. Thermal constants are placeholders.

using System;
using System.Collections.Generic;
using System.Globalization;

namespace ElRobotSysId
{
    /// <summary>Parameters of one ST3215 (all SI at the joint output; ticks are servo encoder steps, 4096 per turn).</summary>
    [Serializable]
    public sealed class St3215Params
    {
        // Electrical (fitted: tools/fit_electrical.py, tools/fit_torque.py)
        public double ResistanceOhm = 4.1;
        public double KeN = 1.8;          // back-EMF at the output, V s/rad
        public double KtEtaN = 1.7;       // torque per motor amp at the output, N m/A
        public double BusVoltage = 12.1;
        public double CopperTempCoef = 0.0039; // 1/K, R(T) = R25 (1 + a (T - 25))

        // Firmware position loop (fitted from the logs: load against goal - position)
        public double DutyPerTick = 0.0042; // duty per tick of error (firmware P 16)
        public double DeadZoneTicks = 4.0;  // no drive inside this |error|
        public double DutyCap = 0.5;        // EEPROM Max Torque 500 / RAM Torque Limit 500 (0.1 % units)
        public double DelaySeconds = 0.08;  // goal -> drive (bus + firmware), ~80 ms measured

        // Protection (EEPROM: protection current 500 x 6.5 mA, overload torque 25 %, protection time 200 x 10 ms,
        // protective torque 20 %)
        public double ProtectionCurrentA = 3.25;
        public double OverloadDuty = 0.25;
        public double ProtectionTimeS = 2.0;
        public double ProtectiveDuty = 0.20;

        // Heating: two nodes, winding -> case -> ambient. PLACEHOLDERS until a thermal test (issue #2).
        public double WindingHeatCapacity = 3.0;  // J/K
        public double CaseHeatCapacity = 40.0;    // J/K
        public double WindingToCaseKPerW = 3.0;
        public double CaseToAmbientKPerW = 15.0;
        public double AmbientC = 25.0;
        public double MaxTemperatureC = 70.0;     // EEPROM max temperature: the servo unloads above it

        // MuJoCo joint settings (applied by ElRobotServoDriver): gearbox stiction as frictionloss, viscous damping
        // beyond the back-EMF, reflected rotor inertia. Friction is measured; damping and armature are not identified.
        public double StaticFrictionNm = 0.3;
        public double DampingNmSPerRad = 0.05;
        public double ArmatureKgM2 = 0.01;

        // Ticks <-> joint angle: rad = Sign * (ticks - ZeroTicks) * 2 pi / 4096 (station viewer mapping, see
        // sysid_safety.toml). The sign must match the MuJoCo joint axis.
        public double ZeroTicks = 2048.0;
        public double Sign = 1.0;

        public static St3215Params Default() => new St3215Params();

        // servo_params.toml key -> setter (the same keys as tools/servo_sim.py FILE_KEYS).
        static readonly Dictionary<string, Action<St3215Params, double>> Keys = new Dictionary<string, Action<St3215Params, double>>
        {
            ["resistance_ohm"] = (p, v) => p.ResistanceOhm = v,
            ["ke_n"] = (p, v) => p.KeN = v,
            ["kt_eta_n"] = (p, v) => p.KtEtaN = v,
            ["bus_voltage"] = (p, v) => p.BusVoltage = v,
            ["copper_temp_coef"] = (p, v) => p.CopperTempCoef = v,
            ["duty_per_tick"] = (p, v) => p.DutyPerTick = v,
            ["dead_zone_ticks"] = (p, v) => p.DeadZoneTicks = v,
            ["duty_cap"] = (p, v) => p.DutyCap = v,
            ["delay_s"] = (p, v) => p.DelaySeconds = v,
            ["protection_current_a"] = (p, v) => p.ProtectionCurrentA = v,
            ["overload_duty"] = (p, v) => p.OverloadDuty = v,
            ["protection_time_s"] = (p, v) => p.ProtectionTimeS = v,
            ["protective_duty"] = (p, v) => p.ProtectiveDuty = v,
            ["static_friction_nm"] = (p, v) => p.StaticFrictionNm = v,
            ["damping_nm_s_per_rad"] = (p, v) => p.DampingNmSPerRad = v,
            ["armature_kg_m2"] = (p, v) => p.ArmatureKgM2 = v,
            ["winding_heat_capacity"] = (p, v) => p.WindingHeatCapacity = v,
            ["case_heat_capacity"] = (p, v) => p.CaseHeatCapacity = v,
            ["winding_to_case_k_per_w"] = (p, v) => p.WindingToCaseKPerW = v,
            ["case_to_ambient_k_per_w"] = (p, v) => p.CaseToAmbientKPerW = v,
            ["ambient_c"] = (p, v) => p.AmbientC = v,
            ["max_temperature_c"] = (p, v) => p.MaxTemperatureC = v,
        };

        /// <summary>
        /// Parameters of joint <paramref name="joint"/> (1..8) from the text of servo_params.toml: [default], then
        /// [servo.M&lt;joint&gt;]. Keys that are missing keep this class's defaults; unknown keys are ignored.
        /// Fill ZeroTicks/Sign from sysid_safety.toml with <see cref="LoadCalibration"/>.
        /// </summary>
        public static St3215Params Load(string servoParamsToml, int joint)
        {
            var doc = St3215Toml.Parse(servoParamsToml);
            var p = Default();
            foreach (var section in new[] { "default", "servo.M" + joint })
                if (doc.TryGetValue(section, out var kv))
                    foreach (var e in kv)
                        if (Keys.TryGetValue(e.Key, out var set) && e.Value is double d)
                            set(p, d);
            return p;
        }

        /// <summary>Sets ZeroTicks and Sign of joint (1..7) from the text of sysid_safety.toml (top-level arrays
        /// `zero` and `sign`); leaves them unchanged when a key is missing.</summary>
        public St3215Params LoadCalibration(string sysidSafetyToml, int joint)
        {
            var top = St3215Toml.Parse(sysidSafetyToml);
            if (top.TryGetValue("", out var kv))
            {
                if (kv.TryGetValue("zero", out var z) && z is List<object> zl && joint - 1 < zl.Count && zl[joint - 1] is double zd)
                    ZeroTicks = zd;
                if (kv.TryGetValue("sign", out var s) && s is List<object> sl && joint - 1 < sl.Count && sl[joint - 1] is double sd)
                    Sign = sd;
            }
            return this;
        }
    }

    /// <summary>
    /// Just enough TOML for servo_params.toml and sysid_safety.toml: [sections], key = number | "string" | true/false |
    /// [arrays, possibly nested and over several lines], # comments. Numbers come back as double, arrays as List&lt;object&gt;.
    /// </summary>
    public static class St3215Toml
    {
        public static Dictionary<string, Dictionary<string, object>> Parse(string text)
        {
            var doc = new Dictionary<string, Dictionary<string, object>> { [""] = new Dictionary<string, object>() };
            string section = "", key = null, pending = null;
            foreach (var raw in text.Replace("\r", "").Split('\n'))
            {
                string line = StripComment(raw).Trim();
                if (pending != null)
                {
                    pending += " " + line;
                    if (Count(pending, '[') <= Count(pending, ']'))
                    {
                        doc[section][key] = Value(pending);
                        pending = null;
                    }
                    continue;
                }
                if (line.Length == 0)
                    continue;
                if (line[0] == '[' && line[line.Length - 1] == ']' && line.IndexOf('=') < 0)
                {
                    section = line.Substring(1, line.Length - 2).Trim();
                    if (!doc.ContainsKey(section))
                        doc[section] = new Dictionary<string, object>();
                    continue;
                }
                int eq = line.IndexOf('=');
                if (eq <= 0)
                    throw new FormatException("TOML: cannot parse line: " + raw);
                key = line.Substring(0, eq).Trim();
                string val = line.Substring(eq + 1).Trim();
                if (val.StartsWith("[") && Count(val, '[') > Count(val, ']'))
                    pending = val;
                else
                    doc[section][key] = Value(val);
            }
            return doc;
        }

        static int Count(string s, char c)
        {
            int n = 0;
            foreach (var ch in s)
                if (ch == c) n++;
            return n;
        }

        static string StripComment(string line)
        {
            bool quoted = false;
            for (int i = 0; i < line.Length; i++)
            {
                if (line[i] == '"') quoted = !quoted;
                if (line[i] == '#' && !quoted) return line.Substring(0, i);
            }
            return line;
        }

        static object Value(string s)
        {
            int i = 0;
            var v = ParseValue(s, ref i);
            return v;
        }

        static object ParseValue(string s, ref int i)
        {
            while (i < s.Length && char.IsWhiteSpace(s[i])) i++;
            if (i >= s.Length) throw new FormatException("TOML: missing value");
            if (s[i] == '[')
            {
                i++;
                var list = new List<object>();
                while (true)
                {
                    while (i < s.Length && (char.IsWhiteSpace(s[i]) || s[i] == ',')) i++;
                    if (i >= s.Length) throw new FormatException("TOML: unclosed array");
                    if (s[i] == ']') { i++; return list; }
                    list.Add(ParseValue(s, ref i));
                }
            }
            if (s[i] == '"')
            {
                int end = s.IndexOf('"', i + 1);
                if (end < 0) throw new FormatException("TOML: unclosed string");
                var str = s.Substring(i + 1, end - i - 1);
                i = end + 1;
                return str;
            }
            int start = i;
            while (i < s.Length && s[i] != ',' && s[i] != ']' && !char.IsWhiteSpace(s[i])) i++;
            string tok = s.Substring(start, i - start);
            if (tok == "true") return true;
            if (tok == "false") return false;
            return double.Parse(tok.Replace("_", ""), NumberStyles.Float, CultureInfo.InvariantCulture);
        }
    }

    /// <summary>One servo. Call <see cref="Command"/> when a goal is sent and <see cref="Step"/> every physics step.</summary>
    public sealed class St3215Servo
    {
        public const double TicksPerRev = 4096.0;
        public const double RadPerTick = 2.0 * Math.PI / TicksPerRev;

        public readonly St3215Params P;

        // Delayed goals: (time it reaches the drive, ticks). A small ring buffer; goals are sent at bus rate.
        readonly double[] _goalTime = new double[256];
        readonly double[] _goalTicks = new double[256];
        int _head, _count;

        double _time;
        double _goal;
        double _overT, _okT;

        public St3215Servo(St3215Params p, double startTicks)
        {
            P = p ?? St3215Params.Default();
            _goal = startTicks;
            Position = startTicks;
            WindingC = CaseC = P.AmbientC;
        }

        // --- what the servo reports (Present* registers) ---
        /// <summary>Present Position, ticks.</summary>
        public double Position { get; private set; }
        /// <summary>Present Speed, ticks/s.</summary>
        public double Speed { get; private set; }
        /// <summary>Present Load in 0.1 % (the PWM duty x 1000). The real follower reports it with the opposite sign to
        /// the speed; this model uses the physical sign (positive duty drives towards increasing ticks).</summary>
        public double Load => Duty * 1000.0;
        /// <summary>Present Current, mA: the SUPPLY current |duty| x |i_motor| (that is what the ST3215 reports;
        /// magnitude only, 6.5 mA resolution on the real servo).</summary>
        public double ReportedCurrentMilliamp => Math.Abs(Duty) * Math.Abs(MotorCurrent) * 1000.0;
        /// <summary>Present Temperature, deg C: the case/board node.</summary>
        public double TemperatureC => CaseC;

        // --- internals, useful for plots ---
        public double Duty { get; private set; }
        public double MotorVoltage => Duty * P.BusVoltage;
        public double MotorCurrent { get; private set; }
        public double Torque { get; private set; }
        public double WindingC { get; private set; }
        public double CaseC { get; private set; }
        /// <summary>Overload protection active: the drive is capped at the protective duty.</summary>
        public bool Tripped { get; private set; }
        /// <summary>Above the EEPROM temperature limit: the real servo unloads (no torque).</summary>
        public bool OverTemperature => CaseC > P.MaxTemperatureC;
        /// <summary>The goal the drive acts on now (after the delay), ticks.</summary>
        public double ActiveGoal => _goal;

        public double TicksToRad(double ticks) => P.Sign * (ticks - P.ZeroTicks) * RadPerTick;
        public double RadToTicks(double rad) => rad / (P.Sign * RadPerTick) + P.ZeroTicks;

        /// <summary>A Goal Position write, ticks. It reaches the drive <see cref="St3215Params.DelaySeconds"/> later.</summary>
        public void Command(double goalTicks)
        {
            if (_count == _goalTime.Length)
            {
                // Full: drop the oldest (goals arrive far more slowly than they are consumed in practice).
                _head = (_head + 1) % _goalTime.Length;
                _count--;
            }
            int i = (_head + _count) % _goalTime.Length;
            _goalTime[i] = _time + P.DelaySeconds;
            _goalTicks[i] = goalTicks;
            _count++;
        }

        /// <summary>Freeze: goal = present position, now (what the sysid tool's SAFETY STOP writes).</summary>
        public void HoldPresent()
        {
            _count = 0;
            _goal = Position;
        }

        /// <summary>
        /// One physics step. Give the joint angle (rad) and angular velocity (rad/s) from the simulator; returns the
        /// motor torque at the joint (N m) to apply. Static friction is NOT in this torque: give it to MuJoCo as the
        /// joint's frictionloss (stiff, with noslip), see docs/SERVO_MODEL.md.
        /// </summary>
        public double Step(double dt, double angleRad, double velocityRadS)
        {
            _time += dt;
            while (_count > 0 && _goalTime[_head] <= _time)
            {
                _goal = _goalTicks[_head];
                _head = (_head + 1) % _goalTime.Length;
                _count--;
            }

            Position = RadToTicks(angleRad);
            Speed = P.Sign * velocityRadS / RadPerTick;

            // Firmware: P on the tick error, dead zone, duty cap (protective cap after a trip; none when too hot).
            double err = _goal - Position;
            double cap = OverTemperature ? 0.0 : Tripped ? P.ProtectiveDuty : P.DutyCap;
            Duty = Math.Abs(err) <= P.DeadZoneTicks ? 0.0 : Clamp(P.DutyPerTick * err, -cap, cap);

            // Motor: V = R(T) i + Ke*N omega (inductance neglected: L/R << 1 ms); torque = Kt*eta*N i.
            double r = P.ResistanceOhm * (1.0 + P.CopperTempCoef * (WindingC - 25.0));
            double omegaTicksDir = P.Sign * velocityRadS; // rad/s in the direction of increasing ticks
            MotorCurrent = (Duty * P.BusVoltage - P.KeN * omegaTicksDir) / r;
            Torque = P.Sign * P.KtEtaN * MotorCurrent;

            // Protection: motor current above the limit, or duty above the overload level, for longer than the
            // protection time -> capped at the protective duty until the cause has been gone as long.
            bool over = Math.Abs(MotorCurrent) > P.ProtectionCurrentA || Math.Abs(Duty) > P.OverloadDuty;
            if (over)
            {
                _overT += dt;
                _okT = 0.0;
                if (_overT > P.ProtectionTimeS)
                    Tripped = true;
            }
            else
            {
                _overT = 0.0;
                _okT += dt;
                if (Tripped && _okT > P.ProtectionTimeS)
                    Tripped = false;
            }

            // Heating: C_w dTw/dt = i^2 R(T) - (Tw - Tc)/R_wc ; C_c dTc/dt = (Tw - Tc)/R_wc - (Tc - Tamb)/R_ca.
            double heat = MotorCurrent * MotorCurrent * r;
            double toCase = (WindingC - CaseC) / P.WindingToCaseKPerW;
            WindingC += dt * (heat - toCase) / P.WindingHeatCapacity;
            CaseC += dt * (toCase - (CaseC - P.AmbientC) / P.CaseToAmbientKPerW) / P.CaseHeatCapacity;
            return Torque;
        }

        static double Clamp(double x, double lo, double hi) => x < lo ? lo : x > hi ? hi : x;
    }
}
