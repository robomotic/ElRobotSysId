// ST3215 servo model for MuJoCo in Unity: position loop, DC motor (electrical), gearbox friction hand-off, overload
// protection, heating, and the readings the real servo reports. Plain C#, no Unity or MuJoCo types: the driver
// (ElRobotServoDriver.cs) feeds it the joint angle and speed every physics step and writes the torque back.
//
// Same equations and numbers as tools/servo_sim.py (ElRobotSysId, issue #2); see docs/SERVO_MODEL.md. Measured on the
// ElRobot follower 2026-11-21..23: R 4.0-4.2 ohm, Ke*N 1.5-2.0 V s/rad, Kt*eta*N 1.6-1.8 N m/A, static friction
// 0.25-0.41 N m loaded (0.02-0.05 unloaded wrist), duty 0.42 %/tick, delay ~80 ms. Thermal constants are placeholders.

using System;

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

        // Ticks <-> joint angle: rad = Sign * (ticks - ZeroTicks) * 2 pi / 4096 (station viewer mapping, see
        // sysid_safety.toml). The sign must match the MuJoCo joint axis.
        public double ZeroTicks = 2048.0;
        public double Sign = 1.0;

        public static St3215Params Default() => new St3215Params();
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
