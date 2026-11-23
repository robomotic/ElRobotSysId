// Behaviour checks of St3215Servo (plain C#, C# 9 like Unity). Exit code 1 on any failure.
using System;
using ElRobotSysId;

static class Program
{
    static int _fail;

    static void Check(bool ok, string what)
    {
        Console.WriteLine((ok ? "ok    " : "FAIL  ") + what);
        if (!ok) _fail++;
    }

    static int Main()
    {
        const double dt = 0.001;

        // 1. Electrical equation: blocked joint (omega 0), 20 ticks of error -> duty 0.084, i = V/R.
        {
            var s = new St3215Servo(St3215Params.Default(), 2048);
            s.Command(2068);
            for (int k = 0; k < 200; k++) s.Step(dt, s.TicksToRad(2048), 0.0);
            double v = 0.0042 * 20 * 12.1;
            Check(Math.Abs(s.Duty - 0.084) < 1e-9, $"duty from the tick error: {s.Duty:F4} (0.0840)");
            double rT = 4.1 * (1 + 0.0039 * (s.WindingC - 25)); // R at the (slightly warmed) winding temperature
            Check(Math.Abs(s.MotorCurrent - v / rT) < 1e-4, $"stall current V/R(T): {s.MotorCurrent:F4} A ({v / rT:F4})");
            Check(Math.Abs(s.ReportedCurrentMilliamp - 0.084 * s.MotorCurrent * 1000) < 1e-6,
                  $"reported = supply current |duty|*|i|: {s.ReportedCurrentMilliamp:F1} mA");
            Check(Math.Abs(s.Torque - 1.7 * s.MotorCurrent) < 1e-9, $"torque = Kt*eta*N * i: {s.Torque:F4} N m");
        }

        // 2. Back-EMF: moving at 1 rad/s with zero duty (inside the dead zone) -> i = -Ke*N/R.
        {
            var s = new St3215Servo(St3215Params.Default(), 2048);
            s.Step(dt, s.TicksToRad(2048), 1.0);
            Check(Math.Abs(s.MotorCurrent + 1.8 / 4.1) < 1e-9, $"back-EMF current: {s.MotorCurrent:F4} A ({-1.8 / 4.1:F4})");
        }

        // 3. Delay: the goal acts 80 ms after the command.
        {
            var s = new St3215Servo(St3215Params.Default(), 2048);
            s.Command(2148);
            int first = -1;
            for (int k = 1; k <= 200 && first < 0; k++) { s.Step(dt, s.TicksToRad(2048), 0.0); if (s.Duty != 0) first = k; }
            Check(first >= 79 && first <= 81, $"transport delay: drive starts after {first} ms (80)");
        }

        // 4. Protection: blocked, 200 ticks away -> duty at the 50 % cap (> 25 % overload); trips after 2 s, then 20 %.
        {
            var s = new St3215Servo(St3215Params.Default(), 2048);
            s.Command(2248);
            for (int k = 0; k < 1500; k++) s.Step(dt, s.TicksToRad(2048), 0.0);
            Check(!s.Tripped && Math.Abs(s.Duty - 0.5) < 1e-9, $"1.5 s: not tripped, duty {s.Duty:F2} (0.50)");
            for (int k = 0; k < 1000; k++) s.Step(dt, s.TicksToRad(2048), 0.0);
            Check(s.Tripped && Math.Abs(s.Duty - 0.2) < 1e-9, $"2.5 s: tripped, duty capped at {s.Duty:F2} (0.20)");
        }

        // 5. Heating: blocked at 40 ticks (duty 0.168, ~1 W) for 2 h -> steady state Tc = Tamb + P R_ca,
        //    Tw = Tc + P R_wc (P with R at the winding temperature).
        {
            var s = new St3215Servo(St3215Params.Default(), 2048);
            s.Command(2088);
            const double h = 0.01;
            for (int k = 0; k < (int)(7200 / h); k++) s.Step(h, s.TicksToRad(2048), 0.0);
            double r = 4.1 * (1 + 0.0039 * (s.WindingC - 25));
            double p = s.MotorCurrent * s.MotorCurrent * r;
            Check(Math.Abs(s.CaseC - (25 + p * 15)) < 0.3, $"case steady state {s.CaseC:F2} C ({25 + p * 15:F2}), P {p:F3} W");
            Check(Math.Abs(s.WindingC - (s.CaseC + p * 3)) < 0.3, $"winding steady state {s.WindingC:F2} C ({s.CaseC + p * 3:F2})");
        }

        // 6. Closed loop on a bare 1-DoF joint (J 0.005 kg m^2, no gravity, no friction): a 50-tick step settles
        //    inside the dead zone, starting after the 80 ms delay.
        {
            var s = new St3215Servo(St3215Params.Default(), 2048);
            double q = s.TicksToRad(2048), w = 0, j = 0.005;
            s.Command(2098);
            double t63 = -1;
            for (int k = 1; k <= 3000; k++)
            {
                double tau = s.Step(dt, q, w);
                w += tau / j * dt; q += w * dt;
                if (t63 < 0 && s.RadToTicks(q) >= 2048 + 0.63 * 50) t63 = k * dt;
            }
            double end = s.RadToTicks(q);
            Check(Math.Abs(end - 2098) <= 4.0 + 1e-6, $"settles within the dead zone: {end:F1} (2098 +- 4)");
            Check(t63 > 0.08 && t63 < 0.4, $"t63 {t63 * 1000:F0} ms (after the 80 ms delay)");
        }

        // 7. Sign: with Sign -1 the torque and the readings flip consistently.
        {
            var p = St3215Params.Default(); p.Sign = -1;
            var s = new St3215Servo(p, 2048);
            s.Command(2068);
            for (int k = 0; k < 200; k++) s.Step(dt, s.TicksToRad(2048), 0.0);
            Check(s.Duty > 0 && s.Torque < 0, $"sign -1: duty {s.Duty:F3} > 0 drives ticks up = negative joint torque {s.Torque:F3}");
            Check(Math.Abs(s.TicksToRad(s.RadToTicks(0.3)) - 0.3) < 1e-12, "ticks <-> rad round trip");
        }

        Console.WriteLine(_fail == 0 ? "all checks passed" : $"{_fail} check(s) failed");
        return _fail == 0 ? 0 : 1;
    }
}
