//! Where goals go and states come from: the real ST3215 bus, or simulated servos for trying the tool without the arm.

use crate::bus::{Bus, MotorState};
use anyhow::Result;
use st3215::protocol::RamRegister;
use std::collections::VecDeque;
use std::time::{Duration, Instant};

#[allow(async_fn_in_trait)]
pub trait Io {
    /// "arm" or "sim".
    fn kind(&self) -> &'static str;
    async fn ping(&mut self, id: u8) -> bool;
    async fn read_state(&mut self, id: u8) -> Result<MotorState>;
    async fn read_positions(&mut self, ids: &[u8]) -> Result<Vec<u16>>;
    async fn read_bytes(&mut self, id: u8, address: u8, len: u8) -> Result<Vec<u8>>;
    async fn write_goals(&mut self, goals: &[(u8, u16)]) -> Result<()>;
    async fn write_acc(&mut self, id: u8, value: u8) -> Result<()>;
    /// Torque on, every joint holding where it is (saved grip re-applied). Returns M1..M8 positions.
    async fn hold_present(&mut self, grip: Option<u16>) -> Result<Vec<u16>>;
}

impl Io for Bus {
    fn kind(&self) -> &'static str {
        "arm"
    }
    async fn ping(&mut self, id: u8) -> bool {
        Bus::ping(self, id).await
    }
    async fn read_state(&mut self, id: u8) -> Result<MotorState> {
        Bus::read_state(self, id).await
    }
    async fn read_positions(&mut self, ids: &[u8]) -> Result<Vec<u16>> {
        Bus::read_positions(self, ids).await
    }
    async fn read_bytes(&mut self, id: u8, address: u8, len: u8) -> Result<Vec<u8>> {
        Ok(Bus::read_bytes(self, id, address, len).await?.to_vec())
    }
    async fn write_goals(&mut self, goals: &[(u8, u16)]) -> Result<()> {
        self.sync_write_goals(goals).await
    }
    async fn write_acc(&mut self, id: u8, value: u8) -> Result<()> {
        self.sync_write_u8(RamRegister::Acc, &[(id, value)]).await
    }
    async fn hold_present(&mut self, grip: Option<u16>) -> Result<Vec<u16>> {
        Bus::hold_present(self, grip).await
    }
}

// ---------------------------------------------------------------------------------------------------------------

/// One simulated servo: goals arrive `delay` late, then a dead zone and a first-order lag; load from gravity and error.
#[derive(Debug, Clone)]
struct SimJoint {
    pos: f64,
    goals: VecDeque<(f64, f64)>,
    target: f64,
    updated: f64,
    speed: f64,
    dead: f64,
    gravity_load: f64,
}

/// Simulated ElRobot follower. Parameters follow the first real paddle runs (PingPongSimBot): ~30 ms transport delay,
/// tau ~70 ms, dead zones of a few ticks (M6 ~20).
pub struct SimIo {
    joints: Vec<SimJoint>,
    acc: [u8; 8],
    t0: Instant,
    pub delay_s: f64,
    pub tau_s: f64,
    /// DC motor model for the reported current (#2): `i = (duty·V_bus − Ke·N·ω)/R`. The motion itself still comes
    /// from the lag model above (not torque-driven); this only makes current, duty, voltage and speed consistent.
    pub r_ohm: f64,
    pub ke_n: f64,
    pub v_bus: f64,
}

/// ST3215 ticks per output revolution.
const TICKS_PER_REV: f64 = 4096.0;

impl SimIo {
    pub fn new(start: [u16; 8]) -> Self {
        let dead = [4.0, 4.0, 4.0, 6.0, 3.0, 20.0, 6.0, 0.0];
        let grav = [0.0, 140.0, 100.0, 60.0, 10.0, 40.0, 10.0, 0.0];
        Self {
            joints: (0..8)
                .map(|j| SimJoint {
                    pos: start[j] as f64,
                    goals: VecDeque::new(),
                    target: start[j] as f64,
                    updated: 0.0,
                    speed: 0.0,
                    dead: dead[j],
                    gravity_load: grav[j],
                })
                .collect(),
            acc: [0; 8],
            t0: Instant::now(),
            delay_s: 0.03,
            tau_s: 0.07,
            r_ohm: 4.4,
            ke_n: 2.5,
            v_bus: 12.1,
        }
    }

    fn now(&self) -> f64 {
        self.t0.elapsed().as_secs_f64()
    }

    fn advance(&mut self, j: usize) {
        let now = self.now();
        let (delay, tau) = (self.delay_s, self.tau_s);
        let s = &mut self.joints[j];
        let dt = 0.001;
        let mut t = s.updated;
        while t + dt <= now {
            t += dt;
            while s.goals.front().is_some_and(|(at, _)| *at <= t - delay) {
                s.target = s.goals.pop_front().unwrap().1;
            }
            let err = s.target - s.pos;
            let before = s.pos;
            if err.abs() > s.dead / 2.0 {
                let aim = s.target - err.signum() * s.dead / 2.0;
                s.pos += (aim - s.pos) * (dt / tau).min(1.0);
            }
            s.speed = (s.pos - before) / dt;
        }
        s.updated = t;
    }

    fn state(&mut self, id: u8) -> MotorState {
        let j = id as usize - 1;
        self.advance(j);
        let s = &self.joints[j];
        let load = (s.gravity_load + 25.0 * (s.target - s.pos)).clamp(-1000.0, 1000.0).round();
        let speed = s.speed.round();
        let omega = speed * std::f64::consts::TAU / TICKS_PER_REV;
        let amps = (load / 1000.0 * self.v_bus - self.ke_n * omega) / self.r_ohm;
        MotorState {
            position: s.pos.round().clamp(0.0, 4095.0) as u16,
            speed: speed as i16,
            load: load as i16,
            voltage_dv: (self.v_bus * 10.0).round() as u8,
            temperature_c: 30,
            current_ma: (amps * 1000.0 / 6.5).round() * 6.5, // the servo's 6.5 mA resolution
        }
    }
}

impl Io for SimIo {
    fn kind(&self) -> &'static str {
        "sim"
    }
    async fn ping(&mut self, id: u8) -> bool {
        (1..=8).contains(&id)
    }
    async fn read_state(&mut self, id: u8) -> Result<MotorState> {
        // A real bus read takes ~1-2 ms; this paces the simulation at a realistic sample rate.
        tokio::time::sleep(Duration::from_micros(1200)).await;
        Ok(self.state(id))
    }
    async fn read_positions(&mut self, ids: &[u8]) -> Result<Vec<u16>> {
        Ok(ids.iter().map(|&id| self.state(id).position).collect())
    }
    async fn read_bytes(&mut self, id: u8, address: u8, len: u8) -> Result<Vec<u8>> {
        // A plausible ST3215 register image: model 777, P/D/I 16/0/0, max torque 1000, acc in RAM.
        let mut mem = vec![0u8; 0x48];
        mem[0x00..0x02].copy_from_slice(&777u16.to_le_bytes());
        mem[0x05] = id;
        mem[0x0B..0x0D].copy_from_slice(&4095u16.to_le_bytes());
        mem[0x0D] = 70; // max temperature, °C
        mem[0x0E] = 140; // max voltage, 0.1 V
        mem[0x0F] = 40; // min voltage, 0.1 V
        mem[0x10..0x12].copy_from_slice(&1000u16.to_le_bytes());
        mem[0x13] = 0x2C; // unload on temperature, current, overload
        mem[0x15] = 16;
        mem[0x1C..0x1E].copy_from_slice(&500u16.to_le_bytes()); // protection current, 6.5 mA units
        mem[0x22] = 20; // protective torque, %
        mem[0x23] = 200; // protection time, 10 ms
        mem[0x24] = 80; // overload torque, %
        mem[0x26] = 200; // overcurrent protection time, 10 ms
        mem[0x28] = 1;
        mem[0x29] = self.acc[id as usize - 1];
        mem[0x30..0x32].copy_from_slice(&1000u16.to_le_bytes());
        let a = address as usize;
        Ok(mem[a..(a + len as usize).min(mem.len())].to_vec())
    }
    async fn write_goals(&mut self, goals: &[(u8, u16)]) -> Result<()> {
        let now = self.now();
        for &(id, g) in goals {
            let j = id as usize - 1;
            self.advance(j);
            self.joints[j].goals.push_back((now, g as f64));
        }
        Ok(())
    }
    async fn write_acc(&mut self, id: u8, value: u8) -> Result<()> {
        self.acc[id as usize - 1] = value;
        Ok(())
    }
    async fn hold_present(&mut self, grip: Option<u16>) -> Result<Vec<u16>> {
        if let Some(g) = grip {
            self.joints[7].pos = g as f64;
            self.joints[7].target = g as f64;
        }
        Ok((1..=8).map(|id| self.state(id).position).collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_servo_follows_a_step_with_delay_and_dead_zone() {
        let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
        rt.block_on(async {
            let mut io = SimIo::new([2048; 8]);
            io.write_goals(&[(6, 2148)]).await.unwrap();
            tokio::time::sleep(Duration::from_millis(15)).await;
            assert_eq!(io.read_state(6).await.unwrap().position, 2048, "nothing moves during the transport delay");
            tokio::time::sleep(Duration::from_millis(600)).await;
            let p = io.read_state(6).await.unwrap().position as i32;
            assert!((p - (2148 - 10)).abs() <= 2, "settles half a dead zone short of the goal: {p}");
        });
    }

    #[test]
    fn sim_current_follows_the_dc_motor_model() {
        let rt = tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap();
        rt.block_on(async {
            let mut io = SimIo::new([2048; 8]);
            io.write_goals(&[(2, 1948)]).await.unwrap();
            let mut moving = false;
            for _ in 0..200 {
                let st = io.read_state(2).await.unwrap();
                let omega = st.speed as f64 * std::f64::consts::TAU / TICKS_PER_REV;
                let v = st.load as f64 / 1000.0 * st.voltage_dv as f64 / 10.0;
                let resid = v - io.r_ohm * st.current_ma / 1000.0 - io.ke_n * omega;
                assert!(resid.abs() < 0.02, "V = R·i + Ke·N·ω within the current resolution: {resid}");
                moving |= st.speed != 0 && st.current_ma < 0.0;
            }
            assert!(moving, "a downward step gives speed and a signed (negative) current");
        });
    }
}
