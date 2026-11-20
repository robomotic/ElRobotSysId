//! Where to find the robot. Taken from command-line flags, optionally filled in from a paddle-balance `paddle.toml`
//! (PingPongSimBot), which already knows the follower port, the level pose and the gripper grip.

use anyhow::{Context, Result};
use serde::Deserialize;
use std::path::Path;

/// Follower bus of the NormaStation ElRobot (CH343 serial 5B14033257). Stable across reboots, unlike /dev/ttyACM*.
pub const DEFAULT_PORT: &str = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14033257-if00";

#[derive(Debug, Default, Deserialize)]
#[serde(default)]
struct File {
    bus: BusSect,
    poses: PosesSect,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default)]
struct BusSect {
    port: Option<String>,
    timeout_ms: Option<u64>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default)]
struct PosesSect {
    level: Option<[u16; 8]>,
    grip: Option<u16>,
}

#[derive(Debug, Clone)]
pub struct Settings {
    pub port: String,
    pub timeout_ms: u64,
    /// Level pose ticks M1..M8, if known.
    pub level: Option<[u16; 8]>,
    /// M8 goal that squeezes the paddle, if known.
    pub grip: Option<u16>,
}

impl Settings {
    /// `config`: optional paddle.toml; a missing file is fine. `port`: overrides the file and the default.
    pub fn load(config: Option<&Path>, port: Option<String>) -> Result<Self> {
        let file = match config {
            Some(p) if p.exists() => {
                let text = std::fs::read_to_string(p).with_context(|| format!("reading {}", p.display()))?;
                toml::from_str::<File>(&text).with_context(|| format!("parsing {}", p.display()))?
            }
            _ => File::default(),
        };
        Ok(Self {
            port: port.or(file.bus.port).unwrap_or_else(|| DEFAULT_PORT.to_string()),
            timeout_ms: file.bus.timeout_ms.unwrap_or(20),
            level: file.poses.level,
            grip: file.poses.grip,
        })
    }

}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_paddle_toml_subset() {
        let dir = std::env::temp_dir().join(format!("sysid-settings-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("paddle.toml");
        std::fs::write(&p, "[bus]\nport = \"/dev/x\"\nwrist_acc = 100\n[poses]\nlevel = [1,2,3,4,5,6,7,8]\ngrip = 1396\nmove_time_s = 3.0\n[control]\nkp = 0.1\n").unwrap();
        let s = Settings::load(Some(&p), None).unwrap();
        assert_eq!(s.port, "/dev/x");
        assert_eq!(s.grip, Some(1396));
        assert_eq!(s.level.unwrap()[7], 8);
        let s = Settings::load(Some(&p), Some("/dev/y".into())).unwrap();
        assert_eq!(s.port, "/dev/y");
        let s = Settings::load(None, None).unwrap();
        assert_eq!(s.port, DEFAULT_PORT);
    }
}
