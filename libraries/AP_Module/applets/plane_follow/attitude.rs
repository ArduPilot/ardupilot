// SPDX-License-Identifier: GPL-3.0-or-later

use ap_rust_sdk::{Context, LogValue, Result};

const ATTITUDE_MSG_ID: u32 = 30;

#[derive(Clone, Copy)]
pub struct Attitude {
    pub timestamp_ms: i64,
    pub delay_ms: f32,
    pub roll: f32,
    pub rollspeed: f32,
}

struct JitterCorrection {
    max_lag_ms: i64,
    convergence_loops: u16,
    link_offset_ms: i64,
    min_sample_ms: i64,
    initialised: bool,
    min_sample_counter: u16,
}

impl JitterCorrection {
    fn new(max_lag_ms: i64, convergence_loops: u16) -> Self {
        Self {
            max_lag_ms,
            convergence_loops,
            link_offset_ms: 0,
            min_sample_ms: 0,
            initialised: false,
            min_sample_counter: 0,
        }
    }

    fn correct(&mut self, offboard_ms: u32, local_ms: u32) -> i64 {
        let offboard_ms = i64::from(offboard_ms);
        let local_ms = i64::from(local_ms);
        let diff_ms = local_ms - offboard_ms;
        if !self.initialised || diff_ms < self.link_offset_ms {
            self.link_offset_ms = diff_ms;
            self.initialised = true;
        }

        let mut estimate_ms = offboard_ms + self.link_offset_ms;
        if estimate_ms + self.max_lag_ms < local_ms {
            estimate_ms = local_ms - self.max_lag_ms;
            self.link_offset_ms = estimate_ms - offboard_ms;
        }

        if self.min_sample_counter == 0 {
            self.min_sample_ms = diff_ms;
        }
        self.min_sample_counter += 1;
        if diff_ms < self.min_sample_ms {
            self.min_sample_ms = diff_ms;
        }
        if self.min_sample_counter == self.convergence_loops {
            self.link_offset_ms = self.min_sample_ms;
            self.min_sample_counter = 0;
        }
        estimate_ms
    }
}

pub struct AttitudeReceiver {
    jitter: JitterCorrection,
    last_timestamp_ms: i64,
}

impl AttitudeReceiver {
    pub fn new(context: &Context<'_>) -> Result<Self> {
        context.mavlink().init(10, 1)?;
        let _ = context.mavlink().register_rx_msgid(ATTITUDE_MSG_ID)?;
        Ok(Self {
            jitter: JitterCorrection::new(5_000, 100),
            last_timestamp_ms: (context.time_us() / 1_000) as i64,
        })
    }

    pub fn get(&mut self, context: &Context<'_>, target_sysid: u32) -> Result<Option<Attitude>> {
        let Some(message) = context.mavlink().receive_chan()? else {
            return Ok(None);
        };
        let bytes = message.bytes();
        if !matches!(bytes[2], 0xfe | 0xfd) || bytes[3] == 0 || (bytes[2] == 0xfe && bytes[3] < 28) {
            return Ok(None);
        }
        let sysid = u32::from_le_bytes(bytes[7..11].try_into().unwrap());
        let msgid =
            u32::from(bytes[12]) | (u32::from(bytes[13]) << 8) | (u32::from(bytes[14]) << 16);
        if msgid != ATTITUDE_MSG_ID {
            return Ok(None);
        }

        let mut payload = [0u8; 28];
        let length = usize::from(bytes[3]).min(payload.len());
        payload[..length].copy_from_slice(&bytes[15..15 + length]);
        let time_boot_ms = u32::from_le_bytes(payload[..4].try_into().unwrap());
        let roll = f32::from_le_bytes(payload[4..8].try_into().unwrap());
        let pitch = f32::from_le_bytes(payload[8..12].try_into().unwrap());
        let yaw = f32::from_le_bytes(payload[12..16].try_into().unwrap());
        let rollspeed = f32::from_le_bytes(payload[16..20].try_into().unwrap());
        let pitchspeed = f32::from_le_bytes(payload[20..24].try_into().unwrap());
        let yawspeed = f32::from_le_bytes(payload[24..28].try_into().unwrap());

        let timestamp_ms = self.jitter.correct(time_boot_ms, message.timestamp_ms);
        if timestamp_ms < self.last_timestamp_ms {
            return Ok(None);
        }
        self.last_timestamp_ms = timestamp_ms;
        let delay_ms = (context.time_us() / 1_000) as f32 - timestamp_ms as f32;
        if sysid != target_sysid {
            return Ok(None);
        }

        context.logger().write(
            "ZATI",
            "Time,TimeJC,Delay,roll,ptch,yaw,rollspd,ptchspd,yawspd",
            Some(("---rrrEEE", "---------")),
            &[
                LogValue::I32(time_boot_ms as i32),
                LogValue::I32(timestamp_ms as i32),
                LogValue::I32(delay_ms as i32),
                LogValue::Float(roll),
                LogValue::Float(pitch),
                LogValue::Float(yaw),
                LogValue::Float(rollspeed),
                LogValue::Float(pitchspeed),
                LogValue::Float(yawspeed),
            ],
        )?;

        if delay_ms <= 600.0 {
            Ok(Some(Attitude {
                timestamp_ms,
                delay_ms,
                roll,
                rollspeed,
            }))
        } else {
            Ok(None)
        }
    }
}
