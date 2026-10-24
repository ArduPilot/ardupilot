// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, Result};
use core::fmt::{self, Write};

ap_rust_sdk::module!(run);

struct Message {
    bytes: [u8; 96],
    length: usize,
}

impl Write for Message {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        let end = self.length + value.len();
        if end > self.bytes.len() {
            return Err(fmt::Error);
        }
        self.bytes[self.length..end].copy_from_slice(value.as_bytes());
        self.length = end;
        Ok(())
    }
}

fn report(context: &Context<'_>, args: fmt::Arguments<'_>) -> Result {
    let mut message = Message {
        bytes: [0; 96],
        length: 0,
    };
    message.write_fmt(args).map_err(|_| Error::Host)?;
    context.text(
        6,
        core::str::from_utf8(&message.bytes[..message.length]).map_err(|_| Error::Host)?,
    )
}

fn run(context: &Context<'_>) -> Result {
    for channel in [0, 17, 255] {
        loop {
            match context.rc().get_pwm(channel) {
                Err(Error::Busy) => context.sleep_ms(10)?,
                result => {
                    assert_eq!(result, Err(Error::Invalid));
                    break;
                }
            }
        }
    }
    // This fixture targets the 16-channel SITL build, including its upper edge.
    for channel in [1, 16] {
        loop {
            match context.rc().get_pwm(channel) {
                Err(Error::Busy) => context.sleep_ms(10)?,
                result => {
                    let _ = result?;
                    break;
                }
            }
        }
    }
    loop {
        let read = || -> Result {
            report(
                context,
                format_args!(
                    "BIND A {} {} {}",
                    (context.ahrs().get_roll_rad()? * 1000.0) as i32,
                    (context.ahrs().get_pitch_rad()? * 1000.0) as i32,
                    (context.ahrs().get_yaw_rad()? * 1000.0) as i32
                ),
            )?;
            report(
                context,
                format_args!(
                    "BIND E {} {} {}",
                    (context.ahrs().get_eas2_tas()? * 1000.0) as i32,
                    context
                        .ahrs().airspeed_estimate()?
                        .map(|v| (v * 1000.0) as i32)
                        .unwrap_or(-9999999),
                    context
                        .ahrs().get_hagl()?
                        .map(|v| (v * 1000.0) as i32)
                        .unwrap_or(-9999999)
                ),
            )?;
            report(
                context,
                format_args!(
                    "BIND S {} {} {} {}",
                    context.vehicle().get_mode()?,
                    u8::from(context.arming().is_armed()?),
                    context.gps().num_sensors()?,
                    context.gps().primary_sensor()?
                ),
            )?;
            report(
                context,
                format_args!(
                    "BIND G {} {} {} {}",
                    context.gps().status(0)?,
                    context.gps().num_sats(0)?,
                    context.gps().get_hdop(0)?,
                    u8::from(context.gps().have_vertical_velocity(0)?)
                ),
            )?;
            report(
                context,
                format_args!(
                    "BIND B {} {} {} {}",
                    context.battery().num_instances()?,
                    u8::from(context.battery().healthy(0)?),
                    context.battery().pack_capacity_mah(0)?,
                    (context.battery().voltage(0)? * 1000.0) as i32
                ),
            )?;
            report(
                context,
                format_args!(
                    "BIND M {} {} {}",
                    context.mission().num_commands()?,
                    context.mission().get_current_nav_index()?,
                    context.mission().state()?
                ),
            )?;
            report(
                context,
                format_args!("BIND R {}", context.rc().get_pwm(1)?.unwrap_or(0)),
            )
        };
        match read() {
            Ok(()) | Err(Error::Busy) => {}
            Err(Error::Stopped) => break,
            Err(error) => return Err(error),
        }
        if let Err(error) = context.sleep_ms(500) {
            if error != Error::Stopped {
                return Err(error);
            }
            break;
        }
    }
    // A stopped context rejects new binding work, even after an interrupted sleep.
    assert_eq!(context.arming().is_armed(), Err(Error::Stopped));
    Ok(())
}
