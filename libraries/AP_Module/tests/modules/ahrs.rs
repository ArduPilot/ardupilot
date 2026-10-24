// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, Result, Vector3f};
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

fn vector(context: &Context<'_>, name: &str, value: Option<Vector3f>) -> Result {
    match value {
        Some(v) => report(
            context,
            format_args!(
                "AHRS {} {} {} {}",
                name,
                (v.x * 1000.0) as i32,
                (v.y * 1000.0) as i32,
                (v.z * 1000.0) as i32
            ),
        ),
        None => report(context, format_args!("AHRS {} nil", name)),
    }
}

fn run(context: &Context<'_>) -> Result {
    loop {
        let read = || -> Result {
            vector(context, "G", Some(context.ahrs().get_gyro()?))?;
            vector(context, "V", context.ahrs().get_velocity_ned()?)?;
            vector(
                context,
                "P",
                context.ahrs().get_relative_position_ned_origin()?,
            )?;
            match context.ahrs().get_variances()? {
                Some((vel, pos, height, mag, tas)) => report(
                    context,
                    format_args!(
                        "AHRS E {} {} {} {} {} {} {}",
                        (vel * 1000.0) as i32,
                        (pos * 1000.0) as i32,
                        (height * 1000.0) as i32,
                        (mag.x * 1000.0) as i32,
                        (mag.y * 1000.0) as i32,
                        (mag.z * 1000.0) as i32,
                        (tas * 1000.0) as i32
                    ),
                )?,
                None => context.text(6, "AHRS E nil")?,
            }
            match context.ahrs().get_quaternion()? {
                Some(q) => report(
                    context,
                    format_args!(
                        "AHRS Q {} {} {} {}",
                        (q.q1 * 1000.0) as i32,
                        (q.q2 * 1000.0) as i32,
                        (q.q3 * 1000.0) as i32,
                        (q.q4 * 1000.0) as i32
                    ),
                ),
                None => context.text(6, "AHRS Q nil"),
            }
        };
        match read().and_then(|()| context.sleep_ms(500)) {
            Ok(()) => {}
            Err(Error::Busy) => match context.sleep_ms(10) {
                Ok(()) => {}
                Err(Error::Stopped) => break,
                Err(error) => return Err(error),
            },
            Err(Error::Stopped) => break,
            Err(error) => return Err(error),
        }
    }
    assert_eq!(context.ahrs().get_gyro(), Err(Error::Stopped));
    assert_eq!(context.ahrs().get_velocity_ned(), Err(Error::Stopped));
    assert_eq!(
        context.ahrs().get_relative_position_ned_origin(),
        Err(Error::Stopped)
    );
    assert_eq!(context.ahrs().get_quaternion(), Err(Error::Stopped));
    Ok(())
}
