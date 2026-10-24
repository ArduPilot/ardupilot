// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{
    enums::{RcChannelAuxFunc, SrvChannelFunction},
    Context, Error, Result,
};
use core::fmt::{self, Write};

ap_rust_sdk::module!(run);

struct Message {
    bytes: [u8; 64],
    length: usize,
}

impl Write for Message {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let end = self.length + text.len();
        if end > self.bytes.len() {
            return Err(fmt::Error);
        }
        self.bytes[self.length..end].copy_from_slice(text.as_bytes());
        self.length = end;
        Ok(())
    }
}

fn exercise(context: &Context<'_>) -> Result {
    if context.arming().is_armed()? {
        return Err(Error::Invalid); // This fixture must never operate in flight.
    }
    let aux = match context.rc().get_aux_cached(RcChannelAuxFunc(300))? {
        Some(value) => value,
        None => return Ok(()),
    };
    let mode = match aux {
        0 => 0, // MANUAL
        1 => 5, // FBWA
        2 => 4, // ACRO
        _ => return Err(Error::Host),
    };
    if context.vehicle().get_mode()? != mode && !context.vehicle().set_mode(mode)? {
        return Err(Error::Host);
    }
    // Exercise reentrant pre-arm checks while the test restarts two workers.
    let _ = context.arming().pre_arm_checks()?;
    let function = SrvChannelFunction(70); // Saved parameter value: throttle.
    let channel = context
        .srv_channels().find_channel(function)?
        .ok_or(Error::Host)?;
    let pwm = context
        .srv_channels().get_output_pwm(function)?
        .ok_or(Error::Host)?;
    let mut message = Message {
        bytes: [0; 64],
        length: 0,
    };
    write!(
        &mut message,
        "ENUM {} {} {} {} {} {}",
        aux,
        channel,
        pwm,
        context.vehicle().get_mode()?,
        context.vehicle().get_control_mode_reason()?,
        context.gcs().frame_type()?.0
    )
    .map_err(|_| Error::Host)?;
    context.text(
        6,
        core::str::from_utf8(&message.bytes[..message.length]).map_err(|_| Error::Host)?,
    )
}

fn run(context: &Context<'_>) -> Result {
    loop {
        match exercise(context).and_then(|()| context.sleep_ms(100)) {
            Ok(()) => {}
            Err(Error::Busy) => match context.sleep_ms(10) {
                Ok(()) => {},
                Err(Error::Stopped) => return Ok(()),
                Err(error) => return Err(error),
            },
            Err(Error::Stopped) => return Ok(()),
            Err(error) => return Err(error),
        }
    }
}
