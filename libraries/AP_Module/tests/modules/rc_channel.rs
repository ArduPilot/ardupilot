// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{enums::RcChannelAuxFunc, Context, Error, RcChannel, Result};
use core::fmt::Write;

ap_rust_sdk::module!(run);

struct Text {
    bytes: [u8; 64],
    len: usize,
}
impl Write for Text {
    fn write_str(&mut self, value: &str) -> core::fmt::Result {
        let end = self.len + value.len();
        if end > self.bytes.len() {
            return Err(core::fmt::Error);
        }
        self.bytes[self.len..end].copy_from_slice(value.as_bytes());
        self.len = end;
        Ok(())
    }
}

fn update(context: &Context<'_>, cached: &mut Option<RcChannel>, last_action: &mut u8) -> Result {
    if context.arming().is_armed()? {
        return Err(Error::Invalid);
    }
    let channel = match *cached {
        Some(channel) => channel,
        None => {
            let Some(channel) = context.rc().find_channel_for_option(RcChannelAuxFunc(301))? else {
                return Ok(());
            };
            if context.rc().get_channel(9)? != Some(channel) {
                return Err(Error::Host);
            }
            *cached = Some(channel);
            channel
        }
    };
    let action = context
        .rc().get_aux_cached(RcChannelAuxFunc(300))?
        .unwrap_or(0);
    if action > 0 {
        channel.set_override(context, if action == 1 { 1600 } else { 1900 })?;
    } else if *last_action > 0 {
        channel.set_override(context, 0)?;
    }
    *last_action = action;
    let mut text = Text {
        bytes: [0; 64],
        len: 0,
    };
    write!(
        text,
        "RCB {} {} {} {}",
        (channel.norm_input(context)? * 1000.0) as i32,
        (channel.norm_input_dz(context)? * 1000.0) as i32,
        (channel.norm_input_ignore_trim(context)? * 1000.0) as i32,
        channel.get_aux_switch_pos(context)?
    )
    .map_err(|_| Error::Host)?;
    context.text(
        6,
        core::str::from_utf8(&text.bytes[..text.len]).map_err(|_| Error::Host)?,
    )
}

fn run(context: &Context<'_>) -> Result {
    let (mut cached, mut last_action) = (None, 0);
    let (mut attempts, mut busy) = (0u32, 0u32);
    loop {
        attempts += 1;
        match update(context, &mut cached, &mut last_action) {
            Ok(()) => context.sleep_ms(100)?,
            Err(Error::Busy) => {
                busy += 1;
                context.sleep_ms(10)?;
            }
            Err(error) => return Err(error),
        }
        if attempts % 100 == 0 {
            let mut text = Text {
                bytes: [0; 64],
                len: 0,
            };
            write!(text, "RCC {attempts} {busy}").map_err(|_| Error::Host)?;
            context.text(
                6,
                core::str::from_utf8(&text.bytes[..text.len]).map_err(|_| Error::Host)?,
            )?;
        }
    }
}
