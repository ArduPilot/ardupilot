// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, Result};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    let channel = context.rc().get_channel(1)?.ok_or(Error::Host)?;
    if context.rc().get_channel(0) != Err(Error::Invalid) {
        return Err(Error::Host);
    }
    for _ in 0..100 {
        if context.rc().get_channel(1)? != Some(channel)
            || channel.norm_input(context)? != 0.25
            || channel.norm_input_dz(context)? != 0.25
            || channel.norm_input_ignore_trim(context)? != 0.25
            || channel.get_aux_switch_pos(context)? != 1
        {
            return Err(Error::Host);
        }
    }
    context.text(6, "MATH ok")
}
