// SPDX-License-Identifier: GPL-3.0-or-later
//! AI-assisted sine demo sharing SINE_FREQ with Lua, Ada and Fortran.
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, Result};

ap_rust_sdk::module!(run);

fn register(context: &Context<'_>) -> Result {
    if context.param().get("SINE_FREQ")?.is_some() {
        return Ok(());
    }
    if !context.param().add_table(120, "SINE_", 1)? {
        // Lua may have created its table but not yet published the entry.
        return Err(Error::Busy);
    }
    if !context.param().add_param(120, 1, "FREQ", 0.2)? {
        return Err(Error::Host);
    }
    Ok(())
}

fn sample(context: &Context<'_>) -> Result {
    let frequency = context.param().get("SINE_FREQ")?.ok_or(Error::Invalid)?;
    let seconds = (context.time_us() / 1000) as u32 as f32 * 0.001;
    let angle = core::f32::consts::TAU * frequency * seconds + core::f32::consts::FRAC_PI_2;
    let value = context.libm().sinf(angle)?;
    context.gcs().send_named_float("Sine_rust", value)
}

fn run(context: &Context<'_>) -> Result {
    loop {
        match register(context) {
            Err(Error::Busy) => context.sleep_ms(10)?,
            result => {
                result?;
                break;
            }
        }
    }
    loop {
        match sample(context) {
            Ok(()) | Err(Error::Busy) => (),
            Err(error) => return Err(error),
        }
        context.sleep_ms(100)?;
    }
}
