// SPDX-License-Identifier: GPL-3.0-or-later
//! AI-assisted sine demo sharing SINE_FREQ with Lua, Ada and Fortran.
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, Result};

ap_rust_sdk::param_table! {
    pub struct Table, enum P;
    key = 120, prefix = "SINE_";
    size = 1;

    // @Param: SINE_FREQ
    // @DisplayName: Sine wave frequency
    // @Description: Frequency of the sine wave reported by the example.
    // @Units: Hz
    // @User: Advanced
    Freq = 1, "FREQ", f32: 0.2;
}

ap_rust_sdk::module!(run);

fn register(context: &Context<'_>) -> Result {
    if context.param().get("SINE_FREQ")?.is_some() {
        return Ok(());
    }
    if !context.param().add_table(Table::KEY, Table::PREFIX, Table::SIZE)? {
        // Lua may have created its table but not yet published the entry.
        return Err(Error::Busy);
    }
    Table::definition(P::Freq).register(context, Table::KEY)?;
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
