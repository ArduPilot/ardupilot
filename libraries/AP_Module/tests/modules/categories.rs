// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, Result};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    // Signed scalar input, scalar return and seven nullable outputs. Exercise
    // both directions through the actual firmware methods, without allocation.
    for _ in 0..100 {
        // Negative i8 survives the wire; the existing RTC method rejects the
        // converted uint8_t month, exactly as the Lua binding does.
        if context.rtc().date_fields_to_clock_s(2024, -1, 29, 12, 34, 56)? != 0 {
            return Err(Error::Host);
        }
        // The existing RTC/Lua API uses zero-based months.
        let seconds = context.rtc().date_fields_to_clock_s(2024, 1, 29, 12, 34, 56)?;
        let date = context
            .rtc().clock_s_to_date_fields(seconds)?
            .ok_or(Error::Host)?;
        if date != (2024, 1, 29, 12, 34, 56, 4) {
            return Err(Error::Host);
        }
        // Real void + three Ref outputs, not nullable outputs or input refs.
        let (_red, _green, _blue) = context.led().get_rgb()?;
    }
    context.text(6, "MATH ok")
}
