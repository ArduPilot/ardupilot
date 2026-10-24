// SPDX-License-Identifier: GPL-3.0-or-later
//! Send the time since boot once per second, like simple_loop.lua.
#![no_std]
#![no_main]

use ap_rust_sdk::{enums::MavSeverity, Context, Result};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    loop {
        let seconds = context.time_us() as f32 / 1_000_000.0;
        context
            .gcs()
            .send_text_printf(MavSeverity::INFO, "Tick %.1f", &[seconds.into()])?;
        context.sleep_ms(1000)?;
    }
}
