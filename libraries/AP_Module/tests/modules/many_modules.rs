// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Result};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    // Every live worker needs a distinct binding owner, not just a thread.
    context.mavlink().init(1, 1)?;
    context.mavlink().register_rx_msgid(0)?;
    loop {
        context.sleep_ms(60_000)?;
    }
}
