// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Result};
use core::fmt;

ap_rust_sdk::module!(run);

struct BadDisplay;

impl fmt::Display for BadDisplay {
    fn fmt(&self, _: &mut fmt::Formatter<'_>) -> fmt::Result {
        panic!("user Display must not run");
    }
}

fn run(context: &Context<'_>) -> Result {
    match option_env!("AP_MODULE_PANIC_TEST") {
        Some("display") => panic!("display {}", BadDisplay),
        Some("stopped") => {
            context.text(6, "panic test ready")?;
            while !context.stop_requested() {
                let _ = context.sleep_ms(1);
            }
            panic!("stopped");
        }
        _ => panic!("P"),
    }
}
