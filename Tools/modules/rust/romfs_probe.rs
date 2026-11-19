// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use ap_rust_sdk::{Context, Error, Result, Vector3f};

// A nonzero mutable static makes every generation exercise the RWPI/r9 path.
static GENERATION_VALUE: AtomicU32 = AtomicU32::new(41);

fn run(context: &Context<'_>) -> Result {
    if GENERATION_VALUE.fetch_add(1, Ordering::Relaxed) != 41 {
        return Err(Error::Host);
    }
    let vector = Vector3f::new(3.0, 4.0, 0.0);
    if vector.length(context)? != 5.0 {
        return Err(Error::Host);
    }
    context.text(6, "PARAM ready")?;
    loop {
        match context.sleep_ms(1000) {
            Ok(()) => {}
            Err(Error::Stopped) => return Ok(()),
            Err(error) => return Err(error),
        }
    }
}

ap_rust_sdk::module!(run);
