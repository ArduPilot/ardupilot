// SPDX-License-Identifier: GPL-3.0-or-later
//! Request fallible allocations on demand without terminating the worker on OOM.
#![no_std]
#![no_main]
extern crate alloc;

use alloc::vec::Vec;
use ap_rust_sdk::{Context, Error, Result};
ap_rust_sdk::module!(run);

fn run(c: &Context<'_>) -> Result {
    if !c.param().add_table(190, "REXP_", 2)?
        || !c.param().add_param(190, 1, "REQUEST", 0.0)?
        || !c.param().add_param(190, 2, "RESULT", 0.0)?
    {
        return Err(Error::Host);
    }
    c.param().set("REXP_REQUEST", 0.0)?;
    c.param().set("REXP_RESULT", 0.0)?;
    c.text(6, "EXPAND ready")?;
    let mut memory = Vec::<u8>::new();
    let mut previous = 0;
    loop {
        c.sleep_ms(25)?;
        let request = c.param().get("REXP_REQUEST")?.ok_or(Error::Unavailable)? as i32;
        if request == previous {
            continue;
        }
        previous = request;
        memory.clear();
        memory.shrink_to_fit();
        // Negative requests return immediately after allocating, exercising
        // the manager's expansion accounting when a worker exits between polls.
        let size = request.unsigned_abs() as usize;
        let result = if memory.try_reserve_exact(size).is_ok() {
            memory.resize(size, 0x5a);
            size as f32
        } else {
            -(size as f32)
        };
        c.param().set("REXP_RESULT", result)?;
        if request < 0 {
            return Ok(());
        }
    }
}
