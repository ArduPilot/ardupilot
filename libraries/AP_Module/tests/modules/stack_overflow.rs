// SPDX-License-Identifier: GPL-3.0-or-later
// Destructive fixture: only run in the guarded H7 test build.
#![no_std]
#![no_main]

extern crate alloc;
use ap_rust_sdk::{Context, Error, Result};

#[inline(never)]
#[allow(unconditional_recursion)]
fn recurse(context: &Context<'_>, depth: u32, value: f32) -> u32 {
    let mut words = [0u32; 32];
    let index = depth as usize % words.len();
    // Keep a real frame and a post-recursion read: neither tail calls nor
    // optimizing away unused local arrays should remove the overflow.
    unsafe { core::ptr::write_volatile(&mut words[index], depth) };
    let next = if option_env!("AP_MODULE_STACK_FP") == Some("1") {
        core::hint::black_box(value) * 1.0001 + 0.1
    } else {
        value
    };
    if option_env!("AP_MODULE_STACK_HOST_CALL") == Some("1") {
        core::hint::black_box(context.time_us());
    }
    let result = recurse(context, core::hint::black_box(depth.wrapping_add(1)), next);
    result.wrapping_add(unsafe { core::ptr::read_volatile(&words[index]) })
}

fn run(context: &Context<'_>) -> Result {
    let mut allocation = alloc::vec::Vec::<u8>::new();
    allocation.try_reserve_exact(512).map_err(|_| Error::NoMemory)?;
    // The owner must reclaim this allocation without running Rust destructors.
    core::hint::black_box(&allocation);
    context.sleep_ms(20)?;
    core::hint::black_box(recurse(context, 0, 0.5));
    Ok(())
}

ap_rust_sdk::module!(run);
