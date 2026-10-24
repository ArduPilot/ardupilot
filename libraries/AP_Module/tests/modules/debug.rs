// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec::Vec;
use ap_rust_sdk::{Context, Error, Result};

#[no_mangle]
static mut APMOD_DEBUG_COUNT: u32 = 7;

#[no_mangle]
static APMOD_DEBUG_MAGIC: u32 = 0x12345678;

#[inline(never)]
#[no_mangle]
fn apmod_debug_tick(values: &mut Vec<u32>, label: &str) -> u32 {
    let count = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(APMOD_DEBUG_COUNT)) };
    core::hint::black_box(label);
    values[0] = count;
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(APMOD_DEBUG_COUNT),
            count.wrapping_add(1),
        )
    };
    core::hint::black_box(&APMOD_DEBUG_MAGIC);
    count
}

fn run(context: &Context<'_>) -> Result {
    let mut values = Vec::new();
    values.try_reserve_exact(3).map_err(|_| Error::NoMemory)?;
    values.extend_from_slice(&[1, 2, 3]);
    context.sleep_ms(100)?;
    loop {
        core::hint::black_box(apmod_debug_tick(&mut values, "apmod debug"));
        context.sleep_ms(100)?;
    }
}

ap_rust_sdk::module!(run);
