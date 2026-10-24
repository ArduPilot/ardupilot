// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

extern crate alloc;

use ap_rust_sdk::{Context, Error, Result};

struct AllocatingDrop;

impl Drop for AllocatingDrop {
    fn drop(&mut self) {
        let allocation = alloc::vec::Vec::<u8>::with_capacity(128);
        core::hint::black_box(&allocation);
    }
}

fn run(context: &Context<'_>) -> Result {
    let mut values = alloc::vec::Vec::<u32>::new();
    values.try_reserve_exact(128).map_err(|_| Error::NoMemory)?;
    for value in 0..128 {
        values.push(value);
    }
    core::hint::black_box(&values);
    context.text(6, "ready")?;
    match option_env!("AP_MODULE_LIFECYCLE") {
        Some("panic") => panic!("lifecycle panic"),
        Some("oom") => {
            let allocation = alloc::vec::Vec::<u8>::with_capacity(32768);
            core::hint::black_box(&allocation);
        }
        Some("return") => return Ok(()),
        Some("bindings_unavailable") => {
            // The manager example has no sensor or vehicle singletons. Its
            // actual generated dispatcher must report absence, not dereference
            // null objects or change the module's ABI fingerprint.
            if context.ahrs().get_roll_rad() != Err(Error::Unavailable)
                || context.arming().is_armed() != Err(Error::Unavailable)
                || context.vehicle().get_mode() != Err(Error::Unavailable)
                || context.gps().num_sensors() != Err(Error::Unavailable)
                || context.rc().get_pwm(1) != Err(Error::Unavailable)
            {
                return Err(Error::Host);
            }
        }
        Some("leak") => {
            core::mem::forget(values);
            return Ok(());
        }
        Some("cleanup") => {
            let _cleanup = AllocatingDrop;
            loop {
                context.sleep_ms(60_000)?;
            }
        }
        Some("never") => loop {
            core::hint::spin_loop();
        },
        Some("late_stop") => {
            while !context.stop_requested() {
                core::hint::spin_loop();
            }
            // Exceed the vehicle manager's stop deadline, then return normally.
            let until = context.time_us() + 15_000_000;
            while context.time_us() < until {
                core::hint::spin_loop();
            }
            return Ok(());
        }
        Some("flood") => {
            for _ in 0..1000 {
                context.text(6, "burst")?;
            }
        }
        Some("late") => {
            // Deliberately ignore stop to exercise retention and a late exit.
            let until = context.time_us() + 500_000;
            while context.time_us() < until {
                core::hint::spin_loop();
            }
            return Ok(());
        }
        _ => {}
    }
    loop {
        context.sleep_ms(60_000)?;
    }
}

ap_rust_sdk::module!(run);
