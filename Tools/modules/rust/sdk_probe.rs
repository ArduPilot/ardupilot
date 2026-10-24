// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use alloc::vec::Vec;
use ap_rust_sdk::{Context, Error, Result};

ap_rust_sdk::module!(run);

struct PanicOnFormat;

impl core::fmt::Display for PanicOnFormat {
    fn fmt(&self, _: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        panic!("panic handler must not invoke this formatter");
    }
}

fn run(context: &Context<'_>) -> Result {
    // Host-selected modes let the harness test independent fresh images.
    match context.time_us() {
        1 => panic!("deliberate SDK panic"),
        2 => {
            let value = Box::new([42_u8; 1024]);
            core::hint::black_box(&value);
        }
        3 => panic!("custom message: {}", PanicOnFormat),
        4 => {
            return if context.arming().is_armed() == Err(Error::Host) {
                Ok(())
            } else {
                Err(Error::Host)
            };
        }
        5 => {
            return if context.ahrs().get_hagl() == Err(Error::Host) {
                Ok(())
            } else {
                Err(Error::Host)
            };
        }
        6 => {
            return if context.rc().get_pwm(1) == Err(Error::Busy) {
                Ok(())
            } else {
                Err(Error::Host)
            };
        }
        7 => {
            return match context.parameter("VALUE") {
                Err(Error::NoMemory) => Ok(()),
                _ => Err(Error::Host),
            }
        }
        8 => {
            return match context.parameter("VALUE") {
                Ok(None) => Ok(()),
                _ => Err(Error::Host),
            }
        }
        9 => {
            return match context.parameter("VALUE") {
                Err(Error::Host) => Ok(()),
                _ => Err(Error::Host),
            }
        }
        10 => {
            return match context.parameter("VALUE") {
                Err(Error::Stopped) => Ok(()),
                _ => Err(Error::Host),
            }
        }
        11 => {
            let parameter = context.parameter("VALUE")?.ok_or(Error::Host)?;
            return if parameter.get() == Err(Error::Stopped) {
                Ok(())
            } else {
                Err(Error::Host)
            };
        }
        12 => {
            return match context.parameter("") {
                Err(Error::Invalid) => Ok(()),
                _ => Err(Error::Host),
            }
        }
        _ => {}
    }
    if context.stop_requested() {
        return Err(Error::Stopped);
    }
    if context.arming().is_armed() != Err(Error::Unavailable) {
        return Err(Error::Host);
    }
    let mut values = Vec::<u32>::new();
    values.try_reserve_exact(64).map_err(|_| Error::NoMemory)?;
    for index in (0..64).rev() {
        values.push(core::hint::black_box(index));
    }
    values.sort_unstable();
    if !values.iter().copied().eq(0..64) {
        return Err(Error::Host);
    }
    context.text(6, "sdk start")?;
    // Keep the allocation live across a stop; returning via ? must drop it.
    context.sleep_ms(60000)?;
    core::hint::black_box(&values);
    context.text(6, "sdk done")?;
    Ok(())
}
