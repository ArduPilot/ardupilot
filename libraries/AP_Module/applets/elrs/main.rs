// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]
mod mavlink_crc;
mod parameters;
#[allow(dead_code)]
mod protocol;
#[allow(dead_code)]
mod receiver_control;
mod stream;
#[allow(dead_code)]
mod tracker;
#[allow(dead_code)]
mod transport;
use ap_rust_sdk::{Context, Error, Result};
use core::fmt::Write;

const DEBUG: bool = false;

struct Message {
    bytes: [u8; 90],
    length: usize,
}
impl Write for Message {
    fn write_str(&mut self, value: &str) -> core::fmt::Result {
        if self.length + value.len() > self.bytes.len() {
            return Err(core::fmt::Error);
        }
        self.bytes[self.length..self.length + value.len()].copy_from_slice(value.as_bytes());
        self.length += value.len();
        Ok(())
    }
}
ap_rust_sdk::module!(run);
fn run(context: &Context<'_>) -> Result {
    let mut pending_bind = parameters::configure(context)?;
    // This exported function uses only its borrowed buffers and bounded receiver policy code.
    if !unsafe {
        context
            .periph()
            .sx1280_register(1, receiver_control::sx1280_receiver_v1 as *const () as u32)?
    } {
        return Err(Error::Invalid);
    }
    if DEBUG {
        let _ = context.periph().debug_write("Rust ELRS service registered");
    }
    loop {
        parameters::update(context, &mut pending_bind)?;
        if DEBUG {
            let stats = context.periph().sx1280_stats()?;
            let mut message = Message {
                bytes: [0; 90],
                length: 0,
            };
            let _ = write!(
                message,
                "Rust ELRS n={} bad={} off={} err={} us={}",
                stats.accepted, stats.rejected, stats.unavailable, stats.faults, stats.max_us
            );
            if let Ok(text) = core::str::from_utf8(&message.bytes[..message.length]) {
                let _ = context.periph().debug_write(text);
            }
        }
        context.sleep_ms(1000)?;
    }
}
