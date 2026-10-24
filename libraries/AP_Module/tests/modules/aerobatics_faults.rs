// SPDX-License-Identifier: GPL-3.0-or-later
//! Test-build-only faults at the runner's fallible read/delivery boundaries.
//! No code or parameter interpretation from this file exists in normal modules.
use ap_rust_sdk::{Context, Error, Result};

#[derive(Default)]
pub struct Faults {
    mode: u8,
    started: u64,
    announced: bool,
}

impl Faults {
    pub fn check(&mut self, c: &Context<'_>, mode: u8, delivery: bool, finish: bool) -> Result {
        if mode != self.mode {
            self.mode = mode;
            self.started = c.time_us();
            self.announced = false;
        }
        if mode == 0 || delivery != (mode == 2 || mode == 4 || mode == 5) {
            return Ok(());
        }
        if mode == 5 {
            if finish {
                c.text(6, "AERO finish fault injected")?;
                return Err(Error::Busy);
            }
            return Ok(());
        }
        if !self.announced {
            c.text(6, "AERO fault injected")?;
            self.announced = true;
        }
        let duration = if mode >= 3 { 1500000 } else { 300000 };
        if c.time_us().saturating_sub(self.started) < duration {
            Err(Error::Busy)
        } else {
            Ok(())
        }
    }
}
