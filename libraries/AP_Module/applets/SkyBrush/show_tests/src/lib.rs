// SPDX-License-Identifier: GPL-3.0-or-later
//! Host tests for the no_std show module.
#![no_std]
extern crate alloc;
extern crate std;

#[path = "../../show/mod.rs"]
pub mod show;

#[cfg(test)]
mod tests;
#[path = "../../lights_out.rs"]
pub mod lights_out;
