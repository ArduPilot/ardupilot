// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{enums::MavMountMode, Context, Error, Result};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    // The harness has a real mount singleton without a backend. Its default
    // mode is RETRACT; range checks must reject bad enum values before calling.
    for _ in 0..100 {
        assert_eq!(context.mount().get_mode(0)?, MavMountMode(0));
        assert_eq!(
            context.mount().set_mode(0, MavMountMode(-1)),
            Err(Error::Invalid)
        );
        assert_eq!(
            context.mount().set_mode(0, MavMountMode(i32::MAX)),
            Err(Error::Invalid)
        );
        context.mount().set_mode(0, MavMountMode(0))?;
        assert_eq!(context.mount().get_mode(0)?, MavMountMode(0));
        // No vehicle singleton in this harness; exercise a literal-argument
        // method without changing any simulated vehicle state.
        assert_eq!(context.vehicle().set_mode(0), Err(Error::Unavailable));
    }
    context.text(6, "MATH ok")
}
