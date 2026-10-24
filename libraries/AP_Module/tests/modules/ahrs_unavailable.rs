// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, Result};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    // The ChibiOS ModuleMath harness has no AHRS singleton. Exercise actual
    // firmware dispatch, including graceful absence, on both Cortex-M targets.
    assert_eq!(context.ahrs().get_gyro(), Err(Error::Unavailable));
    assert_eq!(context.ahrs().get_velocity_ned(), Err(Error::Unavailable));
    assert_eq!(
        context.ahrs().get_relative_position_ned_origin(),
        Err(Error::Unavailable)
    );
    assert_eq!(context.ahrs().get_quaternion(), Err(Error::Unavailable));
    context.text(6, "MATH ok") // completion marker shared with the zero-heap harness
}
