// SPDX-License-Identifier: GPL-3.0-or-later
// Frozen AP_Module ABI-3 compatibility source. Do not rebuild checked-in images
// during tests: their purpose is to exercise older, unchanged machine code.
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Result, Vector3f};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    assert_eq!(Vector3f::new(3.0, 4.0, 0.0).length(context)?, 5.0);
    context.text(6, "MATH ok")
}
