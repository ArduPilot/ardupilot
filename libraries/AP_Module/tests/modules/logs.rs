// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, LogValue as V, Result};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    // All Lua-supported format codes, with stable metadata across reloads.
    for _ in 0..100 {
        let result = context.logger().write(
            "RLG1",
            "A,B,C,D,E,F,G,H,I,J,K,L,M,N,O",
            None,
            &[
                V::I8(-8),
                V::U8(250),
                V::I16(-1234),
                V::U16(65000),
                V::I32(-1234567),
                V::U32(4000000000),
                V::U64(0xfedcba9876543210),
                V::Float(1.25),
                V::CentiI16(-100),
                V::CentiU16(100),
                V::CentiI32(-100000),
                V::CentiU32(100000),
                V::LatLon(-353632610),
                V::Mode(3),
                V::Text4(b"AB"),
            ],
        );
        match result {
            // The file-decoding SITL test proves records were written. The
            // standalone probe also exercises feature-off graceful return.
            Err(Error::Unavailable) => return context.text(6, "MATH ok"),
            value => value?,
        }
        context.logger().write(
            "RLG2",
            "Short,Long,Nan,Inf",
            None,
            &[
                V::Text16(b"sixteen"),
                V::Text64(b"sixty-four"),
                V::Float(f32::from_bits(0x7fc12345)),
                V::Float(f32::INFINITY),
            ],
        )?;
        context.logger().write(
            "RLGP",
            "Height,Lat,Who",
            Some(("mD#", "0G-")),
            &[V::Float(100.5), V::LatLon(-353632610), V::U8(1)],
        )?;
        if !cfg!(target_os = "none") {
            context.sleep_ms(20)?;
        }
    }
    context.text(6, "MATH ok")
}
