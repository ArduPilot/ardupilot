// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{enums::MavSeverity, Context, Error, Result};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    let long = [b'x'; 256];
    if context.fs().crc32(core::str::from_utf8(&long).unwrap()) != Err(Error::Invalid)
        || context.fs().crc32("embedded\0NUL") != Err(Error::Invalid)
        || context.fs().crc32("rust-string-missing-file")? != None
    {
        return Err(Error::Host);
    }
    // AP_Filesystem uses crc_crc32 with initial zero and no final complement,
    // not zlib's complemented CRC. Both harnesses create the same fixture.
    for _ in 0..10 {
        if context.fs().crc32("string-test.txt")? != Some(0x2dfd2d88) {
            return Err(Error::Host);
        }
    }
    // Exercise the two-string wrapper's 512-byte packet even without a GCS.
    let longest = core::str::from_utf8(&long[..255]).unwrap();
    let two_strings = context.gcs().send_named_string(longest, longest);
    match context.gcs().send_text(MavSeverity(6), "STRING literal %s %n") {
        // ModuleMath has no GCS singleton; the vehicle exercises the live calls.
        Err(Error::Unavailable) => {
            if two_strings != Err(Error::Unavailable) {
                return Err(Error::Host);
            }
            context.text(6, "MATH ok")
        }
        Err(error) => Err(error),
        Ok(()) => loop {
            two_strings?;
            context.gcs().send_text(MavSeverity(6), "STRING literal %s %n")?;
            context.gcs().send_named_float("RUST_PERR", 12.5)?;
            context.gcs().send_named_int("RUST_INT", -12345)?;
            context.gcs().send_named_string("RUST_STR", "native %s")?;
            context.sleep_ms(200)?;
        },
    }
}
