// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Error, Result};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    let mut buffer = [0xaau8; 256];
    for path in [
        "",
        "@SYS/version.txt",
        "/@PARAM/param.pck",
        "//@MISSION/mission.dat",
    ] {
        if context.file().read_at(path, 0, &mut buffer) != Err(Error::Invalid) {
            return Err(Error::Host);
        }
    }
    if context.file().read_at("file-not-present.bin", 0, &mut buffer)? != None
        || buffer != [0xaa; 256]
    {
        return Err(Error::Host);
    }
    // Exceed FatFS's 16 file slots on every load, proving per-call close.
    for _ in 0..10 {
        if context.file().read_at("file-test.bin", 0, &mut buffer)? != Some(256) {
            return Err(Error::Host);
        }
        for (i, byte) in buffer.iter().enumerate() {
            if *byte != i as u8 {
                return Err(Error::Host);
            }
        }
        if context.file().read_at("file-test.bin", 257, &mut buffer)? != Some(256) {
            return Err(Error::Host);
        }
        for (i, byte) in buffer.iter().enumerate() {
            if *byte != (i as u8).wrapping_add(1) {
                return Err(Error::Host);
            }
        }
        buffer.fill(0xaa);
        if context.file().read_at("file-test.bin", 768, &mut buffer)? != Some(3)
            || &buffer[..3] != b"end"
            || buffer[3..].iter().any(|b| *b != 0xaa)
            || context.file().read_at("file-test.bin", 771, &mut buffer)? != Some(0)
            || context.file().read_at("file-test.bin", 1000, &mut buffer)? != Some(0)
        {
            return Err(Error::Host);
        }
    }
    // Both FatFS and AP_Filesystem's POSIX wrapper reject directory opens.
    if context.file().read_at("APM", 0, &mut buffer) != Ok(None) {
        return Err(Error::Host);
    }
    context.text(6, "MATH ok")
}
