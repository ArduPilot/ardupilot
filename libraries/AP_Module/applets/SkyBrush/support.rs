// SPDX-License-Identifier: GPL-3.0-or-later
//! Small formatting helper for GCS text without allocation.

use core::fmt::{self, Write};

struct Buf<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl Write for Buf<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for &b in s.as_bytes() {
            if self.len < self.buf.len() {
                self.buf[self.len] = b;
                self.len += 1;
            }
        }
        Ok(())
    }
}

/// Format into `buf`, truncating; the result is always valid UTF-8 (ASCII expected).
pub fn fmt<'a>(buf: &'a mut [u8], args: fmt::Arguments<'_>) -> &'a str {
    let mut w = Buf { buf, len: 0 };
    let _ = w.write_fmt(args);
    let len = w.len;
    let s = &w.buf[..len];
    match core::str::from_utf8(s) {
        Ok(v) => v,
        Err(e) => core::str::from_utf8(&s[..e.valid_up_to()]).unwrap_or(""),
    }
}
