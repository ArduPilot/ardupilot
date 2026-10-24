// SPDX-License-Identifier: GPL-3.0-or-later

use crate::{bindings::Gcs, enums::MavSeverity, Error, Result};
use core::fmt::{self, Write};

struct Message {
    bytes: [u8; 255],
    length: usize,
}

impl Write for Message {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        let end = self.length + value.len();
        if end > self.bytes.len() {
            return Err(fmt::Error);
        }
        self.bytes[self.length..end].copy_from_slice(value.as_bytes());
        self.length = end;
        Ok(())
    }
}

impl Gcs<'_, '_> {
    /// Experimental HAL printf formatting followed by send_text. See
    /// string().format_printf() for supported conversions and bounds.
    pub fn send_text_printf(
        &self,
        severity: MavSeverity,
        format: &str,
        args: &[crate::PrintfArg<'_>],
    ) -> Result {
        let text = self.context.string().format_printf(format, args)?;
        self.send_text(severity, text.as_str())
    }

    /// Format and send text using a bounded stack buffer, without heap allocation.
    ///
    /// Pass `format_args!("Tick {:.1}", seconds)` for compile-time format checking.
    /// Like `send_text`, accepts at most 255 UTF-8 bytes and no NUL characters.
    /// Overflow or a formatting error returns `Error::Invalid` without sending
    /// partial text. Host errors are propagated; delivery is not guaranteed.
    pub fn send_text_fmt(&self, severity: MavSeverity, args: fmt::Arguments<'_>) -> Result {
        let mut message = Message {
            bytes: [0; 255],
            length: 0,
        };
        message.write_fmt(args).map_err(|_| Error::Invalid)?;
        let text =
            core::str::from_utf8(&message.bytes[..message.length]).map_err(|_| Error::Invalid)?;
        self.send_text(severity, text)
    }
}
