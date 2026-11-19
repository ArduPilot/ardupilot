// SPDX-License-Identifier: GPL-3.0-or-later

use ap_rust_sdk::{enums::MavSeverity, Context, Error, Result};
use core::fmt::{self, Write};

pub fn retry<T>(context: &Context<'_>, mut call: impl FnMut() -> Result<T>) -> Result<T> {
    let start = context.time_us();
    loop {
        match call() {
            Err(Error::Busy) if context.time_us().saturating_sub(start) < 100_000 => {
                context.sleep_ms(1)?;
            }
            result => return result,
        }
    }
}

pub struct Text<const N: usize> {
    bytes: [u8; N],
    len: usize,
}

impl<const N: usize> Text<N> {
    pub fn new(args: fmt::Arguments<'_>) -> Result<Self> {
        let mut result = Self {
            bytes: [0; N],
            len: 0,
        };
        result.write_fmt(args).map_err(|_| Error::Invalid)?;
        Ok(result)
    }

    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len]).unwrap()
    }
}

impl<const N: usize> Write for Text<N> {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        if value.len() > N - self.len {
            return Err(fmt::Error);
        }
        self.bytes[self.len..self.len + value.len()].copy_from_slice(value.as_bytes());
        self.len += value.len();
        Ok(())
    }
}

pub fn message(context: &Context<'_>, severity: i32, args: fmt::Arguments<'_>) -> Result {
    let text = Text::<128>::new(args)?;
    retry(context, || {
        context.gcs().send_text(MavSeverity(severity), text.as_str())
    })
}
