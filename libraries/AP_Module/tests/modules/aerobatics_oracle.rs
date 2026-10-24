// SPDX-License-Identifier: GPL-3.0-or-later
use ap_rust_sdk::{Context, Error, LogValue as V, Result};
#[path = "../../applets/aerobatics/support.rs"]
mod support;

pub struct Oracle {
    offset: u32,
    cache: [u8; 256],
    available: usize,
    absolute: f32,
    relative: f32,
    maximum: f32,
    worst_ratio: f32,
    worst: Option<(u32, f32, f32)>,
    bit_exact: bool,
}
impl Oracle {
    pub fn new() -> Self {
        Self {
            offset: 0,
            cache: [0; 256],
            available: 0,
            absolute: 2e-5,
            relative: 2e-6,
            maximum: 0.0,
            worst_ratio: 1.0,
            worst: None,
            bit_exact: false,
        }
    }

    pub fn controller() -> Self {
        Self {
            absolute: 0.0,
            relative: 0.0,
            bit_exact: true,
            ..Self::new()
        }
    }

    fn expected(&mut self, c: &Context<'_>) -> Result<f32> {
        let index = self.offset as usize % self.cache.len();
        if index == 0 {
            self.available = c
                .file().read_at("aerobatics-golden.bin", self.offset, &mut self.cache)?
                .ok_or(Error::Invalid)?;
        }
        if index + 4 > self.available {
            return Err(Error::Invalid);
        }
        self.offset += 4;
        Ok(f32::from_le_bytes(
            self.cache[index..index + 4].try_into().unwrap(),
        ))
    }

    pub fn record(
        &mut self,
        c: &Context<'_>,
        name: &str,
        labels: &str,
        values: &[V<'_>],
    ) -> Result {
        if !cfg!(target_os = "none") {
            return c.logger().write(name, labels, None, values);
        }
        for value in values {
            let actual = match value {
                V::U8(v) => *v as f32,
                V::U16(v) => *v as f32,
                V::Float(v) => *v,
                _ => return Err(Error::Invalid),
            };
            let expected = self.expected(c)?;
            let equal = if self.bit_exact {
                actual.to_bits() == expected.to_bits()
            } else {
                actual == expected
            };
            if equal || (actual.is_nan() && expected.is_nan()) {
                continue;
            }
            self.maximum = self.maximum.max((actual - expected).abs());
            let tolerance = (expected.abs() * self.relative).max(self.absolute);
            if self.bit_exact
                || !actual.is_finite()
                || !expected.is_finite()
                || (actual - expected).abs() > tolerance
            {
                let ratio = (actual - expected).abs() / tolerance;
                if !ratio.is_finite() || ratio > self.worst_ratio {
                    self.worst_ratio = ratio;
                    self.worst = Some((self.offset - 4, actual, expected));
                }
            }
        }
        Ok(())
    }

    pub fn finish(&self, c: &Context<'_>) -> Result {
        if let Some((offset, actual, expected)) = self.worst {
            let text = support::Text::<96>::new(format_args!(
                "oracle byte {} actual {:08x} expected {:08x} maxabs {:08x}",
                offset,
                actual.to_bits(),
                expected.to_bits(),
                self.maximum.to_bits()
            ))?;
            c.text(3, text.as_str())?;
            return Err(Error::Invalid);
        }
        if cfg!(target_os = "none")
            && c.file().read_at("aerobatics-golden.bin", self.offset, &mut [0u8; 1])? != Some(0)
        {
            return Err(Error::Invalid);
        }
        if cfg!(target_os = "none") {
            let text = support::Text::<96>::new(format_args!(
                "oracle maximum absolute {:08x}",
                self.maximum.to_bits()
            ))?;
            c.text(6, text.as_str())?;
        }
        Ok(())
    }
}
