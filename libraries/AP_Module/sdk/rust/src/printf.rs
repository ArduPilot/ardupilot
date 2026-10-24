// SPDX-License-Identifier: GPL-3.0-or-later

use crate::{result, Context, Error, Result};

/// Typed arguments for the experimental firmware printf bridge. Conversions
/// must match: Int32 for %d/%i/%c, Uint32 for %u/%o/%x/%X, Int64/Uint64 for
/// ll forms, Float for %f/%e/%g (also uppercase), and Str for %s.
pub enum PrintfArg<'a> {
    Int32(i32),
    Uint32(u32),
    Int64(i64),
    Uint64(u64),
    Float(f64),
    Str(&'a str),
}

macro_rules! argument_from {
    ($type:ty, $variant:ident) => {
        impl From<$type> for PrintfArg<'_> {
            fn from(value: $type) -> Self {
                Self::$variant(value.into())
            }
        }
    };
}
argument_from!(i32, Int32);
argument_from!(u32, Uint32);
argument_from!(i64, Int64);
argument_from!(u64, Uint64);
argument_from!(f32, Float);
argument_from!(f64, Float);

impl<'a> From<&'a str> for PrintfArg<'a> {
    fn from(value: &'a str) -> Self {
        Self::Str(value)
    }
}

/// Bounded UTF-8 text formatted by firmware, without heap allocation.
pub struct PrintfText {
    bytes: [u8; 258],
    length: usize,
}

impl PrintfText {
    pub fn as_str(&self) -> &str {
        // format() validates UTF-8 before constructing the public value.
        unsafe { core::str::from_utf8_unchecked(&self.bytes[2..2 + self.length]) }
    }
}

pub(crate) fn format(
    context: &Context<'_>,
    slot: u32,
    format: &str,
    args: &[PrintfArg<'_>],
) -> Result<PrintfText> {
    if format.len() > 255 || format.as_bytes().contains(&0) || args.len() > 8 {
        return Err(Error::Invalid);
    }
    let mut input = [0u8; 585];
    input[..format.len()].copy_from_slice(format.as_bytes());
    input[256] = args.len() as u8;
    let mut strings = 0;
    for (index, arg) in args.iter().enumerate() {
        let (tag, bits) = match arg {
            PrintfArg::Int32(value) => (1, *value as u32 as u64),
            PrintfArg::Uint32(value) => (2, *value as u64),
            PrintfArg::Int64(value) => (3, *value as u64),
            PrintfArg::Uint64(value) => (4, *value),
            PrintfArg::Float(value) => (5, value.to_bits()),
            PrintfArg::Str(value) => {
                if value.as_bytes().contains(&0) || value.len() >= 256 - strings {
                    return Err(Error::Invalid);
                }
                let offset = strings;
                input[329 + offset..329 + offset + value.len()].copy_from_slice(value.as_bytes());
                strings += value.len() + 1;
                (6, offset as u64)
            }
        };
        input[257 + index] = tag;
        input[265 + index * 8..273 + index * 8].copy_from_slice(&bits.to_le_bytes());
    }
    let mut text = PrintfText {
        bytes: [0; 258],
        length: 0,
    };
    result(unsafe {
        context.api.binding_call.unwrap()(
            context.raw,
            slot,
            input.as_ptr(),
            input.len() as u32,
            text.bytes.as_mut_ptr(),
            text.bytes.len() as u32,
        )
    })?;
    text.length = u16::from_le_bytes(text.bytes[..2].try_into().unwrap()) as usize;
    if text.length > 255 || text.bytes[2 + text.length] != 0 {
        return Err(Error::Host);
    }
    let bytes = &text.bytes[2..2 + text.length];
    if bytes.contains(&0) || core::str::from_utf8(bytes).is_err() {
        return Err(Error::Invalid);
    }
    Ok(text)
}
