// SPDX-License-Identifier: GPL-3.0-or-later
//! Bounded numeric schedules. Executable Lua custom functions require a module rebuild.

use super::{
    maneuvers::{self, Command},
    path::{Attributes, Evaluation, Path},
};
use alloc::{string::String, vec::Vec};
use ap_rust_sdk::{Context, Error, Result};

pub const MAX_BYTES: usize = 16384;
const MAX_COMMANDS: usize = 256;

pub struct Schedule {
    pub name: String,
    pub fingerprint: u32,
    commands: Vec<Specification>,
}

struct Specification {
    command: Command,
    args: [f32; 4],
    attributes: Attributes,
    message: Option<String>,
}

fn copy_text(value: &str) -> Result<String> {
    let mut s = String::new();
    s.try_reserve_exact(value.len())
        .map_err(|_| Error::NoMemory)?;
    s.push_str(value);
    Ok(s)
}

fn number(value: &str, convert: &impl Fn(&str) -> Result<(f32, usize)>) -> Result<f32> {
    // Keep decimal-only, whole-token, finite schedule numbers. The firmware
    // parser also accepts hexadecimal, whitespace and partial conversions.
    if value.is_empty()
        || value.len() > 255
        || !value
            .bytes()
            .all(|b| b.is_ascii_digit() || matches!(b, b'+' | b'-' | b'.' | b'e' | b'E'))
    {
        return Err(Error::Invalid);
    }
    let (n, consumed) = convert(value)?;
    if consumed == value.len() && n.is_finite() {
        Ok(n)
    } else {
        Err(Error::Invalid)
    }
}

fn boolean(value: &str) -> Result<bool> {
    match value {
        "true" => Ok(true),
        "false" => Ok(false),
        _ => Err(Error::Invalid),
    }
}

impl Schedule {
    fn parse(text: &str, convert: impl Fn(&str) -> Result<(f32, usize)>) -> Result<Self> {
        if text.len() > MAX_BYTES || text.as_bytes().contains(&0) {
            return Err(Error::Invalid);
        }
        let mut name = copy_text("Trick")?;
        let mut commands = Vec::new();
        let mut attributes = Attributes::default();
        let mut message = None;
        let mut pending = false;
        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            if let Some((key, value)) = line.split_once(':') {
                let value = value.trim();
                match key {
                    "name" => {
                        if value.is_empty()
                            || value.len() > 48
                            || !value
                                .bytes()
                                .all(|c| c.is_ascii_alphanumeric() || c == b'_')
                        {
                            return Err(Error::Invalid);
                        }
                        name = copy_text(value)?;
                        continue;
                    }
                    "message" => {
                        if value.is_empty()
                            || value.len() > 50
                            || !value.bytes().all(|b| {
                                b.is_ascii_alphanumeric()
                                    || matches!(b, b'_' | b':' | b' ' | b'-' | b'\t')
                            })
                        {
                            return Err(Error::Invalid);
                        }
                        message = Some(copy_text(value)?);
                    }
                    "roll_ref" => attributes.roll_ref = Some(number(value, &convert)?),
                    "thr_boost" => attributes.thr_boost = Some(boolean(value)?),
                    "pos_corr" => attributes.pos_corr = Some(number(value, &convert)?),
                    "pos_gain_mul" => attributes.pos_gain_mul = Some(number(value, &convert)?),
                    "shift_xy" => attributes.shift_xy = Some(boolean(value)?),
                    // Timestamp synchronization/rate overrides need their
                    // own controller support; do not silently ignore them.
                    _ => return Err(Error::Invalid),
                }
                pending = true;
                continue;
            }
            let mut words = line.split_whitespace();
            let command =
                Command::named(words.next().ok_or(Error::Invalid)?).ok_or(Error::Invalid)?;
            let mut args = [0.0; 4];
            for (i, word) in words.enumerate() {
                if i >= args.len() {
                    return Err(Error::Invalid);
                }
                args[i] = number(word, &convert)?;
            }
            if commands.len() == MAX_COMMANDS {
                return Err(Error::Invalid);
            }
            commands.try_reserve(1).map_err(|_| Error::NoMemory)?;
            commands.push(Specification {
                command,
                args,
                attributes,
                message: message.take(),
            });
            attributes = Attributes::default();
            pending = false;
        }
        if commands.is_empty() || pending {
            return Err(Error::Invalid);
        }
        Ok(Self {
            name,
            commands,
            fingerprint: text
                .bytes()
                .fold(2166136261u32, |h, b| (h ^ b as u32).wrapping_mul(16777619)),
        })
    }

    pub fn load(c: &Context<'_>, filename: &str) -> Result<Option<Self>> {
        let mut bytes = Vec::new();
        let mut chunk = [0u8; 256];
        loop {
            let n = match c.file().read_at(filename, bytes.len() as u32, &mut chunk)? {
                Some(n) => n,
                None if bytes.is_empty() => return Ok(None),
                None => return Err(Error::Host),
            };
            if n == 0 {
                break;
            }
            if bytes.len() + n > MAX_BYTES {
                return Err(Error::Invalid);
            }
            bytes.try_reserve(n).map_err(|_| Error::NoMemory)?;
            bytes.extend_from_slice(&chunk[..n]);
        }
        let text = core::str::from_utf8(&bytes).map_err(|_| Error::Invalid)?;
        Ok(Some(Self::parse(text, |value| c.string().strtof(value))?))
    }

    pub fn build(&self, c: &Context<'_>, box_width: f32, eval: Evaluation) -> Result<Path> {
        let mut paths = Vec::new();
        paths
            .try_reserve_exact(self.commands.len())
            .map_err(|_| Error::NoMemory)?;
        for spec in &self.commands {
            let mut path = maneuvers::build(c, spec.command, spec.args, box_width, eval)?;
            path.attributes = spec.attributes;
            path.rebuild_on_reentry = true;
            path.message = spec.message.as_deref().map(copy_text).transpose()?;
            paths.push(path);
        }
        Path::compose(c, paths, box_width, eval)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // Exercise the same prefix/end-offset contract as the firmware. The binding
    // tests cover the C-library adapter; these tests cover schedule validation.
    fn convert(value: &str) -> Result<(f32, usize)> {
        use std::ffi::{c_char, CString};
        extern "C" {
            fn strtof(text: *const c_char, end: *mut *mut c_char) -> f32;
        }
        let text = CString::new(value).map_err(|_| Error::Invalid)?;
        let mut end = core::ptr::null_mut();
        let n = unsafe { strtof(text.as_ptr(), &mut end) };
        Ok((n, unsafe { end.offset_from(text.as_ptr()) } as usize))
    }
    fn parse(text: &str) -> Result<Schedule> {
        Schedule::parse(text, convert)
    }
    #[test]
    fn decimal_numbers() {
        for (text, expected) in [
            ("25", 25.0f32),
            ("-.5", -0.5),
            ("+1.25e2", 125.0),
            ("1.", 1.0),
            ("-0", -0.0),
            ("1e-999", 0.0),
        ] {
            assert_eq!(
                number(text, &convert).unwrap().to_bits(),
                expected.to_bits()
            );
        }
        for text in [
            "", "+", ".", "1e", "1e+", "1.2.3", "1-2", "1e999", "NaN", "inf", "0x1p2", " 1", "1 ",
            "1\0", "1_0", "١",
        ] {
            assert_eq!(number(text, &convert), Err(Error::Invalid), "{text}");
        }
        let long = "0".repeat(256);
        assert_eq!(number(&long[..255], &convert), Ok(0.0));
        assert_eq!(number(&long, &convert), Err(Error::Invalid));
        assert_eq!(number("1", &|_| Err(Error::Stopped)), Err(Error::Stopped));
    }
    #[test]
    fn numeric_schedule() {
        let s = parse("# hello\nname: Test\nmessage: Loop\nloop 25 0 1\nalign_box 1\nloop -20\n")
            .unwrap();
        assert_eq!(s.name, "Test");
        assert_eq!(s.commands.len(), 3);
        assert_eq!(s.commands[0].args, [25.0, 0.0, 1.0, 0.0]);
        assert_eq!(s.commands[0].message.as_deref(), Some("Loop"));
        assert!(s.commands[1].message.is_none());
    }
    #[test]
    fn reject_unsupported_or_malformed() {
        for text in [
            "",
            "#empty",
            "name: Only",
            "unknown 5",
            "loop 1 2 3 4 5",
            "loop NaN",
            "loop inf",
            "function triangular_loop(r)",
            "loop 25\nmessage: dangling",
            "timestamp: 1\nloop 25",
            "thr_boost: maybe\nloop 25",
            "name: two words\nloop 25",
            "loop 25\0",
            "message: Loop #1\nloop 25",
            "loop 25 # trailing comment",
        ] {
            assert!(parse(text).is_err(), "{text}");
        }
    }
    #[test]
    fn text_bounds_and_whitespace() {
        assert_eq!(
            parse("name: Test\r\n\tloop\t25\t0\t1\r\n")
                .unwrap()
                .commands
                .len(),
            1
        );
        let mut exact = copy_text("loop 25\n#").unwrap();
        while exact.len() < MAX_BYTES {
            exact.push('x');
        }
        assert!(parse(&exact).is_ok());
        exact.push('x');
        assert!(parse(&exact).is_err());
        let mut commands = String::new();
        for _ in 0..MAX_COMMANDS {
            commands.push_str("loop 25\n");
        }
        assert!(parse(&commands).is_ok());
        commands.push_str("loop 25\n");
        assert!(parse(&commands).is_err());
        let mut long_name = copy_text("name: ").unwrap();
        for _ in 0..49 {
            long_name.push('x');
        }
        long_name.push_str("\nloop 25");
        assert!(parse(&long_name).is_err());
    }
}
