# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Typed, bounded equivalent of Lua's manual logger.write binding."""

FIELDS = {
    'b': ('I8', 'i8', 1), 'B': ('U8', 'u8', 1), 'h': ('I16', 'i16', 2), 'H': ('U16', 'u16', 2),
    'i': ('I32', 'i32', 4), 'I': ('U32', 'u32', 4), 'Q': ('U64', 'u64', 8), 'f': ('Float', 'f32', 4),
    'c': ('CentiI16', 'i16', 2), 'C': ('CentiU16', 'u16', 2), 'e': ('CentiI32', 'i32', 4),
    'E': ('CentiU32', 'u32', 4), 'L': ('LatLon', 'i32', 4), 'M': ('Mode', 'u8', 1),
    'n': ('Text4', "&'a [u8]", 4), 'N': ('Text16', "&'a [u8]", 16), 'Z': ('Text64', "&'a [u8]", 64),
}
# name, labels, format, units, multipliers, metadata flag, payload length, payload
SIZES = [5, 58, 16, 16, 16, 1, 2, 244]
POLICY = ('v1:lua-dynamic-logger,no-scheduler-lock,host-TimeUS-u64,all-f32-bits,'
          'firmware-owned-copied-metadata,persistent-until-reboot,shared-message-id-pool,'
          'checked-payload-and-labels-and-units,post-lookup-format-match,SITL-conflict-config-error,'
          'best-effort-record-no-durability-guarantee')


def specification(first_id):
    return dict(id=first_id, name='logger.write',
                arguments=[dict(type=dict(name='LogRecord',
                                          fields=[['name', 'nonempty-cstring5'], ['labels', 'nonempty-cstring58'],
                                                  ['format', 'nonempty-cstring16'], ['units', 'cstring16'],
                                                  ['multipliers', 'cstring16'], ['have_units', 'bool-u8'],
                                                  ['payload_len', 'u16-le'], ['payload', 'bytes-max244-zero-pad244']],
                                          formats={code: dict(size=size, scalar=kind if not kind.startswith('&') else
                                                              'bytes-zero-padded,embedded-nul-allowed')
                                                   for code, (_, kind, size) in sorted(FIELDS.items())}), direction='in')],
                result='void', policy=POLICY)


def header(notice):
    lines = [notice, '#pragma once', '#include <stdint.h>', '', 'namespace AP_Module', '{',
             f'static constexpr uint32_t LOG_WIRE_SIZE = {sum(SIZES)}U;',
             'static constexpr uint16_t LOG_WIRE_PAYLOAD_MAX = 244U;',
             'static inline uint8_t log_field_size(char code)', '{', '    switch (code) {']
    for code, (_, _, size) in sorted(FIELDS.items()):
        lines += [f"    case '{code}':", f'        return {size};']
    return '\n'.join(lines + ['    default:', '        return 0;', '    }', '}', '}', ''])


def rust(method, notice):
    lines = [notice, 'use crate::{result, Error, Result};', '',
             '/// DataFlash field encodings. Centi/LatLon variants preserve the raw integer:',
             '/// they select the log format character, not a numeric scaling operation.',
             '/// Byte strings may contain NUL and non-UTF-8 bytes, as in Lua.',
             '#[derive(Clone, Copy, Debug)]', "pub enum LogValue<'a> {"]
    for _, (variant, kind, _) in sorted(FIELDS.items()):
        lines.append(f'    {variant}({kind}),')
    lines += ['}', '', "impl LogValue<'_> {", '    fn format_size(&self) -> (u8, usize) {', '        match self {']
    for code, (variant, _, size) in sorted(FIELDS.items()):
        lines.append(f"            Self::{variant}(..) => (b'{code}', {size}),")
    lines += ['        }', '    }', '    fn encode(&self, bytes: &mut [u8]) -> Result {', '        match self {']
    for _, (variant, kind, _) in sorted(FIELDS.items()):
        if kind.startswith('&'):
            lines += [f'            Self::{variant}(value) => {{',
                      '                if value.len() > bytes.len() { return Err(Error::Invalid); }',
                      '                bytes[..value.len()].copy_from_slice(value);', '            }']
        else:
            lines.append(f'            Self::{variant}(value) => bytes.copy_from_slice(&value.to_le_bytes()),')
    lines += ['        }', '        Ok(())', '    }', '}', '',
              'fn metadata(dest: &mut [u8], text: &str) -> Result {',
              '    if text.len() >= dest.len() || text.as_bytes().contains(&0) { return Err(Error::Invalid); }',
              '    dest[..text.len()].copy_from_slice(text.as_bytes());', '    Ok(())', '}', '',
              "impl crate::bindings::Logger<'_, '_> {",
              '    /// Write a timestamped DataFlash record using the same host logger as Lua.',
              '    /// Formats are derived from typed values, up to 15 fields and 244 payload bytes.',
              '    /// Names are 1–4 UTF-8 bytes, labels at most 57; optional units/multipliers',
              '    /// each have one character per value. Labels must be nonempty and unique',
              '    /// (including the implicit TimeUS field). No module-heap allocation.',
              '    /// Use stable names/formats: copied metadata and shared message IDs persist',
              '    /// in firmware memory until reboot, outside the module heap limit.',
              '    /// A conflicting name/format triggers SITL config_error, like Lua; hardware',
              '    /// returns Invalid. IO/format locks may block. Success is best effort, not durable.',
              '    pub fn write(&self, name: &str, labels: &str, units: Option<(&str, &str)>,',
              "                     values: &[LogValue<'_>]) -> Result {",
              '        if name.is_empty() || labels.is_empty() || values.is_empty() || values.len() > 15 {',
              '            return Err(Error::Invalid);', '        }', f'        let mut input = [0u8; {sum(SIZES)}];',
              '        metadata(&mut input[..5], name)?;', '        metadata(&mut input[5..63], labels)?;',
              '        if let Some((units, multipliers)) = units {',
              '            if units.len() != values.len() || multipliers.len() != values.len() {',
              '                return Err(Error::Invalid);', '            }',
              '            metadata(&mut input[79..95], units)?;',
              '            metadata(&mut input[95..111], multipliers)?;',
              '            input[111] = 1;', '        }', '        let mut length = 0;',
              '        for (i, value) in values.iter().enumerate() {', '            let (format, size) = value.format_size();',
              '            if length + size > 244 { return Err(Error::Invalid); }', '            input[63+i] = format;',
              '            value.encode(&mut input[114+length..114+length+size])?;',
              '            length += size;', '        }',
              '        input[112..114].copy_from_slice(&(length as u16).to_le_bytes());',
              '        result(unsafe {',
              '            self.context.api.binding_call.unwrap()(self.context.raw,',
              f'                crate::imports::BINDING_{method["id"]}.slot(),',
              '                input.as_ptr(), input.len() as u32, core::ptr::null_mut(), 0)', '        })', '    }', '}', '']
    return '\n'.join(lines)
