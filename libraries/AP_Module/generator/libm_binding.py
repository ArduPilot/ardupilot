# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Float libm used by Lua's LUA_32BITS math library, without a Lua dependency."""

FUNCTIONS = {
    'acosf': 1, 'asinf': 1, 'atan2f': 2, 'ceilf': 1, 'cosf': 1, 'expf': 1,
    'floorf': 1, 'fmodf': 2, 'logf': 1, 'powf': 2, 'sinf': 1, 'sqrtf': 1, 'tanf': 1,
}
POLICY = 'v1:c99-libm-float32,all-f32-bits,board-fp-environment,no-allocation'


def specification(first_id):
    return [dict(id=first_id + i, name=name, arity=arity) for i, (name, arity) in enumerate(sorted(FUNCTIONS.items()))]


def cpp(methods):
    lines = []
    for method in methods:
        name, arity = method['name'], method['arity']
        lines += [f'static int32_t binding_libm_{name}(const uint8_t *input, uint32_t,',
                  '                                    uint8_t *output, uint32_t)', '{']
        args = ', '.join(f'BindingWire::read_float(input + {i * 4})' for i in range(arity))
        lines += [f'    const float value = ::{name}({args});', '    BindingWire::write_float(output, value);',
                  '    return AP_MODULE_STATUS_OK;', '}', '']
    return lines


def rust(methods, notice):
    lines = [notice, 'use crate::{result, Result};', '', "impl crate::bindings::Libm<'_, '_> {"]
    for method in methods:
        number, name, arity = method['id'], method['name'], method['arity']
        args = ''.join(f', arg{i}: f32' for i in range(arity))
        lines += [f'    /// Firmware `{name}` (same float C library as Lua on this board).',
                  '    /// NaN/Inf and domain errors follow the board floating-point environment,',
                  '    /// including enabled traps. No allocation; stop is checked on every call.',
                  f'    pub fn {name}(&self{args}) -> Result<f32> {{',
                  f'        let mut input = [0u8; {arity * 4}];']
        for i in range(arity):
            lines.append(f'        input[{i * 4}..{(i + 1) * 4}].copy_from_slice(&arg{i}.to_le_bytes());')
        lines += ['        let mut output = [0u8; 4];', '        result(unsafe {',
                  f'            self.context.api.binding_call.unwrap()(self.context.raw, '
                  f'crate::imports::BINDING_{number}.slot(),',
                  '                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)',
                  '        })?;', '        Ok(f32::from_le_bytes(output))', '    }', '']
    return '\n'.join(lines + ['}', ''])
