# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Audited, allocation-free value adapters for Lua's AP_Math userdata."""

import naming

TYPES = {
    'Vector3f': dict(fields=['x', 'y', 'z'], default=[0, 0, 0], depends=None, operators={'+': 'add', '-': 'sub'}),
    'Quaternion': dict(fields=['q1', 'q2', 'q3', 'q4'], default=[1, 0, 0, 0],
                       depends='AP_AHRS_ENABLED', operators={'*': 'mul'}),
}
# result, receiver direction, (argument type, direction). Userdata direction
# is audited against AP_Math, NOT inferred from Lua's optional 'Ref annotation.
METHODS = {
    'Vector3f': {
        'length': ('float', 'in', []),
        'normalize': ('void', 'inout', []),
        'is_nan': ('boolean', 'in', []),
        'is_inf': ('boolean', 'in', []),
        'is_zero': ('boolean', 'in', []),
        'dot': ('float', 'in', [('Vector3f', 'in')]),
        'cross': ('Vector3f', 'in', [('Vector3f', 'in')]),
        'scale': ('Vector3f', 'in', [('float', 'in')]),
        'rotate_xy': ('void', 'inout', [('float', 'in')]),
        'angle': ('float', 'in', [('Vector3f', 'in')]),
    },
    'Quaternion': {
        'length': ('float', 'in', []),
        'normalize': ('void', 'inout', []),
        'get_euler_roll': ('float', 'in', []),
        'get_euler_pitch': ('float', 'in', []),
        'get_euler_yaw': ('float', 'in', []),
        'from_euler': ('void', 'inout', [('float', 'in')] * 3),
        'inverse': ('Quaternion', 'in', []),
        'earth_to_body': ('void', 'in', [('Vector3f', 'inout')]),
        'to_axis_angle': ('void', 'in', [('Vector3f', 'out')]),
        'from_axis_angle': ('void', 'inout', [('Vector3f', 'in'), ('float', 'in')]),
        'from_angular_velocity': ('void', 'inout', [('Vector3f', 'in'), ('float', 'in')]),
    },
}
POLICY = 'v1:component-le-f32,allow-nonfinite,cpp-math,copyback-on-success,no-allocation'
ARGUMENT_NAMES = {
    'Vector3f': dict(dot=['other'], cross=['other'], scale=['factor'], rotate_xy=['angle_rad'], angle=['other']),
    'Quaternion': dict(from_euler=['roll', 'pitch', 'yaw'], earth_to_body=['vector'], to_axis_angle=['vector'],
                       from_axis_angle=['axis', 'theta'], from_angular_velocity=['rate', 'dt']),
}


def ir_type(kind, argument=False):
    return dict(kind='userdata' if kind in TYPES else kind, name=kind if kind in TYPES else None,
                nullable=False, reference=False, pointer=False, skip_check=argument and kind == 'float', range=None)


def select(ir, selection, first_id):
    if not any(item == dict(name='AP_Math/AP_Math.h', depends=None) for item in ir['includes']):
        raise ValueError('unaudited AP_Math include')
    result = []
    for name, layout in sorted(TYPES.items()):
        config = dict(include='AP_Math/AP_Math.h', methods=sorted(METHODS[name]), operators=sorted(layout['operators']))
        if selection[name] != config:
            raise ValueError(f'unaudited {name} selection')
        owner = next((item for item in ir['userdata'] if item['name'] == name), None)
        expected = dict(rename=None, depends=layout['depends'], get=None, creation=None, creation_manual=False,
                        creation_args=0, has_fields=True, has_enums=False, operator_getter=None,
                        literal=False, reference=False, semaphore=False, scheduler_semaphore=False,
                        semaphore_pointer=False, aliases=[], operators_manual=False)
        if owner is None or any(owner[key] != value for key, value in expected.items()):
            raise ValueError(f'unaudited {name} owner metadata')
        if sorted(owner['operator_names']) != sorted(layout['operators']):
            raise ValueError(f'unaudited {name} operators')
        if sorted(field['name'] for field in owner['fields']) != sorted(layout['fields']):
            raise ValueError(f'unaudited {name} fields')
        for field in owner['fields']:
            expected = dict(name=field['name'], rename=None, depends=None, array_len=None,
                            valid_mask_name=None, valid_mask_value=None, read=True, write=True, type=ir_type('float', True))
            if {key: value for key, value in field.items() if key != 'line'} != expected:
                raise ValueError(f'unaudited {name}.{field["name"]} field')
        methods = {method['name']: method for method in owner['methods']}
        if not set(ARGUMENT_NAMES[name]).issubset(METHODS[name]):
            raise ValueError(f'stale {name} argument names')
        entries = []
        for method_name, (return_type, receiver, arguments) in sorted(METHODS[name].items()):
            argument_names = ARGUMENT_NAMES[name].get(method_name, [])
            if len(argument_names) != len(arguments):
                raise ValueError(f'missing {name}.{method_name} argument names')
            method = methods.get(method_name)
            if (method is None or any(method[key] is not None for key in ('rename', 'depends', 'deprecate')) or
                    method['return'] != ir_type(return_type) or
                    method['arguments'] != [ir_type(kind, True) for kind, _ in arguments]):
                raise ValueError(f'unaudited {name}.{method_name} signature')
            entries.append(dict(name=method_name, result=return_type, receiver=receiver,
                                arguments=[dict(type=kind, direction=direction, name=arg_name)
                                           for (kind, direction), arg_name in
                                           zip(arguments, argument_names)]))
        for operator, method_name in sorted(layout['operators'].items()):
            entries.append(dict(name=method_name, operator=operator, result=name, receiver='in',
                                arguments=[dict(type=name, direction='in', name='other')]))
        for entry in entries:
            entry['id'] = first_id
            first_id += 1
        result.append(dict(name=name, fields=layout['fields'], default=layout['default'],
                           depends=layout['depends'], methods=entries))
    return dict(policy=POLICY, types=result)


def size(kind):
    if kind in TYPES:
        return 4 * len(TYPES[kind]['fields'])
    return {'float': 4, 'boolean': 1, 'void': 0}[kind]


def wire_fields(owner, method):
    inputs = [('ud', owner)]
    outputs = [] if method['result'] == 'void' else [('value', method['result'])]
    if method['receiver'] == 'inout':
        outputs.append(('ud', owner))
    for index, arg in enumerate(method['arguments']):
        if arg['direction'] != 'out':
            inputs.append((f'arg{index}', arg['type']))
        if arg['direction'] != 'in':
            outputs.append((f'arg{index}', arg['type']))
    return inputs, outputs


def cpp_read(kind, offset):
    if kind == 'float':
        return f'BindingWire::read_float(input + {offset})'
    components = ', '.join(cpp_read('float', offset + 4 * i) for i in range(len(TYPES[kind]['fields'])))
    return f'{kind}({components})'


def cpp(contract, notice):
    # Import hashes cover the exact wire contract and the generated Rust side
    # supplies fixed-size buffers. Per-case length checks do not protect trusted
    # native pointers, so the dispatcher deliberately omits them.
    lines = [notice, '#include <AP_Module/Bindings.h>', '#if AP_MODULE_ENABLED',
             '#include <AP_Module/BindingSupport.h>', '#include <AP_Module/HostApi.h>',
             '#include <AP_Math/AP_Math.h>', '#include <AP_AHRS/AP_AHRS_config.h>',
             '', 'namespace AP_Module', '{']
    for owner in contract['types']:
        lines.append(f'static_assert(sizeof({owner["name"]}) == {size(owner["name"])}, '
                     f'"audit {owner["name"]} value fields");')
        for index, field in enumerate(owner['fields']):
            lines.append(f'static_assert(offsetof({owner["name"]}, {field}) == {index * 4}, '
                         f'"audit {owner["name"]}.{field}");')
    lines += [
             'int32_t binding_math_dispatch(BindingId id, const uint8_t *input, uint32_t,',
             '                              uint8_t *output, uint32_t)', '{', '    switch (id) {']
    for owner in contract['types']:
        for method in owner['methods']:
            inputs, outputs = wire_fields(owner['name'], method)
            ni, no = (sum(size(kind) for _, kind in fields) for fields in (inputs, outputs))
            binding_id = naming.binding_constant(owner['name'] + '.' + method['name'])
            lines += [f'    case BindingId::{binding_id}: {{ // {owner["name"]}::{method["name"]}']
            if owner['depends']:
                lines.append(f'#if {owner["depends"]}')
            offset = 0
            for name, kind in inputs:
                qualifier = '' if (name, kind) in outputs else 'const '
                lines.append(f'        {qualifier}{kind} {name} = {cpp_read(kind, offset)};')
                offset += size(kind)
            for name, kind in outputs:
                if name != 'value' and (name, kind) not in inputs:
                    lines.append(f'        {kind} {name}{{}};')
            arguments = ', '.join(f'arg{i}' for i in range(len(method['arguments'])))
            call = (f'ud {method["operator"]} arg0' if 'operator' in method else f'ud.{method["name"]}({arguments})')
            lines.append('        ' + ('' if method['result'] == 'void' else 'const auto value = ') + call + ';')
            offset = 0
            for name, kind in outputs:
                if kind in TYPES:
                    lines.append(f'        BindingWire::write_native(output + {offset}, {name});')
                    offset += size(kind)
                elif kind == 'float':
                    lines.append(f'        BindingWire::write_float(output + {offset}, {name});')
                    offset += 4
                else:
                    lines.append(f'        output[{offset}] = {name} ? 1 : 0;')
                    offset += 1
            lines.append('        return AP_MODULE_STATUS_OK;')
            if owner['depends']:
                lines += ['#else', '        return AP_MODULE_STATUS_UNAVAILABLE;', '#endif']
            lines.append('    }')
    return '\n'.join(lines + ['    default:', '        return AP_MODULE_STATUS_INVALID;', '    }', '}', '}', '#endif', ''])


def rust_kind(kind):
    return {'float': 'f32', 'boolean': 'bool', 'void': '()'}.get(kind, kind)


def rust(contract, notice):
    lines = [notice, 'use crate::{result, Context, Error, Result};', '']
    for owner in contract['types']:
        name, fields = owner['name'], owner['fields']
        lines += ['/// A local, allocation-free copy of the Lua/AP_Math value.',
                  '/// Arithmetic calls the same C++ implementation as Lua, including its',
                  '/// non-finite and degenerate-input behavior. Fields are unchecked f32 values.',
                  '#[derive(Clone, Copy, Debug, PartialEq)]', f'pub struct {name} {{']
        lines += [f'    pub {field}: f32,' for field in fields]
        lines += ['}', '', f'impl Default for {name} {{', '    fn default() -> Self {',
                  '        Self::new(' + ', '.join(f'{value}.0' for value in owner['default']) + ')', '    }', '}', '',
                  f'impl {name} {{', '    pub const fn new(' + ', '.join(f'{field}: f32' for field in fields) + ') -> Self {',
                  '        Self { ' + ', '.join(fields) + ' }', '    }', '',
                  f'    pub(crate) fn encode(&self) -> [u8; {size(name)}] {{', f'        let mut bytes = [0u8; {size(name)}];']
        for i, field in enumerate(fields):
            lines.append(f'        bytes[{4*i}..{4*i+4}].copy_from_slice(&self.{field}.to_le_bytes());')
        lines += ['        bytes', '    }', '', '    pub(crate) fn decode(bytes: &[u8]) -> Self {', '        Self {']
        for i, field in enumerate(fields):
            lines.append(f'            {field}: f32::from_le_bytes(bytes[{4*i}..{4*i+4}].try_into().unwrap()),')
        lines += ['        }', '    }', '']
        for method in owner['methods']:
            inputs, outputs = wire_fields(name, method)
            ni, no = (sum(size(kind) for _, kind in items) for items in (inputs, outputs))
            args = ''.join(f', {arg["name"]}: {"&mut " if arg["direction"] != "in" else ""}{rust_kind(arg["type"])}'
                           for arg in method['arguments'])
            receiver = '&mut self' if method['receiver'] == 'inout' else '&self'
            lines.append(f'    /// `{name}::{method.get("operator", method["name"])}` from bindings.desc.')
            if method['name'] == 'normalize':
                lines.append('    /// As in Lua, zero length ' +
                             ('sets a firmware internal error (SITL debug builds panic).'
                              if name == 'Quaternion' else 'divides by zero, producing NaNs or an enabled FP trap.'))
            if method['name'] == 'from_axis_angle':
                lines.append('    /// Axis must have unit length; theta is in radians, as in Lua.')
            if method['name'] == 'from_angular_velocity':
                lines.append('    /// Same small-angle integration as Lua; rate is rad/s and dt is seconds.')
            lines += [f'    pub fn {method["name"]}({receiver}, context: &Context<\'_>{args})'
                      f' -> Result<{rust_kind(method["result"])}> {{', f'        let mut input = [0u8; {ni}];']
            offset = 0
            for field, kind in inputs:
                value = 'self' if field == 'ud' else method['arguments'][int(field[3:])]['name']
                encode = 'encode' if kind in TYPES else 'to_le_bytes'
                lines.append(f'        input[{offset}..{offset+size(kind)}].copy_from_slice(&{value}.{encode}());')
                offset += size(kind)
            lines += [f'        let mut output = [0u8; {no}];', '        result(unsafe {',
                      ('            context.api.binding_call.unwrap()(context.raw, '
                       f'crate::imports::BINDING_{method["id"]}.slot(),'),
                      '                input.as_ptr(),',
                      '                input.len() as u32, output.as_mut_ptr(), output.len() as u32)', '        })?;']
            offset = 0
            for field, kind in outputs:
                if kind in TYPES:
                    decode = f'{kind}::decode(&output[{offset}..{offset+size(kind)}])'
                elif kind == 'float':
                    decode = f'f32::from_le_bytes(output[{offset}..{offset+4}].try_into().unwrap())'
                else:
                    decode = f'match output[{offset}] {{ 0 => false, 1 => true, _ => return Err(Error::Host) }}'
                lines.append(f'        let {field} = {decode};')
                offset += size(kind)
            for field, _ in outputs:
                if field != 'value':
                    target = 'self' if field == 'ud' else method['arguments'][int(field[3:])]['name']
                    lines.append(f'        *{target} = {field};')
            lines += ['        Ok(())' if method['result'] == 'void' else '        Ok(value)', '    }', '']
        lines += ['}', '']
    return '\n'.join(lines)
