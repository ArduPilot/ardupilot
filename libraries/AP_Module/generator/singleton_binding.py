# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Signature-category codecs and adapters for declarative singleton methods."""

import re
import xml.etree.ElementTree as ET

from pathlib import Path

import borrowed_binding
import fwversion_binding
import math_binding
import value_binding

from native_templates import render_lines

MAVLINK_COMMON = Path(__file__).resolve().parents[3] / 'modules/mavlink/message_definitions/v1.0/common.xml'

SCALARS = {'boolean': ('bool', 1), 'uint8_t': ('u8', 1), 'uint16_t': ('u16', 2),
           'uint32_t': ('u32', 4), 'int8_t': ('i8', 1), 'int16_t': ('i16', 2),
           'int32_t': ('i32', 4), 'float': ('f32', 4)}
TYPES = dict(SCALARS, string=('&str', 17), cstring=('&str', 256),
             fwstring=('FirmwareString', fwversion_binding.MAX_STRING + 1),
             void=('()', 0), enum=('i32', 4), literal=('()', 0),
             **{name: (name, math_binding.size(name)) for name in math_binding.TYPES},
             **{name: (p['rust'], value_binding.size(name)) for name, p in value_binding.TYPES.items()},
             **{name: (f'Option<crate::{p["rust"]}>', 4) for name, p in borrowed_binding.TYPES.items()})


def field(value, argument=False, parameter=False):
    kind = value['name'] if value['kind'] in ('userdata', 'ap_object') else value['kind']
    output = value['nullable'] or value['reference']
    if value['pointer'] or (value['nullable'] and value['reference']) or (output and not argument):
        raise ValueError('unsupported-pointer-or-direction')
    if kind not in TYPES:
        raise ValueError('unsupported-type:' + (value['name'] or value['kind']))
    if kind in borrowed_binding.TYPES and (argument or output or value['kind'] != 'ap_object'):
        raise ValueError('object-input-ownership')
    if kind == 'literal':
        if (not argument or output or value['skip_check'] or value['range'] or
                not re.fullmatch(r'(?:[+-]?\d+|[A-Za-z_]\w*(?:::\w+)*|"[^"\\\n]*")', value['name'] or '')):
            raise ValueError('unsupported-literal')
        if re.fullmatch(r'(?:ud|value|lock|scheduler|owner|token|input|input_size|output|output_size|'
                        r'arg\d+|bits\d+)(?:::\w+)*',
                        value['name']):
            raise ValueError('literal-captures-local')
        return dict(type='literal', output=False, expression=value['name'])
    if kind == 'enum' and not re.fullmatch(r'[A-Za-z_]\w*(?:::\w+)*', value['name'] or ''):
        raise ValueError('unsupported-enum-name')
    if kind == 'enum' and output:
        raise ValueError('enum-output-reference-unspecified')
    if kind == 'string':
        if not argument or output:
            raise ValueError('string-output-ownership')
        if not parameter:
            kind = 'cstring'
    if argument and kind == 'void':
        raise ValueError('void-argument')
    if value['name'] is not None and value['kind'] not in ('userdata', 'enum', 'ap_object'):
        raise ValueError('unexpected-type-name')
    if ((output or value_binding.is_value(kind) or kind in ('void', 'string', 'cstring')) and
            (value['range'] or value['skip_check'])):
        raise ValueError('unsupported-type-annotation')
    result = dict(type=kind, output=output)
    if value_binding.is_value(kind) and argument and not output:
        result['inout'] = True
    if kind == 'enum':
        result['enum'] = value['name']
        if argument and (not value['range'] or value['skip_check']):
            raise ValueError('enum-input-needs-range')
    if value['reference']:
        result['reference'] = True
    if value['range']:
        # Trusted source expressions, not module input. Reject statement syntax;
        # keep bound expressions for runtime validation in the owning firmware.
        if len(value['range']) != 2 or not all(re.fullmatch(r'[A-Za-z_0-9():+*/.<> -]+', b) for b in value['range']):
            raise ValueError('unsupported-range-expression')
        result['range'] = value['range']
    if argument and not output and kind == 'float' and not parameter:
        if value['skip_check']:
            result['allow_nonfinite'] = True
        elif not value['range']:
            raise ValueError('float-input-needs-range-or-skip-check')
    return result


def io(method):
    inputs = [(i, arg) for i, arg in enumerate(method['arguments']) if not arg['output'] and arg['type'] != 'literal']
    outputs = [(i, arg) for i, arg in enumerate(method['arguments']) if arg['output']]
    optional = bool(outputs and not outputs[0][1].get('reference'))
    return inputs, outputs, optional


def sizes(method):
    inputs, outputs, _ = io(method)
    receiver_size = TYPES[method['value_receiver']][1] if method.get('value_receiver') else 0
    return (4 * bool(method.get('receiver')) + receiver_size + sum(TYPES[a['type']][1] for _, a in inputs),
            TYPES[method['result']['type']][1] + sum(TYPES[a['type']][1] for _, a in outputs) + receiver_size +
            sum(TYPES[a['type']][1] for _, a in inputs if a.get('inout')))


def cpp(method):
    inputs, outputs, optional = io(method)
    ni, no = sizes(method)
    # The loader resolves a 64-bit hash over the complete wire contract, and
    # the generated Rust wrapper fixes both buffer lengths. Modules are trusted
    # native code, so repeating length/null checks in every binding adds flash
    # without making an invalid pointer safe. Keep the size arguments only in
    # the shared HostApi ABI.
    lines = render_lines(
        'host_cpp/function.cpp', comment=f'// {method["owner"]}::{method["method"]}', binding_id=method['id'],
        context='void *context, ' if method['owner'] == 'AP_ONVIF' else '',
        input='input' if ni else '', output='output' if no else '',
    )
    if method['owner'] == 'AP_ONVIF':
        lines.append('    (void)context;')
    if ni:
        lines.append('    (void)input;')
    if no:
        lines.append('    (void)output;')
    needs_zero = optional or method['result']['type'] == 'fwstring' or any(
        arg['output'] and arg['type'] == 'fwstring' for arg in method['arguments'])
    if no and needs_zero:
        lines.append(f'    memset(output, 0, {no});')
    guards = [method.get(k) for k in ('include_depends', 'owner_depends', 'depends') if method.get(k)]
    for f in [method['result']] + method['arguments']:
        policy = value_binding.TYPES.get(f['type'], math_binding.TYPES.get(f['type'], {}))
        if policy.get('depends') and policy['depends'] not in guards:
            guards.append(policy['depends'])
    if guards:
        lines.append('#if ' + ' && '.join(f'({g})' for g in guards))
    if method['owner'] == 'AP_ONVIF':
        lines += ['    BindingLock onvif_lock(binding_onvif_semaphore());',
                  '    const int32_t onvif_status = binding_onvif_check_owner(context);',
                  '    if (onvif_status != AP_MODULE_STATUS_OK) {', '        return onvif_status;', '    }']
    if method.get('receiver'):
        lines += borrowed_binding.cpp_token(method['receiver'])
    access = '.' if method.get('reference') or method.get('value_receiver') else '->'
    getter = method.get('get') or (method['owner'] if method.get('literal') else method['owner'] + '::get_singleton()')
    if method.get('value_receiver'):
        lines += value_binding.cpp_read(method['value_receiver'], 'ud', 0)
    elif not method.get('receiver'):
        lines.append(f'    auto {"&" if method.get("reference") else "*"}ud = {getter};')
        if not method.get('reference'):
            lines += ['    if (ud == nullptr) {', '        return AP_MODULE_STATUS_UNAVAILABLE;', '    }']
    offset = (TYPES[method['value_receiver']][1] if method.get('value_receiver') else
              4 * bool(method.get('receiver')))
    dynamic_ranges = []
    for index, arg in enumerate(method['arguments']):
        kind = arg['type']
        if kind == 'literal':
            continue
        ctype = arg.get('enum', 'bool' if kind == 'boolean' else kind)
        if arg['output']:
            lines.append(f'    {ctype} arg{index}{{}};')
            continue
        size = TYPES[kind][1]
        if value_binding.is_value(kind):
            lines += value_binding.cpp_read(kind, f'arg{index}', offset)
        elif kind in ('string', 'cstring'):
            lines += [f'    const char *arg{index} = BindingWire::read_{kind}(input + {offset});',
                      f'    if (arg{index} == nullptr) {{', '        return AP_MODULE_STATUS_INVALID;', '    }']
        elif kind == 'float':
            lines.append(f'    const float arg{index} = BindingWire::read_float(input + {offset});')
            if method.get('adapter'):
                lines += [f'    if (!std::isfinite(arg{index})) {{', '        return AP_MODULE_STATUS_INVALID;', '    }']
        elif kind.startswith('int') or kind == 'enum':
            integer = 'int32_t' if kind == 'enum' else kind
            unsigned = 'u' + integer
            lines += [f'    const {unsigned} bits{index} = {unsigned}(BindingWire::read(input + {offset}, {size}));',
                      f'    {integer} arg{index};', f'    memcpy(&arg{index}, &bits{index}, sizeof(arg{index}));']
            if kind == 'enum':
                lines += [f'    if (!BindingWire::valid_enum<{ctype}>(arg{index})) {{',
                          '        return AP_MODULE_STATUS_INVALID;', '    }']
        else:
            if kind == 'boolean':
                lines += [f'    if (input[{offset}] > 1) {{', '        return AP_MODULE_STATUS_INVALID;', '    }']
            lines.append(f'    const {ctype} arg{index} = {ctype}(BindingWire::read(input + {offset}, {size}));')
        if 'range' in arg:
            low, high = arg['range']
            number = 'float' if kind == 'float' else 'int64_t'
            if kind == 'enum':
                low, high = f'static_cast<int32_t>({low})', f'static_cast<int32_t>({high})'
            check = [f'    if (!BindingWire::in_range<{number}>(arg{index}, ({low}), ({high}))) {{',
                     '        return AP_MODULE_STATUS_INVALID;', '    }']
            (dynamic_ranges if any(re.search(r'\bud\b', bound) for bound in arg['range']) else lines).extend(check)
        offset += size
    if method['lock']:
        lock = method['lock']
        if lock == 'scheduler-semaphore':
            lines += ['#if AP_SCHEDULER_ENABLED', '    auto *scheduler = AP_Scheduler::get_singleton();',
                      '    if (scheduler == nullptr) {', '        return AP_MODULE_STATUS_UNAVAILABLE;', '    }']
            semaphore = 'scheduler->get_semaphore()'
        else:
            semaphore = f'ud{access}get_semaphore()'
            if lock == 'semaphore-pointer':
                lines += [f'    if ({semaphore} == nullptr) {{', '        return AP_MODULE_STATUS_UNAVAILABLE;', '    }']
                semaphore = '*' + semaphore
        lines.append(f'    BindingLock lock({semaphore});')
        if lock == 'scheduler-semaphore':
            lines.append('#endif')
    if method.get('receiver'):
        lines += borrowed_binding.cpp_receiver(method['receiver'])
    lines += dynamic_ranges
    target = method['method']
    if method.get('adapter'):
        if target.startswith('set_'):
            lines += ['    if (!BindingParam::valid_value(arg0, arg1)) {',
                      '        return AP_MODULE_STATUS_INVALID;', '    }']
        elif target == 'add_table':
            target = 'add_table_persistent'
    args = ', '.join(a['expression'] if a['type'] == 'literal' else
                     f'static_cast<{a["enum"]}>(arg{i})' if a['type'] == 'enum' and not a['output'] else f'arg{i}'
                     for i, a in enumerate(method['arguments']))
    prefix = '' if method['result']['type'] == 'void' else 'const auto value = '
    call = (f'ud{access}{method["field"]}' if method.get('field') else
            'ud' if method.get('copy') else
            f'ud.{method["assignment"]} = arg0' if method.get('assignment') else
            f'ud {method["operator"]} arg0' if method.get('operator') else f'ud{access}{target}({args})')
    lines.append(f'    {prefix}{call};')
    fields = [] if method['result']['type'] == 'void' else [('value', method['result'], 0)]
    offset = TYPES[method['result']['type']][1]
    if optional:
        lines += ['    output[0] = value ? 1 : 0;', '    if (value) {']
        fields = []
    for index, arg in outputs:
        fields.append((f'arg{index}', arg, offset))
        offset += TYPES[arg['type']][1]
    indent = '        ' if optional else '    '
    for value, arg, offset in fields:
        kind = arg['type']
        if kind == 'fwstring':
            lines += [f'{indent}if ({value} == nullptr) {{',
                      f'{indent}    return AP_MODULE_STATUS_HOST_ERROR;', f'{indent}}}',
                      f'{indent}const size_t length = strnlen({value}, {fwversion_binding.MAX_STRING + 1});',
                      f'{indent}if (length > {fwversion_binding.MAX_STRING}) {{',
                      f'{indent}    return AP_MODULE_STATUS_HOST_ERROR;', f'{indent}}}',
                      f'{indent}output[{offset}] = uint8_t(length);',
                      f'{indent}memcpy(output + {offset + 1}, {value}, length);']
        elif kind in borrowed_binding.TYPES:
            lines += [f'{indent}if (!write_handle_{kind}(output + {offset}, {value})) {{',
                      f'{indent}    return AP_MODULE_STATUS_HOST_ERROR;', f'{indent}}}']
        elif value_binding.is_value(kind):
            lines += value_binding.cpp_write(kind, value, offset, indent)
        elif kind == 'float':
            lines.append(f'{indent}BindingWire::write_float(output + {offset}, {value});')
        elif kind == 'enum':
            lines += [f'{indent}if (!BindingWire::write_enum(output + {offset}, {value})) {{',
                      f'{indent}    return AP_MODULE_STATUS_HOST_ERROR;', f'{indent}}}']
        else:
            lines.append(f'{indent}BindingWire::write(output + {offset}, uint32_t({value}), {TYPES[kind][1]});')
    if optional:
        lines.append('    }')
    # In/out copies are independent of a bool result's optional output values.
    offset = TYPES[method['result']['type']][1] + sum(TYPES[a['type']][1] for _, a in outputs)
    copybacks = ([('ud', method['value_receiver'])] if method.get('value_receiver') else [])
    copybacks += [(f'arg{i}', a['type']) for i, a in inputs if a.get('inout')]
    for value, kind in copybacks:
        lines += value_binding.cpp_write(kind, value, offset)
        offset += TYPES[kind][1]
    lines.append('    return AP_MODULE_STATUS_OK;')
    if guards:
        lines += ['#else', '    return AP_MODULE_STATUS_UNAVAILABLE;', '#endif']
    return lines + ['}', '']


def enum_name(name):
    return ''.join(part.capitalize() if part.isupper() else part[0].upper() + part[1:]
                   for part in re.split(r'::|_', name) if part)


def rust_type(field):
    return 'crate::enums::' + enum_name(field['enum']) if field['type'] == 'enum' else TYPES[field['type']][0]


def mavlink_constants(path, name):
    enums = ET.parse(path).getroot().findall(f"./enums/enum[@name='{name}']")
    if len(enums) != 1:
        raise ValueError(f'missing or duplicate MAVLink enum {name}')
    constants = {}
    prefix = name + '_'
    for entry in enums[0].findall('entry'):
        full_name = entry.get('name', '')
        member = full_name[len(prefix):]
        if not full_name.startswith(prefix) or not re.fullmatch(r'[A-Z][A-Z0-9_]*', member):
            raise ValueError(f'invalid MAVLink enum member {entry.get("name")}')
        value = int(entry.attrib['value'], 0)
        if member in constants or not -(1 << 31) <= value < 1 << 31:
            raise ValueError(f'duplicate or out-of-range MAVLink enum member {member}')
        constants[member] = value
    if not constants:
        raise ValueError(f'empty MAVLink enum {name}')
    return constants


def rust_enums(methods, notice, mavlink_xml=MAVLINK_COMMON):
    names = sorted({a['enum'] for m in methods for a in [m['result']] + m['arguments'] if a['type'] == 'enum'})
    if len({enum_name(name) for name in names}) != len(names):
        raise ValueError('duplicate Rust enum name')
    lines = [notice, '// Distinct numeric enum types; firmware validates binding-specific ranges.',
             '// Named constants are generated from protocol definitions where available.', '']
    for name in names:
        lines += [f'/// `{name}` from bindings.desc. Unknown numeric values remain representable.',
                  '#[repr(transparent)]', '#[derive(Clone, Copy, Debug, Eq, PartialEq)]',
                  f'pub struct {enum_name(name)}(pub i32);', '']
        if name == 'MAV_SEVERITY':
            lines += ['// Values from MAVLink common.xml.', f'impl {enum_name(name)} {{']
            lines += [f'    pub const {member}: Self = Self({value});'
                      for member, value in mavlink_constants(mavlink_xml, name).items()]
            lines += ['}', '']
    return '\n'.join(lines)


def rust(method, context_override=None):
    inputs, outputs, optional = io(method)
    # Host arguments retain their IR positions; public arguments omit literals.
    inputs = list(enumerate(arg for _, arg in inputs))
    ni, no = sizes(method)
    fields = ([arg for _, arg in outputs] if outputs else
              ([] if method['result']['type'] == 'void' else [method['result']]))
    kinds = [rust_type(f) for f in fields]
    value_type = kinds[0] if len(kinds) == 1 else '(' + ', '.join(kinds) + ')'
    return_type = f'Option<{value_type}>' if optional else value_type
    value_receiver = method.get('value_receiver')
    receiver = method.get('receiver') or value_receiver
    args = (", context: &Context<'_>" if receiver else '') + ''.join(
        f", arg{i}: {'&mut ' if a.get('inout') else ''}{rust_type(a)}" for i, a in inputs)
    lines = [f'    /// `{method["owner"]}::{method["method"]}` from bindings.desc.']
    if method.get('getter_effects'):
        lines.extend('    /// ' + line for line in method['getter_effects'].splitlines())
    if method.get('effects'):
        lines.append('    /// ' + method['effects'])
    if any(a['type'] == 'cstring' for _, a in inputs):
        lines += ['    /// Strings are borrowed for this call, NUL-free and at most 255 bytes (empty allowed).',
                  '    /// Longer strings return Invalid, not truncation. The host must not retain the pointer.']
    if method.get('adapter'):
        lines += ['    /// Names are nonempty, NUL-free and at most 16 bytes.',
                  '    /// Float inputs are finite; integer setters check the destination range.',
                  '    /// Saved values may wait for IO; success does not imply durable storage.']
    if value_receiver or any(a.get('inout') for _, a in inputs):
        lines += ['    /// Copies all value state back on host success, including a false boolean result.',
                  '    /// Host-produced field values are not range-checked again; assignments are checked.']
    receiver_arg = '&mut self' if value_receiver else '&self'
    docs = lines
    lines = [f'        let {"mut " if inputs or receiver else ""}input = [0u8; {ni}];']
    if value_receiver:
        lines.append(f'        input[..{TYPES[value_receiver][1]}].copy_from_slice(&self.encode());')
    elif receiver:
        lines.append('        input[..4].copy_from_slice(&self.token.get().to_le_bytes());')
    offset = TYPES[value_receiver][1] if value_receiver else 4 * bool(receiver)
    for index, arg in inputs:
        kind = arg['type']
        size = TYPES[kind][1]
        if value_binding.is_value(kind):
            lines.append(f'        input[{offset}..{offset + size}].copy_from_slice(&arg{index}.encode());')
        elif kind in ('string', 'cstring'):
            empty = f'arg{index}.is_empty() || ' if kind == 'string' else ''
            lines += [f'        if {empty}arg{index}.len() > {size - 1} || arg{index}.as_bytes().contains(&0) {{',
                      '            return Err(Error::Invalid);', '        }',
                      f'        input[{offset}..{offset} + arg{index}.len()].copy_from_slice(arg{index}.as_bytes());']
        elif kind == 'boolean':
            lines.append(f'        input[{offset}] = u8::from(arg{index});')
        elif kind == 'enum':
            lines.append(f'        input[{offset}..{offset + size}].copy_from_slice(&arg{index}.0.to_le_bytes());')
        else:
            lines.append(f'        input[{offset}..{offset + size}].copy_from_slice(&arg{index}.to_le_bytes());')
        offset += size
    context = 'context' if receiver else context_override or 'self'
    lines += render_lines('rust/packet_buffer.rs', name='output', size=no)
    lines += render_lines(
        'rust/host_call.rs', context=context, slot=f'crate::imports::BINDING_{method["id"]}.slot()',
        input='input.as_ptr()', input_size='input.len() as u32',
        output='output.as_mut_ptr()', output_size='output.len() as u32',
    )
    offset = TYPES[method['result']['type']][1] + sum(TYPES[a['type']][1] for _, a in outputs)
    copybacks = ([('*self', value_receiver)] if value_receiver else [])
    copybacks += [(f'*arg{i}', a['type']) for i, a in inputs if a.get('inout')]
    for value, kind in copybacks:
        n = TYPES[kind][1]
        lines.append(f'        {value} = {TYPES[kind][0]}::decode(&output[{offset}..{offset+n}]);')
        offset += n
    offset = int(optional)
    if optional:
        lines += ['        match output[0] {', '            0 => return Ok(None),', '            1 => {},',
                  '            _ => return Err(Error::Host),', '        }']
    for index, arg in enumerate(fields):
        kind, size = TYPES[arg['type']]
        if arg['type'] in borrowed_binding.TYPES:
            name = borrowed_binding.TYPES[arg['type']]['rust']
            lines.append(f'        let value{index} = crate::{name}::from_wire('
                         f'u32::from_le_bytes(output[{offset}..{offset + size}].try_into().unwrap()));')
        elif arg['type'] == 'fwstring':
            lines.append(f'        let value{index} = FirmwareString::decode('
                         f'&output[{offset}..{offset + size}])?;')
        elif arg['type'] == 'enum':
            lines.append(f'        let value{index} = {rust_type(arg)}('
                         f'i32::from_le_bytes(output[{offset}..{offset + size}].try_into().unwrap()));')
        elif kind == 'bool':
            lines += [f'        let value{index} = match output[{offset}] {{', '            0 => false,',
                      '            1 => true,', '            _ => return Err(Error::Host),', '        };']
        elif value_binding.is_value(arg['type']):
            lines.append(f'        let value{index} = {kind}::decode(&output[{offset}..{offset + size}]);')
        else:
            lines.append(f'        let value{index} = {kind}::from_le_bytes('
                         f'output[{offset}..{offset + size}].try_into().unwrap());')
        offset += size
    values = ', '.join(f'value{i}' for i in range(len(fields)))
    value = values if len(fields) == 1 else '(' + values + ')'
    lines += [f'        Ok({"Some(" + value + ")" if optional else value})']
    lines = docs + render_lines(
        'rust/method.rs', name=method['rust_name'], receiver=receiver_arg, arguments=args, result=return_type,
        body=''.join(line + '\n' for line in lines),
    ) + ['']
    for alias in method['aliases']:
        passed = ', '.join((['context'] if receiver else []) + [f'arg{i}' for i, _ in inputs])
        lines += render_lines(
            'rust/alias.rs', name=alias, receiver=receiver_arg, arguments=args, result=return_type,
            target=method['rust_name'], passed=passed,
        ) + ['']
    return lines
