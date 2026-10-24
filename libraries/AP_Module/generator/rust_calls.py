# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Rust source views over typed call plans, independent of firmware subsystems."""

from collections import Counter

import call_model as model
import naming

from native_templates import render


def kind(type):
    if type is None:
        return '()'
    if isinstance(type, model.Scalar):
        return type.name
    if isinstance(type, model.Buffer):
        return '&mut [u8]' if type.direction == 'out' else '&[u8]'
    if isinstance(type, model.String):
        return '&str'
    if isinstance(type, (model.BoundedCount, model.BufferResult)):
        return 'usize' if isinstance(type, model.BoundedCount) or type.length else 'bool'
    if isinstance(type, (model.OptionalValue, model.SelectedHandle)):
        return 'Option<' + kind(type.type) + '>'
    return type.name


def encode(type, value, offset):
    if isinstance(type, model.Length):
        type = type.scalar
    if isinstance(type, model.OptionalValue):
        return render('rust/encode_optional.rs', value=value, offset=offset,
                      body=encode(type.type, 'value', offset + 1))
    if isinstance(type, (model.Buffer, model.String)):
        return render('rust/encode_buffer.rs', value=value + '.as_bytes()' if isinstance(type, model.String) else value,
                      offset=offset)
    if isinstance(type, model.Record):
        return ''.join(encode(f.type, value + '.' + naming.rust_identifier(f.name), offset + pos)
                       for f, pos in model.layout(type.fields))
    if isinstance(type, model.Enum):
        value = value + '.0' if type.external else '(' + value + ' as ' + kind(type.scalar) + ')'
    if type.size == 1:
        return render('rust/encode_byte.rs', offset=offset, value=value + ' as u8')
    return render('rust/encode_scalar.rs', offset=offset, end=offset + type.size, value=value)


def decode(type, offset=0):
    if type is None:
        return '()'
    if isinstance(type, model.Record):
        fields = ''.join('            ' + naming.rust_identifier(f.name) + ': ' + decode(f.type, offset + pos) + ',\n'
                         for f, pos in model.layout(type.fields))
        return render('rust/decode_record.rs', name=type.name, fields=fields).rstrip('\n')
    if isinstance(type, model.Enum):
        value = decode(type.scalar, offset)
        return type.name + '(' + value + ')' if type.external else type.name + '::try_from(' + value + ')?'
    if type == model.BOOL:
        return render('rust/decode_bool.rs', value=f'output[{offset}]').rstrip('\n')
    if type.size == 1:
        return f'output[{offset}]' + (' as i8' if type == model.I8 else '')
    return kind(type) + f'::from_le_bytes(output[{offset}..{offset + type.size}].try_into().unwrap())'


def wire_result(result):
    if isinstance(result, model.BoundedCount):
        return result.scalar
    if isinstance(result, model.SelectedHandle):
        return model.BOOL
    if isinstance(result, model.Enum):
        return result.scalar
    if isinstance(result, model.OptionalValue) and isinstance(result.type, model.Handle):
        return result.type.scalar
    return result


def helper_key(call):
    return tuple(input_type(f.type) for f in call.inputs), wire_result(call.result)


def input_type(type):
    if isinstance(type, model.Enum):
        return type.scalar
    return model.U8 if type == model.BOOL else type


def defaultable(type):
    if isinstance(type, model.Enum):
        return type.external or type.default is not None
    if isinstance(type, model.Record):
        return all(defaultable(f.type) for f in type.fields)
    return isinstance(type, model.Scalar)


def packet(call, context, slot, values, input_types=None):
    types = input_types or tuple(f.type for f in call.inputs)
    encoded = ''.join(encode(type, value, pos) for ((_, pos), type, value) in
                      zip(model.layout(call.inputs), types, values))
    buffers = render('rust/packet_buffer.rs', name='input', size=call.input_size) if call.input_size else ''
    buffers += render('rust/packet_buffer.rs', name='output', size=call.output_size) if call.output_size else ''
    output = wire_result(call.result)
    if isinstance(output, model.BufferResult):
        buffer = naming.rust_identifier(output.buffer)
        decoded = render(
            'rust/decode_count_buffer.rs' if output.length else 'rust/decode_present_buffer.rs',
            buffer=buffer, value=decode(output.length or model.BOOL),
            start=output.length.size if output.length else 1,
        ).rstrip('\n')
    elif isinstance(output, model.OptionalValue):
        decoded = render(
            'rust/decode_optional.rs', present=decode(model.BOOL), value=decode(output.type, 1),
            start=1, end=output.size,
        ).rstrip('\n')
    else:
        decoded = decode(wire_result(output))
    called = render(
        'rust/host_call.rs', context=context, slot=slot,
        input='input.as_ptr()' if call.input_size else 'core::ptr::null()', input_size=call.input_size,
        output='output.as_mut_ptr()' if call.output_size else 'core::ptr::null_mut()', output_size=call.output_size,
    )
    return render('rust/packet.rs', buffers=buffers, encode=encoded, call=called, result=decoded)


def method(call, helper=None):
    context = 'context' if call.receiver else 'self.context'
    slot = f'crate::imports::BINDING_{call.binding_id}.slot()'
    args = []
    arguments_values = {}
    if call.receiver:
        receiver = call.receiver
        if receiver.ownership == 'borrowed-index':
            arguments_values['receiver'] = 'self.index'
        else:
            arguments_values['receiver'] = 'self.token.get()'
        args.append("context: &Context<'_>")
    checks = ''
    for argument in call.arguments:
        name = naming.rust_identifier(argument.name)
        reference = '&' if isinstance(argument.type, model.Record) else ''
        args.append(name + ': ' + reference + kind(argument.type))
        arguments_values[argument.name] = name
        if isinstance(argument.type, model.Buffer):
            checks += render('rust/check_buffer.rs', name=name,
                             capacity=argument.type.capacity,
                             lower=f'{name}.len() < {argument.type.minimum} || ' if argument.type.minimum else '')
            if argument.type.empty_returns_zero:
                checks += f'        if {name}.is_empty() {{ return Ok(0); }}\n'
        elif isinstance(argument.type, model.String):
            checks += render('rust/check_string.rs', name=name, capacity=argument.type.capacity,
                             nonempty=f'{name}.is_empty() || ' if argument.type.nonempty else '')
        elif isinstance(argument.type, model.OptionalValue) and argument.type.nonzero:
            checks += f'        if {name} == Some(0) {{ return Err(Error::Invalid); }}\n'
        if argument.minimum is not None:
            checks += render('rust/range_check.rs', name=name, minimum=argument.minimum, maximum=argument.maximum)
    values = [f'({naming.rust_identifier(f.type.buffer)}.len() as {kind(f.type.scalar)})'
              if isinstance(f.type, model.Length) else arguments_values[f.name] for f in call.inputs]
    if helper:
        values = [(value + '.0' if f.type.external else f'{value} as {kind(f.type.scalar)}')
                  if isinstance(f.type, model.Enum) else f'{value} as u8' if f.type == model.BOOL else value
                  for f, value in zip(call.inputs, values)]
        passed = ', '.join([context, slot] + values)
        body = '        let value = ' + helper + '(' + passed + ')?;\n'
    else:
        body = packet(call, context, slot, values)
    if isinstance(call.result, model.SelectedHandle):
        result = call.result
        value = result.type.name + ' { index: ' + naming.rust_identifier(result.argument) + ' }'
        # then_some constructs only a small selector value, never an owned resource.
        returned = 'value.then_some(' + value + ')'
    elif isinstance(call.result, model.OptionalValue) and isinstance(call.result.type, model.Handle):
        handle = call.result.type
        returned = ('core::num::NonZeroU32::new(value).map(|token| ' + handle.name + ' { token })')
    elif isinstance(call.result, model.Enum):
        returned = call.result.name + '(value)' if call.result.external else call.result.name + '::try_from(value)?'
    elif isinstance(call.result, model.BoundedCount):
        limit = naming.rust_identifier(call.result.buffer)
        body += f'        if value as usize > {limit}.len() {{ return Err(Error::Host); }}\n'
        returned = 'value as usize'
    else:
        returned = 'value'
    body += '        Ok(' + returned + ')\n'
    name = naming.rust_identifier(call.method_name or call.name.partition('.')[2])
    receiver = 'self' if call.consume else '&self'
    arguments = ''.join(', ' + a for a in args)
    return ''.join(('    /// ' + d).rstrip() + '\n' for d in call.docs) + render(
        'rust/method.rs',
        name=name, receiver=receiver, arguments=arguments, result=kind(call.result), body=checks + body,
    ) + '\n'


def emit(module, notice):
    declarations = []
    imports = {'result', 'Context', 'Error', 'Result'}
    for type in module.types:
        if getattr(type, 'external', False):
            path = 'crate::enums' if isinstance(type, model.Enum) else 'crate'
            declarations.append('use ' + path + '::' + type.name + ';\n')
        elif isinstance(type, model.Handle):
            borrowed = type.ownership.startswith('borrowed-')
            # Owned handles cannot be copied; consuming close leaves no live
            # closed handle, so no optional token or runtime closed-state check.
            scalar = 'core::num::NonZeroU32'
            declarations.append(render(
                'rust/handle.rs', derives='Clone, Copy, Debug, Eq, PartialEq' if borrowed else 'Debug',
                docs=''.join('/// ' + line + '\n' for line in type.docs),
                name=type.name, field='index' if type.ownership == 'borrowed-index' else 'token',
                type='u8' if type.ownership == 'borrowed-index' else scalar,
            ) + '\n')
        elif isinstance(type, model.Enum):
            variants = ''.join(('    #[default]\n' if name == type.default else '') +
                               '    ' + name + ' = ' + str(value) + ',\n' for name, value in type.values)
            cases = ''.join('            ' + str(value) + ' => Ok(Self::' + name + '),\n' for name, value in type.values)
            declarations.append(render(
                'rust/enum.rs', name=type.name, scalar=kind(type.scalar), variants=variants, cases=cases,
                default=', Default' if type.default else '',
            ) + '\n')
        elif isinstance(type, model.Record):
            declarations.append(render(
                'rust/record.rs', name=type.name,
                default=', Default' if defaultable(type) else '',
                fields=''.join('    pub ' + naming.rust_identifier(f.name) + ': ' + kind(f.type) + ',\n' for f in type.fields),
            ) + '\n')
        else:
            raise ValueError('unsupported Rust model declaration')
    groups = {}
    helpers = {}
    helper_names = set()
    counts = Counter(helper_key(c) for c in module.calls)
    for call in module.calls:
        key = helper_key(call)
        helper = None
        if (counts[key] > 1 and not isinstance(call.result, (model.BufferResult, model.BoundedCount)) and
                all(isinstance(f.type, (model.Scalar, model.Enum, model.Record)) for f in call.inputs)):
            if key not in helpers:
                inputs = '_'.join(naming.snake_case(kind(t)) for t in key[0]) or 'empty'
                output = naming.snake_case(kind(key[1])) if key[1] else 'unit'
                base_name = 'call_' + inputs + '_' + output
                name = base_name
                suffix = 2
                while name in helper_names:
                    name = base_name + '_' + str(suffix)
                    suffix += 1
                helper_names.add(name)
                fields = ', '.join('arg' + str(i) + ': ' + ('&' if isinstance(type, model.Record) else '') + kind(type)
                                   for i, type in enumerate(key[0]))
                body = packet(call, 'context', 'slot', ['arg' + str(i) for i in range(len(call.inputs))], key[0])
                body += '        Ok(value)\n'
                body = ''.join(line[4:] if line.startswith('    ') else line for line in body.splitlines(keepends=True))
                helpers[key] = (name, render(
                    'rust/call_helper.rs', name=name, arguments=', ' + fields if fields else '',
                    result=kind(key[1]), body=body,
                ) + '\n')
            helper = helpers[key][0]
        namespace = call.namespace or call.name.partition('.')[0]
        owner = call.receiver.name if call.receiver else 'crate::bindings::' + naming.rust_type(namespace) + "<'_, '_>"
        groups.setdefault(owner, []).append(method(call, helper))
    for projection in module.projections:
        groups.setdefault(projection.owner, []).append(render(
            'rust/projection.rs', name=projection.name, result=kind(projection.type), method=projection.method,
            field=projection.field,
        ) + '\n')
    for owner, name, target, direction in module.byte_methods:
        groups.setdefault(owner, []).append(render(
            'rust/read_byte.rs' if direction == 'out' else 'rust/write_byte.rs', name=name, target=target,
        ) + '\n')
    declarations += [f'pub const {name}: usize = {value};\n' for name, value in module.constants]
    return render(
        'rust/client_module.rs', notice=notice, imports=', '.join(sorted(imports)),
        declarations=''.join(declarations), helpers=''.join(value[1] for value in helpers.values()),
        implementations=''.join(render('rust/impl.rs', type=owner, methods=''.join(methods)) + '\n'
                                for owner, methods in groups.items()),
    )
