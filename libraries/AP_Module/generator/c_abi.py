# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Local C value bridges shared by ISO C language clients, not a firmware ABI.

Typed C++ clients still own wire encoding, validation and handle policies. Each
frontend supplies C-compatible mirror types; conversion never aliases C++ object
layouts. Stage inputs before writing outputs, and publish in/out values only on
success. This also gives every out parameter a defined value on failure.
"""

import cpp_bindings

from native_templates import render


def decode(item, source, target):
    if item.cpp == 'bool' and item.abi != 'bool':
        return f'if ({source} > 1) {{ return false; }} {target} = {source} != 0;'
    if item.kind in ('scalar', 'enum'):
        return f'{target} = static_cast<ap_module::{item.cpp}>({source});' if item.kind == 'enum' else \
            f'{target} = static_cast<{item.cpp}>({source});'
    return f'if (!decode_{item.name}({source}, {target})) {{ return false; }}'


def encode(item, source, target):
    if item.kind in ('scalar', 'enum'):
        return f'{target} = static_cast<{item.abi}>({source});'
    return f'encode_{item.name}({source}, {target});'


def emit_types(types):
    abi, conversions = [], []
    for item in types.types.values():
        if item.kind in ('scalar', 'enum'):
            continue
        if item.kind == 'array':
            abi.append(f'struct {item.abi} {{ {item.element.abi} data[{item.count}]; }};\n')
            decoded = '    ' + decode(item.element, 'source.data[i]', 'target.data[i]') + '\n'
            encoded = '    ' + encode(item.element, 'source.data[i]', 'target.data[i]') + '\n'
            conversions.append(render('c_abi/array_bridge.cpp', abi=item.abi, cpp=item.cpp, name=item.name,
                                      count=item.count, decode=decoded, encode=encoded))
        else:
            fields = ''.join(f'    {value.abi} {field};\n' for field, value, _ in item.fields)
            abi.append(f'struct {item.abi} {{\n' + fields + '};\n')
            abi += [f'static_assert(offsetof({item.abi}, {field}) == {position}, "C field layout");\n'
                    for field, _, position in item.fields]
            decoded = ''.join('    ' + decode(value, 'source.' + field, 'target.' + cpp_bindings.identifier(field)) + '\n'
                              for field, value, _ in item.fields)
            encoded = ''.join('    ' + encode(value, 'source.' + cpp_bindings.identifier(field), 'target.' + field) + '\n'
                              for field, value, _ in item.fields)
            if item.kind == 'optional':
                inner = item.fields[1][1]
                validate = '    if (source.present > 1) { return false; }\n' if item.fields[0][1].abi != 'bool' else ''
                decoded = (validate + '    target.present = source.present != 0;\n'
                           '    if (target.present) { ' + decode(inner, 'source.value', 'target.value') + ' }\n')
                encoded = ('    target = {};\n    target.present = source.present;\n'
                           '    if (source.present) { ' + encode(inner, 'source.value', 'target.value') + ' }\n')
            conversions.append(render('c_abi/record_bridge.cpp', abi=item.abi, cpp=item.cpp, name=item.name,
                                      decode=decoded, encode=encoded))
        abi.append(f'static_assert(sizeof({item.abi}) == {item.size}, "C value size");\n')
        abi.append(f'static_assert(alignof({item.abi}) == {item.alignment}, "C value alignment");\n')
    return ''.join(abi), ''.join(conversions)


def method(types, name, inputs, results, copybacks, symbol):
    group, fn = name.split('.', 1)
    group = 'parameter_cache' if group == 'Parameter' else group.lower()
    arguments, locals_, calls, commits, invalid, failed = ['const ap_module::ContextBase *base'], [], [], [], [], []
    for index, (arg, kind, policy) in enumerate(inputs):
        if kind == 'StringView':
            length_type = getattr(types, 'string_length', 'uint32_t')
            arguments += [f'const char *a{index}', f'{length_type} a{index}_length']
            if length_type == 'int32_t':
                locals_.append(f'    if (a{index}_length < 0) {{ INVALID_INPUT }}')
            calls.append(f'ap_module::StringView(a{index}, a{index}_length)')
            continue
        modified = any(n == arg for n, _ in copybacks)
        item = types.get(kind)
        arguments.append(f'{"" if modified else "const "}{item.abi} *a{index}')
        local = f'value{index}'
        locals_.append(f'    ap_module::{kind} {local}{{}};' if item.kind != 'scalar' else f'    {kind} {local}{{}};')
        if policy and policy.get('out_only'):
            invalid.append(f'*a{index} = {{}};')
            failed.append(f'    if (!result) {{ *a{index} = {{}}; }}\n')
        else:
            locals_.append('    if (![&]() -> bool { ' + decode(item, f'*a{index}', local) +
                           ' return true; }()) { INVALID_INPUT }')
        calls.append(local)
        if modified:
            commits.append('        ' + encode(item, local, f'*a{index}'))
    for arg, kind in results:
        item = types.get(kind)
        arguments.append(item.abi + ' *result_' + arg)
        invalid.append(f'*result_{arg} = {{}};')
    locals_ = [line.replace('INVALID_INPUT', ' '.join(invalid) + ' return AP_MODULE_STATUS_INVALID;') for line in locals_]
    outputs = ''.join('    ' + encode(types.get(kind), 'result.value', 'result_' + arg + '[0]') + '\n'
                      for arg, kind in results) + ''.join(failed)
    return render('c_abi/bridge_method.cpp', symbol=symbol, arguments=', '.join(arguments),
                  locals='\n'.join(locals_) + '\n' if locals_ else '', group=cpp_bindings.identifier(group),
                  name=cpp_bindings.identifier(fn), values=', '.join(calls), outputs=outputs,
                  copybacks='\n'.join(commits) + '\n' if commits else '')


def bridge(types, methods):
    abi, conversions = emit_types(types)
    return render('c_abi/bridge.cpp', abi=abi, conversions=conversions, methods=''.join(methods))
