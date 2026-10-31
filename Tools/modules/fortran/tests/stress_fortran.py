# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Retain and invoke every generated Fortran API with typed nonzero inputs."""

import sys

import build

sys.path.insert(0, str(build.SDK_ROOT / 'generator'))
import bindings  # noqa: E402
import fortran_bindings  # noqa: E402


def leaves(item, path):
    if item.kind in ('scalar', 'enum'):
        return [(item, path)]
    if item.kind == 'array':
        return [leaf for i in range(item.count) for leaf in leaves(item.element, f'{path}%data({i})')]
    return [leaf for field, value, _ in item.fields for leaf in leaves(value, path + '%' + field.lower())]


def literal(item, value):
    scalar = item.element if item.kind == 'enum' else item
    if scalar.cpp == 'bool':
        return '.true._c_bool' if value else '.false._c_bool'
    if scalar.cpp == 'float':
        return str(value) + '.0_c_float'
    return str(value) + '_' + scalar.f90.split('(')[1].split(')')[0]


def source(directory):
    types, entries, plans = fortran_bindings.describe(bindings.specification())
    lines = ['function run(ctx) bind(C, name="ap_module_fortran_run") result(status)',
             '    use ap_module_bindings', '    implicit none', '    type(c_ptr), value :: ctx',
             '    integer(c_int32_t) :: status, wanted', '    wanted = 1000']
    for name, inputs, results, copybacks, _, _ in plans:
        lines += ['    block']
        args, initialize, checks = ['ctx'], [], []
        for index, (arg, kind, policy) in enumerate(inputs):
            if kind == 'StringView':
                args.append('"x"')
                continue
            item = types.get(kind)
            field = 'a' + str(index)
            lines.append(f'        {item.f90} :: {field}')
            initialize += [f'        {path} = {literal(leaf, 1)}' for leaf, path in leaves(item, field)]
            if policy and policy.get('out_only'):
                checks += [(leaf, path, False) for leaf, path in leaves(item, field)]
            elif any(n == arg for n, _ in copybacks):
                checks += [(leaf, path, True) for leaf, path in leaves(item, field)]
            args.append(field)
        for index, (_, kind) in enumerate(results):
            item = types.get(kind)
            field = 'r' + str(index)
            lines.append(f'        {item.f90} :: {field}')
            initialize += [f'        {path} = {literal(leaf, 1)}' for leaf, path in leaves(item, field)]
            checks += [(leaf, path, False) for leaf, path in leaves(item, field)]
            args.append(field)
        lines += initialize
        lines += ['        status = ' + fortran_bindings.method_name(name) + '( &',
                  '            ' + ', &\n            '.join(args) + ')',
                  '        if (wanted == 1000) wanted = status', '        if (status /= wanted) return']
        for item, path, modified in checks:
            expected = ('merge(' + literal(item, 1) + ', ' + literal(item, 0) + ', status /= module_ok)' if modified
                        else literal(item, 0))
            operator = '.neqv.' if item.cpp == 'bool' else '/='
            lines += [f'        if ({path} {operator} {expected}) then',
                      '            status = module_host_error', '            return', '        end if']
        lines += ['    end block']
    lines += ['    status = module_ok', 'end function']
    path = directory / 'all_calls.f90'
    path.write_text('\n'.join(lines) + '\n')
    return path, {entry['contract']['name'] for entry in entries}
