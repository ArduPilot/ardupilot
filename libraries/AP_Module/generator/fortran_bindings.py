# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Runtime-free ISO_C_BINDING clients over the shared typed C++ method plans.

Unsigned values use same-width signed C integers at this local boundary. The
bridge converts the bit patterns to the unsigned wire type; Fortran callers use
IBITS/IAND/IOR/TRANSFER, not signed arithmetic, for the upper half of the range.
There are no descriptors, allocatable results, or libgfortran operations here.
"""

import hashlib
import re

from dataclasses import dataclass

import c_abi
import cpp_bindings
import naming

from native_templates import render

SCALARS = {
    'bool': ('logical(c_bool)', 'bool', 1),
    'uint8_t': ('integer(c_int8_t)', 'int8_t', 1),
    'int8_t': ('integer(c_int8_t)', 'int8_t', 1),
    'uint16_t': ('integer(c_int16_t)', 'int16_t', 2),
    'int16_t': ('integer(c_int16_t)', 'int16_t', 2),
    'uint32_t': ('integer(c_int32_t)', 'int32_t', 4),
    'int32_t': ('integer(c_int32_t)', 'int32_t', 4),
    'uint64_t': ('integer(c_int64_t)', 'int64_t', 8),
    'float': ('real(c_float)', 'float', 4),
}


def identifier(name):
    value = re.sub('[^a-z0-9]+', '_', name.lower()).strip('_')
    if not value or not value[0].isalpha():
        raise ValueError('invalid Fortran identifier: ' + name)
    if len(value) > 63:
        value = value[:54] + '_' + hashlib.sha256(value.encode()).hexdigest()[:8]
    return value


def method_name(name):
    group, method = name.split('.', 1)
    group = 'parameter_cache' if group == 'Parameter' else group
    return identifier(group + '_' + method)


@dataclass
class Type:
    cpp: str
    name: str
    abi: str
    size: int
    alignment: int
    kind: str
    fields: tuple = ()
    element: object = None
    count: int = 0

    @property
    def f90(self):
        return SCALARS[self.cpp][0] if self.kind == 'scalar' else self.element.f90 if self.kind == 'enum' else \
            'type(' + self.name + ')'


class Types:
    string_length = 'int32_t'

    def __init__(self, cpp):
        self.cpp = cpp
        self.types = {}
        self.names = {}

    def get(self, kind):
        if kind in self.types:
            return self.types[kind]
        if kind in SCALARS:
            _, abi, size = SCALARS[kind]
            item = Type(kind, '', abi, size, size, 'scalar')
        elif 'enum:' + kind in self.cpp.records:
            scalar = self.get(self.cpp.records['enum:' + kind])
            item = Type(kind, identifier(kind), scalar.abi, scalar.size, scalar.alignment, 'enum', element=scalar)
        elif kind.startswith('Array<'):
            value, count = kind[6:-1].rsplit(', ', 1)
            element = self.get(value)
            name = identifier('array_' + (element.name or value) + '_' + count)
            item = Type(kind, name, 'F90_' + name, element.size * int(count), element.alignment,
                        'array', element=element, count=int(count))
        else:
            optional = kind.startswith('Optional<')
            if optional:
                inner = self.get(kind[9:-1])
                name = identifier('optional_' + (inner.name or inner.cpp))
                fields = [('present', self.get('bool')), ('value', inner)]
            else:
                name = identifier(kind)
                fields = [(n, self.get(t)) for n, t in self.cpp.records[kind]]
            position, alignment, layout = 0, 1, []
            for field, value in fields:
                position = (position + value.alignment - 1) // value.alignment * value.alignment
                layout.append((field, value, position))
                position += value.size
                alignment = max(alignment, value.alignment)
            size = (position + alignment - 1) // alignment * alignment
            item = Type(kind, name, 'F90_' + name, size, alignment,
                        'optional' if optional else 'record', tuple(layout))
        if item.name:
            previous = self.names.setdefault(item.name, kind)
            if previous != kind:
                raise ValueError('Fortran type name collision: ' + item.name)
        self.types[kind] = item
        return item

    def declarations(self):
        declarations = []
        for item in self.types.values():
            if item.kind == 'scalar':
                continue
            if item.kind == 'enum':
                # BOZ conversion preserves flag bits even beyond signed HUGE().
                kind = re.search(r'\(([^)]+)\)', item.f90)[1]
                for name, value in self.cpp.enum_values[item.cpp].items():
                    name = identifier(item.name + '_' + name)
                    bits = int(value) % (1 << (item.size * 8))
                    declarations.append(render('fortran/constant.f90', declaration=item.f90, name=name,
                                               value=f"int(z'{bits:X}', {kind})"))
            else:
                fields = (f'        {item.element.f90} :: data(0:{item.count - 1})\n' if item.kind == 'array' else
                          ''.join(f'        {value.f90} :: {identifier(field)}\n' for field, value, _ in item.fields))
                declarations.append(render('fortran/type.f90', name=item.name, fields=fields))
        return ''.join(declarations)


def describe(spec):
    cpp, entries, plans = cpp_bindings.describe(spec)
    types = Types(cpp)
    for _, inputs, results, copybacks, _, _ in plans:
        for _, kind in results + copybacks:
            types.get(kind)
        for _, kind, _ in inputs:
            if kind != 'StringView':
                types.get(kind)
    return types, entries, plans


def emit(spec, host):
    types, _, plans = describe(spec)
    methods, bridges, names = [], [], set()
    for name, inputs, results, copybacks, _, _ in plans:
        fn = method_name(name)
        if fn in names or fn in types.names:
            raise ValueError('Fortran binding name collision: ' + fn)
        names.add(fn)
        symbol = 'ap_module_f90_binding_' + naming.binding_name(name)
        arguments, declarations, raw_declarations, values = ['ctx'], [], [], ['ctx']
        for arg, kind, policy in inputs:
            field = 'a_' + identifier(arg)
            arguments.append(field)
            if kind == 'StringView':
                declarations.append(f'        character(kind=c_char, len=*), intent(in) :: {field}\n')
                raw_declarations += [f'                character(kind=c_char), intent(in) :: {field}(*)\n',
                                     f'                integer(c_int32_t), value :: {field}_length\n']
                # Saturate before narrowing: an oversized Fortran string must
                # be rejected by the C++ client, never wrap to a short length.
                values += [field, f'int(min(len({field}, kind=c_size_t), '
                                  'int(huge(0_c_int32_t), c_size_t)), c_int32_t)']
            else:
                mode = 'out' if policy and policy.get('out_only') else \
                    'inout' if any(n == arg for n, _ in copybacks) else 'in'
                declaration = f'{types.get(kind).f90}, intent({mode}) :: {field}\n'
                declarations.append('        ' + declaration)
                raw_declarations.append('                ' + declaration)
                values.append(field)
        for arg, kind in results:
            field = 'out_' + identifier(arg)
            arguments.append(field)
            declaration = f'{types.get(kind).f90}, intent(out) :: {field}\n'
            declarations.append('        ' + declaration)
            raw_declarations.append('                ' + declaration)
            values.append(field)
        # Use one argument per continuation line to stay within free-form 132 columns.
        signature = ', &\n            '.join(arguments)
        raw_arguments = []
        for arg, kind, _ in inputs:
            field = 'a_' + identifier(arg)
            raw_arguments += [field, field + '_length'] if kind == 'StringView' else [field]
        raw_arguments = ['ctx', *raw_arguments, *('out_' + identifier(arg) for arg, _ in results)]
        methods.append(render('fortran/method.f90', name=fn, arguments=signature,
                              declarations=''.join(declarations),
                              raw_arguments=', &\n                    '.join(raw_arguments),
                              raw_declarations=''.join(raw_declarations), symbol=symbol,
                              values=', &\n            '.join(values)))
        bridges.append(c_abi.method(types, name, inputs, results, copybacks, symbol))
    constants = ''.join(render('fortran/constant.f90', declaration='integer(c_int32_t)',
                               name='module_' + identifier(name), value=str(value))
                        for name, value in host['status'].items())
    source = render('fortran/bindings.f90', declarations=types.declarations(),
                    constants=constants, methods=''.join(methods))
    return {
        'sdk/fortran/ap_module_bindings.f90': source,
        'sdk/fortran/bridge.cpp': c_abi.bridge(types, bridges),
    }
