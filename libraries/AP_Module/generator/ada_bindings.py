# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Ada clients over the audited typed C++ plans, without another wire-policy parser.

Only C-compatible values cross the local Ada/C++ boundary. The C++ clients
retain encoding, validation and ownership policies; firmware never sees these
compiler layouts. Rust output and the module ABI are independent of this layer.
"""

import re

from dataclasses import dataclass

import c_abi
import cpp_bindings
import naming

from native_templates import render

KEYWORDS = set('abort abs abstract accept access aliased all and array at begin body case constant declare delay delta '
               'digits do else elsif end entry exception exit for function generic goto if in interface is limited loop '
               'mod new not null of or others out overriding package parallel pragma private procedure protected raise '
               'range record rem renames requeue return reverse select separate some subtype synchronized tagged task '
               'terminate then type until use when while with xor'.split())
SCALARS = {
    'bool': ('Interfaces.C.C_bool', 'uint8_t', 1, "Interfaces.C.C_bool'Val (0)"),
    'uint8_t': ('Interfaces.Unsigned_8', 'uint8_t', 1, '0'),
    'int8_t': ('Interfaces.Integer_8', 'int8_t', 1, '0'),
    'uint16_t': ('Interfaces.Unsigned_16', 'uint16_t', 2, '0'),
    'int16_t': ('Interfaces.Integer_16', 'int16_t', 2, '0'),
    'uint32_t': ('Interfaces.Unsigned_32', 'uint32_t', 4, '0'),
    'int32_t': ('Interfaces.Integer_32', 'int32_t', 4, '0'),
    'uint64_t': ('Interfaces.Unsigned_64', 'uint64_t', 8, '0'),
    'float': ('Interfaces.C.C_float', 'float', 4, '0.0'),
}


def identifier(name):
    name = re.sub('[^A-Za-z0-9]+', '_', name).strip('_')
    if not name:
        raise ValueError('empty Ada identifier')
    if name[0].isdigit():
        name = 'N_' + name
    return name + '_Call' if name.lower() in KEYWORDS else name


@dataclass
class Type:
    cpp: str
    name: str
    abi: str
    size: int
    alignment: int
    default: str
    kind: str
    fields: tuple = ()
    element: object = None
    count: int = 0

    @property
    def ada(self):
        return SCALARS[self.cpp][0] if self.kind == 'scalar' else 'AP_Module.Types.' + self.name


class Types:
    def __init__(self, cpp):
        self.cpp = cpp
        self.types = {}
        self.names = {}

    def get(self, kind):
        if kind in self.types:
            return self.types[kind]
        if kind in SCALARS:
            _, abi, size, default = SCALARS[kind]
            item = Type(kind, '', abi, size, size, default, 'scalar')
        elif 'enum:' + kind in self.cpp.records:
            scalar = self.get(self.cpp.records['enum:' + kind])
            item = Type(kind, identifier(kind), scalar.abi, scalar.size, scalar.alignment, '0', 'enum', element=scalar)
        elif kind.startswith('Array<'):
            value, count = kind[6:-1].rsplit(', ', 1)
            element = self.get(value)
            name = 'Array_' + (element.name or identifier(value)) + '_' + count
            item = Type(kind, name, 'Ada_' + name, element.size * int(count), element.alignment,
                        '(others => ' + element.default + ')', 'array', element=element, count=int(count))
        else:
            optional = kind.startswith('Optional<')
            if optional:
                inner = self.get(kind[9:-1])
                name = 'Optional_' + (inner.name or identifier(inner.cpp))
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
            item = Type(kind, name, 'Ada_' + name, size, alignment, '(others => <>)',
                        'optional' if optional else 'record', tuple(layout))
        if item.name:
            previous = self.names.setdefault(item.name.lower(), kind)
            if previous != kind:
                raise ValueError('Ada type name collision: ' + item.name)
        self.types[kind] = item
        return item

    def decode(self, item, source, target):
        return c_abi.decode(item, source, target)

    def encode(self, item, source, target):
        return c_abi.encode(item, source, target)

    def emit(self):
        declarations = []
        for item in self.types.values():
            if item.kind == 'scalar':
                continue
            if item.kind == 'enum':
                constants = ''.join(f'   {identifier(item.name + "_" + n)} : constant {item.name} := {v};\n'
                                    for n, v in self.cpp.enum_values[item.cpp].items())
                declarations.append(render('ada/enum.ads', name=item.name, scalar=item.element.ada, constants=constants))
                continue
            if item.kind == 'array':
                declarations.append(render('ada/array.ads', name=item.name, element=item.element.ada, count=item.count,
                                           size=item.size * 8, alignment=item.alignment))
            else:
                fields = ''.join(f'      {identifier(field)} : {value.ada} := {value.default};\n'
                                 for field, value, _ in item.fields)
                layout = ''.join(f'      {identifier(field)} at {position} range 0 .. {value.size * 8 - 1};\n'
                                 for field, value, position in item.fields)
                declarations.append(render('ada/record.ads', name=item.name, fields=fields, layout=layout,
                                           size=item.size * 8, alignment=item.alignment))
        abi, conversions = c_abi.emit_types(self)
        return ''.join(declarations), abi, conversions


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
    groups, bodies, bridges = {}, {}, []
    names = set()
    for name, inputs, results, copybacks, _, _ in plans:
        group, fn = name.split('.', 1)
        cpp_group = 'parameter_cache' if group == 'Parameter' else group.lower()
        group, fn = identifier(cpp_group), identifier(fn)
        if (group.lower(), fn.lower()) in names:
            raise ValueError('Ada binding name collision: ' + name)
        names.add((group.lower(), fn.lower()))
        symbol = 'ap_module_ada_binding_' + naming.binding_name(name)
        public, raw, arguments = ['Ctx : AP_Module.Context'], ['Ctx : AP_Module.Context'], ['Ctx']
        for arg, kind, policy in inputs:
            field = 'A_' + identifier(arg)
            modified = any(n == arg for n, _ in copybacks)
            out_only = policy and policy.get('out_only')
            mode = 'out ' if out_only else 'in out ' if modified else ''
            if kind == 'StringView':
                public.append(f'{field} : Standard.String')
                raw += [f'{field} : System.Address', f'{field}_Length : Interfaces.Unsigned_32']
                arguments += [field + "'Address", field + "'Length"]
                continue
            item = types.get(kind)
            public.append(f'{field} : {mode}{item.ada}')
            raw.append(f'{field} : System.Address')
            arguments.append(field + "'Address")
        for arg, kind in results:
            item = types.get(kind)
            field = 'Result_' + identifier(arg)
            public.append(f'{field} : out {item.ada}')
            raw.append(field + ' : System.Address')
            arguments.append(field + "'Address")
        bridges.append(c_abi.method(types, name, inputs, results, copybacks, symbol))
        signature = ';\n         '.join(public)
        groups.setdefault(group, []).append(render('ada/method.ads', name=fn, arguments=signature))
        body = render('ada/method.adb', name=fn, arguments=signature, raw_arguments='; '.join(raw),
                      symbol=symbol, values=', '.join(arguments))
        bodies.setdefault(group, []).append(body)
    declarations, abi, conversions = types.emit()
    return {
        'sdk/ada/ap_module-types.ads': render('ada/types.ads', declarations=declarations),
        'sdk/ada/ap_module-statuses.ads': render('ada/statuses.ads', constants=''.join(
            f'   {identifier(n)} : constant AP_Module.Status := {v};\n' for n, v in host['status'].items())),
        'sdk/ada/ap_module-bindings.ads': render('ada/bindings.ads', namespaces=''.join(
            render('ada/namespace.ads', name=n, methods=''.join(m)) for n, m in groups.items())),
        'sdk/ada/ap_module-bindings.adb': render('ada/bindings.adb', namespaces=''.join(
            render('ada/namespace.adb', name=n, methods=''.join(m)) for n, m in bodies.items())),
        'sdk/ada/bridge.cpp': render('c_abi/bridge.cpp', abi=abi, conversions=conversions, methods=''.join(bridges)),
    }
