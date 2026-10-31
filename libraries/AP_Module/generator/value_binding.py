# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Complete, layout-independent copies of audited plain-value userdata."""

import math_binding

SCALARS = {'boolean': ('bool', 1), 'uint8_t': ('u8', 1), 'uint16_t': ('u16', 2),
           'int32_t': ('i32', 4), 'float': ('f32', 4)}
# Wire order is explicit, never the C++ layout or IR declaration order. Hidden
# fields are carried too: get/modify/set must not discard firmware-owned state.
TYPES = {
    'Location': dict(rust='Location', depends=None, private=True, size=16,
                     fields=[('relative_alt', 'boolean', None), ('loiter_ccw', 'boolean', None),
                             ('terrain_alt', 'boolean', None), ('origin_alt', 'boolean', None),
                             ('loiter_xtrack', 'boolean', None), ('alt', 'int32_t', 4),
                             ('lat', 'int32_t', 8), ('lng', 'int32_t', 12)], hidden=['loiter_ccw']),
    'Vector2f': dict(rust='Vector2f', depends=None, private=False, size=8,
                     fields=[('x', 'float', 0), ('y', 'float', 4)], hidden=[]),
    'mavlink_mission_item_int_t': dict(
        rust='MissionItem', depends='AP_MISSION_ENABLED', private=False, size=40,
        fields=[('param1', 'float', 0), ('param2', 'float', 4), ('param3', 'float', 8), ('param4', 'float', 12),
                ('x', 'int32_t', 16), ('y', 'int32_t', 20), ('z', 'float', 24),
                ('seq', 'uint16_t', 28), ('command', 'uint16_t', 30),
                ('target_system', 'uint8_t', 32), ('target_component', 'uint8_t', 33),
                ('frame', 'uint8_t', 34), ('current', 'uint8_t', 35),
                ('autocontinue', 'uint8_t', 36), ('mission_type', 'uint8_t', 37)],
        hidden=['target_system', 'target_component', 'autocontinue', 'mission_type']),
}
COMMON_WRITERS = {'Vector2f', 'Vector3f', 'Quaternion'}


def is_value(kind):
    return kind in TYPES or kind in math_binding.TYPES


def fields(kind):
    return TYPES[kind]['fields'] if kind in TYPES else [(f, 'float', i * 4)
                                                        for i, f in enumerate(math_binding.TYPES[kind]['fields'])]


def size(kind):
    return sum(SCALARS[t][1] for _, t, _ in fields(kind))


def wire_type(kind):
    return dict(name=kind, fields=[[name, {'boolean': 'bool-u8', 'uint8_t': 'u8'}.get(t, SCALARS[t][0] + '-le')]
                                   for name, t, _ in fields(kind)])


def cpp_helpers(names):
    lines = []
    for name in sorted(names):
        p = TYPES.get(name, math_binding.TYPES.get(name))
        if p['depends']:
            lines.append('#if ' + p['depends'])
        if name in TYPES:
            lines.append(f'static_assert(sizeof({name}) == {p["size"]}, "audit {name} value fields");')
            for field, _, offset in p['fields']:
                if offset is not None:
                    lines.append(f'static_assert(offsetof({name}, {field}) == {offset}, "audit {name}.{field}");')
        elif name in COMMON_WRITERS:
            lines.append(f'static_assert(sizeof({name}) == {size(name)}, "audit {name} value fields");')
            for field, _, offset in fields(name):
                lines.append(f'static_assert(offsetof({name}, {field}) == {offset}, "audit {name}.{field}");')
        lines += [f'static inline __attribute__((unused)) bool read_value_{name}(const uint8_t *input, {name} &value)', '{']
        offset = 0
        for field, kind, _ in fields(name):
            n = SCALARS[kind][1]
            if kind == 'boolean':
                lines += [f'    if (input[{offset}] > 1) {{', '        return false;', '    }',
                          f'    value.{field} = input[{offset}];']
            elif kind == 'float':
                lines.append(f'    value.{field} = BindingWire::read_float(input + {offset});')
            elif kind.startswith('int'):
                lines += [f'    const uint32_t bits_{field} = BindingWire::read(input + {offset}, {n});',
                          f'    memcpy(&value.{field}, &bits_{field}, {n});']
            else:
                lines.append(f'    value.{field} = {kind}(BindingWire::read(input + {offset}, {n}));')
            offset += n
        lines += ['    return true;', '}']
        if name not in COMMON_WRITERS:
            lines += [f'static __attribute__((noinline, unused)) void write_value_{name}('
                      f'uint8_t *output, const {name} &value)', '{']
            lines += cpp_write_fields(name, 'value', 0)
            lines += ['}']
        if p['depends']:
            lines.append('#endif')
        lines.append('')
    return lines


def cpp_read(kind, name, offset):
    return [f'    {kind} {name}{{}};', f'    if (!read_value_{kind}(input + {offset}, {name})) {{',
            '        return AP_MODULE_STATUS_INVALID;', '    }']


def cpp_write(kind, value, offset, indent='    '):
    if kind in COMMON_WRITERS:
        return [f'{indent}BindingWire::write_native(output + {offset}, {value});']
    return [f'{indent}write_value_{kind}(output + {offset}, {value});']


def cpp_write_fields(kind, value, offset, indent='    '):
    lines = []
    for field, t, _ in fields(kind):
        n = SCALARS[t][1]
        lines.append(f'{indent}BindingWire::write_float(output + {offset}, {value}.{field});' if t == 'float' else
                     f'{indent}BindingWire::write(output + {offset}, uint32_t({value}.{field}), {n});')
        offset += n
    return lines


def select(ir, first_id, method_contract):
    methods, reports = [], []
    policies = dict(TYPES, **{n: dict(p, private=False, hidden=[], fields=fields(n))
                              for n, p in math_binding.TYPES.items()})
    for name, p in sorted(policies.items()):
        owner = next(o for o in ir['userdata'] if o['name'] == name)
        if (owner['depends'] != p['depends'] or owner['get'] or owner['literal'] or owner['reference'] or
                owner['creation'] or owner['creation_manual'] or owner['creation_args'] or owner['operator_getter'] or
                any(owner[k] for k in ('semaphore', 'scheduler_semaphore', 'semaphore_pointer'))):
            raise ValueError('unaudited changed value owner: ' + name)
        public = {f: t for f, t, _ in p['fields'] if f not in p['hidden']}
        if {f['name'] for f in owner['fields']} != set(public):
            raise ValueError('unaudited changed value fields: ' + name)
        for f in owner['fields']:
            if (f['type']['kind'] != public[f['name']] or f['rename'] or f['depends'] or f['array_len'] or
                    f['valid_mask_name'] or f['valid_mask_value'] or not f['read'] or not f['write'] or
                    any(f['type'][k] for k in ('nullable', 'reference', 'pointer')) or
                    (f['type']['range'] and not p['private'])):
                raise ValueError('unaudited value field: ' + name + '.' + f['name'])
        candidates = [(m, None) for m in owner['methods'] if m['name'] not in math_binding.METHODS.get(name, {})]
        # Range-checked assignment is a host call, so macro bounds stay in the
        # firmware. Reusing a host-produced value does NOT revalidate its fields.
        if p['private']:
            for f in owner['fields']:
                candidates.append((dict(name='set_' + f['name'], rename=None, depends=f['depends'], deprecate=None,
                                        line=f['line'], arguments=[f['type']],
                                        **{'return': math_binding.ir_type('void')}), f['name']))
        operators = {'+': 'add', '-': 'sub'}
        if name not in math_binding.TYPES and (owner['operators_manual'] or
                                               any(op not in operators for op in owner['operator_names'])):
            raise ValueError('unsupported value operators: ' + name)
        for op in (owner['operator_names'] if name not in math_binding.TYPES else []):
            value_type = dict(math_binding.ir_type('float'), kind='userdata', name=name)
            candidates.append((dict(name=operators[op], rename=None, depends=None, deprecate=None, line=0,
                                    arguments=[value_type], operator=op, **{'return': value_type}), None))
        for method, assignment in sorted(candidates, key=lambda v: v[0]['name']):
            report = dict(owner=name, name=method['name'], line=method['line'], kind='userdata-method')
            try:
                item = method_contract(owner, method)
                item.update(id=first_id + len(methods), value_receiver=name,
                            rust_name=(method['rename'] or method['name']).lower())
                if method.get('operator'):
                    item['operator'] = method['operator']
                item['aliases'] = [a['alias'] for a in owner['aliases'] if a['name'] == method['name'] and not a['manual']]
                if assignment:
                    item['assignment'] = assignment
                    item['api_name'] = name + '.' + assignment + '.set'
                if method['name'] == 'copy':
                    if item['result']['type'] != name or item['arguments']:
                        raise ValueError('unsupported-copy-intrinsic')
                    item['copy'] = True
                methods.append(item)
                report.update(status='generated', api=item['api_name'])
            except ValueError as error:
                report.update(status='unsupported', reason=str(error))
            if not assignment and not method.get('operator'):
                reports.append(report)
        for alias in owner['aliases']:
            target = next((r for r in reports if r['owner'] == name and r['name'] == alias['name']), None)
            supported = target and target['status'] == 'generated' and not alias['manual']
            reports.append(dict(owner=name, name=alias['alias'], kind='userdata-alias',
                                status='generated' if supported else 'unsupported',
                                reason='alias' if supported else 'manual-lua' if alias['manual'] else 'userdata-alias'))
    names = [(m['owner'], n) for m in methods for n in [m['rust_name']] + m['aliases']]
    if len(names) != len(set(names)):
        raise ValueError('duplicate value method name')
    return methods, reports


def rust(methods, notice, emit_method):
    lines = [notice, '#[allow(unused_imports)]',
             'use crate::{result, Context, Error, Result, Quaternion, Vector3f};', '']
    for name, p in sorted(TYPES.items()):
        rust_name = p['rust']
        lines += ['/// A complete local value, not a firmware pointer. Hidden fields survive copy-back.',
                  '/// Methods conservatively require mutable values; copies do not share Lua-style identity.',
                  '#[derive(Clone, Copy, Debug, Default, PartialEq)]', f'pub struct {rust_name} {{']
        for field, kind, _ in p['fields']:
            public = 'pub ' if not p['private'] and field not in p['hidden'] else ''
            lines.append(f'    {public}{field}: {SCALARS[kind][0]},')
        lines += ['}', '', f'impl {rust_name} {{', f'    pub(crate) fn encode(&self) -> [u8; {size(name)}] {{',
                  f'        let mut bytes = [0u8; {size(name)}];']
        offset = 0
        for field, kind, _ in p['fields']:
            n = SCALARS[kind][1]
            lines.append(f'        bytes[{offset}] = u8::from(self.{field});' if kind == 'boolean' else
                         f'        bytes[{offset}..{offset+n}].copy_from_slice(&self.{field}.to_le_bytes());')
            offset += n
        lines += ['        bytes', '    }', '    pub(crate) fn decode(bytes: &[u8]) -> Self {', '        Self {']
        offset = 0
        for field, kind, _ in p['fields']:
            r, n = SCALARS[kind]
            expr = f'bytes[{offset}] != 0' if kind == 'boolean' else (
                f'{r}::from_le_bytes(bytes[{offset}..{offset+n}].try_into().unwrap())')
            lines.append(f'            {field}: {expr},')
            offset += n
        lines += ['        }', '    }', '']
        if p['private']:
            for field, kind, _ in p['fields']:
                if field not in p['hidden']:
                    lines += [f'    pub fn {field}(&self) -> {SCALARS[kind][0]} {{ self.{field} }}', '']
        for method in methods:
            if method.get('value_receiver') == name:
                lines += emit_method(method)
        lines += ['}', '']
    for name in sorted(math_binding.TYPES):
        extra = [m for m in methods if m.get('value_receiver') == name]
        if extra:
            lines.append(f'impl {name} {{')
            for method in extra:
                lines += emit_method(method)
            lines += ['}', '']
    return '\n'.join(lines)
