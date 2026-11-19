# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Named handle strategies for pointer-backed Lua objects, never raw pointers."""

TYPES = {
    'RC_Channel': dict(rust='RcChannel', strategy='indexed-singleton-array',
                       singleton='RC_Channels', accessor='channel', count='NUM_RC_CHANNELS', base=1,
                       depends='AP_RC_CHANNEL_ENABLED', lock='scheduler-semaphore'),
}


def wire_type(name):
    policy = TYPES[name]
    return dict(handle=name, wire='u32-le,zero-is-null', strategy=policy['strategy'], base=policy['base'],
                lifetime='revalidate-index-each-call,no-owned-resource')


def cpp_helpers(names):
    lines = []
    for name in sorted(names):
        p = TYPES[name]
        lines += [f'#if {p["depends"]}', f'static bool write_handle_{name}(uint8_t *output, const {name} *value)', '{',
                  '    if (value == nullptr) {', '        BindingWire::write(output, 0, 4);', '        return true;', '    }',
                  f'    auto *owner = {p["singleton"]}::get_singleton();',
                  '    if (owner == nullptr) {', '        return false;', '    }',
                  f'    for (uint32_t i = 0; i < {p["count"]}; i++) {{',
                  f'        if (owner->{p["accessor"]}(i) == value) {{',
                  f'            BindingWire::write(output, i + {p["base"]}, 4);', '            return true;',
                  '        }', '    }', '    return false;', '}', '#endif', '']
    return lines


def cpp_token(name):
    p = TYPES[name]
    return ['    const uint32_t token = BindingWire::read(input, 4);',
            f'    if (token < {p["base"]} || token - {p["base"]} >= {p["count"]}) {{',
            '        return AP_MODULE_STATUS_INVALID;', '    }']


def cpp_receiver(name):
    p = TYPES[name]
    return [f'    auto *owner = {p["singleton"]}::get_singleton();',
            '    if (owner == nullptr) {', '        return AP_MODULE_STATUS_UNAVAILABLE;', '    }',
            f'    auto *ud = owner->{p["accessor"]}(token - {p["base"]});',
            '    if (ud == nullptr) {', '        return AP_MODULE_STATUS_UNAVAILABLE;', '    }']


def select(ir, first_id, method_contract, effects):
    methods, reports = [], []
    for name, policy in TYPES.items():
        if (policy['strategy'] != 'indexed-singleton-array' or policy['base'] != 1 or
                policy['lock'] != 'scheduler-semaphore'):
            raise ValueError('unsupported borrowed-object strategy: ' + name)
        if not any(o['name'] == name for o in ir.get('ap_objects', [])):
            raise ValueError('missing borrowed-object owner: ' + name)
    for owner in sorted(ir.get('ap_objects', []), key=lambda o: o['name']):
        name = owner['name']
        policy = TYPES.get(name)
        if policy and (owner['depends'] != policy['depends'] or owner['get'] or owner['literal'] or owner['reference'] or
                       any(owner[k] for k in ('semaphore', 'scheduler_semaphore', 'semaphore_pointer'))):
            raise ValueError('changed borrowed-object owner: ' + name)
        for method in sorted(owner['methods'], key=lambda m: m['name']):
            report = dict(owner=name, name=method['name'], line=method['line'], kind='ap-object-method')
            try:
                if policy is None:
                    raise ValueError('object-ownership-or-lifetime')
                # Native calls serialize RC input/override state with the main
                # loop and each other, unlike Lua's unlocked object methods.
                host_owner = dict(owner, scheduler_semaphore=policy['lock'] == 'scheduler-semaphore')
                item = method_contract(host_owner, method)
                item.update(id=first_id + len(methods), receiver=name, rust_name=method['rename'] or method['name'])
                item['aliases'] = [a['alias'] for a in owner['aliases'] if a['name'] == method['name'] and not a['manual']]
                if name + '.' + method['name'] in effects:
                    item['effects'] = effects[name + '.' + method['name']]
                methods.append(item)
                report.update(status='generated', api=item['api_name'])
            except ValueError as error:
                report.update(status='unsupported', reason=str(error))
            reports.append(report)
        for alias in owner['aliases']:
            target = next((r for r in reports if r['owner'] == name and r['name'] == alias['name']), None)
            supported = target and target['status'] == 'generated' and not alias['manual']
            reports.append(dict(owner=name, name=alias['alias'], kind='ap-object-alias',
                                status='generated' if supported else 'unsupported',
                                reason=('alias' if supported else 'manual-lua' if alias['manual']
                                        else 'object-ownership-or-lifetime')))
    return methods, reports


def rust(methods, notice, emit_method):
    lines = [notice, '#[allow(unused_imports)]', 'use crate::{result, Context, Error, Result};',
             'use core::num::NonZeroU32;', '']
    for name in sorted({m['receiver'] for m in methods}):
        p = TYPES[name]
        lines += ['/// A copyable channel index, not a firmware pointer or an owned resource.',
                  '/// Every operation revalidates it through the supplied Context.',
                  '/// A cached handle stays on the same channel if its RC option changes.',
                  '#[derive(Clone, Copy, Debug, Eq, PartialEq)]',
                  f'pub struct {p["rust"]} {{ token: NonZeroU32 }}', '', f'impl {p["rust"]} {{',
                  '    pub(crate) fn from_wire(token: u32) -> Option<Self> {',
                  '        NonZeroU32::new(token).map(|token| Self { token })', '    }', '']
        for method in methods:
            if method['receiver'] == name:
                lines += emit_method(method)
        lines += ['}', '']
    return '\n'.join(lines)
