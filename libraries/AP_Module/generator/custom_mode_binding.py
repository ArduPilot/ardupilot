# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Lifecycle-scoped handles for persistent custom vehicle modes."""

from native_templates import render

OWNER = 'AP_Vehicle'
STATE = 'AP_Vehicle::custom_mode_state'
FULL_NAME_MAX = 31
SHORT_NAME_MAX = 4
POLICY = ('v1:persistent-mode-registration,module-context-owned-state-handle,'
          'cleanup-forces-allow-entry-false,full-name-max31,short-name-max4,'
          'blocking-scheduler-lock,no-retained-module-pointers')


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    methods = [] if owner is None else [method for method in owner['methods']
                                        if method['name'] == 'register_custom_mode']
    if len(methods) != 1:
        raise ValueError('missing AP_Vehicle.register_custom_mode')
    method = methods[0]
    args = [(item['kind'], item['name'], item['skip_check']) for item in method['arguments']]
    if (owner['rename'] != 'vehicle' or owner['depends'] != 'AP_SCRIPTING_BINDING_VEHICLE_ENABLED' or
            not owner['scheduler_semaphore'] or method['return']['kind'] != 'ap_object' or
            method['return']['name'] != STATE or args != [
                ('uint8_t', None, True), ('string', None, False), ('string', None, False)]):
        raise ValueError('unaudited custom mode registration declaration')
    state = next((item for item in ir['ap_objects'] if item['name'] == STATE), None)
    if state is None or state['depends'] != 'AP_SCRIPTING_BINDING_VEHICLE_ENABLED':
        raise ValueError('unaudited custom mode state declaration')
    fields = state['fields']
    if (len(fields) != 1 or fields[0]['name'] != 'allow_entry' or not fields[0]['read'] or
            not fields[0]['write'] or fields[0]['type']['kind'] != 'boolean'):
        raise ValueError('unaudited custom mode allow_entry field')
    return specification(first_id)


def specification(first_id):
    return dict(methods=[
        dict(id=first_id, name='vehicle.register_custom_mode',
             arguments=[dict(type='u8', name='number'),
                        dict(type='nonempty-nul-free-bytes-max31-zero-pad32', name='full_name'),
                        dict(type='nonempty-nul-free-bytes-max4-zero-pad5', name='short_name')],
             result='optional-nonzero-u32-le', policy=POLICY),
        dict(id=first_id + 1, name='CustomMode.allow_entry',
             arguments=[dict(type='nonzero-u32-le', name='handle')], result='bool-u8', policy=POLICY),
        dict(id=first_id + 2, name='CustomMode.set_allow_entry',
             arguments=[dict(type='nonzero-u32-le', name='handle'), dict(type='bool-u8', name='value')],
             result='void', policy=POLICY),
    ], full_name_max=FULL_NAME_MAX, short_name_max=SHORT_NAME_MAX)


def rust(spec, notice):
    register, get, set_value = spec['methods']
    return notice + render(
        'rust/custom_mode.rs',
        full_name_max=FULL_NAME_MAX,
        short_name_max=SHORT_NAME_MAX,
        register_id=register['id'],
        get_id=get['id'],
        set_value_id=set_value['id'],
    )
