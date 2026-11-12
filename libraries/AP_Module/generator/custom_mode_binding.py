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
CONTROL_POLICY = ('v1:module-context-owned-state-handle,owner-checked,setters-store-under-scheduler-lock,'
                  'requests-applied-on-main-thread,request-rejected-unless-active-and-owner-live,'
                  'generation-u16-nonzero-wrapping,nonfinite-takeoff-alt-invalid,'
                  'supervision-opt-in-latched-owner-loss,cleanup-publishes-owner-loss-event,'
                  'blocking-scheduler-lock,no-retained-module-pointers')
# EKF_FS_DISARM applies in every flight mode while the owner is live
OPTIONS_POLICY = CONTROL_POLICY + ',ekf-fs-disarm-all-modes-while-owner-live'
REQUESTS = ('GUIDED', 'TAKEOFF', 'LAND', 'RTL', 'LOITER')
OPTIONS = ('NO_EXTERNAL_GUIDED', 'EKF_FS_DISARM', 'ARMING_GATED')
STATUS = dict(name='CustomModeStatus', fields=[
    ['request', 'u8'], ['generation', 'u16-le'], ['applied_generation', 'u16-le'], ['init_ok', 'bool-u8'],
    ['complete', 'bool-u8'], ['active', 'bool-u8'], ['takeoff_event', 'bool-u8'], ['owner_lost', 'bool-u8']])


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
    ] + [dict(id=first_id + 3 + i, name='CustomMode.' + name,
              arguments=[dict(type='nonzero-u32-le', name='handle')] + arguments, result=result,
              policy=OPTIONS_POLICY if name == 'set_options' else CONTROL_POLICY)
         for i, (name, arguments, result) in enumerate(controls())],
        full_name_max=FULL_NAME_MAX, short_name_max=SHORT_NAME_MAX)


def controls():
    return [
        ('set_request', [dict(type='u8', name='request', range=[0, len(REQUESTS) - 1]),
                         dict(type='f32-le', name='takeoff_alt_m')], 'u16-le'),
        ('status', [], STATUS),
        ('set_options', [dict(type='u8', name='options', range=[0, (1 << len(OPTIONS)) - 1])], 'void'),
        ('set_arming_ready', [dict(type='bool-u8', name='ready')], 'void'),
        ('set_supervision', [dict(type='u16-le', name='timeout_ms')], 'void'),
        ('keepalive', [], 'void'),
        ('ack_takeoff_event', [], 'bool-u8'),
        ('request_arm', [dict(type='bool-u8', name='arm')], 'bool-u8'),
        ('set_target_posvelaccel',
         [dict(type='f32-le', name=name) for name in ('pos_n_m', 'pos_e_m', 'pos_d_m', 'vel_n_ms', 'vel_e_ms',
                                                       'vel_d_ms', 'acc_n_mss', 'acc_e_mss', 'acc_d_mss')] +
         [dict(type='bool-u8', name='use_yaw'), dict(type='f32-le', name='yaw_deg'),
          dict(type='bool-u8', name='use_yaw_rate'), dict(type='f32-le', name='yaw_rate_degs'),
          dict(type='bool-u8', name='yaw_relative')], 'bool-u8'),
    ]


def rust(spec, notice):
    register, get, set_value = spec['methods'][:3]
    controls = {method['name'].split('.')[1] + '_id': method['id'] for method in spec['methods'][3:]}
    return notice + render(
        'rust/custom_mode.rs',
        full_name_max=FULL_NAME_MAX,
        short_name_max=SHORT_NAME_MAX,
        register_id=register['id'],
        get_id=get['id'],
        set_value_id=set_value['id'],
        **controls,
    )
