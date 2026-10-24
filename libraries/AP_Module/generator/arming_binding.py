# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Module-owned auxiliary arming authorisation bindings."""

from native_templates import render

OWNER = 'AP_Arming'
FAILURE_MESSAGE_MAX = 41
POLICY = ('v1:module-context-owned-id,owner-validated-updates,cleanup-after-worker-join,'
          'lua-reset-isolation,max3-firmware-wide,failure-message-nul-free-bytes-max41-zero-pad41,'
          'no-retained-module-pointers')


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected_owner = dict(rename='arming', depends='AP_ARMING_ENABLED', get=None, literal=False, reference=False,
                          semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected_owner.items()):
        raise ValueError('unaudited AP_Arming owner metadata')

    expected = {
        'get_aux_auth_id': (
            ('boolean', None, False, False, False, False, None),
            [('uint8_t', None, True, False, False, False, None)]),
        'set_aux_auth_passed': (
            ('void', None, False, False, False, False, None),
            [('uint8_t', None, False, False, False, True, None)]),
        'set_aux_auth_failed': (
            ('void', None, False, False, False, False, None),
            [('uint8_t', None, False, False, False, True, None),
             ('string', None, False, False, False, False, None)]),
    }
    methods = {method['name']: method for method in owner['methods'] if method['name'] in expected}
    if set(methods) != set(expected):
        raise ValueError('missing auxiliary arming method')
    for name, (result, arguments) in expected.items():
        method = methods[name]
        actual = (_field_shape(method['return']), [_field_shape(argument) for argument in method['arguments']])
        if (method['rename'] is not None or method['depends'] is not None or method['deprecate'] is not None or
                actual != (result, arguments)):
            raise ValueError(f'unaudited AP_Arming.{name} declaration')
    return specification(first_id)


def specification(first_id):
    methods = [
        dict(id=first_id, name='arming.get_aux_auth_id', arguments=[],
             result=dict(presence='bool-u8', auth_id='u8,module-owned-token'), policy=POLICY),
        dict(id=first_id + 1, name='arming.set_aux_auth_passed',
             arguments=[dict(type='u8,module-owned-token', name='auth_id')], result='void', policy=POLICY),
        dict(id=first_id + 2, name='arming.set_aux_auth_failed',
             arguments=[dict(type='u8,module-owned-token', name='auth_id'),
                        dict(type=f'nul-free-bytes-max{FAILURE_MESSAGE_MAX}-zero-pad{FAILURE_MESSAGE_MAX}',
                             name='message')],
             result='void', policy=POLICY),
    ]
    return dict(methods=methods)


def rust(spec, notice):
    get, passed, failed = spec['methods']
    return notice + render(
        'rust/arming.rs',
        failure_message_max=FAILURE_MESSAGE_MAX,
        get_id=get['id'],
        passed_id=passed['id'],
        failed_id=failed['id'],
    )
