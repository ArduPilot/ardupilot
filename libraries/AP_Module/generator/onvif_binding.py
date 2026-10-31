# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Single-module initialization adapter for the persistent ONVIF client."""

import call_model as model
import rust_calls

OWNER = 'AP_ONVIF'
METHOD = 'start'
STRING_MAX = 255
STRING_WIRE_SIZE = STRING_MAX + 1
POLICY = ('v1:single-module-owner-claimed-before-start,cleanup-owner-after-worker-join,'
          'persistent-client-allocation-not-released,reconfigurable-after-restart,'
          'three-nul-free-byte-strings-max255-zero-pad256,blocking-lock,'
          'blocking-firmware-network-call,no-retained-module-pointers')


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected_owner = dict(rename='onvif', depends='ENABLE_ONVIF == 1', get=None, literal=False,
                          reference=False, semaphore=False, scheduler_semaphore=False,
                          semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected_owner.items()):
        raise ValueError('unaudited AP_ONVIF owner metadata')
    methods = [method for method in owner['methods'] if method['name'] == METHOD]
    expected_result = ('boolean', None, False, False, False, False, None)
    expected_argument = ('string', None, False, False, False, False, None)
    if len(methods) != 1:
        raise ValueError('missing AP_ONVIF.start method')
    method = methods[0]
    actual = (_field_shape(method['return']), [_field_shape(argument) for argument in method['arguments']])
    if (method['rename'] is not None or method['depends'] is not None or method['deprecate'] is not None or
            actual != (expected_result, [expected_argument] * 3)):
        raise ValueError('unaudited AP_ONVIF.start declaration')
    return specification(first_id)


def specification(first_id):
    return dict(
        id=first_id,
        name='onvif.start',
        arguments=[dict(type=f'nul-free-bytes-max{STRING_MAX}-zero-pad{STRING_WIRE_SIZE}', name=name)
                   for name in ('username', 'password', 'hostname')],
        result='bool-u8',
        policy=POLICY,
    )


def client_model(method):
    call = model.Call(
        method['name'], method['id'],
        tuple(model.Field(name, model.String(STRING_WIRE_SIZE)) for name in ('username', 'password', 'hostname')),
        model.BOOL, docs=(
            'Start or reconfigure the firmware-wide ONVIF client for this module.',
            '',
            'A start attempt claims the client until this module exits, even when the',
            'camera probe returns false. Firmware allocations and configuration persist;',
            'cleanup releases only the module ownership claim after the worker has joined.',
        ),
    )
    return model.Module((call,), constants=(('ONVIF_STRING_MAX', STRING_MAX),))


def rust(method, notice):
    return rust_calls.emit(client_model(method), notice)
