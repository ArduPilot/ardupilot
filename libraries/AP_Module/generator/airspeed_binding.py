# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Revalidated scripting-air-speed backend adapters."""

import rust_calls

from call_model import BOOL
from call_model import F32
from call_model import U8
from call_model import Call
from call_model import Field
from call_model import Handle
from call_model import Module
from call_model import SelectedHandle

OWNER = 'AP_Airspeed'
BACKEND = 'AP_Airspeed_Backend'
DEPENDS = 'AP_AIRSPEED_SCRIPTING_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane)'
POLICY = ('v1:non-owning-backend-index,revalidate-every-call,no-retained-firmware-pointers,'
          'all-f32-bits,native-backend-return,no-lifecycle-rollback')


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected_owner = dict(rename='airspeed', depends=DEPENDS, get=None, literal=False, reference=False,
                          semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected_owner.items()):
        raise ValueError('unaudited AP_Airspeed metadata')
    get_backend = next((method for method in owner['methods'] if method['name'] == 'get_backend'), None)
    if (get_backend is None or _field_shape(get_backend['return']) !=
            ('ap_object', BACKEND, False, False, False, False, None) or
            [_field_shape(arg) for arg in get_backend['arguments']] !=
            [('uint8_t', None, False, False, False, True, None)]):
        raise ValueError('unaudited AP_Airspeed.get_backend method')

    backend = next((item for item in ir['ap_objects'] if item['name'] == BACKEND), None)
    expected_backend = dict(rename=None, depends=DEPENDS, get=None, creation=None, creation_manual=False,
                            creation_args=0, has_fields=False, has_enums=False, operations=0,
                            operator_getter=None, operators_manual=False, operator_names=[], literal=False,
                            reference=False, semaphore=False, scheduler_semaphore=False,
                            semaphore_pointer=False)
    if backend is None or any(backend[key] != value for key, value in expected_backend.items()):
        raise ValueError('unaudited AP_Airspeed_Backend metadata')
    expected_methods = {'handle_script_airspeed', 'handle_script_differential_pressure',
                        'handle_script_temperature'}
    if {method['name'] for method in backend['methods']} != expected_methods:
        raise ValueError('unaudited AP_Airspeed_Backend methods')
    for method in backend['methods']:
        if (_field_shape(method['return']) != ('boolean', None, False, False, False, False, None) or
                [_field_shape(arg) for arg in method['arguments']] !=
                [('float', None, False, False, False, True, None)] or
                method['rename'] is not None or method['depends'] is not None or method['deprecate'] is not None):
            raise ValueError('unaudited AP_Airspeed_Backend.' + method['name'])
    return specification(first_id)


def specification(first_id):
    receiver = dict(type='u8,revalidated-airspeed-backend-index', direction='borrowed-receiver')
    return dict(methods=[
        dict(id=first_id, name='airspeed.get_backend', arguments=['u8-index'],
             result='optional-backend-index-bool-u8', policy=POLICY),
        dict(id=first_id + 1, name='AirspeedBackend.handle_script_airspeed',
             arguments=[receiver, 'f32-le,all-bits'], result='bool-u8', policy=POLICY),
        dict(id=first_id + 2, name='AirspeedBackend.handle_script_differential_pressure',
             arguments=[receiver, 'f32-le,all-bits'], result='bool-u8', policy=POLICY),
        dict(id=first_id + 3, name='AirspeedBackend.handle_script_temperature',
             arguments=[receiver, 'f32-le,all-bits'], result='bool-u8', policy=POLICY),
    ])


def client_model(spec):
    get_backend, airspeed, pressure, temperature = spec['methods']
    backend = Handle('AirspeedBackend', 'borrowed-index')
    calls = [Call(get_backend['name'], get_backend['id'], (Field('index', U8),),
                  SelectedHandle(backend, 'index'))]
    for method, argument in ((airspeed, 'airspeed_ms'), (pressure, 'press_pa'), (temperature, 'temperature_c')):
        calls.append(Call(method['name'], method['id'], (Field(argument, F32),), BOOL, receiver=backend))
    return Module(tuple(calls), (backend,))


def rust(spec, notice):
    return rust_calls.emit(client_model(spec), notice)
