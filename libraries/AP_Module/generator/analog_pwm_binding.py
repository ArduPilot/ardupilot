# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Opaque analog-input and lifecycle-owned PWM source handles."""

import rust_calls

from call_model import BOOL
from call_model import F32
from call_model import U8
from call_model import U16
from call_model import Call
from call_model import Field
from call_model import Handle
from call_model import Module
from call_model import OptionalValue

ANALOG_OWNER = 'hal.analogin'
ANALOG_SOURCE = 'AP_HAL::AnalogSource'
PWM_SOURCE = 'AP_HAL::PWMSource'
POLICY = ('v1:opaque-context-owned-handles,max4-each-per-module,analog-source-reused-after-cleanup,'
          'pwm-source-deleted-after-close-or-worker-join,no-retained-module-pointers')


def _shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def _method(method):
    return (_shape(method['return']), tuple(_shape(argument) for argument in method['arguments']),
            method['rename'], method['depends'], method['deprecate'])


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == ANALOG_OWNER), None)
    channel = None if owner is None else next(
        (method for method in owner['methods'] if method['name'] == 'channel'), None)
    expected_channel = (('ap_object', ANALOG_SOURCE, False, False, False, False, None),
                        (('literal', 'ANALOG_INPUT_NONE', False, False, False, False, None),),
                        None, None, None)
    if (owner is None or owner['rename'] != 'analog' or
            owner['depends'] != '!defined(HAL_DISABLE_ADC_DRIVER)' or not owner['literal'] or
            channel is None or _method(channel) != expected_channel):
        raise ValueError('unaudited analog.channel declaration')

    analog = next((item for item in ir['ap_objects'] if item['name'] == ANALOG_SOURCE), None)
    expected_analog = {
        'set_pin': (('boolean', None, False, False, False, False, None),
                    (('uint8_t', None, False, False, False, True, None),), None, None, None),
        'voltage_average': (('float', None, False, False, False, False, None), (), None, None, None),
        'voltage_latest': (('float', None, False, False, False, False, None), (), None, None, None),
        'voltage_average_ratiometric': (
            ('float', None, False, False, False, False, None), (), None, None, None),
    }
    if (analog is None or analog['depends'] != '!defined(HAL_DISABLE_ADC_DRIVER)' or
            analog['aliases'] or {method['name']: _method(method) for method in analog['methods']} !=
            expected_analog):
        raise ValueError('unaudited AnalogSource declaration')

    pwm = next((item for item in ir['ap_objects'] if item['name'] == PWM_SOURCE), None)
    expected_pwm = {
        'set_pin': (('boolean', None, False, False, False, False, None),
                    (('uint8_t', None, False, False, False, True, None),
                     ('literal', '"Scripting"', False, False, False, False, None)), None, None, None),
        'get_pwm_us': (('uint16_t', None, False, False, False, False, None), (), None, None, None),
        'get_pwm_avg_us': (('uint16_t', None, False, False, False, False, None), (), None, None, None),
    }
    if (pwm is None or pwm['depends'] is not None or pwm['aliases'] or
            {method['name']: _method(method) for method in pwm['methods']} != expected_pwm):
        raise ValueError('unaudited PWMSource declaration')
    return specification(first_id)


def specification(first_id):
    names = [
        'analog.channel', 'AnalogSource.set_pin', 'AnalogSource.voltage_average',
        'AnalogSource.voltage_latest', 'AnalogSource.voltage_average_ratiometric',
        'pwm_source.open', 'pwm_source.close', 'PwmSource.set_pin', 'PwmSource.get_pwm_us',
        'PwmSource.get_pwm_avg_us',
    ]
    arguments = [
        [],
        [dict(type='nonzero-u32-le', name='handle'), dict(type='u8', name='pin')],
        [dict(type='nonzero-u32-le', name='handle')],
        [dict(type='nonzero-u32-le', name='handle')],
        [dict(type='nonzero-u32-le', name='handle')],
        [],
        [dict(type='nonzero-u32-le', name='handle')],
        [dict(type='nonzero-u32-le', name='handle'), dict(type='u8', name='pin')],
        [dict(type='nonzero-u32-le', name='handle')],
        [dict(type='nonzero-u32-le', name='handle')],
    ]
    results = ['optional-nonzero-u32-le', 'bool-u8', 'f32-le', 'f32-le', 'f32-le',
               'optional-nonzero-u32-le', 'void', 'bool-u8', 'u16-le', 'u16-le']
    return dict(methods=[dict(id=first_id + index, name=name, arguments=arguments[index],
                              result=results[index], policy=POLICY)
                         for index, name in enumerate(names)])


def client_model(spec):
    analog = Handle('AnalogSource', 'context-token')
    pwm = Handle('PwmSource', 'closeable-token')
    calls = []
    for method in spec['methods']:
        owner, _, name = method['name'].partition('.')
        if owner in ('analog', 'pwm_source') and name != 'close':
            calls.append(Call(method['name'], method['id'], result=OptionalValue(analog if owner == 'analog' else pwm)))
            continue
        receiver = analog if owner == 'AnalogSource' else pwm
        arguments = (Field('pin', U8),) if name == 'set_pin' else ()
        result = BOOL if arguments else F32 if receiver == analog else U16
        if name == 'close':
            result = None
        calls.append(Call(method['name'], method['id'], arguments, result, receiver=receiver, consume=name == 'close'))
    return Module(tuple(calls), (analog, pwm))


def rust(spec, notice):
    return rust_calls.emit(client_model(spec), notice)
