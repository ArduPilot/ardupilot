# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Module-owned setup adapters for persistent serial-LED drivers."""

import rust_calls

from call_model import BOOL
from call_model import U8
from call_model import Call
from call_model import Field
from call_model import Module

OWNER = 'AP_SerialLED'
METHODS = ('set_num_neopixel', 'set_num_neopixel_rgb', 'set_num_profiled')
MAX_LEDS = 128
POLICY = ('v1:module-context-channel-claim-on-success,cleanup-owner-after-worker-join,'
          'persistent-driver-allocation-not-released,compatible-reclaim-after-restart,'
          'firmware-validates-existing-driver,blocking-lock,no-retained-module-pointers')


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected_owner = dict(rename='serialLED', depends='AP_SERIALLED_ENABLED', get=None, literal=False,
                          reference=False, semaphore=False, scheduler_semaphore=False,
                          semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected_owner.items()):
        raise ValueError('unaudited AP_SerialLED owner metadata')

    expected_result = ('boolean', None, False, False, False, False, None)
    expected_arguments = [
        ('uint8_t', None, False, False, False, False, ('1', '16')),
        ('uint8_t', None, False, False, False, False, ('0', 'AP_SERIALLED_MAX_LEDS')),
    ]
    methods = {method['name']: method for method in owner['methods'] if method['name'] in METHODS}
    if set(methods) != set(METHODS):
        raise ValueError('missing serial-LED setup method')
    for name in METHODS:
        method = methods[name]
        actual = (_field_shape(method['return']), [_field_shape(argument) for argument in method['arguments']])
        if (method['rename'] is not None or method['depends'] is not None or method['deprecate'] is not None or
                actual != (expected_result, expected_arguments)):
            raise ValueError(f'unaudited AP_SerialLED.{name} declaration')
    return specification(first_id)


def specification(first_id):
    methods = []
    for offset, name in enumerate(METHODS):
        methods.append(dict(
            id=first_id + offset,
            name=f'serialLED.{name}',
            arguments=[dict(type='u8', name='channel', range=[1, 16]),
                       dict(type='u8', name='num_leds', range=[0, MAX_LEDS])],
            result='bool-u8',
            policy=POLICY,
        ))
    return dict(methods=methods)


def client_model(spec):
    arguments = (Field('channel', U8, 1, 16), Field('num_leds', U8, 0, MAX_LEDS))
    docs = ('Configure a persistent serial-LED driver and claim its channel for this module.',
            'The ownership claim is released at module exit, but firmware cannot undo the',
            'underlying driver allocation; a restarted module may reclaim a compatible setup.')
    return Module(tuple(Call(m['name'], m['id'], arguments, BOOL, docs=docs) for m in spec['methods']))


def rust(spec, notice):
    return rust_calls.emit(client_model(spec), notice)
