# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""AP_Periph vehicle-notification state with an owned u64 wire value."""

import rust_calls

from call_model import U64
from call_model import Call
from call_model import Module

OWNER = 'AP_Periph_FW'
POLICY = 'v1:read-singleton-state,u64-little-endian,no-retained-firmware-pointers'


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected = dict(rename='periph', depends='defined(HAL_BUILD_AP_PERIPH)', get=None,
                    literal=False, reference=False, semaphore=False,
                    scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected.items()):
        raise ValueError('unaudited AP_Periph_FW metadata')
    method = next((item for item in owner['methods'] if item['name'] == 'get_vehicle_state'), None)
    if (method is None or method['return']['kind'] != 'userdata' or method['return']['name'] != 'uint64_t' or
            method['arguments'] or method['depends'] != 'AP_PERIPH_NOTIFY_ENABLED'):
        raise ValueError('unaudited AP_Periph_FW.get_vehicle_state')
    return specification(first_id)


def specification(first_id):
    return dict(methods=[dict(id=first_id, name='periph.get_vehicle_state', arguments=[],
                              result='u64-le', policy=POLICY)])


def client_model(spec):
    method = spec['methods'][0]
    return Module((Call(method['name'], method['id'], result=U64),))


def rust(spec, notice):
    return rust_calls.emit(client_model(spec), notice)
