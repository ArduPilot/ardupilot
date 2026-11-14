# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Leak-free old-parameter value lookup replacing Parameter.init_by_info."""

import rust_calls

from call_model import F32
from call_model import U8
from call_model import U16
from call_model import U32
from call_model import Call
from call_model import Enum
from call_model import Field
from call_model import Module
from call_model import OptionalValue

OWNER = 'Parameter'
POLICY = ('v1:stack-owned-conversion-value,types-int8-int16-int32-float,'
          'optional-f32-all-bits,no-retained-param-pointer,no-module-reload-leak')


def _shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def select(ir, first_id):
    owner = next((item for item in ir['userdata'] if item['name'] == OWNER), None)
    method = next((item for item in owner['methods'] if item['name'] == 'init_by_info'), None) if owner else None
    expected_arguments = [
        ('uint16_t', None, False, False, False, True, None),
        ('uint32_t', None, False, False, False, True, None),
        ('enum', 'ap_var_type', False, False, False, False, ('AP_PARAM_INT8', 'AP_PARAM_FLOAT')),
    ]
    if (method is None or _shape(method['return']) !=
            ('boolean', None, False, False, False, False, None) or
            [_shape(arg) for arg in method['arguments']] != expected_arguments or
            any(method[key] is not None for key in ('rename', 'depends', 'deprecate'))):
        raise ValueError('unaudited Parameter.init_by_info')
    return specification(first_id)


def specification(first_id):
    return dict(methods=[dict(id=first_id, name='parameter.old_value',
                              arguments=['u16-le-key', 'u32-le-group-element', 'type-u8-range1-4'],
                              result='optional-f32-le', policy=POLICY)])


def client_model(spec):
    method = spec['methods'][0]
    type = Enum('OldParameterType', U8, (('Int8', 1), ('Int16', 2), ('Int32', 3), ('Float', 4)))
    arguments = (Field('key', U16), Field('group_element', U32), Field('parameter_type', type))
    docs = ('Read a legacy parameter conversion value without retaining or allocating an AP_Param object.',)
    return Module((Call(method['name'], method['id'], arguments, OptionalValue(F32), namespace='param', docs=docs),), (type,))


def rust(spec, notice):
    return rust_calls.emit(client_model(spec), notice)
