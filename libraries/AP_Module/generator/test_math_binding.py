# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Reject changes to the manually audited value/mutation contracts."""

import copy
import json

import bindings
import generate
import math_binding
import pytest


@pytest.fixture(scope='module')
def ir():
    return bindings.read_ir()


@pytest.fixture(scope='module')
def selection():
    return json.loads((bindings.ROOT / 'generator/bindings.json').read_text())


@pytest.mark.parametrize('owner_name', math_binding.TYPES)
@pytest.mark.parametrize('change', [
    'field_type', 'range', 'read', 'write', 'array', 'mask', 'field_order',
    'field_missing', 'operator', 'manual', 'depends', 'creation', 'return',
    'argument', 'reference', 'skip_check', 'alias', 'method_missing',
])
def test_math_audit(ir, selection, owner_name, change):
    ir = copy.deepcopy(ir)
    owner = next(item for item in ir['userdata'] if item['name'] == owner_name)
    field = owner['fields'][0]
    method = next(item for item in owner['methods'] if item['name'] ==
                  ('scale' if owner_name == 'Vector3f' else 'from_euler'))
    if change == 'field_type':
        field['type']['kind'] = 'uint32_t'
    elif change == 'range':
        field['type']['range'] = ['0', '1']
    elif change in ('read', 'write'):
        field[change] = False
    elif change == 'array':
        field['array_len'] = '3'
    elif change == 'mask':
        field['valid_mask_name'] = 'flags'
    elif change == 'field_missing':
        owner['fields'].pop()
    elif change == 'operator':
        owner['operator_names'].append('/')
    elif change == 'manual':
        owner['operators_manual'] = True
    elif change == 'depends':
        owner['depends'] = 'OTHER_ENABLED'
    elif change == 'creation':
        owner['creation'] = 'custom_new'
    elif change == 'return':
        method['return']['kind'] = 'uint32_t'
    elif change == 'argument':
        method['arguments'].append(math_binding.ir_type('float', True))
    elif change in ('reference', 'skip_check'):
        method['arguments'][0][change] = not method['arguments'][0][change]
    elif change == 'alias':
        owner['aliases'].append(dict(name='scale', alias='scaled', manual=False, depends=None))
    elif change == 'method_missing':
        owner['methods'].remove(method)
    else:
        # Component wire order is explicitly audited, independent of parser order.
        original = bindings.select(ir, selection)
        owner['fields'].reverse()
        owner['methods'].reverse()
        owner['operator_names'].reverse()
        assert bindings.select(ir, selection) == original
        return
    with pytest.raises(ValueError, match='unaudited'):
        bindings.select(ir, selection)


def test_math_directions_and_fingerprint(ir, selection):
    contract = bindings.select(ir, selection)
    types = {owner['name']: owner for owner in contract['math']['types']}
    quaternion = {method['name']: method for method in types['Quaternion']['methods']}
    assert quaternion['earth_to_body']['receiver'] == 'in'
    assert quaternion['earth_to_body']['arguments'] == [dict(type='Vector3f', direction='inout', name='vector')]
    assert quaternion['to_axis_angle']['arguments'] == [dict(type='Vector3f', direction='out', name='vector')]
    for name in ('from_axis_angle', 'from_angular_velocity'):
        assert quaternion[name]['receiver'] == 'inout'
        names = ['axis', 'theta'] if name == 'from_axis_angle' else ['rate', 'dt']
        assert quaternion[name]['arguments'] == [dict(type='Vector3f', direction='in', name=names[0]),
                                                 dict(type='float', direction='in', name=names[1])]
    assert types['Quaternion']['fields'] == ['q1', 'q2', 'q3', 'q4']
    assert types['Quaternion']['default'] == [1, 0, 0, 0]
    assert types['Vector3f']['fields'] == ['x', 'y', 'z']
    assert types['Vector3f']['default'] == [0, 0, 0]
    spec = json.loads((bindings.ROOT / 'generator/host_api.json').read_text())
    original = generate.generate(spec, contract)
    cpp = original['BindingMath_generated.cpp']
    assert 'ud.from_axis_angle(arg0, arg1);' in cpp
    assert 'ud.from_euler(arg0, arg1, arg2);' in cpp
    for operator in ('+', '-', '*'):
        assert f'const auto value = ud {operator} arg0;' in cpp
    assert 'isfinite' not in cpp and 'is_finite' not in original['sdk/rust/src/math.rs']
    assert len([method for owner in types.values() for method in owner['methods']]) == 24
    for change in ('direction', 'default', 'fields', 'policy', 'guard'):
        changed = copy.deepcopy(contract)
        if change == 'direction':
            changed['math']['types'][0]['methods'][0]['arguments'][0]['direction'] = 'in'
        elif change == 'policy':
            changed['math']['policy'] += '-changed'
        elif change == 'guard':
            changed['math']['types'][0]['depends'] = '0'
        else:
            changed['math']['types'][0][change].reverse()
        generated = generate.generate(spec, changed)
        assert generated['sdk/abi.json'] == original['sdk/abi.json']
        different = generated['generator/imports.json'] != original['generator/imports.json']
        # Defaults are created by the SDK, not part of a host method ABI.
        assert different == (change not in ('guard', 'default'))


@pytest.mark.parametrize('change', ['stale', 'missing', 'extra'])
def test_argument_name_table(ir, selection, monkeypatch, change):
    names = copy.deepcopy(math_binding.ARGUMENT_NAMES)
    if change == 'stale':
        names['Quaternion']['removed'] = []
    elif change == 'missing':
        names['Quaternion']['from_euler'].pop()
    else:
        names['Quaternion']['from_euler'].append('extra')
    monkeypatch.setattr(math_binding, 'ARGUMENT_NAMES', names)
    with pytest.raises(ValueError, match='argument names'):
        bindings.select(ir, selection)
