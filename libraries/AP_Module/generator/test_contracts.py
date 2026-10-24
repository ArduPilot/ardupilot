# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later

import copy
import hashlib
import json

import bindings
import contracts
import pytest


@pytest.fixture(scope='module')
def spec():
    return bindings.specification()


def hashes(spec):
    return {entry['contract']['name']: entry['hash'] for entry in contracts.describe(spec)}


def test_canonical_hash():
    value = {'name': 'ahrs.get_gyro', 'revision': 1}
    wire = b'AP_Module.binding.v1\n{"name":"ahrs.get_gyro","revision":1}'
    assert contracts.canonical(value) == wire
    assert contracts.digest(value) == int.from_bytes(hashlib.sha256(wire).digest()[:8], 'little')
    assert contracts.digest(dict(reversed(list(value.items())))) == contracts.digest(value)


def test_import_section_includes_diagnostic_name(spec):
    entry = next(entry for entry in contracts.describe(spec) if entry['contract']['name'] == 'ahrs.get_gyro')
    imports = contracts.emit(spec, '')['sdk/rust/src/imports.rs']
    assert f'#[link_section = ".ap_module_import_{entry["hash"]}.ahrs.get_gyro"]' in imports
    assert f'#[export_name = "__ap_module_import_{entry["hash"]}.ahrs.get_gyro"]' in imports


@pytest.mark.parametrize('text,value', [('+360', 360), ('-20', -20), ('0.5', 0.5), ('+.5', 0.5),
                                        ('-1.25e-2', -0.0125), ('1e2', 100), ('UINT8_MAX', 'firmware-bound')])
def test_numeric_range_bounds(text, value):
    assert contracts.range_bound(text) == value


def test_numeric_range_hash(spec):
    changed = copy.deepcopy(spec)
    method = next(m for m in changed['methods'] if m['api_name'] == 'battery.reset_remaining')
    method['arguments'][1]['range'] = ['-0.5', '+360']
    before = hashes(changed)
    method['arguments'][1]['range'][0] = '-0.25'
    assert hashes(changed)[method['api_name']] != before[method['api_name']]
    with pytest.raises(ValueError, match='non-finite'):
        contracts.range_bound('1e999')


def test_implementation_changes_and_additions(spec):
    before = hashes(spec)
    changed = copy.deepcopy(spec)
    for method in changed['methods']:
        method.update(id=method['id'] + 1000, owner='DifferentClass', method='different_method',
                      include='different.h', depends='0', owner_depends='0', include_depends='0', rust_name='cosmetic_name')
    changed['methods'].reverse()
    assert hashes(changed) == before
    extra = copy.deepcopy(changed['methods'][0])
    extra.update(id=9999, api_name='new.api', api_aliases=[])
    changed['methods'].append(extra)
    assert all(hashes(changed)[name] == binding_hash for name, binding_hash in before.items())


def test_only_used_types_affect_contract(spec):
    before = hashes(spec)
    changed = copy.deepcopy(spec)
    next(owner for owner in changed['math']['types'] if owner['name'] == 'Quaternion')['fields'].reverse()
    after = hashes(changed)
    assert after['ahrs.get_quaternion'] != before['ahrs.get_quaternion']
    assert after['ahrs.get_gyro'] == before['ahrs.get_gyro']
    assert after['Parameter.get'] == before['Parameter.get']
    assert after['Vector3f.length'] == before['Vector3f.length']


def test_used_signature_changes(spec):
    before = hashes(spec)
    changed = copy.deepcopy(spec)
    next(method for method in changed['methods'] if method['api_name'] == 'ahrs.get_gyro')['result']['type'] = 'Quaternion'
    assert hashes(changed)['ahrs.get_gyro'] != before['ahrs.get_gyro']
    assert hashes(changed)['ahrs.get_quaternion'] == before['ahrs.get_quaternion']


def test_enum_and_literal_identity(spec):
    before = hashes(spec)
    changed = copy.deepcopy(spec)
    method = next(m for m in changed['methods'] if m['api_name'] == 'vehicle.set_mode')
    assert method['arguments'][1] == dict(type='literal', output=False, expression='ModeReason::SCRIPTING')
    method['arguments'][1]['expression'] = 'ModeReason::GCS_COMMAND'
    assert hashes(changed)['vehicle.set_mode'] != before['vehicle.set_mode']
    assert hashes(changed)['vehicle.get_mode'] == before['vehicle.get_mode']
    method = next(m for m in changed['methods'] if m['api_name'] == 'rc.get_aux_cached')
    method['arguments'][0]['enum'] = 'Other::Function'
    assert hashes(changed)['rc.get_aux_cached'] != before['rc.get_aux_cached']
    # Rust names and private dispatch IDs still do not enter the identity.
    method['rust_name'] = 'another_name'
    method['id'] += 1000
    enum_hash = hashes(changed)['rc.get_aux_cached']
    method['id'] += 1000
    assert hashes(changed)['rc.get_aux_cached'] == enum_hash


def test_alias_preserves_old_contract(spec):
    changed = copy.deepcopy(spec)
    method = next(method for method in changed['methods'] if method['api_name'] == 'ahrs.airspeed_EAS')
    old_name = method['api_name']
    method['api_name'], method['api_aliases'] = method['api_aliases'][0], [old_name]
    assert hashes(changed) == hashes(spec)


def test_dronecan_handle_check_message_is_same_id_contract_alias(spec):
    entries = {entry['contract']['name']: entry for entry in contracts.describe(spec)}
    subscription = entries['DroneCanSubscription.check_message']
    handle = entries['DroneCanHandle.check_message']
    assert handle['id'] == subscription['id']
    assert handle['hash'] != subscription['hash']
    subscription_contract = copy.deepcopy(subscription['contract'])
    subscription_contract['name'] = handle['contract']['name']
    assert handle['contract'] == subscription_contract
    emitted = contracts.emit(spec, '')
    binding_name = contracts.binding_names(spec)[handle['id']].upper()
    assert emitted['sdk/rust/src/imports.rs'].count(f'BINDING_{binding_name}:') == 1
    dispatch_id = f'uint32_t(BindingId::{binding_name})'
    assert f'UINT64_C(0x{subscription["hash"]}), {dispatch_id}' in emitted['BindingImports_generated.cpp']
    assert f'UINT64_C(0x{handle["hash"]}), {dispatch_id}' in emitted['BindingImports_generated.cpp']


def test_collision_fails_closed(spec, monkeypatch):
    monkeypatch.setattr(contracts, 'digest', lambda _: 42)
    with pytest.raises(ValueError, match='collision'):
        contracts.describe(spec)


def test_release_catalogue_check(spec):
    previous = contracts.describe(spec)
    changed = copy.deepcopy(spec)
    method = next(m for m in changed['methods'] if m['api_name'] == 'ahrs.get_gyro')
    method['lock'] = None
    current = contracts.describe(changed)
    with pytest.raises(ValueError, match='incompatible binding'):
        contracts.check_compatibility(previous, current)
    contracts.check_compatibility([entry for entry in previous if entry['id'] != method['id']], current)
    collision = copy.deepcopy(previous)
    collision[0]['contract']['revision'] = 2
    with pytest.raises(ValueError, match='collision with prior'):
        contracts.check_compatibility(previous, collision)


def test_frozen_probe_contract(spec):
    baseline = json.loads((bindings.ROOT / 'tests/compat/imports-v3.json').read_text())
    contracts.check_compatibility(baseline, contracts.describe(spec))


def test_pre_category_catalogue(spec):
    baseline = json.loads((bindings.ROOT / 'tests/compat/imports-v3-layout.json').read_text())
    timed = [entry for entry in baseline if isinstance(entry['contract']['policy'], dict) and
             entry['contract']['policy'].get('lock_timeout_ms') is not None]
    untimed = [entry for entry in baseline if entry not in timed]
    contracts.check_compatibility(untimed, contracts.describe(spec))
    with pytest.raises(ValueError, match='incompatible binding'):
        contracts.check_compatibility(timed, contracts.describe(spec))


def test_parameter_layout_contract(spec):
    before = hashes(spec)
    layout = next(entry['contract'] for entry in contracts.describe(spec)
                  if entry['contract']['name'] == 'Parameter.__layout')
    assert layout['arguments'] == []
    assert layout['result'] == dict(fields=[['size', 'u32-le'], ['alignment', 'u32-le']])
    # No current C++ size or alignment is part of the wire contract.
    assert layout['policy'] == dict(ownership='module-heap,trivial-drop', size_min=1, size_max=16777216,
                                    alignment='power-of-two,size-multiple', alignment_max=4096)
    changed = copy.deepcopy(spec)
    changed['parameter']['layout']['policy']['alignment_max'] = 8192
    after = hashes(changed)
    assert after['Parameter.__layout'] != before['Parameter.__layout']
    assert all(after[name] == value for name, value in before.items() if name != 'Parameter.__layout')
    # HEAD 3350e5465af used fixed two-pointer-word storage. Those imports must
    # not resolve to the variable-sized host protocol, even on today's layout.
    legacy = {'823ecd93d2844080', 'd83f55f38a0b265d', '355c3474cab8937a',
              'c8a7d2e7b16aa804', '4f1f4a8c3d4d4dfa', 'ac4b916ee8e4a87c'}
    assert not legacy.intersection(before.values())


def test_wire_encoding_affects_all_categories(spec):
    before = hashes(spec)
    changed = copy.deepcopy(spec)
    changed['encoding'] += '-changed'
    after = hashes(changed)
    assert all(after[name] != binding_hash for name, binding_hash in before.items())


def test_revision_configuration(spec, monkeypatch):
    before = hashes(spec)
    # Exercise the real configuration reader without editing the working tree.
    monkeypatch.setattr(contracts, 'read_revisions', lambda: {'Vector3f.length': 2})
    after = hashes(spec)
    assert after['Vector3f.length'] != before['Vector3f.length']
    assert after['Vector3f.dot'] == before['Vector3f.dot']


@pytest.mark.parametrize('revision', [0, -1, '2', True, 0x100000000])
def test_invalid_revision(spec, monkeypatch, revision):
    monkeypatch.setattr(contracts, 'read_revisions', lambda: {'Vector3f.length': revision})
    with pytest.raises(ValueError, match='invalid binding contract revision'):
        contracts.describe(spec)


def test_unknown_revision_and_alias(spec, monkeypatch):
    before = hashes(spec)
    monkeypatch.setattr(contracts, 'read_revisions', lambda: {'unknown.api': 1})
    with pytest.raises(ValueError, match='unknown public binding'):
        contracts.describe(spec)
    monkeypatch.setattr(contracts, 'read_revisions', lambda: {'ahrs.airspeed_estimate': 2})
    after = hashes(spec)
    assert after['ahrs.airspeed_estimate'] != before['ahrs.airspeed_estimate']
    assert after['ahrs.airspeed_EAS'] == before['ahrs.airspeed_EAS']


def test_revision_for_omitted_optional_section(spec, monkeypatch):
    spec = copy.deepcopy(spec)
    spec['crsf'] = None
    monkeypatch.setattr(contracts, 'read_revisions', lambda: {'crsf.pop_menu_event': 2})
    contracts.describe(spec)
