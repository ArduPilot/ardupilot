# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Freeze honest per-language method coverage, separately from host-call totals."""

import copy
import json

import bindings
import generate
import language_coverage
import pytest


@pytest.fixture(scope='module')
def generated():
    spec = bindings.specification()
    outputs = generate.generate(json.loads((generate.ROOT / 'generator/host_api.json').read_text()), spec)
    return spec, outputs


def test_languages_count_helpers_and_replacements_separately(generated):
    _, outputs = generated
    report = json.loads(outputs['generator/coverage.json'])
    assert report['schema'] == 2 and len(report['entries']) == 560
    assert report['native_host_calls'] == dict(rust=585, cpp=585, ada=585, fortran=585)
    assert report['summary']['rust'] == {
        'host-call': 533, 'local-helper': 10, 'native-operation': 8, 'replacement-only': 8, 'value-field': 1}
    assert report['summary']['cpp'] == {
        'host-call': 533, 'native-operation': 8, 'replacement-only': 18, 'value-field': 1}
    assert report['summary']['ada'] == {
        'host-call': 533, 'native-operation': 6, 'replacement-only': 20, 'value-field': 1}
    assert report['summary']['fortran'] == report['summary']['ada']
    rows = {(r['owner'], r['name']): r['languages'] for r in report['entries']}
    assert rows['AP_AHRS', 'get_roll']['rust']['status'] == 'replacement-only'
    assert rows['AP_AHRS', 'get_roll']['cpp']['status'] == 'replacement-only'
    assert rows['AP_HAL::CANFrame', 'isExtended']['rust']['status'] == 'local-helper'
    assert rows['AP_HAL::CANFrame', 'isExtended']['cpp']['status'] == 'replacement-only'
    assert rows['AP_Filesystem::stat_t', 'is_directory']['cpp']['status'] == 'value-field'
    assert rows['AP_Filesystem::stat_t', 'is_directory']['ada']['status'] == 'value-field'
    assert rows['uint32_t', '__tostring']['ada'] == dict(
        status='replacement-only', calls=['string.format_printf'],
        note="Ada 'Image may require runtime/secondary-stack support.")
    assert rows['Location', 'get_vector_from_origin_NEU']['rust']['status'] == 'local-helper'


def test_missing_ada_client_does_not_keep_coverage(generated):
    spec, outputs = generated
    changed = dict(outputs)
    changed['sdk/ada/ap_module-bindings.adb'] = changed['sdk/ada/ap_module-bindings.adb'].replace(
        'ap_module_ada_binding_ahrs_get_gyro', 'missing_ada_client')
    with pytest.raises(ValueError, match='Ada coverage and generated clients disagree'):
        language_coverage.report(spec, changed)


@pytest.mark.parametrize('source', ['sdk/fortran/ap_module_bindings.f90', 'sdk/fortran/bridge.cpp'])
def test_missing_fortran_client_does_not_keep_coverage(generated, source):
    spec, outputs = generated
    changed = dict(outputs)
    changed[source] = changed[source].replace(
        'ap_module_f90_binding_ahrs_get_gyro', 'missing_fortran_client')
    with pytest.raises(ValueError, match='Fortran coverage and generated clients disagree'):
        language_coverage.report(spec, changed)


def test_missing_rust_helper_does_not_keep_a_coverage_claim(generated):
    spec, outputs = generated
    changed = dict(outputs)
    changed['sdk/rust/src/can.rs'] = changed['sdk/rust/src/can.rs'].replace('pub fn is_extended(', 'pub fn renamed(')
    changed['sdk/rust/src/can.rs'] += '\nstruct Other;\nimpl Other {\n    pub fn is_extended(&self) -> bool { true }\n}\n'
    with pytest.raises(ValueError, match='local Rust helper missing.*CanFrame.is_extended'):
        language_coverage.report(spec, changed)


def test_unknown_successful_manual_adapter_needs_a_classification(generated):
    spec, outputs = generated
    changed = copy.deepcopy(spec)
    changed['coverage']['entries'].append(dict(owner='Future', name='new_manual', kind='singleton-manual', status='generated'))
    with pytest.raises(ValueError, match='unclassified language coverage: Future.new_manual'):
        language_coverage.report(changed, outputs)
