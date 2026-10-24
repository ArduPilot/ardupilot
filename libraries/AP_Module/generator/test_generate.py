# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later

import copy
import json
import re

import generate
import pytest


def specification():
    return json.loads((generate.ROOT / 'generator/host_api.json').read_text())


def test_generated_files_match(tmp_path):
    outputs = generate.generate(specification())
    generate.write_outputs(outputs, tmp_path)
    for name, content in outputs.items():
        assert (tmp_path / name).read_text() == content
    generate.write_outputs(outputs, tmp_path, check=True)
    assert outputs == generate.generate(specification())


def test_generated_binding_names_are_readable():
    outputs = generate.generate(specification())
    cpp = outputs['Bindings_generated.cpp']
    rust = outputs['sdk/rust/src/bindings.rs']
    ids = outputs['BindingIds.h']
    assert 'binding_ahrs_get_gyro' in cpp
    assert 'case BindingId::AHRS_GET_GYRO:' in cpp
    assert 'binding_libm_acosf' in cpp
    assert 'binding_string_strtof' in cpp
    assert '#include <stdlib.h>' in cpp
    assert 'AP_Common/strtod.h' not in cpp
    assert re.search(r'\bbinding_(?:\d+|[A-Za-z_]+_\d+)\b', cpp) is None
    assert re.search(r'\bBINDING_\d+\b', rust) is None
    assert re.search(r'(?m)^\s*case \d+:', cpp) is None
    assert 'AHRS_GET_GYRO = 44U' in ids
    assert 'pub fn ahrs(&self)' in rust
    assert 'pub fn get_gyro(&self)' in rust
    assert 'pub fn ahrs_get_gyro(' not in rust


def test_generated_adapters_trust_wrapper_buffers():
    cpp = generate.generate(specification())['Bindings_generated.cpp']
    # Includes the specialised adapters and the inline Parameter dispatch cases.
    # Value/content checks remain; fixed packet sizes and pointer validity are
    # established by the generated wrapper, not rechecked in each adapter.
    assert re.search(r'\b(?:input_size|output_size)\s*[!=<>]=?', cpp) is None
    assert re.search(r'\b(?:input|output)\s*[!=]=\s*nullptr', cpp) is None


def test_fingerprint_changes_with_contract():
    original = specification()
    expected = generate.generate(original)['sdk/abi.json']
    for change in ('version', 'status', 'signature', 'order'):
        spec = copy.deepcopy(original)
        if change == 'version':
            spec['version'] += 1
        elif change == 'status':
            spec['status']['STOPPED'] += 1
        elif change == 'signature':
            spec['functions'][0]['args'][1][1] = 'u64'
        else:
            spec['functions'].reverse()
        assert generate.generate(spec)['sdk/abi.json'] != expected


def test_appended_host_callback_preserves_core_identity():
    spec = specification()
    before = generate.generate(spec)['sdk/abi.json']
    spec['functions'].append(dict(name='future', return_='void', args=[['context', 'pointer']]))
    spec['functions'][-1]['return'] = spec['functions'][-1].pop('return_')
    assert generate.generate(spec)['sdk/abi.json'] == before


@pytest.mark.parametrize('bad', ['duplicate', 'void_argument', 'missing_context', 'return_type', 'identifier'])
def test_reject_invalid_specification(bad):
    spec = specification()
    if bad == 'duplicate':
        spec['functions'].append(copy.deepcopy(spec['functions'][0]))
    elif bad == 'void_argument':
        spec['functions'][0]['args'][1][1] = 'void'
    elif bad == 'missing_context':
        spec['functions'][0]['args'].pop(0)
    elif bad == 'return_type':
        spec['functions'][0]['return'] = 'String'
    else:
        spec['functions'][0]['name'] = 'not a name'
    with pytest.raises(ValueError):
        generate.generate(spec)
