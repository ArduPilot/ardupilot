# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Check local C mirrors independently of the Fortran compiler's ABI tests."""

import json
import subprocess

import bindings
import fortran_bindings
import generate
import pytest


def visit(item, path):
    if item.kind in ('scalar', 'enum'):
        value = 'true' if item.cpp == 'bool' else '3.25f' if item.cpp == 'float' else '-1'
        return [(path, value)]
    if item.kind == 'array':
        return [leaf for i in range(item.count) for leaf in visit(item.element, f'{path}.data[{i}]')]
    return [leaf for field, value, _ in item.fields for leaf in visit(value, path + '.' + field)]


def test_fortran_c_value_roundtrips(tmp_path):
    types, _, _ = fortran_bindings.describe(bindings.specification())
    generated = tmp_path / 'generated'
    generate.write_outputs(generate.generate(json.loads((generate.ROOT / 'generator/host_api.json').read_text())), generated)
    lines = ['#include "sdk/fortran/bridge.cpp"', '#include <assert.h>', 'int main() {']
    for item in types.types.values():
        if item.kind in ('scalar', 'enum'):
            continue
        lines += ['{', item.abi + ' source{};', item.abi + ' destination{};', 'ap_module::' + item.cpp + ' value{};']
        lines += [field + ' = ' + value + ';' for field, value in visit(item, 'source')]
        lines += [f'assert(decode_{item.name}(source, value));', f'encode_{item.name}(value, destination);']
        lines += [f'assert({field} == {value});' for field, value in visit(item, 'destination')]
        lines += ['}']
    lines += ['}', '']
    source = tmp_path / 'roundtrip.cpp'
    source.write_text('\n'.join(lines))
    executable = tmp_path / 'roundtrip'
    subprocess.run(['g++', '-std=c++17', '-fsanitize=address,undefined', '-g',
                    '-I' + str(generate.ROOT / 'sdk/cpp'), '-I' + str(generated), '-I' + str(generated / 'sdk/cpp'),
                    str(source), str(generated / 'sdk/cpp/imports.cpp'), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


def test_fortran_names_and_unknown_types():
    types, _, _ = fortran_bindings.describe(bindings.specification())
    name = fortran_bindings.identifier('optional_' + 'long_name_' * 10)
    assert len(name) == 63 and name == fortran_bindings.identifier('optional_' + 'long_name_' * 10)
    assert name != fortran_bindings.identifier('optional_' + 'long_name_' * 11)
    with pytest.raises(ValueError, match='invalid Fortran identifier'):
        fortran_bindings.identifier('123')
    with pytest.raises(KeyError):
        types.get('Unknown_Contract')
