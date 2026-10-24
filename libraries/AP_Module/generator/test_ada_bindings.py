# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise every generated Ada C bridge and every C-compatible value layout."""

import json
import subprocess

import ada_bindings
import bindings
import generate
import naming
import pytest


@pytest.fixture(scope='module')
def description():
    return bindings.specification()


def leaves(item, path):
    if item.kind in ('scalar', 'enum'):
        return [path]
    if item.kind == 'array':
        return [field for i in range(item.count) for field in leaves(item.element, f'{path}.data[{i}]')]
    return [leaf for field, value, _ in item.fields for leaf in leaves(value, path + '.' + field)]


def test_every_ada_bridge_matches_cpp_packets(tmp_path, description):
    types, entries, plans = ada_bindings.describe(description)
    assert len(entries) == len(plans) == 585
    generated = tmp_path / 'generated'
    generate.write_outputs(generate.generate(json.loads((generate.ROOT / 'generator/host_api.json').read_text())), generated)
    lines = ['#include "sdk/ada/bridge.cpp"', '#include <stdio.h>', '#include <string.h>', '#include <assert.h>',
             'static unsigned current;', 'static bool called;', 'static int32_t status;',
             'static int32_t call(void *, uint32_t, const uint8_t *, uint32_t ni, uint8_t *out, uint32_t no) {',
             '    called = true; printf("%u %u %u\\n", current, ni, no);',
             '    for (uint32_t i = 0; i < no; i++) { out[i] = 0; } return status;', '}',
             'int main() { AP_ModuleHostApi api{}; api.binding_call = call; ContextBase base(api, nullptr);',
             'for (status = AP_MODULE_STATUS_UNAVAILABLE; status <= 0; status += 3) {']
    expected = []
    for index, (name, inputs, results, copybacks, sizes, _) in enumerate(plans):
        sizes = sizes or (sum(types.cpp.size(t) for _, t, _ in inputs), sum(types.cpp.size(t) for _, t in results))
        expected.append(f'{index} {sizes[0]} {sizes[1]}')
        lines += ['{', f'current = {index}; called = false;']
        args = ['&base']
        checks = []
        for i, (arg, kind, policy) in enumerate(inputs):
            if kind == 'StringView':
                args += ['"x"', '1']
            else:
                item = types.get(kind)
                lines += [f'{item.abi} a{i}{{}};']
                if policy and policy.get('out_only'):
                    lines += [f'memset(&a{i}, 0xA5, sizeof(a{i}));']
                    checks += [f'assert({field} == 0);' for field in leaves(item, f'a{i}')]
                elif any(arg == field for field, _ in copybacks):
                    lines += [field + ' = 1;' for field in leaves(item, f'a{i}')]
                    checks += [f'assert({field} == (status == 0 ? 0 : 1));' for field in leaves(item, f'a{i}')]
                args.append(f'&a{i}')
        for i, (_, kind) in enumerate(results):
            item = types.get(kind)
            lines += [f'{item.abi} out{i};', f'memset(&out{i}, 0xA5, sizeof(out{i}));']
            checks += [f'assert({field} == 0);' for field in leaves(item, f'out{i}')]
            args.append(f'&out{i}')
        lines += [f'if (ap_module_ada_binding_{naming.binding_name(name)}({", ".join(args)}) != status) {{ return 1; }}',
                  'if (!called) { return 2; }', *checks, '}']
    lines += ['}', '}']
    source = tmp_path / 'all.cpp'
    source.write_text('\n'.join(lines))
    executable = tmp_path / 'all'
    subprocess.run(['g++', '-std=c++17', '-fsanitize=address,undefined', '-g',
                    '-I' + str(generate.ROOT / 'sdk/cpp'), '-I' + str(generated), '-I' + str(generated / 'sdk/cpp'),
                    str(source), str(generated / 'sdk/cpp/imports.cpp'), '-o', str(executable)], check=True)
    assert subprocess.check_output([str(executable)], text=True).splitlines() == expected * 2


def test_ada_type_roundtrips(tmp_path, description):
    types, _, _ = ada_bindings.describe(description)
    generated = tmp_path / 'generated'
    generate.write_outputs(generate.generate(json.loads((generate.ROOT / 'generator/host_api.json').read_text())), generated)
    lines = ['#include "sdk/ada/bridge.cpp"', '#include <assert.h>', 'int main() {']

    def visit(item, path, statement):
        if item.kind in ('scalar', 'enum'):
            literal = '3.25f' if item.cpp == 'float' else '-5' if item.cpp.startswith('int') else '1'
            return [statement(path, literal)]
        if item.kind == 'array':
            return [line for i in range(item.count) for line in visit(item.element, f'{path}.data[{i}]', statement)]
        return [line for field, value, _ in item.fields for line in visit(value, path + '.' + field, statement)]

    for item in types.types.values():
        if item.kind in ('scalar', 'enum'):
            continue
        lines += ['{', item.abi + ' source{};', item.abi + ' destination{};', 'ap_module::' + item.cpp + ' value{};']
        lines += visit(item, 'source', lambda path, value: path + ' = ' + value + ';')
        lines += [f'assert(decode_{item.name}(source, value));', f'encode_{item.name}(value, destination);']
        lines += visit(item, 'destination', lambda path, value: f'assert({path} == {value});')
        lines += ['}']
    lines += ['}']
    source = tmp_path / 'roundtrip.cpp'
    source.write_text('\n'.join(lines))
    executable = tmp_path / 'roundtrip'
    subprocess.run(['g++', '-std=c++17', '-fsanitize=address,undefined', '-g',
                    '-I' + str(generate.ROOT / 'sdk/cpp'), '-I' + str(generated), '-I' + str(generated / 'sdk/cpp'),
                    str(source), str(generated / 'sdk/cpp/imports.cpp'), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


def test_negative_decode_and_string_validation(tmp_path):
    generated = tmp_path / 'generated'
    generate.write_outputs(generate.generate(json.loads((generate.ROOT / 'generator/host_api.json').read_text())), generated)
    source = tmp_path / 'invalid.cpp'
    source.write_text('''#include "sdk/ada/bridge.cpp"
#include <assert.h>
static unsigned calls;
static int32_t call(void *, uint32_t, const uint8_t *input, uint32_t ni, uint8_t *, uint32_t) {
    calls++;
    assert(ni == 260 && input[0] == 99);
    return AP_MODULE_STATUS_INVALID;
}
int main() {
    AP_ModuleHostApi api{}; api.binding_call = call; ContextBase base(api, nullptr);
    uint8_t camera = 0, bad = 2, output = 0xA5;
    assert(ap_module_ada_binding_camera_record_video(&base, &camera, &bad, &output) == AP_MODULE_STATUS_INVALID);
    assert(calls == 0 && output == 0);
    Ada_Location location{}; location.relative_alt = 2; location.alt = 123;
    float offset = 0;
    assert(ap_module_ada_binding_location_offset(&base, &location, &offset, &offset) == AP_MODULE_STATUS_INVALID);
    assert(calls == 0 && location.relative_alt == 2 && location.alt == 123);
    Ada_Optional_bool optional{0, 2}; Optional<bool> value{};
    assert(decode_Optional_bool(optional, value) && !value.present);
    optional.present = 1;
    assert(!decode_Optional_bool(optional, value));
    int32_t severity = 6;
    const char nul[] = {'a', 0, 'b'};
    assert(ap_module_ada_binding_gcs_send_text(&base, &severity, nul, 3) == AP_MODULE_STATUS_INVALID);
    assert(ap_module_ada_binding_gcs_send_text(&base, &severity, nullptr, 1) == AP_MODULE_STATUS_INVALID);
    assert(calls == 0);
    // Unknown enum bits are forwarded intact; firmware owns semantic ranges.
    severity = 99;
    assert(ap_module_ada_binding_gcs_send_text(&base, &severity, "x", 1) == AP_MODULE_STATUS_INVALID);
    assert(calls == 1);
}
''')
    executable = tmp_path / 'invalid'
    subprocess.run(['g++', '-std=c++17', '-fsanitize=address,undefined', '-g',
                    '-I' + str(generate.ROOT / 'sdk/cpp'), '-I' + str(generated), '-I' + str(generated / 'sdk/cpp'),
                    str(source), str(generated / 'sdk/cpp/imports.cpp'), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


def test_ada_identifiers_and_enum_values(description):
    types, _, _ = ada_bindings.describe(description)
    assert ada_bindings.identifier('__layout') == 'layout'
    assert ada_bindings.identifier('begin') == 'begin_Call'
    assert ada_bindings.identifier('some__name') == 'some_name'
    assert types.cpp.enum_values['MavSeverity']['INFO'] == 6
    with pytest.raises(ValueError, match='empty Ada identifier'):
        ada_bindings.identifier('___')


def test_unknown_ada_type_is_not_a_raw_buffer_escape_hatch(description):
    types, _, _ = ada_bindings.describe(description)
    with pytest.raises(KeyError):
        types.get('Unknown_Contract')
