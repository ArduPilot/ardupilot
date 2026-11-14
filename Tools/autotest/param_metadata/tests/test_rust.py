# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Rust metadata discovery, tag parsing and the actual vehicle emitters."""

import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

from pathlib import Path

from Tools.autotest.param_metadata.rust import parameter_matches, parameter_paths

ROOT = Path(__file__).resolve().parents[4]


def test_discovery_includes_nested_applets_drivers_and_examples(tmp_path):
    for name in ('applets/demo/params.rs', 'drivers/device.rs', 'examples/demo/main.rs',
                 'sdk/src/lib.rs', 'tests/fixture.rs', 'applets/demo/readme.md'):
        p = tmp_path / 'libraries/AP_Module' / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.touch()
    assert parameter_paths(tmp_path) == [
        'AP_Module/applets/demo/params.rs', 'AP_Module/drivers/device.rs', 'AP_Module/examples/demo/main.rs']


def test_rust_metadata_supports_tags_lowercase_variants_and_eof():
    text = '''ap_rust_sdk::param_table! {
    pub struct Params, enum P;
    key = 100, prefix = "TEST_";
    // @Param{Plane, Copter}: TEST_RATE
    // @DisplayName: Test rate
    // @Description: An annotated parameter
    // @Values{Plane}: 0:Off,1:On
    // @User: Advanced
    rate = 63, "RATE", f32: 1.0;
}
// @Param: FINAL
// @Description: Last metadata block'''
    matches = parameter_matches(text)
    assert [(m[0], m[1]) for m in matches] == [('Plane, Copter', 'TEST_RATE'), ('', 'FINAL')]
    assert '// @Values{Plane}: 0:Off,1:On' in matches[0][2]
    assert 'rate = ' not in matches[0][2]


def check_table_documentation(text, source):
    # Match the macro first so an unsupported declaration cannot silently escape
    # validation. Reserved size is optional and does not affect parameter names.
    starts = re.findall(r'\bparam_table!\s*\{', text)
    tables = re.findall(r'\bparam_table!\s*\{(.*?)^}', text, re.MULTILINE | re.DOTALL)
    assert len(tables) == len(starts), source
    count = 0
    for table in tables:
        prefix = re.search(r'\bprefix\s*=\s*"([^"\n]*)"\s*;', table)
        assert prefix is not None, source
        entries = re.findall(r'^\s*\w+\s*=\s*[^,\n]+,\s*"([^"\n]+)"\s*,\s*f32\s*:',
                             table, re.MULTILINE)
        assert entries, source
        names = [m[1] for m in parameter_matches(table)]
        assert names == [prefix[1] + name for name in entries], source
        count += len(entries)
    return count


@pytest.mark.parametrize('size', ['', '    size = 63;\n'])
@pytest.mark.parametrize('annotation', ['TEST_VALUE', 'WRONG_NAME', None])
def test_documentation_check_covers_optional_size(size, annotation):
    docs = '' if annotation is None else f'    // @Param: {annotation}\n    // @Description: Test parameter\n'
    text = ('ap_rust_sdk::param_table! {\n'
            '    struct Params, enum P;\n'
            '    key = 100, prefix = "TEST_";\n'
            f'{size}{docs}'
            '    Value = 1, "VALUE", f32: 0.0;\n'
            '}\n')
    if annotation == 'TEST_VALUE':
        assert check_table_documentation(text, 'fixture') == 1
    else:
        with pytest.raises(AssertionError, match='fixture'):
            check_table_documentation(text, 'fixture')


def test_documentation_check_rejects_unparsed_table():
    text = ('ap_rust_sdk::param_table! {\n'
            '    struct Params, enum P;\n'
            '    key = 100, prefix = PREFIX;\n'
            '    Value = 1, "VALUE", f32: 0.0;\n'
            '}\n')
    with pytest.raises(AssertionError, match='fixture'):
        check_table_documentation(text, 'fixture')


def test_all_rust_declarations_have_matching_documentation():
    count = 0
    for relative in parameter_paths(ROOT):
        text = (ROOT / 'libraries' / relative).read_text()
        count += check_table_documentation(text, relative)
    assert count > 150


def test_vehicle_xml_keeps_lua_and_documents_rust(tmp_path):
    subprocess.run([sys.executable, str(ROOT / 'Tools/autotest/param_metadata/param_parse.py'),
                    '--vehicle', 'ArduPlane', '--format', 'xml'], cwd=tmp_path, check=True)
    tree = ET.parse(tmp_path / 'apm.pdef.xml')
    groups = {group.attrib['name']: group for group in tree.findall('./libraries/parameters')}
    lua = {p.attrib['name'] for p in groups['Lua Script'].findall('param')}
    rust = {p.attrib['name'] for p in groups['Rust Module'].findall('param')}
    assert {'WEB_ENABLE', 'QUIK_ENABLE', 'DCS_NUM_CIRCUITS'} <= lua & rust
    assert {'AEROM_LANG', 'TRIK11_ARG4', 'DCS9_CIRCUIT_ID', 'SINE_FREQ',
            'FOLLP_ALT_OVR', 'PLND_DIST_CUTOFF', 'ARM_C_RTL_ALT'} <= rust
    assert len(rust) == sum(len(parameter_matches((ROOT / 'libraries' / p).read_text()))
                            for p in parameter_paths(ROOT))
