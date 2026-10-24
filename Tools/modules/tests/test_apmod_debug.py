# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Debug metadata stays on the host and follows independently relocated segments."""

import io

import apmod_debug
import pack
import pytest

from elftools.elf.elffile import ELFFile

import build


@pytest.fixture(scope='module', params=list(build.TARGETS))
def symbols(tmp_path_factory, request):
    path = tmp_path_factory.mktemp('apmod-debug') / 'debug.apmod'
    source = build.SDK_ROOT / 'tests/modules/debug.rs'
    build.build(source, path, request.param, 16384, debug=True, opt_level='1')
    return apmod_debug.ModuleSymbols(path)


def test_debug_metadata_is_host_only(symbols):
    elf = ELFFile(io.BytesIO(symbols.elf_data))
    assert elf.get_section_by_name('.debug_info')['sh_size'] > 1000
    assert elf.get_section_by_name('.debug_gdb_scripts') is None
    assert elf.get_section_by_name('.debug_frame') is not None
    assert all(not s['sh_flags'] & 2 for s in elf.iter_sections() if s.name.startswith('.debug'))
    assert pack.pack_elf(symbols.elf_data, symbols.module['contract']) == symbols.data


@pytest.mark.parametrize('rx', [0x24000120, 0x08040000])
def test_split_symbol_placement(symbols, rx):
    rw = 0x20001000 if symbols.module['machine'] == 40 else rx + symbols.module['rx_size']
    addresses = symbols.addresses(rx, rw)
    assert addresses['.text'] == rx
    assert addresses['.data'] == rw
    assert '-s .data 0x' in symbols.command('/tmp/module symbols.elf', rx, rw)
    debug_data = symbols.debug_elf(rx, rw)
    assert pack.pack_elf(debug_data, symbols.module['contract']) == symbols.data
    patched = ELFFile(io.BytesIO(debug_data))
    if symbols.module['machine'] == 40:
        globals_found = []
        for unit in patched.get_dwarf_info(relocate_dwarf_sections=False).iter_CUs():
            for die in unit.iter_DIEs():
                name = die.attributes.get('DW_AT_name')
                if name and name.value == b'APMOD_DEBUG_COUNT':
                    expr = bytes(die.attributes['DW_AT_location'].value)
                    assert expr[0] == 0x0c and expr[5:] == b'\x96' * 3
                    assert rw <= int.from_bytes(expr[1:5], 'little') < rw + symbols.module['memory_size']
                    globals_found.append(name.value)
        assert globals_found


def test_mismatched_elf_rejected(symbols, tmp_path):
    path = tmp_path / 'bad.apmod'
    path.write_bytes(symbols.data)
    elf = ELFFile(io.BytesIO(symbols.elf_data))
    modified = bytearray(symbols.elf_data)
    modified[elf.get_section_by_name('.text')['sh_offset']] ^= 1
    path.with_suffix('.elf').write_bytes(modified)
    with pytest.raises(ValueError, match='does not match'):
        apmod_debug.ModuleSymbols(path)


@pytest.mark.parametrize('path', ['bad\nquit', 'bad\x00file', 'bad\rfile'])
def test_reject_debugger_command_injection(path):
    with pytest.raises(ValueError):
        apmod_debug.quote(path)
