# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Linker GC, load-time import contracts and malformed container checks."""

import hashlib
import io
import shutil
import struct
import subprocess
import zlib

from pathlib import Path

import apmod_info
import pack
import pytest

from elftools.elf.elffile import ELFFile
from elftools.elf.relocation import RelocationSection
from test_modules import upgrade_frozen_container

import build


@pytest.mark.parametrize('target,sha256', [
    ('thumbv7em', 'c4e56da5111ea5ceb3718c65009dc649ca720c88f549155e39b504cf4b6051e3'),
    ('x86_64', '14b7350c67c967ce261c4be7f57d9a24cace5232e1775c6c9fa725577a0df7fb'),
])
def test_frozen_probe_identity(target, sha256):
    root = Path(__file__).resolve().parents[3]
    data = bytes.fromhex((root / f'libraries/AP_Module/tests/compat/{target}.apmod.hex').read_text())
    assert hashlib.sha256(data).hexdigest() == sha256
    assert {item[2] for item in pack.unpack(upgrade_frozen_container(data))['imports']} == {
        'Vector3f.length', 'string.format_printf'}


@pytest.fixture(params=list(build.TARGETS))
def one_import(tmp_path, request):
    source = tmp_path / 'one.rs'
    source.write_text('''#![no_std]
#![no_main]
use ap_rust_sdk::{Context, Result};
ap_rust_sdk::module!(run);
fn run(context: &Context<'_>) -> Result { let _ = context.arming().is_armed()?; Ok(()) }
#[allow(dead_code)]
fn unused(context: &Context<'_>) -> Result { let _ = context.ahrs().get_gyro()?; Ok(()) }
''')
    first = build.build(source, tmp_path / 'one.apmod', request.param, 8192)
    second = build.build(source, tmp_path / 'two.apmod', request.param, 8192, base=0x24123000)
    assert first.read_bytes() == second.read_bytes()
    return first.read_bytes()


def test_only_live_imports_survive(one_import):
    info = pack.unpack(one_import)
    assert {name for _, _, name in info['imports']} == {'arming.is_armed', 'string.format_printf'}
    binding_hash, offset, name = next(entry for entry in info['imports'] if entry[2] == 'arming.is_armed')
    assert binding_hash != 0 and offset >= info['rx_size']
    assert name == 'arming.is_armed'
    assert info['image'][offset:offset + 4] == b'\xff' * 4


def test_apmod_info(one_import):
    info = pack.unpack(one_import)
    binding_hash, offset, name = info['imports'][0]
    output = apmod_info.format_module(Path('one.apmod'), one_import)
    assert 'Module: one.apmod' in output
    assert 'Format: APMOD07 (version 7)' in output
    assert f'Pointer width: {info["width"] * 8} bits' in output
    assert f'Initialized image: {len(info["image"])} bytes' in output
    assert f'Memory image: {info["memory_size"]} bytes' in output
    assert f'RX image: {info["rx_size"]} bytes' in output
    assert f'Imports: {len(info["imports"])}' in output
    assert f'{binding_hash:016x}  0x{offset:08x}  {name}' in output
    assert output.endswith('Validation: OK')


@pytest.mark.parametrize('target', list(build.TARGETS))
def test_zero_imports(tmp_path, target):
    source = tmp_path / 'zero.rs'
    source.write_text('''#![no_std]
#![no_main]
use ap_rust_sdk::{Context, Result};
ap_rust_sdk::module!(run);
fn run(_: &Context<'_>) -> Result { Ok(()) }
''')
    image = build.build(source, tmp_path / 'zero.apmod', target, 8192)
    assert pack.unpack(image.read_bytes())['imports'] == []


def crc(data):
    data[84:88] = bytes(4)
    data[84:88] = struct.pack('<I', zlib.crc32(data))
    return data


def test_invalid_import_metadata(one_import):
    info = pack.unpack(one_import)
    records = pack.RUNTIME_HEADER.size + len(info['image']) + len(info['fixups']) * pack.FIXUP.size
    _, slot, _ = info['imports'][0]
    for field, value in [(60, 0), (60, 4097), (60, 0xffffffff),
                         (records + 8, 0), (records + 8, slot + 1),
                         (records + 8, len(info['image'])), (records + 8, 0xfffffffc),
                         (records + 12, 1), (records + 12, 0xffffffff),
                         (pack.RUNTIME_HEADER.size + slot, 0)]:
        bad = bytearray(one_import)
        struct.pack_into('<I', bad, field, value)
        with pytest.raises(ValueError):
            pack.unpack(crc(bad))
    bad = bytearray(one_import)
    bad[records] ^= 1
    with pytest.raises(ValueError, match='checksum'):
        pack.unpack(bad)
    bad = bytearray(one_import)
    bad[records + len(info['imports']) * pack.IMPORT.size] = 0
    with pytest.raises(ValueError, match='import name'):
        pack.unpack(crc(bad))
    with pytest.raises(ValueError):
        pack.unpack(one_import[:-1])
    with pytest.raises(ValueError):
        pack.unpack(one_import + b'\0')


def test_stripped_elf_rejected(one_import, tmp_path):
    objcopy = {40: 'arm-none-eabi-objcopy', 62: 'objcopy', 183: 'aarch64-linux-gnu-objcopy'}[
        pack.unpack(one_import)['machine']]
    if shutil.which(objcopy) is None:
        pytest.skip(f'{objcopy} not found')
    stripped = tmp_path / 'stripped.elf'
    subprocess.run([objcopy, '--strip-all', str(tmp_path / 'one.elf'), str(stripped)], check=True)
    with pytest.raises(ValueError, match='unstripped ELF symbol table required'):
        pack.pack_elf(stripped.read_bytes(), pack.unpack(one_import)['contract'])


def test_import_in_nobits_rejected(one_import, tmp_path):
    data = bytearray((tmp_path / 'one.elf').read_bytes())
    elf = ELFFile(io.BytesIO(data))
    index = next(index for index, section in enumerate(elf.iter_sections()) if section.name == '.data')
    # An import symbol may only identify initialized storage.
    struct.pack_into('<I', data, elf['e_shoff'] + index * elf['e_shentsize'] + 4, 8)
    with pytest.raises(ValueError, match='invalid import symbol'):
        pack.pack_elf(data, pack.unpack(one_import)['contract'])


def test_referenced_rust_personality_rejected(one_import, tmp_path):
    data = bytearray((tmp_path / 'one.elf').read_bytes())
    elf = ELFFile(io.BytesIO(data))
    symbols = elf.get_section_by_name('.symtab')
    references = {reloc['r_info_sym'] for section in elf.iter_sections()
                  if isinstance(section, RelocationSection) and elf.get_section(section['sh_info'])['sh_flags'] & 2
                  for reloc in section.iter_relocations() if reloc['r_info_type'] != 0}
    index, symbol = next((index, symbol) for index, symbol in enumerate(symbols.iter_symbols())
                         if index in references and symbol.name.startswith('__ap_module_import_'))
    name = b'rust_eh_personality\0'
    strings = elf.get_section(symbols['sh_link'])
    offset = strings['sh_offset'] + symbol['st_name']
    data[offset:offset + len(name)] = name
    entry = symbol.entry
    entry['st_shndx'] = 'SHN_UNDEF'
    offset = symbols['sh_offset'] + index * symbols['sh_entsize']
    data[offset:offset + symbols['sh_entsize']] = elf.structs.Elf_Sym.build(entry)
    with pytest.raises(ValueError, match='undefined symbol rust_eh_personality|non-image relocation symbol'):
        pack.pack_elf(data, pack.unpack(one_import)['contract'])
