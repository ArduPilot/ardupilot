#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Run the H743 native Rust manager with ROMFS RX executing in place."""

import os
import re
import shutil
import subprocess
import zlib

from pathlib import Path

import pack
import pytest

from elftools.elf.elffile import ELFFile
from test_threads import ROOT  # noqa: F401
from test_threads import gen_board  # noqa: F401
from test_threads import renode  # noqa: F401
from test_threads import renode_data  # noqa: F401
from test_threads import run  # noqa: F401

import build

CHECKSUM_MASK = 0x007FFFFF


def validate_prelinked_romfs(executable, build_directory, module_path):
    header = (build_directory / 'ap_romfs_embedded.h').read_text()
    match = re.search(r'\{\s*"modules/romfs\.apmod"[^\n]*\b(ap_romfs_[0-9]+)\b', header)
    assert match is not None, 'ROMFS module is absent from generated header'
    module = pack.unpack(module_path.read_bytes())
    with executable.open('rb') as stream:
        elf = ELFFile(stream)
        symbol = elf.get_section_by_name('.symtab').get_symbol_by_name(match.group(1))[0]
        base = symbol['st_value']
        segment = next((item for item in elf.iter_segments()
                        if item['p_type'] == 'PT_LOAD' and item['p_vaddr'] <= base <
                        item['p_vaddr'] + item['p_filesz']), None)
        assert segment is not None and segment['p_flags'] & 1, 'ROMFS module is not in executable flash'
        assert 0x08000000 <= base < 0x09000000 or 0x90000000 <= base < 0xA0000000
        for offset, target in module['fixups']:
            if offset >= module['rx_size']:
                continue
            address = base + pack.RUNTIME_HEADER.size + offset
            stream.seek(segment['p_offset'] + address - segment['p_vaddr'])
            actual = int.from_bytes(stream.read(module['width']), 'little')
            expected = base + pack.RUNTIME_HEADER.size + target
            assert actual == expected, (offset, target, hex(actual), hex(expected))


@pytest.mark.parametrize('sd_language', ['rust', 'cpp', 'fortran'])
@pytest.mark.parametrize('cpu', ['cortex-m4', 'cortex-m7'])
def test_chibios_romfs_runtime(tmp_path, renode, monkeypatch, sd_language, cpu):  # noqa: F811
    board = os.environ.get('OWNED_THREAD_BOARD', 'MatekH743')
    build_root = Path(os.environ.get('AP_MODULE_ROMFS_BUILD_ROOT', ROOT / 'build/romfs-runtime'))
    executable = build_root / board / 'examples/ModuleRuntime'
    embedded = ROOT / 'build/modules/romfs_runtime/romfs.apmod'
    if not executable.is_file() or not embedded.is_file():
        pytest.skip('Run Tools/modules/build_romfs_runtime.py first')
    if not shutil.which('mcopy') or not shutil.which('mmd'):
        pytest.skip('Requires mtools')

    validate_prelinked_romfs(executable, build_root / board, embedded)
    names = ['result', 'cycles', 'before', 'after', 'loaded_checksum',
             'running_checksum', 'romfs_rx', 'romfs_rw', 'sd_rx', 'sd_rw', 'stack_free', 'stack_total']
    with executable.open('rb') as stream:
        symbols = ELFFile(stream).get_section_by_name('.symtab')
        addresses = [symbols.get_symbol_by_name(f'rust_runtime_test_{name}')[0]['st_value'] for name in names]

    monkeypatch.setenv('AP_MODULE_LIFECYCLE', 'cleanup')
    sd_module = tmp_path / 'sd.apmod'
    sources = {'rust': 'libraries/AP_Module/tests/modules/lifecycle.rs', 'cpp': 'Tools/modules/cpp/probe.cpp',
               'fortran': 'Tools/modules/fortran/probe.F90'}
    source = ROOT / sources[sd_language]
    if sd_language == 'fortran':
        monkeypatch.setenv('FC', os.environ.get('ARM_FC', 'arm-linux-gnueabihf-gfortran'))
    build.build(source, sd_module,
                'thumbv7em-none-eabihf', 8193, cpu=cpu)
    expected_checksum = (zlib.crc32(embedded.read_bytes()) ^ zlib.crc32(sd_module.read_bytes())) & CHECKSUM_MASK

    generated = gen_board.generate(ROOT, board, tmp_path / 'generated', uart_port=0, state_dir=tmp_path)
    data = renode_data.ensure_mcu_data(generated['mcu_type'])
    sdcard = run.make_fat_image(tmp_path / 'sdcard.img', 16 * 1024 * 1024, fat_bits=16)
    subprocess.run(['mmd', '-i', str(sdcard), '::/fixtures'], check=True)
    subprocess.run(['mcopy', '-i', str(sdcard), str(sd_module), '::/fixtures/sd.apmod'], check=True)
    subprocess.run(['mcopy', '-i', str(sdcard), str(sd_module), '::/fixtures/shadow.apmod'], check=True)

    output = renode('''
$elf=@%s
$mcu_svd=@%s
$sdcard=@%s
$vector_base=0x%X
include @%s
emulation SetGlobalAdvanceImmediately true
emulation RunFor "60"
%s
''' % (executable.resolve(), data['svd'], sdcard, generated['app_base'], generated['resc'],
       '\n'.join(f'sysbus ReadDoubleWord 0x{address:X}' for address in addresses)), timeout=180)
    values = [int(value, 16) for value in re.findall(r'^0x[0-9A-Fa-f]{8}$', output, re.M)]
    (tmp_path / 'renode.log').write_text(output)
    assert len(values) == len(names), output
    result = dict(zip(names, values))
    print(f'{board} ROMFS: {result}', flush=True)
    assert result['result'] == 0x600DF00D, (result, output)
    assert result['cycles'] == 10 and result['before'] == result['after'], result
    assert result['stack_free'] > 1024 and 4096 <= result['stack_total'] < 4096 + 1024, result
    assert expected_checksum != 0, result
    assert result['loaded_checksum'] == expected_checksum, result
    assert result['running_checksum'] == expected_checksum, result
    assert result['romfs_rx'] == 0 and result['romfs_rw'] > 0, result
    assert result['sd_rx'] > 0 and result['sd_rw'] > 0, result
    readback = tmp_path / 'readback.apmod'
    subprocess.run(['mcopy', '-i', str(sdcard), '::/fixtures/readback.apmod', str(readback)], check=True)
    assert readback.read_bytes() == embedded.read_bytes(), 'ROMFS filesystem readback must restore canonical fixups'
