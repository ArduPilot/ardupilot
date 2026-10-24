# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Run the native module manager on ChibiOS, including a spinning worker."""

import os
import re
import shutil
import subprocess

from pathlib import Path

import pytest

from elftools.elf.elffile import ELFFile
from test_threads import ROOT  # noqa: F401
from test_threads import gen_board  # noqa: F401
from test_threads import renode  # noqa: F401
from test_threads import renode_data  # noqa: F401
from test_threads import run  # noqa: F401

import build


@pytest.mark.parametrize('count', [12, 32])
def test_chibios_many_modules(tmp_path, renode, count):  # noqa: F811
    board = os.environ.get('OWNED_THREAD_BOARD', 'MatekH743')
    build_root = Path(os.environ.get('AP_MODULE_BUILD_ROOT', ROOT / 'build'))
    executable = build_root / board / 'examples/ModuleRuntime'
    if not executable.is_file() or not shutil.which('mcopy') or not shutil.which('mmd'):
        pytest.skip('Requires the ModuleRuntime example and mtools')
    names = ['result', 'cycles', 'before', 'after', 'empty_before', 'empty_after', 'stack_free', 'stack_total']
    with executable.open('rb') as stream:
        symbols = ELFFile(stream).get_section_by_name('.symtab')
        addresses = [symbols.get_symbol_by_name(f'rust_runtime_test_{name}')[0]['st_value'] for name in names]
        generations = symbols.get_symbol_by_name('rust_runtime_test_generation_free')[0]['st_value']
        addresses += list(range(generations, generations + 21 * 4, 4))
        error = symbols.get_symbol_by_name('rust_runtime_test_error')[0]['st_value']
        addresses += list(range(error, error + 100, 4))
    generated = gen_board.generate(ROOT, board, tmp_path / 'generated', uart_port=0, state_dir=tmp_path)
    data = renode_data.ensure_mcu_data(generated['mcu_type'])
    sdcard = run.make_fat_image(tmp_path / 'sdcard.img', 16 * 1024 * 1024, fat_bits=16)
    subprocess.run(['mmd', '-i', str(sdcard), '::/fixtures'], check=True)
    image = build.build(ROOT / 'libraries/AP_Module/tests/modules/many_modules.rs', tmp_path / 'many.apmod',
                        'thumbv7em-none-eabihf', 1024, cpu='cortex-m7', heap_size=256)
    fixture = tmp_path / 'many.count'
    fixture.write_bytes(bytes([count]))
    for path in (image, fixture):
        subprocess.run(['mcopy', '-i', str(sdcard), str(path), '::/fixtures/' + path.name], check=True)
    output = renode('''
$elf=@%s
$mcu_svd=@%s
$sdcard=@%s
$vector_base=0x%X
include @%s
emulation SetGlobalAdvanceImmediately true
emulation RunFor "180"
%s
''' % (executable.resolve(), data['svd'], sdcard, generated['app_base'], generated['resc'],
       '\n'.join(f'sysbus ReadDoubleWord 0x{address:X}' for address in addresses)), timeout=360)
    (tmp_path / 'renode.log').write_text(output)
    values = [int(value, 16) for value in re.findall(r'^0x[0-9A-Fa-f]{8}$', output, re.M)]
    assert len(values) == len(addresses), output
    result = dict(zip(names, values))
    result['error'] = b''.join(value.to_bytes(4, 'little') for value in values[-25:]).split(b'\0')[0].decode()
    print(f'{board} {count} modules: {result}', flush=True)
    assert result['result'] == 0x600DF00D and result['cycles'] == 20, (result, output)
    assert values[len(names):len(names) + 21] == [result['before']] * 21, result
    assert result['before'] == result['after'], result
    assert result['empty_before'] == result['empty_after'], result
    assert result['stack_free'] > 1024 and 4096 <= result['stack_total'] < 5120, result


@pytest.mark.parametrize('ending', ['never', 'panic'])
@pytest.mark.parametrize('priority_max', [2, 5])
def test_chibios_runtime_manager(tmp_path, renode, monkeypatch, ending, priority_max):  # noqa: F811
    board = os.environ.get('OWNED_THREAD_BOARD', 'MatekH743')
    build_root = Path(os.environ.get('AP_MODULE_BUILD_ROOT', ROOT / 'build'))
    executable = build_root / board / 'examples/ModuleRuntime'
    if not executable.is_file():
        pytest.skip('Build examples/ModuleRuntime with --enable-modules')
    if not shutil.which('mcopy') or not shutil.which('mmd'):
        pytest.skip('Requires mtools')
    names = ['result', 'cycles', 'before', 'after', 'stack_free', 'stack_total',
             'manager_priority', 'worker_priority', 'manager_stack', 'worker_stack', 'timeouts', 'panics',
             'timeout_elapsed', 'worker_stack_total']
    with executable.open('rb') as stream:
        symbols = ELFFile(stream).get_section_by_name('.symtab')
        addresses = [symbols.get_symbol_by_name(f'rust_runtime_test_{name}')[0]['st_value'] for name in names]
        error_address = symbols.get_symbol_by_name('rust_runtime_test_error')[0]['st_value']
        addresses += list(range(error_address, error_address + 100, 4))
        memory_address = symbols.get_symbol_by_name('rust_runtime_test_memory')[0]['st_value']
        addresses += list(range(memory_address, memory_address + 256, 4))
        generation_address = symbols.get_symbol_by_name('rust_runtime_test_generation_free')[0]['st_value']
        addresses += list(range(generation_address, generation_address + 21 * 4, 4))
    generated = gen_board.generate(ROOT, board, tmp_path / 'generated', uart_port=0, state_dir=tmp_path)
    data = renode_data.ensure_mcu_data(generated['mcu_type'])
    sdcard = run.make_fat_image(tmp_path / 'sdcard.img', 16 * 1024 * 1024, fat_bits=16)
    subprocess.run(['mmd', '-i', str(sdcard), '::/fixtures'], check=True)
    priority_file = tmp_path / 'priority.bin'
    priority_file.write_bytes(bytes([priority_max]))
    subprocess.run(['mcopy', '-i', str(sdcard), str(priority_file), '::/fixtures/priority.bin'], check=True)
    for name, mode in [('one', 'cleanup'), ('two', 'parameters'), ('end', ending)]:
        monkeypatch.setenv('AP_MODULE_LIFECYCLE', mode)
        monkeypatch.setenv('AP_MODULE_PARAM_MODE', 'renode')
        output = tmp_path / f'{name}.apmod'
        source = 'parameters.rs' if mode == 'parameters' else 'lifecycle.rs'
        build.build(ROOT / 'libraries/AP_Module/tests/modules' / source, output,
                    'thumbv7em-none-eabihf', 8193, thread_priority=priority_max if name == 'end' else 2)
        subprocess.run(['mcopy', '-i', str(sdcard), str(output), f'::/fixtures/{name}.apmod'], check=True)
    output = renode('''
$elf=@%s
$mcu_svd=@%s
$sdcard=@%s
$vector_base=0x%X
include @%s
emulation SetGlobalAdvanceImmediately true
emulation RunFor "40"
%s
''' % (executable.resolve(), data['svd'], sdcard, generated['app_base'], generated['resc'],
       '\n'.join(f'sysbus ReadDoubleWord 0x{address:X}' for address in addresses)))
    values = [int(value, 16) for value in re.findall(r'^0x[0-9A-Fa-f]{8}$', output, re.M)]
    (tmp_path / 'renode.log').write_text(output)
    assert len(values) == len(names) + 25 + 64 + 21, output
    result = dict(zip(names, values[:len(names)]))
    error_words = values[len(names):len(names)+25]
    result['error'] = b''.join(value.to_bytes(4, 'little') for value in error_words).split(b'\0')[0].decode()
    memory = b''.join(value.to_bytes(4, 'little') for value in values[len(names)+25:len(names)+25+64])
    memory = memory.split(b'\0')[0].decode()
    print(f'Generation free memory: {values[-21:]}', flush=True)
    print(f'{board} {ending}: {result}', flush=True)
    print(memory, flush=True)
    assert result['result'] == 0x600DF00D, (result, output)
    assert result['cycles'] == 20 and result['before'] == result['after'], result
    assert values[-21:] == [result['before']] * 21, 'Every measured generation must restore the same free memory'
    assert result['stack_free'] > 1024 and 4096 <= result['stack_total'] < 4096 + 1024, result
    # The registry includes port context overhead, but not a 4 KiB rounding gap.
    assert 8193 + 4096 <= result['worker_stack_total'] < 8193 + 4096 + 1024, result
    assert result['manager_priority'] == priority_max + 1, result
    assert result['worker_priority'] == priority_max, result
    fast_regions = {'STM32F405xx': (0x10000000, 0x10010000), 'STM32H743xx': (0x20000000, 0x20020000)}
    low, high = fast_regions[generated['mcu_type']]
    assert low <= result['manager_stack'] < high and low <= result['worker_stack'] < high, result
    assert result['timeouts'] == 1 and result['panics'] == int(ending == 'panic'), result
    assert 10000 <= result['timeout_elapsed'] < 12000, result
