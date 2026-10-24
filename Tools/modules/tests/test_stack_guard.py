# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Destructive H7 stack tests in Renode; never run on a flying vehicle."""

import os
import re
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


def run_image(tmp_path, emulator, executable, commands, timeout=60):
    board = os.environ.get('STACK_GUARD_BOARD', 'MatekH743')
    generated = gen_board.generate(ROOT, board, tmp_path / 'generated', uart_port=0, state_dir=tmp_path)
    data = renode_data.ensure_mcu_data(generated['mcu_type'])
    sdcard = tmp_path / 'sdcard.img'
    if not sdcard.is_file():
        run.make_fat_image(sdcard, 16 * 1024 * 1024, fat_bits=16)
    return emulator('''
$elf=@%s
$mcu_svd=@%s
$sdcard=@%s
$vector_base=0x%X
include @%s
emulation SetGlobalAdvanceImmediately true
%s
''' % (executable.resolve(), data['svd'], sdcard, generated['app_base'], generated['resc'], commands), timeout=timeout)


@pytest.mark.parametrize('mode', range(7))
def test_stack_guard(tmp_path, renode, mode):  # noqa: F811
    board = os.environ.get('STACK_GUARD_BOARD', 'MatekH743')
    build_root = Path(os.environ.get('AP_MODULE_BUILD_ROOT', ROOT / 'build'))
    executable = build_root / board / 'examples/StackGuard'
    if not executable.is_file():
        pytest.skip('Build examples/StackGuard on H7 with owned threads enabled')
    names = ['result', 'cycles', 'before', 'after', 'mode', 'depth', 'heartbeat', 'waiters', 'faults']
    with executable.open('rb') as stream:
        symbols = ELFFile(stream).get_section_by_name('.symtab')
        addresses = [symbols.get_symbol_by_name(f'stack_guard_test_{name}')[0]['st_value'] for name in names]
        fault = symbols.get_symbol_by_name('ap_stack_guard_recover')[0]['st_value'] & ~1
        fatal = symbols.get_symbol_by_name('AP_HardFault_Handler') or symbols.get_symbol_by_name('ap_stack_guard_disable')
        refused = fatal[0]['st_value'] & ~1
        start = symbols.get_symbol_by_name('stack_guard_test_start')[0]['st_value']
    generated = gen_board.generate(ROOT, board, tmp_path / 'generated', uart_port=0, state_dir=tmp_path)
    data = renode_data.ensure_mcu_data(generated['mcu_type'])
    sdcard = run.make_fat_image(tmp_path / 'sdcard.img', 16 * 1024 * 1024, fat_bits=16)
    output = renode('''
$elf=@%s
$mcu_svd=@%s
$sdcard=@%s
$vector_base=0x%X
include @%s
cpu AddHook 0x%X "print('GUARD_FAULT', self.GetRegister(0), self.GetRegister(1), self.FaultStatus, self.FPCCR)"
cpu AddHook 0x%X "print('GUARD_REFUSED'); self.IsHalted = True"
emulation SetGlobalAdvanceImmediately true
emulation RunFor "5"
sysbus WriteDoubleWord 0x%X %u
sysbus WriteDoubleWord 0x%X 1
emulation RunFor "15"
%s
sysbus ReadDoubleWord 0xE000ED28
sysbus ReadDoubleWord 0xE000ED2C
sysbus ReadDoubleWord 0xE000ED34
sysbus ReadDoubleWord 0xE000EF34
cpu PC
''' % (executable.resolve(), data['svd'], sdcard, generated['app_base'], generated['resc'], fault, refused,
       addresses[4], mode, start, '\n'.join(f'sysbus ReadDoubleWord 0x{a:X}' for a in addresses)))
    (tmp_path / 'renode.log').write_text(output)
    values = [int(v, 16) for v in re.findall(r'^0x[0-9A-Fa-f]{8}$', output, re.M)]
    result = dict(zip(names, values))
    print(result)
    assert 'Python runtime error' not in output, output
    assert result['mode'] == mode, output
    if mode in (0, 1, 2, 3, 4):
        assert result['result'] == 0x600DF00D, output
        assert result['cycles'] == 100 and result['before'] == result['after'], output
        assert result['faults'] == 101, output
        assert result['heartbeat'] > 100, output
    else:
        assert result['result'] == 4 and result['cycles'] == 0, output
        assert result['faults'] == 0, output
        assert 'GUARD_REFUSED' in output, output


@pytest.mark.parametrize('floating_point,host_call', [
    (False, False),
    (True, False),
    (False, True),
])
def test_rust_stack_overflow(tmp_path, renode, monkeypatch, floating_point, host_call):  # noqa: F811
    build_root = Path(os.environ.get('AP_MODULE_BUILD_ROOT', ROOT / 'build'))
    executable = build_root / os.environ.get('STACK_GUARD_BOARD', 'MatekH743') / 'examples/ModuleRuntime'
    if not executable.is_file():
        pytest.skip('Build examples/ModuleRuntime with Tools/modules/stack-guard.hwdef')
    names = ['result', 'cycles', 'before', 'after', 'overflows']
    with executable.open('rb') as stream:
        symbols = ELFFile(stream).get_section_by_name('.symtab')
        addresses = [symbols.get_symbol_by_name(f'rust_runtime_test_{n}')[0]['st_value'] for n in names]
        fault = symbols.get_symbol_by_name('ap_stack_guard_recover')[0]['st_value'] & ~1
        fatal = symbols.get_symbol_by_name('AP_HardFault_Handler') or symbols.get_symbol_by_name('ap_stack_guard_disable')
        refused = fatal[0]['st_value'] & ~1
    sdcard = run.make_fat_image(tmp_path / 'sdcard.img', 16 * 1024 * 1024, fat_bits=16)
    monkeypatch.setenv('AP_MODULE_STACK_FP', str(int(floating_point)))
    monkeypatch.setenv('AP_MODULE_STACK_HOST_CALL', str(int(host_call)))
    module = tmp_path / 'overflow.apmod'
    build.build(ROOT / 'libraries/AP_Module/tests/modules/stack_overflow.rs', module, 'thumbv7em-none-eabihf', 4096)
    subprocess.run(['mmd', '-i', str(sdcard), '::/fixtures'], check=True)
    subprocess.run(['mcopy', '-i', str(sdcard), str(module), '::/fixtures/overflow.apmod'], check=True)
    commands = '''
cpu AddHook 0x%X "print('GUARD_FAULT', self.GetRegister(0), self.GetRegister(1), self.FaultStatus, self.FPCCR)"
cpu AddHook 0x%X "print('GUARD_REFUSED'); self.IsHalted = True"
emulation RunFor "120"
%s
''' % (fault, refused, '\n'.join(f'sysbus ReadDoubleWord 0x{a:X}' for a in addresses))
    output = run_image(tmp_path, renode, executable, commands, timeout=180)
    (tmp_path / 'renode.log').write_text(output)
    values = [int(v, 16) for v in re.findall(r'^0x[0-9A-Fa-f]{8}$', output, re.M)]
    result = dict(zip(names, values))
    print(result)
    assert 'Python runtime error' not in output, output
    assert result['result'] == 0x600DF00D, output
    assert result['cycles'] == 100 and result['overflows'] == 102, output
    assert result['before'] == result['after'], output
