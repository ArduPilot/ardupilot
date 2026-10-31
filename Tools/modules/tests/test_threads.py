# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Run the owned-thread HAL example on a real ChibiOS build under Renode."""

import os
import re
import sys

from pathlib import Path

import pytest

from elftools.elf.elffile import ELFFile

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'Tools/renode/tests'))
sys.path.insert(0, str(ROOT / 'Tools/renode'))
import gen_board  # noqa: E402
import renode_data  # noqa: E402
import run  # noqa: E402

from test_peripherals import renode  # noqa: E402,F401


@pytest.mark.parametrize('example,names', [
    ('OwnedThread', ('owned_thread_test_result', 'owned_thread_test_cycles',
                     'owned_thread_test_memory_before', 'owned_thread_test_memory_after')),
    ('ExecutableMemory', ('executable_memory_test_result', 'executable_memory_test_address',
                          'executable_memory_test_before', 'executable_memory_test_after')),
    ('RustHeap', ('rust_heap_test_result', 'rust_heap_test_cycles',
                  'rust_heap_test_before', 'rust_heap_test_after')),
])
def test_chibios_runtime_primitives(tmp_path, renode, example, names):  # noqa: F811
    board = os.environ.get('OWNED_THREAD_BOARD', 'MatekH743')
    build_root = Path(os.environ.get('AP_MODULE_BUILD_ROOT', ROOT / 'build'))
    executable = build_root / board / 'examples' / example
    if not executable.is_file():
        pytest.skip(f'Build examples/{example} with the native runtime HAL options enabled')
    with executable.open('rb') as stream:
        symbols = ELFFile(stream).get_section_by_name('.symtab')
        addresses = [symbols.get_symbol_by_name(name)[0]['st_value'] for name in names]
    generated = gen_board.generate(ROOT, board, tmp_path / 'generated', uart_port=0, state_dir=tmp_path)
    data = renode_data.ensure_mcu_data(generated['mcu_type'])
    sdcard = run.make_fat_image(tmp_path / 'sdcard.img', 16 * 1024 * 1024, fat_bits=16)
    output = renode('''
$elf=@%s
$mcu_svd=@%s
$sdcard=@%s
$vector_base=0x%X
include @%s
emulation SetGlobalAdvanceImmediately true
emulation RunFor "3"
sysbus ReadDoubleWord 0x%X
sysbus ReadDoubleWord 0x%X
sysbus ReadDoubleWord 0x%X
sysbus ReadDoubleWord 0x%X
''' % (executable.resolve(), data['svd'], sdcard, generated['app_base'], generated['resc'], *addresses))
    values = re.findall(r'^0x[0-9A-Fa-f]{8}$', output, re.M)
    values = [int(value, 16) for value in values]
    assert len(values) == 4 and values[0] == 0x600DF00D, output
    if example in ('OwnedThread', 'RustHeap'):
        assert values[1] == 100, output
        if example == 'RustHeap':
            assert values[3] == values[2], output
    else:
        # The allocator must not select ITCM/DTCM/CCM or an unrelated SRAM bank.
        expected = {'STM32H743xx': (0x24000000, 0x24080000), 'STM32F405xx': (0x20000000, 0x20020000)}
        lower, upper = expected[generated['mcu_type']]
        assert lower <= values[1] < upper and values[1] % 32 == 0, output
    print(f'{board} {example}: free memory before={values[2]} after={values[3]}')
