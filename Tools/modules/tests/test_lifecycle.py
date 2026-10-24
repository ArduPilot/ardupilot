# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise actual ChibiOS module threads with Rust images loaded from SD."""

import os
import re
import shutil
import subprocess

import pytest

from elftools.elf.elffile import ELFFile
from test_threads import ROOT  # noqa: F401
from test_threads import gen_board  # noqa: F401
from test_threads import renode  # noqa: F401
from test_threads import renode_data  # noqa: F401
from test_threads import run  # noqa: F401

import build


def test_chibios_module_lifecycle(tmp_path, renode, monkeypatch):  # noqa: F811
    board = os.environ.get('OWNED_THREAD_BOARD', 'MatekH743')
    executable = ROOT / 'build' / board / 'examples/ModuleLoader'
    if not executable.is_file():
        pytest.skip('Build examples/ModuleLoader with the native runtime HAL options enabled')
    if not shutil.which('mcopy') or not shutil.which('mmd'):
        pytest.skip('Requires mtools for the SD image')
    names = ['rust_module_test_result', 'rust_module_test_cycles', 'rust_module_test_before',
             'rust_module_test_after', 'rust_module_test_retained', 'rust_module_test_loads',
             'rust_module_test_free_on_failure']
    with executable.open('rb') as stream:
        symbols = ELFFile(stream).get_section_by_name('.symtab')
        addresses = [symbols.get_symbol_by_name(name)[0]['st_value'] for name in names]
    generated = gen_board.generate(ROOT, board, tmp_path / 'generated', uart_port=0, state_dir=tmp_path)
    data = renode_data.ensure_mcu_data(generated['mcu_type'])
    sdcard = run.make_fat_image(tmp_path / 'sdcard.img', 16 * 1024 * 1024, fat_bits=16)
    subprocess.run(['mmd', '-i', str(sdcard), '::/APM', '::/APM/modules'], check=True)
    for mode in ('cleanup', 'panic'):
        monkeypatch.setenv('AP_MODULE_LIFECYCLE', mode)
        output = tmp_path / f'{mode}.apmod'
        build.build(ROOT / 'libraries/AP_Module/tests/modules/lifecycle.rs', output,
                    'thumbv7em-none-eabihf', 8192)
        subprocess.run(['mcopy', '-i', str(sdcard), str(output), f'::/APM/modules/{mode}.apmod'], check=True)
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
       '\n'.join(f'sysbus ReadDoubleWord 0x{address:X}' for address in addresses)))
    values = [int(value, 16) for value in re.findall(r'^0x[0-9A-Fa-f]{8}$', output, re.M)]
    assert len(values) == 7 and values[0] == 0x600DF00D and values[1] == 100, output
    assert values[2] == values[3] and values[4] > 0, output
    assert values[5] == 203 and values[6] == 0, output  # 2 warm-up + 200 cycle + 1 panic loads
    print(f'{board}: 100 two-module cycles; free {values[2]} -> {values[3]}, '
          f'known retained panic bytes {values[4]} (heap used, not reserved; excludes backend overhead)')
