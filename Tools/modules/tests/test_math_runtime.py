# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Real SDK math through SITL and ChibiOS SD-loaded modules."""

import hashlib
import os
import re
import shutil
import subprocess

from pathlib import Path

import pytest

from elftools.elf.elffile import ELFFile
from rust.build import sdk_environment
from test_modules import upgrade_frozen_container
from test_runtime import ROOT
from test_runtime import Vehicle
from test_threads import gen_board  # noqa: F401
from test_threads import renode  # noqa: F401
from test_threads import renode_data  # noqa: F401
from test_threads import run  # noqa: F401

import build


def test_sdk_math_unit(tmp_path):
    executable = tmp_path / 'sdk-math-tests'
    environment = sdk_environment()
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', '--test',
                    str(ROOT / 'libraries/AP_Module/sdk/rust/src/lib.rs'), '-o', str(executable)], check=True, env=environment)
    subprocess.run([str(executable)], check=True)


def test_frozen_host_dispatch(tmp_path):
    executable = ROOT / 'build/sitl/tests/test_rust_sdk'
    if not executable.is_file():
        pytest.skip('Build tests/test_rust_sdk with Rust enabled')
    image = tmp_path / 'compat.apmod'
    image.write_bytes(upgrade_frozen_container(bytes.fromhex(
        (ROOT / 'libraries/AP_Module/tests/compat/x86_64.apmod.hex').read_text())))
    environment = dict(os.environ, AP_MODULE_TEST_COMPAT_MODULE=str(image))
    subprocess.run([str(executable), '--gtest_filter=ModuleMathSDK.FrozenImageSurvives*'], env=environment, check=True)


@pytest.mark.parametrize('lua', [False, True])
@pytest.mark.parametrize('module', ['math', 'compat', 'categories', 'values', 'files'])
def test_sitl_math(tmp_path, lua, module):
    if lua and os.environ.get('AP_MODULE_TEST_NO_LUA'):
        pytest.skip('Explicit Rust-only build')
    if module == 'compat':
        # Intentionally do not invoke the compiler on the frozen old SDK probe.
        data = bytes.fromhex((ROOT / 'libraries/AP_Module/tests/compat/x86_64.apmod.hex').read_text())
        data = upgrade_frozen_container(data)
    else:
        image = build.build(ROOT / f'libraries/AP_Module/tests/modules/{module}.rs', tmp_path / 'math.apmod',
                            'x86_64-unknown-linux-gnu', 8192)
        data = image.read_bytes()
    settings = {'SCR_ENABLE': 1, 'SCR_HEAP_SIZE': 100000} if lua else {}
    if module == 'categories':
        settings['NTF_LED_TYPES'] = 1 << 10
    vehicle = Vehicle(tmp_path / 'vehicle', {'math.apmod': data}, settings)
    if module == 'files':
        (vehicle.directory / 'file-test.bin').write_bytes(bytes(range(256)) * 3 + b'end')
        (vehicle.directory / 'APM').mkdir()
    if lua:
        scripts = vehicle.directory / 'scripts'
        scripts.mkdir()
        shutil.copyfile(ROOT / 'libraries/AP_Module/tests/modules/math.lua', scripts / 'math.lua')
    try:
        vehicle.start()
        vehicle.text('Module: MATH ok')
        if lua:
            vehicle.text('LUA MATH ok')
        before = len(vehicle.texts)
        vehicle.parameter('MOD_RESTART', 1)
        vehicle.text('Module: MATH ok', after=before)
        assert not any(re.search(r'Module: .* (panic|returned -)', line) for line in vehicle.texts)
    finally:
        vehicle.stop()


@pytest.mark.parametrize('module', [
    'math', 'ahrs_unavailable', 'compat', 'categories', 'enums', 'strings', 'libm',
    'rc_channel_harness', 'values', 'files', 'logs', 'printf', 'alignment',
])
def test_chibios_math(tmp_path, renode, module):  # noqa: F811
    chibios_probe(tmp_path, renode, module)


def chibios_probe(tmp_path, emulator, module, oracle=None, files=None, heap_limit=128, expected_heap=4096,
                  expected_code=65536, cycles=20, timeout_ms=5000):
    board = os.environ.get('OWNED_THREAD_BOARD', 'MatekH743')
    build_root = Path(os.environ.get('AP_MODULE_BUILD_ROOT', ROOT / 'build'))
    executable = build_root / board / 'examples/ModuleMath'
    if not executable.is_file():
        pytest.skip('Build examples/ModuleMath with --enable-modules')
    if not shutil.which('mcopy') or not shutil.which('mmd'):
        pytest.skip('Requires mtools')
    names = ['result', 'cycles', 'before', 'after', 'heap_peak', 'status', 'stack_used', 'stack_total',
             'warmup_before', 'heap_limit', 'code_limit', 'cycle_limit', 'timeout_ms']
    with executable.open('rb') as stream:
        symbols = ELFFile(stream).get_section_by_name('.symtab')
        addresses = []
        for name in names:
            symbol = symbols.get_symbol_by_name(f'rust_math_test_{name}')
            assert symbol, f'Missing rust_math_test_{name}: rebuild examples/ModuleMath'
            addresses.append(symbol[0]['st_value'])
        error_symbol = symbols.get_symbol_by_name('rust_math_test_error')
        assert error_symbol, 'Missing diagnostics: rebuild examples/ModuleMath with the matching hwdef'
        error_address = error_symbol[0]['st_value']
    print(f'Native ELF SHA256 {hashlib.sha256(executable.read_bytes()).hexdigest()} '
          f'mtime_ns={executable.stat().st_mtime_ns}', flush=True)
    generated = gen_board.generate(ROOT, board, tmp_path / 'generated', uart_port=0, state_dir=tmp_path)
    data = renode_data.ensure_mcu_data(generated['mcu_type'])
    sdcard = run.make_fat_image(tmp_path / 'sdcard.img', 16 * 1024 * 1024, fat_bits=16)
    subprocess.run(['mmd', '-i', str(sdcard), '::/APM', '::/APM/modules'], check=True)
    if module == 'compat':
        image = tmp_path / 'math.apmod'
        image.write_bytes(upgrade_frozen_container(bytes.fromhex(
            (ROOT / 'libraries/AP_Module/tests/compat/thumbv7em.apmod.hex').read_text())))
    else:
        image = build.build(ROOT / f'libraries/AP_Module/tests/modules/{module}.rs', tmp_path / 'math.apmod',
                            'thumbv7em-none-eabihf', 8192)
    subprocess.run(['mcopy', '-i', str(sdcard), str(image), '::/APM/modules/math.apmod'], check=True)
    if module == 'strings':
        fixture = tmp_path / 'string-test.txt'
        fixture.write_bytes(b'123456789')
        subprocess.run(['mcopy', '-i', str(sdcard), str(fixture), '::/string-test.txt'], check=True)
    if module == 'files':
        fixture = tmp_path / 'file-test.bin'
        fixture.write_bytes(bytes(range(256)) * 3 + b'end')
        subprocess.run(['mcopy', '-i', str(sdcard), str(fixture), '::/file-test.bin'], check=True)
    if oracle is not None:
        fixture = tmp_path / 'aerobatics-golden.bin'
        fixture.write_bytes(oracle)
        subprocess.run(['mcopy', '-i', str(sdcard), str(fixture), '::/aerobatics-golden.bin'], check=True)
    for name, contents in (files or {}).items():
        fixture = tmp_path / name
        fixture.write_bytes(contents)
        subprocess.run(['mcopy', '-i', str(sdcard), str(fixture), f'::/{name}'], check=True)
    output = emulator('''
$elf=@%s
$mcu_svd=@%s
$sdcard=@%s
$vector_base=0x%X
include @%s
emulation SetGlobalAdvanceImmediately true
emulation RunFor "%s"
%s
''' % (executable.resolve(), data['svd'], sdcard, generated['app_base'], generated['resc'],
       (cycles + 1) * timeout_ms / 1000 + 10,
       '\n'.join(f'sysbus ReadDoubleWord 0x{address:X}' for address in addresses +
                 list(range(error_address, error_address + 96, 4)))),
        timeout=max(60, (cycles + 1) * timeout_ms / 1000 + 30))
    (tmp_path / 'renode.log').write_text(output)
    values = [int(value, 16) for value in re.findall(r'^0x[0-9A-Fa-f]{8}$', output, re.M)]
    assert len(values) == len(names) + 24, output
    error = b''.join(value.to_bytes(4, 'little') for value in values[len(names):]).split(b'\0')[0]
    result = dict(zip(names, values[:len(names)]))
    print(f'{board} {module}: {result}', flush=True)
    if error:
        print(error.decode('utf-8', errors='replace'), flush=True)
    assert result['heap_limit'] == expected_heap, 'Rebuild ModuleMath with the test-specific heap hwdef'
    assert (result['code_limit'], result['cycle_limit'], result['timeout_ms']) == (expected_code, cycles, timeout_ms)
    assert result['result'] == 0x600DF00D and result['cycles'] == cycles, (result, error, output)
    assert result['before'] == result['after'] and result['status'] == 0, result
    if oracle is None:
        assert result['heap_peak'] == 0, result
    else:
        assert 0 < result['heap_peak'] <= heap_limit, result
    assert 0 < result['stack_used'] < result['stack_total'] - 4096, result
    return result
