#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Unit tests for Renode firmware image handling."""

import base64
import importlib.util
import json
import os
import select
import shutil
import signal
import socket
import struct
import subprocess
import sys
import time
import zlib

from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / 'run.py'
sys.path.insert(0, str(MODULE_PATH.parent))
SPEC = importlib.util.spec_from_file_location('renode_run', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
renode_run = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = renode_run
SPEC.loader.exec_module(renode_run)

APP_BASE = 0x08020000
FLASH_SIZE = 2 * 1024 * 1024


@pytest.mark.skipif(not sys.platform.startswith('linux'),
                    reason='requires Linux parent-death signals')
def test_parent_death_signal_stops_child():
    child_code = '''
import os
import signal
import sys
import time
sys.path.insert(0, sys.argv[1])
from process_utils import arm_parent_death_signal
arm_parent_death_signal(os.getppid(), signal.SIGTERM)
print(os.getpid(), flush=True)
time.sleep(30)
'''
    launcher_code = '''
import subprocess
import sys
child = subprocess.Popen(
    [sys.executable, '-c', sys.argv[2], sys.argv[1]],
    stdout=subprocess.PIPE, text=True)
print(child.stdout.readline().strip(), flush=True)
'''
    launcher = subprocess.Popen(
        [sys.executable, '-c', launcher_code, str(MODULE_PATH.parent),
         child_code], stdout=subprocess.PIPE, text=True)
    child_pid = int(launcher.stdout.readline())
    assert launcher.wait(timeout=5) == 0

    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        try:
            stat_fields = Path('/proc/%u/stat' % child_pid).read_text().split()
        except FileNotFoundError:
            break
        if stat_fields[2] == 'Z':
            break
        time.sleep(0.05)
    else:
        os.kill(child_pid, signal.SIGKILL)
        pytest.fail('child survived its parent')


@pytest.mark.skipif(not sys.platform.startswith('linux'),
                    reason='requires Linux parent-death signals')
def test_parent_death_signal_overrides_deferring_handler():
    # the parent is already gone, as seen by a child whose inherited
    # handler would otherwise swallow the signal
    child_code = '''
import os
import signal
import sys
sys.path.insert(0, sys.argv[1])
from process_utils import arm_parent_death_signal
signal.signal(signal.SIGTERM, lambda signum, frame: None)
arm_parent_death_signal(os.getppid() + 1, signal.SIGTERM)
'''
    child = subprocess.run([sys.executable, '-c', child_code, str(MODULE_PATH.parent)],
                           timeout=10)
    assert child.returncode == -signal.SIGTERM


@pytest.mark.skipif(not sys.platform.startswith('linux'),
                    reason='requires Linux parent-death signals')
def test_parent_death_signal_resets_handler_while_parent_alive():
    child_code = '''
import os
import signal
import sys
sys.path.insert(0, sys.argv[1])
from process_utils import arm_parent_death_signal
signal.signal(signal.SIGTERM, lambda signum, frame: None)
arm_parent_death_signal(os.getppid(), signal.SIGTERM)
sys.exit(0 if signal.getsignal(signal.SIGTERM) == signal.SIG_DFL else 1)
'''
    child = subprocess.run([sys.executable, '-c', child_code, str(MODULE_PATH.parent)],
                           timeout=10)
    assert child.returncode == 0


@pytest.mark.skipif(not hasattr(select, 'poll'),
                    reason='requires poll support')
def test_output_pipe_disconnect_detection():
    read_fd, write_fd = os.pipe()
    write_stream = os.fdopen(write_fd, 'w')
    try:
        assert not renode_run.output_pipe_disconnected((write_stream,))
        os.close(read_fd)
        read_fd = None
        assert renode_run.output_pipe_disconnected((write_stream,))
    finally:
        if read_fd is not None:
            os.close(read_fd)
        write_stream.close()


@pytest.mark.skipif(not hasattr(socket, 'AF_UNIX'),
                    reason='requires Unix domain sockets')
def test_prepare_unix_socket_rejects_live_and_removes_stale(tmp_path):
    path = tmp_path / 'APM-UDS-serial1'
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(str(path))
    listener.listen(1)
    try:
        with pytest.raises(ValueError, match='already in use'):
            renode_run.prepare_unix_socket(path)
        renode_run.cleanup_unix_socket(path)
        assert path.exists()
    finally:
        listener.close()

    renode_run.cleanup_unix_socket(path)
    assert not path.exists()


def test_prepare_unix_socket_rejects_regular_file(tmp_path):
    path = tmp_path / 'APM-UDS-serial1'
    path.write_text('keep me')

    with pytest.raises(ValueError, match='non-socket'):
        renode_run.prepare_unix_socket(path)
    renode_run.cleanup_unix_socket(path)
    assert path.read_text() == 'keep me'


@pytest.mark.skipif(not hasattr(socket, 'AF_UNIX'),
                    reason='requires Unix domain sockets')
def test_prepare_unix_socket_uses_native_path_limit(tmp_path):
    path = tmp_path / ('x' * 200)

    with pytest.raises(ValueError, match='cannot probe Unix socket'):
        renode_run.prepare_unix_socket(path)


@pytest.mark.skipif(renode_run.fcntl is None,
                    reason='requires advisory file locking')
def test_reserve_unix_socket_prevents_bound_socket_replacement(tmp_path):
    path = tmp_path / 'APM-UDS-serial1'
    reservation = renode_run.reserve_unix_socket(path)
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    listener.bind(str(path))
    try:
        with pytest.raises(ValueError, match='already reserved'):
            renode_run.reserve_unix_socket(path)
        assert path.exists()
    finally:
        listener.close()
        os.close(reservation)
        renode_run.cleanup_unix_socket(path)


def test_firmware_format_uses_elf_magic(tmp_path):
    elf = tmp_path / 'extensionless'
    elf.write_bytes(b'\x7fELFpayload')
    assert renode_run.firmware_format(elf) == 'elf'

    invalid_elf = tmp_path / 'firmware.elf'
    invalid_elf.write_bytes(b'not an ELF')
    with pytest.raises(SystemExit, match='not an ELF file'):
        renode_run.firmware_format(invalid_elf)


def test_convert_bin_firmware(tmp_path):
    source = tmp_path / 'firmware.bin'
    destination = tmp_path / 'current-firmware.bin'
    source.write_bytes(b'raw firmware')

    renode_run.make_firmware_binary(
        source, destination, APP_BASE, FLASH_SIZE)

    assert destination.read_bytes() == b'raw firmware'


def test_convert_elf_bootloader(tmp_path):
    objcopy = shutil.which('arm-none-eabi-objcopy')
    if objcopy is None:
        pytest.skip('requires arm-none-eabi-objcopy')
    source = tmp_path / 'bootloader.elf'
    destination = tmp_path / 'bootloader.bin'
    raw = tmp_path / 'raw.bin'
    payload = struct.pack('<II', 0x20001000, 0x08000101) + bytes(0x200)
    raw.write_bytes(payload)
    subprocess.run([
        objcopy, '-I', 'binary', '-O', 'elf32-littlearm', '-B', 'arm',
        '--rename-section', '.data=.text,alloc,load,readonly,code,contents',
        '--change-addresses', '0x08000000', str(raw), str(source),
    ], check=True)

    binary, address = renode_run.make_bootloader_binary(source, destination)

    assert address == 0x08000000
    assert binary.read_bytes() == payload


def test_wrap_binary_in_runtime_elf(tmp_path):
    reset_vector = APP_BASE + 0x101
    firmware = struct.pack('<II', 0x30001000, reset_vector) + b'\x00' * 0x200
    source = tmp_path / 'firmware.bin'
    destination = tmp_path / 'firmware-runtime.elf'
    source.write_bytes(firmware)

    renode_run.wrap_binary_in_runtime_elf(source, destination, APP_BASE)

    wrapped = destination.read_bytes()
    assert wrapped[:4] == b'\x7fELF'
    assert struct.unpack_from('<I', wrapped, 24)[0] == reset_vector
    assert struct.unpack_from('<I', wrapped, 52)[0] == 1
    assert struct.unpack_from('<I', wrapped, 60)[0] == APP_BASE
    assert wrapped[0x100:] == firmware


def test_convert_and_validate_apj_firmware(tmp_path):
    payload = b'APJ firmware payload'
    descriptor = {
        'magic': 'APJFWv1',
        'board_id': 42,
        'image_size': len(payload),
        'image': base64.b64encode(zlib.compress(payload)).decode('ascii'),
    }
    source = tmp_path / 'firmware.apj'
    source.write_text(json.dumps(descriptor))
    destination = tmp_path / 'current-firmware.bin'

    renode_run.make_firmware_binary(
        source, destination, APP_BASE, FLASH_SIZE, expected_board_id=42)
    assert destination.read_bytes() == payload

    with pytest.raises(SystemExit, match='does not match target board ID'):
        renode_run.make_firmware_binary(
            source, destination, APP_BASE, FLASH_SIZE,
            expected_board_id=43)


def test_convert_hex_firmware_ignores_bootloader(tmp_path):
    intelhex = pytest.importorskip('intelhex')
    source = tmp_path / 'firmware.hex'
    destination = tmp_path / 'current-firmware.bin'
    image = intelhex.IntelHex()
    image[0x08000000] = 0xaa
    image[APP_BASE] = 1
    image[APP_BASE + 1] = 2
    image[APP_BASE + 2] = 3
    image.write_hex_file(str(source))

    renode_run.make_firmware_binary(
        source, destination, APP_BASE, FLASH_SIZE)

    assert destination.read_bytes() == b'\x01\x02\x03'


def run_startup_script(tmp_path, commands):
    executable = Path(os.environ.get(
        'RENODE', MODULE_PATH.parents[2] / 'build/renode/renode'))
    if not executable.is_file():
        pytest.skip('build Renode or set RENODE')
    script = renode_run.make_startup_script(tmp_path / 'startup.py', commands)
    return subprocess.run(
        [str(executable.resolve()), '--disable-xwt', '--console', '--plain',
         '-e', 'include @%s; quit' % script],
        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, timeout=120,
        env=dict(os.environ, TMPDIR=str(tmp_path),
                 XDG_CONFIG_HOME=str(tmp_path / 'xdg')))


def test_startup_script_quotes_commands(tmp_path):
    commands = ['$a=@/path with space', 'cpu AddHook 0 "monitor.Parse(\'x\')"',
                '$b=@/home/j\u00f6rg']
    script = renode_run.make_startup_script(tmp_path / 'startup.py', commands)
    assert script.read_bytes().isascii()
    line = next(line for line in script.read_text().splitlines()
                if line.startswith('commands = '))
    assert eval(line.split('=', 1)[1]) == commands


def test_startup_script_exits_on_nested_compile_error(tmp_path):
    model = tmp_path / 'Broken.cs'
    model.write_text('namespace X { class Broken { int f = "x"; } }\n')
    nested = tmp_path / 'nested.resc'
    nested.write_text('include @%s\n' % model)
    result = run_startup_script(
        tmp_path, ['include @%s' % nested, 'echo "not reached"'])
    assert result.returncode == renode_run.STARTUP_FAILED
    assert 'error CS0029' in result.stdout
    assert 'not reached' not in result.stdout


def test_startup_script_runs_all_commands(tmp_path):
    result = run_startup_script(tmp_path, ['$x=@/home/j\u00f6rg', 'echo "reached"'])
    assert result.returncode == 0
    assert 'reached' in result.stdout
