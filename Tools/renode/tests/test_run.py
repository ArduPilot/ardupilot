#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Unit tests for Renode firmware image handling."""

import base64
import importlib.util
import json
import os
import socket
import struct
import sys
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
