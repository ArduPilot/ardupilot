#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Tests for the standalone STM32 USB DFU endpoint."""

import importlib.util
import socket
import struct
import sys

from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / 'dfu.py'
SPEC = importlib.util.spec_from_file_location('renode_dfu', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
dfu = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = dfu
SPEC.loader.exec_module(dfu)


def make_device(tmp_path, family='f405', flash_size=1024 * 1024):
    flash = tmp_path / 'flash.img'
    flash.write_bytes(b'\xff' * flash_size)
    return dfu.DfuDevice(flash, family, flash_size, port=0)


def setup_packet(request, value=0, length=0, direction_in=False):
    request_type = 0xA1 if direction_in else 0x21
    return struct.pack('<BBHHH', request_type, request, value, 0, length)


def test_flash_geometry_and_memory_descriptor():
    f405 = dfu.flash_sectors('f405', 1024 * 1024)
    assert len(f405) == 12
    assert f405[:5] == (
        (0x08000000, 16 * 1024),
        (0x08004000, 16 * 1024),
        (0x08008000, 16 * 1024),
        (0x0800C000, 16 * 1024),
        (0x08010000, 64 * 1024),
    )
    assert f405[-1] == (0x080E0000, 128 * 1024)
    assert dfu.memory_layout('f405', 1024 * 1024) == (
        '@Internal Flash /0x08000000/04*016Kg,01*064Kg,07*128Kg')

    h743 = dfu.flash_sectors('h743', 2 * 1024 * 1024)
    assert len(h743) == 16
    assert h743[-1] == (0x081E0000, 128 * 1024)
    assert dfu.memory_layout('h743', 2 * 1024 * 1024) == (
        '@Internal Flash /0x08000000/16*128Kg')

    with pytest.raises(ValueError, match='unsupported'):
        dfu.flash_sectors('f767', 2 * 1024 * 1024)


def test_dfuse_program_upload_erase_and_manifest(tmp_path):
    device = make_device(tmp_path)
    try:
        address = 0x08004000
        command = bytes((0x21,)) + struct.pack('<I', address)
        assert device._control(
            setup_packet(dfu.DFU_DNLOAD, value=0, length=len(command)),
            command) == (dfu.USB_OK, b'')
        assert device.address == address
        assert device._get_status()[4] == dfu.STATE_DFU_DNBUSY
        assert device._get_status()[4] == dfu.STATE_DFU_DNLOAD_IDLE

        payload = b'flight-controller bootloader'
        assert device._control(
            setup_packet(dfu.DFU_DNLOAD, value=2, length=len(payload)),
            payload) == (dfu.USB_OK, b'')
        status, uploaded = device._control(
            setup_packet(dfu.DFU_UPLOAD, value=2, length=len(payload),
                         direction_in=True), b'')
        assert status == dfu.USB_OK
        assert uploaded == payload

        erase = bytes((0x41,)) + struct.pack('<I', address)
        device._control(
            setup_packet(dfu.DFU_DNLOAD, value=0, length=len(erase)), erase)
        assert device.flash.read_bytes()[address - dfu.FLASH_BASE:
                                         address - dfu.FLASH_BASE + len(payload)] == (
                                             b'\xff' * len(payload))

        device._control(setup_packet(dfu.DFU_DNLOAD), b'')
        assert device._get_status()[4] == dfu.STATE_DFU_MANIFEST
        assert device.wait_for_manifest()
    finally:
        device.close()


def test_usbip_import_exposes_stm32_dfu_identity(tmp_path):
    device = make_device(tmp_path)
    connection = socket.create_connection((device.host, device.port), timeout=2)
    try:
        connection.sendall(
            struct.pack('>HHI', dfu.USBIP_VERSION, dfu.OP_REQ_IMPORT, 0) +
            dfu.BUS_ID.encode().ljust(32, b'\0'))
        reply = connection.recv(8 + 312)
        while len(reply) < 8 + 312:
            reply += connection.recv(8 + 312 - len(reply))
        version, command, status = struct.unpack('>HHI', reply[:8])
        assert (version, command, status) == (
            dfu.USBIP_VERSION, dfu.OP_REP_IMPORT, 0)
        vendor, product = struct.unpack('>HH', reply[8 + 300:8 + 304])
        assert (vendor, product) == (
            dfu.STM32_VENDOR_ID, dfu.STM32_DFU_PRODUCT_ID)
    finally:
        connection.close()
        device.close()


def test_close_releases_usbip_port_for_renode_handoff(tmp_path):
    device = make_device(tmp_path)
    connection = socket.create_connection((device.host, device.port), timeout=2)
    port = device.port
    connection.sendall(
        struct.pack('>HHI', dfu.USBIP_VERSION, dfu.OP_REQ_IMPORT, 0) +
        dfu.BUS_ID.encode().ljust(32, b'\0'))
    reply = connection.recv(8 + 312)
    while len(reply) < 8 + 312:
        reply += connection.recv(8 + 312 - len(reply))

    device.close()
    connection.close()
    replacement = socket.socket()
    try:
        replacement.bind(('', port))
    finally:
        replacement.close()


def test_ended_session_releases_usbip_port(tmp_path):
    device = make_device(tmp_path)
    port = device.port
    try:
        with socket.create_connection((device.host, port), timeout=2) as connection:
            # A rejected import ends the session before device.close().
            connection.sendall(
                struct.pack('>HHI', dfu.USBIP_VERSION, dfu.OP_REQ_IMPORT, 0) +
                b'unknown'.ljust(32, b'\0'))
            reply = dfu.DfuDevice._receive(connection, 8)
            assert struct.unpack('>HHI', reply)[2] == 1
            with pytest.raises(ConnectionResetError):
                connection.recv(1)
    finally:
        device.close()
    with socket.socket() as replacement:
        replacement.bind(('', port))
