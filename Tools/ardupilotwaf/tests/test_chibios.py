# AP_FLAKE8_CLEAN
"""Tests for ChibiOS post-link ELF handling."""

import importlib
import struct

from pathlib import Path

import pytest


def load_chibios(monkeypatch):
    root = Path(__file__).resolve().parents[3]
    monkeypatch.syspath_prepend(str(root / "modules/waf"))
    monkeypatch.syspath_prepend(str(root / "Tools/ardupilotwaf"))
    monkeypatch.syspath_prepend(str(root / "Tools/scripts"))
    monkeypatch.syspath_prepend(str(root / "libraries/AP_HAL_ChibiOS/hwdef/scripts"))
    monkeypatch.syspath_prepend(str(root))
    return importlib.import_module("Tools.ardupilotwaf.chibios")


def test_add_elf_load_segment(tmp_path, monkeypatch):
    chibios = load_chibios(monkeypatch)
    elf_path = tmp_path / "firmware"
    address = 0x08001000
    section_offset = 0x100
    section_size = 4
    section_header_offset = 0x200

    elf = bytearray(0x300)
    struct.pack_into(
        '<16sHHIIIIIHHHHHH', elf, 0,
        b'\x7fELF\x01\x01' + bytes(10), 2, 40, 1, address,
        52, section_header_offset, 0, 52, 32, 1, 40, 2, 0)
    struct.pack_into(
        '<IIIIIIII', elf, 52,
        1, 0x80, 0x08000000, 0x08000000, section_size, section_size, 4, 1)
    struct.pack_into(
        '<IIIIIIIIII', elf, section_header_offset + 40,
        0, 1, 0, address, section_offset, section_size, 0, 0, 1, 0)
    elf[section_offset:section_offset + section_size] = b'sign'
    elf_path.write_bytes(elf)

    chibios.add_elf_load_segment(elf_path, address, section_size)

    result = elf_path.read_bytes()
    assert struct.unpack_from('<H', result, 44)[0] == 2
    assert struct.unpack_from('<I', result, section_header_offset + 48)[0] == 2
    assert struct.unpack_from('<IIIIIIII', result, 84) == (
        1, section_offset, address, address, section_size, section_size, 4, 1)
    assert result[section_offset:section_offset + section_size] == b'sign'


def test_find_app_descriptor_ignores_magic_collision(monkeypatch):
    chibios = load_chibios(monkeypatch)
    descriptor = b'12345678'
    collision = descriptor + bytes(range(1, 17))
    marker = descriptor + bytes(16)
    image = collision + b'padding' + marker + b'trailer'

    assert chibios.find_app_descriptor(image, descriptor) == len(collision) + len(b'padding')


def test_find_app_descriptor_does_not_accept_magic_alone(monkeypatch):
    chibios = load_chibios(monkeypatch)
    descriptor = b'12345678'

    assert chibios.find_app_descriptor(descriptor + bytes(range(1, 17)), descriptor) == -1


def test_find_app_descriptor_rejects_multiple_markers(monkeypatch):
    chibios = load_chibios(monkeypatch)
    descriptor = b'12345678'
    marker = descriptor + bytes(16)

    with pytest.raises(chibios.Errors.WafError, match="multiple zero-initialized"):
        chibios.find_app_descriptor(marker + marker, descriptor)
