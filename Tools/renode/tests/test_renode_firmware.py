#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Tests for Renode firmware selection and caching."""

import gzip
import importlib.util
import io
import json

from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / 'renode_firmware.py'
SPEC = importlib.util.spec_from_file_location('renode_firmware', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
firmware = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(firmware)


class Response(io.BytesIO):
    def __init__(self, data):
        super().__init__(data)
        self.headers = {'Content-Length': str(len(data))}

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()


def entry(channel='stable', board='CubeBlack', vehicle='Copter'):
    latest = 1 if channel == 'latest' else 0
    return {
        'platform': board,
        'vehicletype': vehicle,
        'git-sha': 'a' * 40,
        'url': ('https://firmware.ardupilot.org/%s/%s/%s/firmware.elf' %
                (vehicle, channel, board)),
        'format': 'elf',
        'latest': latest,
        'mav-firmware-version-str': 'V1.2.3',
    }


def arm_elf():
    data = bytearray(64)
    data[:6] = b'\x7fELF\x01\x01'
    data[18:20] = (40).to_bytes(2, 'little')
    return bytes(data)


def test_fetch_and_select_exact_alias(monkeypatch):
    entries = [
        entry(),
        entry(board='CubeBlack-heli'),
        entry(channel='latest'),
    ]
    encoded = gzip.compress(json.dumps({
        'format-version': '1.0.0', 'firmware': entries,
    }).encode())
    monkeypatch.setattr(firmware.urllib.request, 'urlopen',
                        lambda *_args, **_kwargs: Response(encoded))

    manifest = firmware.fetch_manifest()
    assert firmware.select_firmware(
        manifest, 'CubeBlack', 'Copter', 'stable') == entries[0]
    assert firmware.select_firmware(
        manifest, 'CubeBlack', 'Copter', 'latest') == entries[2]


def test_install_validates_and_reuses_cached_elf(tmp_path, monkeypatch):
    payload = arm_elf()
    downloads = []

    def urlopen(*_args, **_kwargs):
        downloads.append(True)
        return Response(payload)

    monkeypatch.setattr(firmware.urllib.request, 'urlopen', urlopen)
    selected = entry()
    path, downloaded = firmware.install_firmware(tmp_path, selected)
    assert downloaded
    assert path.read_bytes() == payload
    assert json.loads(path.with_name('firmware.json').read_text())['size'] == 64

    again, downloaded = firmware.install_firmware(tmp_path, selected)
    assert again == path
    assert not downloaded
    assert len(downloads) == 1

    path.write_bytes(b'corrupt')
    _again, downloaded = firmware.install_firmware(tmp_path, selected)
    assert downloaded
    assert len(downloads) == 2


def test_rejects_unsafe_or_invalid_firmware(tmp_path, monkeypatch):
    selected = entry()
    selected['url'] = 'https://example.com/Copter/stable/CubeBlack/a.elf'
    with pytest.raises(ValueError, match='unsafe firmware URL'):
        firmware.install_firmware(tmp_path, selected)

    selected = entry()
    monkeypatch.setattr(firmware.urllib.request, 'urlopen',
                        lambda *_args, **_kwargs: Response(b'not an ELF'))
    with pytest.raises(ValueError, match='32-bit ARM ELF'):
        firmware.install_firmware(tmp_path, selected)


@pytest.mark.parametrize('field', ('platform', 'vehicletype', 'git-sha'))
def test_rejects_cache_path_traversal(tmp_path, field):
    selected = entry()
    selected[field] = '..'
    with pytest.raises(ValueError, match='unsafe firmware'):
        firmware.install_firmware(tmp_path, selected)


def test_rejects_manifest_while_expanding(monkeypatch):
    monkeypatch.setattr(firmware, 'MAX_MANIFEST_SIZE', 64)
    encoded = gzip.compress(b'A' * 65)
    monkeypatch.setattr(firmware.urllib.request, 'urlopen',
                        lambda *_args, **_kwargs: Response(encoded))

    with pytest.raises(ValueError, match='expanded firmware manifest'):
        firmware.fetch_manifest()


def test_rejects_non_object_manifest(monkeypatch):
    encoded = gzip.compress(b'[]')
    monkeypatch.setattr(firmware.urllib.request, 'urlopen',
                        lambda *_args, **_kwargs: Response(encoded))

    with pytest.raises(ValueError, match='unsupported firmware manifest'):
        firmware.fetch_manifest()
