#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Tests for portable Renode FAT image creation and extraction."""

import importlib.util
import os
import shutil
import struct
import subprocess
import sys

from pathlib import Path

import pytest

from pyfatfs.PyFatFS import PyFatFS

MODULE_PATH = Path(__file__).resolve().parents[1] / 'fat_image.py'
SPEC = importlib.util.spec_from_file_location('fat_image', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
fat_image = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = fat_image
SPEC.loader.exec_module(fat_image)

EXTRACT_LOGS_PATH = Path(__file__).resolve().parents[1] / 'extract_logs.py'
EXTRACT_LOGS_SPEC = importlib.util.spec_from_file_location(
    'renode_extract_logs', EXTRACT_LOGS_PATH)
assert EXTRACT_LOGS_SPEC is not None and EXTRACT_LOGS_SPEC.loader is not None
extract_logs = importlib.util.module_from_spec(EXTRACT_LOGS_SPEC)
EXTRACT_LOGS_SPEC.loader.exec_module(extract_logs)


def root_directory_offset(path):
    with path.open('rb') as stream:
        boot = stream.read(fat_image.SECTOR_SIZE)
    bytes_per_sector = struct.unpack_from('<H', boot, 11)[0]
    sectors_per_cluster = boot[13]
    reserved_sectors = struct.unpack_from('<H', boot, 14)[0]
    number_of_fats = boot[16]
    fat_sectors = struct.unpack_from('<I', boot, 36)[0]
    root_cluster = struct.unpack_from('<I', boot, 44)[0]
    first_data_sector = reserved_sectors + number_of_fats * fat_sectors
    root_sector = first_data_sector + (root_cluster - 2) * sectors_per_cluster
    return root_sector * bytes_per_sector


def test_create_sparse_fat32_image(tmp_path, monkeypatch):
    image = tmp_path / 'sdcard.img'
    sparse_calls = []

    def mark_sparse(stream):
        sparse_calls.append(stream.name)

    monkeypatch.setattr(fat_image, '_mark_windows_sparse', mark_sparse)
    fat_image.create_image(
        image, 512 * 1024 * 1024, fat_bits=32,
        sectors_per_cluster=fat_image.FAT32_SECTORS_PER_CLUSTER)

    assert sparse_calls == [str(image)]
    geometry = fat_image.read_geometry(image)
    assert geometry['fat_bits'] == 32
    assert geometry['bytes_per_sector'] == 512
    assert geometry['sectors_per_cluster'] == 8
    assert geometry['number_of_fats'] == 2
    assert geometry['data_clusters'] >= 65525
    assert image.stat().st_size == 512 * 1024 * 1024
    if os.name != 'nt' and hasattr(image.stat(), 'st_blocks'):
        assert image.stat().st_blocks * 512 < 2 * 1024 * 1024

    with image.open('rb') as stream:
        primary_boot = stream.read(512)
        backup_sector = struct.unpack_from('<H', primary_boot, 50)[0]
        stream.seek(backup_sector * 512)
        backup_boot = stream.read(512)
        stream.seek(root_directory_offset(image))
        volume_entry = stream.read(32)
    for boot in (primary_boot, backup_boot):
        assert struct.unpack_from('<HH', boot, 24) == (63, 32)
        assert boot[71:82] == b'ARDUPILOT  '
    assert volume_entry[:11] == b'ARDUPILOT  '
    assert volume_entry[11] == 0x08

    fsck = shutil.which('fsck.fat')
    if fsck is not None:
        result = subprocess.run(
            [fsck, '-vn', str(image)], check=False,
            capture_output=True, text=True)
        assert result.returncode == 0, result.stdout + result.stderr
        assert '4096 bytes per cluster' in result.stdout
        assert 'different' not in result.stdout


def test_create_reverse_debug_fat16_image(tmp_path):
    image = tmp_path / 'reverse.img'
    fat_image.create_image(image, 16 * 1024 * 1024, fat_bits=16)

    geometry = fat_image.read_geometry(image)
    assert geometry['fat_bits'] == 16
    assert geometry['bytes_per_sector'] == 512
    assert geometry['number_of_fats'] == 2


def test_existing_image_must_match_geometry(tmp_path):
    image = tmp_path / 'sdcard.img'
    fat_image.create_image(image, 64 * 1024 * 1024, fat_bits=32)

    assert fat_image.create_image(
        image, 64 * 1024 * 1024, fat_bits=32) == image
    with pytest.raises(RuntimeError, match='expected FAT16'):
        fat_image.create_image(image, 64 * 1024 * 1024, fat_bits=16)
    with pytest.raises(RuntimeError, match='move it aside'):
        fat_image.create_image(image, 32 * 1024 * 1024, fat_bits=32)


def test_extract_files(tmp_path, monkeypatch):
    image = tmp_path / 'sdcard.img'
    fat_image.create_image(image, 64 * 1024 * 1024, fat_bits=32)
    with PyFatFS(str(image)) as filesystem:
        filesystem.makedirs('/APM/LOGS')
        filesystem.writebytes('/APM/LOGS/00000001.BIN', b'first log')
        filesystem.writebytes('/APM/LOGS/00000002.BIN', b'second log')
        filesystem.writetext('/APM/LOGS/LASTLOG.TXT', '2\n')

    output_directory = tmp_path / 'logs'
    outputs = fat_image.extract_files(
        image, '/APM/LOGS', '*.BIN', output_directory)
    assert outputs == [
        output_directory / '00000001.BIN',
        output_directory / '00000002.BIN',
    ]
    assert [output.read_bytes() for output in outputs] == [
        b'first log', b'second log']

    crashdump = tmp_path / 'CrashDump.DAT'
    fat_image.extract_file(
        image, '/APM/LOGS/00000002.BIN', crashdump)
    assert crashdump.read_bytes() == b'second log'

    default_output = tmp_path / 'default-output'
    default_output.mkdir()
    monkeypatch.chdir(default_output)
    assert extract_logs.main([str(image)]) == 0
    assert (default_output / '00000001.BIN').read_bytes() == b'first log'
    assert (default_output / '00000002.BIN').read_bytes() == b'second log'
