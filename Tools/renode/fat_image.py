#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Create and read FAT images without external host filesystem tools."""

import contextlib
import ctypes
import fnmatch
import os
import shutil
import struct
import tempfile

from pathlib import Path

try:
    from fs.errors import FSError
    from pyfatfs import PyFATException
    from pyfatfs.PyFat import PyFat
    from pyfatfs.PyFatFS import PyFatFS
except ImportError as error:
    FSError = OSError
    PyFat = None
    PyFatFS = None
    PyFATException = OSError
    PYFATFS_IMPORT_ERROR = error
else:
    PYFATFS_IMPORT_ERROR = None


SECTOR_SIZE = 512
FAT32_SECTORS_PER_CLUSTER = 8
FSCTL_SET_SPARSE = 0x000900C4


def _require_pyfatfs():
    if PYFATFS_IMPORT_ERROR is not None:
        raise RuntimeError(
            'pyfatfs 1.1.0 is required for Renode SD images '
            '(install it with: python3 -m pip install pyfatfs==1.1.0)') \
            from PYFATFS_IMPORT_ERROR


def _mark_windows_sparse(stream):
    """Mark a new Windows file sparse before extending it."""
    if os.name != 'nt':
        return

    import msvcrt

    from ctypes import wintypes

    kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel32.DeviceIoControl.argtypes = (
        wintypes.HANDLE,
        wintypes.DWORD,
        wintypes.LPVOID,
        wintypes.DWORD,
        wintypes.LPVOID,
        wintypes.DWORD,
        ctypes.POINTER(wintypes.DWORD),
        wintypes.LPVOID,
    )
    kernel32.DeviceIoControl.restype = wintypes.BOOL
    returned = wintypes.DWORD()
    handle = msvcrt.get_osfhandle(stream.fileno())
    if not kernel32.DeviceIoControl(
            handle, FSCTL_SET_SPARSE, None, 0, None, 0,
            ctypes.byref(returned), None):
        raise ctypes.WinError(ctypes.get_last_error())


def read_geometry(path):
    """Return the FAT geometry needed to validate a generated image."""
    path = Path(path)
    with path.open('rb') as image:
        boot = image.read(SECTOR_SIZE)
    if len(boot) != SECTOR_SIZE or boot[510:512] != b'\x55\xaa':
        raise RuntimeError('%s has no valid FAT boot sector' % path)

    bytes_per_sector = struct.unpack_from('<H', boot, 11)[0]
    sectors_per_cluster = boot[13]
    reserved_sectors = struct.unpack_from('<H', boot, 14)[0]
    number_of_fats = boot[16]
    root_entries = struct.unpack_from('<H', boot, 17)[0]
    total_sectors_16 = struct.unpack_from('<H', boot, 19)[0]
    fat_sectors_16 = struct.unpack_from('<H', boot, 22)[0]
    total_sectors_32 = struct.unpack_from('<I', boot, 32)[0]
    fat_sectors_32 = struct.unpack_from('<I', boot, 36)[0]
    total_sectors = total_sectors_16 or total_sectors_32
    fat_sectors = fat_sectors_16 or fat_sectors_32
    if (bytes_per_sector == 0 or sectors_per_cluster == 0 or
            total_sectors == 0 or fat_sectors == 0):
        raise RuntimeError('%s has invalid FAT geometry' % path)

    root_dir_sectors = (
        root_entries * 32 + bytes_per_sector - 1) // bytes_per_sector
    first_data_sector = (
        reserved_sectors + number_of_fats * fat_sectors + root_dir_sectors)
    data_clusters = (
        total_sectors - first_data_sector) // sectors_per_cluster
    if data_clusters < 4085:
        fat_bits = 12
    elif data_clusters < 65525:
        fat_bits = 16
    else:
        fat_bits = 32

    return {
        'fat_bits': fat_bits,
        'bytes_per_sector': bytes_per_sector,
        'sectors_per_cluster': sectors_per_cluster,
        'reserved_sectors': reserved_sectors,
        'number_of_fats': number_of_fats,
        'fat_sectors': fat_sectors,
        'root_entries': root_entries,
        'total_sectors': total_sectors,
        'data_clusters': data_clusters,
    }


def _patch_compatibility(path, fat_bits, label):
    """Normalize fields that pyfatfs 1.1.0 leaves host-tool unfriendly."""
    label_bytes = label.encode('ascii').ljust(11, b' ')
    with Path(path).open('r+b') as image:
        boot = bytearray(image.read(SECTOR_SIZE))
        bytes_per_sector = struct.unpack_from('<H', boot, 11)[0]
        sectors_per_cluster = boot[13]
        reserved_sectors = struct.unpack_from('<H', boot, 14)[0]
        number_of_fats = boot[16]
        fat_sectors_16 = struct.unpack_from('<H', boot, 22)[0]
        fat_sectors_32 = struct.unpack_from('<I', boot, 36)[0]
        fat_sectors = fat_sectors_16 or fat_sectors_32

        boot_sectors = [0]
        if fat_bits == 32:
            backup_sector = struct.unpack_from('<H', boot, 50)[0]
            if backup_sector:
                boot_sectors.append(backup_sector)
            root_cluster = struct.unpack_from('<I', boot, 44)[0]
            first_data_sector = reserved_sectors + number_of_fats * fat_sectors
            root_sector = (
                first_data_sector +
                (root_cluster - 2) * sectors_per_cluster)
        else:
            root_sector = reserved_sectors + number_of_fats * fat_sectors

        for boot_sector in boot_sectors:
            image.seek(boot_sector * bytes_per_sector + 24)
            image.write(struct.pack('<HH', 63, 32))

        root_offset = root_sector * bytes_per_sector
        image.seek(root_offset)
        volume_entry = bytearray(image.read(32))
        if len(volume_entry) != 32 or volume_entry[11] != 0x08:
            raise RuntimeError('%s has no root volume-label entry' % path)
        volume_entry[:11] = label_bytes
        image.seek(root_offset)
        image.write(volume_entry)


def _validate_geometry(path, size, fat_bits, sectors_per_cluster):
    geometry = read_geometry(path)
    if Path(path).stat().st_size != size:
        raise RuntimeError('%s is not %u bytes' % (path, size))
    if geometry['fat_bits'] != fat_bits:
        raise RuntimeError(
            '%s is FAT%u; expected FAT%u' %
            (path, geometry['fat_bits'], fat_bits))
    if geometry['bytes_per_sector'] != SECTOR_SIZE:
        raise RuntimeError(
            '%s uses %u-byte sectors; expected %u' %
            (path, geometry['bytes_per_sector'], SECTOR_SIZE))
    if (sectors_per_cluster is not None and
            geometry['sectors_per_cluster'] != sectors_per_cluster):
        raise RuntimeError(
            '%s uses %u sectors per cluster; expected %u' %
            (path, geometry['sectors_per_cluster'], sectors_per_cluster))
    return geometry


def create_image(path, size, fat_bits=32, sectors_per_cluster=None,
                 label='ARDUPILOT'):
    """Create a persistent sparse FAT image without replacing existing state."""
    path = Path(path)
    if path.exists():
        if path.stat().st_size != size:
            raise RuntimeError(
                '%s is %u bytes; expected %u (move it aside to reinitialize)' %
                (path, path.stat().st_size, size))
        _validate_geometry(path, size, fat_bits, sectors_per_cluster)
        return path

    _require_pyfatfs()
    if not 1 <= len(label) <= 11 or not label.isascii():
        raise ValueError('FAT volume label must contain 1 to 11 ASCII characters')
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open('xb+') as stream:
            _mark_windows_sparse(stream)

        fat = PyFat()
        try:
            fat.mkfs(
                str(path), fat_bits, size=size, sector_size=SECTOR_SIZE,
                number_of_fats=2, label=label)
        except Exception:
            with contextlib.suppress(Exception):
                fat.close()
            raise
        fat.close()
        _patch_compatibility(path, fat_bits, label)
        _validate_geometry(path, size, fat_bits, sectors_per_cluster)
    except Exception:
        with contextlib.suppress(OSError):
            path.unlink()
        raise
    return path


def _copy_member(filesystem, source, output):
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix='.%s-' % output.name, dir=output.parent)
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        with filesystem.openbin(source, 'r') as source_stream, \
                temporary.open('wb') as output_stream:
            shutil.copyfileobj(source_stream, output_stream, 1024 * 1024)
        temporary.chmod(0o644)
        os.replace(temporary, output)
    except Exception:
        with contextlib.suppress(OSError):
            temporary.unlink()
        raise


def extract_file(image, source, output):
    """Extract one file from a FAT image, replacing the destination atomically."""
    _require_pyfatfs()
    try:
        with PyFatFS(str(image), read_only=True) as filesystem:
            _copy_member(filesystem, source, output)
    except (FSError, OSError, PyFATException) as error:
        raise RuntimeError(
            'failed to extract %s from %s: %s' %
            (source, image, error)) from error
    return Path(output)


def extract_files(image, directory, pattern, output_directory):
    """Extract matching regular files from one directory in a FAT image."""
    _require_pyfatfs()
    output_directory = Path(output_directory)
    try:
        with PyFatFS(str(image), read_only=True) as filesystem:
            names = sorted(
                name for name in filesystem.listdir(directory)
                if fnmatch.fnmatchcase(name.upper(), pattern.upper()) and
                filesystem.isfile('%s/%s' % (directory.rstrip('/'), name)))
            outputs = []
            for name in names:
                if Path(name).name != name:
                    raise RuntimeError('unsafe FAT directory entry %s' % name)
                output = output_directory / name
                _copy_member(
                    filesystem, '%s/%s' % (directory.rstrip('/'), name),
                    output)
                outputs.append(output)
            return outputs
    except (FSError, OSError, PyFATException) as error:
        raise RuntimeError(
            'failed to extract %s/%s from %s: %s' %
            (directory, pattern, image, error)) from error
