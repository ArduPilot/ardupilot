#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Decode and verify the metadata trailer in an SD crash dump."""

import argparse
import dataclasses
import struct
import subprocess
import sys

from typing import Optional

TRAILER_MAGIC = b"APCDUMP\0"
TRAILER_VERSION = 1
TRAILER_STRUCT = struct.Struct("<8sHHIIIII")
TRAILER_CRC_OFFSET = 24
SECTOR_SIZE = 512
ELF_PT_LOAD = 1
ELF_SHT_NOBITS = 8
ELF_SHF_ALLOC = 0x2

FIRMWARE_RANGE_SYMBOLS = (
    "__firmware_crc_start__",
    "__firmware_crc_end__",
    "__firmware_crc_ext_start__",
    "__firmware_crc_ext_end__",
)


class CrashDumpError(Exception):
    """Crash dump metadata or firmware verification failed."""


@dataclasses.dataclass(frozen=True)
class CrashDumpTrailer:
    git_hash: int
    firmware_crc: int
    firmware_size: int
    dump_size: int


@dataclasses.dataclass(frozen=True)
class CrashDumpInfo:
    dump_size: Optional[int]
    trailer: Optional[CrashDumpTrailer]


@dataclasses.dataclass(frozen=True)
class FirmwareIdentity:
    crc: int
    size: int


def _make_crc32_table():
    table = []
    for value in range(256):
        crc = value
        for _ in range(8):
            crc = (crc >> 1) ^ ((-(crc & 1)) & 0xEDB88320)
        table.append(crc)
    return table


CRC32_TABLE = _make_crc32_table()


def crc32(data, crc=0):
    """Return the CRC used by AP_Math's crc_crc32()."""
    for value in data:
        crc = CRC32_TABLE[(crc ^ value) & 0xFF] ^ (crc >> 8)
    return crc


def _valid_dump_size(data, dump_size):
    if not 0 < dump_size < len(data):
        return False
    padding_end = min(dump_size + 16, len(data))
    return all(value == 0xFF for value in data[dump_size:padding_end])


def read_dump_info(dump_file):
    with open(dump_file, "rb") as file:
        data = file.read()

    if len(data) < SECTOR_SIZE or len(data) % SECTOR_SIZE != 0:
        return CrashDumpInfo(None, None)

    raw_trailer = data[-TRAILER_STRUCT.size:]
    values = TRAILER_STRUCT.unpack(raw_trailer)
    magic, version, size, git_hash, firmware_crc, firmware_size, trailer_crc, dump_size = values

    if magic == TRAILER_MAGIC:
        if version != TRAILER_VERSION:
            raise CrashDumpError(f"Unsupported crash dump trailer version {version}")
        if size != TRAILER_STRUCT.size:
            raise CrashDumpError(f"Invalid crash dump trailer size {size}")
        crc_data = bytearray(raw_trailer)
        struct.pack_into("<I", crc_data, TRAILER_CRC_OFFSET, 0)
        calculated_crc = crc32(crc_data)
        if calculated_crc != trailer_crc:
            raise CrashDumpError(
                f"Invalid crash dump trailer CRC 0x{trailer_crc:08x}; "
                f"expected 0x{calculated_crc:08x}"
            )
        if not _valid_dump_size(data, dump_size):
            raise CrashDumpError(f"Invalid crash dump size {dump_size}")
        trailer = CrashDumpTrailer(git_hash, firmware_crc, firmware_size, dump_size)
        return CrashDumpInfo(dump_size, trailer)

    legacy_dump_size = struct.unpack_from("<I", data, len(data) - 4)[0]
    if _valid_dump_size(data, legacy_dump_size):
        return CrashDumpInfo(legacy_dump_size, None)
    return CrashDumpInfo(None, None)


def _read_firmware_symbols(elf_file, nm="arm-none-eabi-nm"):
    result = subprocess.run(
        [nm, "-an", elf_file],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    )
    symbols = {}
    wanted = set(FIRMWARE_RANGE_SYMBOLS)
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[2] in wanted:
            symbols[fields[2]] = int(fields[0], 16)
    missing = wanted - symbols.keys()
    if missing:
        raise CrashDumpError(
            "ELF lacks crash dump firmware CRC symbols: " + ", ".join(sorted(missing))
        )
    return symbols


def _read_elf_load_sections(elf_file):
    with open(elf_file, "rb") as file:
        data = file.read()

    if len(data) < 52 or data[:4] != b"\x7fELF":
        raise CrashDumpError(f"{elf_file} is not an ELF file")
    if data[4] != 1 or data[5] != 1:
        raise CrashDumpError("Only little-endian ELF32 firmware is supported")

    header = struct.unpack_from("<16sHHIIIIIHHHHHH", data)
    program_offset = header[5]
    program_entry_size = header[9]
    program_count = header[10]
    section_offset = header[6]
    section_entry_size = header[11]
    section_count = header[12]
    if program_entry_size < 32:
        raise CrashDumpError("Invalid ELF program header size")
    if section_entry_size < 40:
        raise CrashDumpError("Invalid ELF section header size")

    load_segments = []
    for index in range(program_count):
        offset = program_offset + index * program_entry_size
        if offset + 32 > len(data):
            raise CrashDumpError("ELF program header is outside the file")
        program = struct.unpack_from("<IIIIIIII", data, offset)
        program_type, file_offset, _, physical_address, file_size, _, _, _ = program
        if program_type != ELF_PT_LOAD or file_size == 0:
            continue
        if file_offset + file_size > len(data):
            raise CrashDumpError("ELF load segment is outside the file")
        load_segments.append((file_offset, physical_address, file_size))

    segments = []
    for index in range(section_count):
        offset = section_offset + index * section_entry_size
        if offset + 40 > len(data):
            raise CrashDumpError("ELF section header is outside the file")
        section = struct.unpack_from("<IIIIIIIIII", data, offset)
        _, section_type, section_flags, _, file_offset, size, _, _, _, _ = section
        if section_type == ELF_SHT_NOBITS or (section_flags & ELF_SHF_ALLOC) == 0 or size == 0:
            continue
        if file_offset + size > len(data):
            raise CrashDumpError("ELF section is outside the file")
        for load_offset, physical_address, load_size in load_segments:
            if file_offset < load_offset or file_offset + size > load_offset + load_size:
                continue
            section_address = physical_address + file_offset - load_offset
            segments.append((section_address, data[file_offset:file_offset + size]))
            break
    return segments


def _read_elf_range(segments, start, end):
    if end <= start:
        return b""
    result = bytearray(b"\xFF" * (end - start))
    found_segment = False
    for segment_start, segment_data in segments:
        segment_end = segment_start + len(segment_data)
        copy_start = max(start, segment_start)
        copy_end = min(end, segment_end)
        if copy_end <= copy_start:
            continue
        found_segment = True
        source_offset = copy_start - segment_start
        target_offset = copy_start - start
        result[target_offset:target_offset + copy_end - copy_start] = (
            segment_data[source_offset:source_offset + copy_end - copy_start]
        )
    if not found_segment:
        raise CrashDumpError(f"ELF has no load segment for 0x{start:08x}-0x{end:08x}")
    return result


def firmware_identity_from_elf(elf_file):
    symbols = _read_firmware_symbols(elf_file)
    segments = _read_elf_load_sections(elf_file)
    ranges = (
        (symbols["__firmware_crc_start__"], symbols["__firmware_crc_end__"]),
        (symbols["__firmware_crc_ext_start__"], symbols["__firmware_crc_ext_end__"]),
    )

    firmware_crc = 0
    firmware_size = 0
    for start, end in ranges:
        range_data = _read_elf_range(segments, start, end)
        firmware_crc = crc32(range_data, firmware_crc)
        firmware_size += len(range_data)
    return FirmwareIdentity(firmware_crc, firmware_size)


def verify_elf(trailer, elf_file):
    identity = firmware_identity_from_elf(elf_file)
    if identity.crc != trailer.firmware_crc or identity.size != trailer.firmware_size:
        raise CrashDumpError(
            "Firmware mismatch: crash dump is from "
            f"git {trailer.git_hash:08x}, CRC 0x{trailer.firmware_crc:08x}, "
            f"{trailer.firmware_size} bytes; ELF has CRC 0x{identity.crc:08x}, "
            f"{identity.size} bytes"
        )
    return identity


def describe_trailer(trailer):
    return (
        f"Crash dump firmware: git {trailer.git_hash:08x}, "
        f"CRC 0x{trailer.firmware_crc:08x}, {trailer.firmware_size} bytes"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf_file")
    parser.add_argument("dump_file")
    args = parser.parse_args()

    try:
        info = read_dump_info(args.dump_file)
        if info.trailer is not None:
            verify_elf(info.trailer, args.elf_file)
            print(describe_trailer(info.trailer) + " (ELF matches)", file=sys.stderr)
        elif info.dump_size is not None:
            print("Warning: legacy crash dump has no firmware identity trailer", file=sys.stderr)
        if info.dump_size is not None:
            print(info.dump_size)
    except (CrashDumpError, OSError, subprocess.SubprocessError) as error:
        print(f"crashdump_info.py: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
