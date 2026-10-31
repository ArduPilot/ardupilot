# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Host-only symbol placement for independently relocated native modules."""

import io
import json
import struct
import zlib

from pathlib import Path

import pack

from elftools.dwarf.enums import ENUM_DW_LANG
from elftools.elf.elffile import ELFFile


def quote(value):
    value = str(value)
    if any(c in value for c in '\r\n\x00'):
        raise ValueError('debugger paths must not contain newlines or NUL')
    return json.dumps(value)


class ModuleSymbols:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.data = self.path.read_bytes()
        self.module = pack.unpack(self.data)
        if self.data[:8] != pack.RUNTIME_MAGIC:
            raise ValueError('module debugging requires APMOD07')
        self.elf_path = self.path.with_suffix('.elf')
        self.elf_data = self.elf_path.read_bytes()
        if pack.pack_elf(self.elf_data, self.module['contract']) != self.data:
            raise ValueError('companion ELF does not match module')
        elf = ELFFile(io.BytesIO(self.elf_data))
        if elf.get_section_by_name('.debug_info') is None:
            raise ValueError('ELF has no DWARF; rebuild with --debug')
        self.is_rust = any(
            unit.get_top_DIE().attributes.get('DW_AT_language') is not None and
            unit.get_top_DIE().attributes['DW_AT_language'].value == ENUM_DW_LANG['DW_LANG_Rust']
            for unit in elf.get_dwarf_info(relocate_dwarf_sections=False).iter_CUs())
        self.sections = [(s.name, s['sh_addr'], s['sh_size']) for s in elf.iter_sections()
                         if s['sh_flags'] & 2 and s['sh_size']]
        self.base = min(address for _, address, _ in self.sections)
        self.checksum = zlib.crc32(self.data)

    def addresses(self, rx, rw):
        boundary = self.module['rx_size']
        if self.module['machine'] == 62 and rw != rx + boundary:
            raise ValueError('native RX/RW addresses must be contiguous')
        return {name: (rx + address - self.base if address - self.base < boundary else
                       rw + address - self.base - boundary) for name, address, _ in self.sections}

    def debug_elf(self, rx, rw):
        """Fix LLVM RWPI static expressions in a host copy, never in the module.

        Linked ARM DWARF uses const4u(linked VMA), breg9(0), plus for
        static variables. The VMA is not an offset from the module's RW allocation.
        Replace exactly that expression with its absolute runtime address;
        this also permits inspection while a different thread is selected.
        Register-relative local-variable expressions remain unchanged.
        """
        data = bytearray(self.elf_data)
        if self.module['machine'] != 40:
            return data
        elf = ELFFile(io.BytesIO(data))
        # --emit-relocs retains records which the ELF linker already applied.
        dwarf = elf.get_dwarf_info(relocate_dwarf_sections=False)
        section = elf.get_section_by_name('.debug_info')
        start = self.base + self.module['rx_size']
        end = self.base + self.module['memory_size']
        for unit in dwarf.iter_CUs():
            for die in unit.iter_DIEs():
                attr = die.attributes.get('DW_AT_location')
                if attr is None or attr.form != 'DW_FORM_exprloc':
                    continue
                expression = bytes(attr.value)
                if len(expression) != 8 or expression[0] != 0x0c or expression[5:] != b'\x79\x00\x22':
                    continue
                address, = struct.unpack_from('<I', expression, 1)
                if address != 0 and not start <= address < end:
                    raise ValueError('RWPI static DWARF address is outside RW')
                # attr.offset includes the ULEB128 length; this exact expression
                # has a one-byte length and is followed by eight opcode bytes.
                offset = section['sh_offset'] + attr.offset
                if data[offset:offset + 9] != b'\x08' + expression:
                    raise ValueError('unexpected DWARF expression encoding')
                # Linker-GC tombstones for unused bindings remain unavailable,
                # rather than accidentally describing the first RW variable.
                relocated = rw + address - start if address else 0
                # const4u (without stack_value) denotes an absolute memory
                # location. Unlike DW_OP_addr, GDB won't rebase it a second time.
                data[offset + 1:offset + 9] = b'\x0c' + struct.pack('<I', relocated) + b'\x96' * 3
        return data

    def command(self, path, rx, rw):
        addresses = self.addresses(rx, rw)
        command = f'add-symbol-file {quote(path)} 0x{addresses.pop(".text"):x}'
        for name, address in addresses.items():
            command += f' -s {name} 0x{address:x}'
        return command
