#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Convert a fully linked, --emit-relocs ELF to an experimental module image.

This is an offline prototype, not a stable module ABI. All runtime fixups are
internal absolute pointers; unsupported compiler output is rejected.
"""

import argparse
import io
import json
import re
import struct
import zlib

from pathlib import Path

from elftools.elf.elffile import ELFFile
from elftools.elf.relocation import RelocationSection
from elftools.elf.sections import SymbolTableSection
from targets import CPU_PROFILES

MAGIC = b'APMOD01\0'
HEADER = struct.Struct('<8s12I')
RUNTIME_MAGIC = b'APMOD07\0'
RUNTIME_HEADER = struct.Struct('<8s14I16s2I')
FIXUP = struct.Struct('<I')
LEGACY_FIXUP = struct.Struct('<II')
IMPORT = struct.Struct('<QII')
PAGE = 4096
MAX_IMAGE = 16 * 1024 * 1024
MAX_IMPORTS = 4096
# The linker has already resolved these internal PC-relative references.
PROFILES = {
    'EM_ARM': (40, 4, 2, {3, 10, 30, 42, 49, 50, 87, 88}),
    'EM_X86_64': (62, 8, 1, {2, 4, 9, 41, 42}),
}
SECTIONS = {'.text', '.rodata', '.ARM.exidx', '.data', '.got', '.bss'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate_contract(machine, abi, profile, stack, fingerprint, heap, thread_priority=1):
    require(type(abi) is int and 0 < abi <= 0xFFFFFFFF, 'invalid ABI version')
    require(type(profile) is int and profile in CPU_PROFILES and CPU_PROFILES[profile].machine == machine,
            'invalid CPU profile')
    require(type(stack) is int and 256 <= stack <= MAX_IMAGE, 'invalid stack requirement')
    require(type(heap) is int and 256 <= heap <= MAX_IMAGE, 'invalid initial heap requirement')
    require(type(thread_priority) is int and 1 <= thread_priority <= 254, 'invalid module thread priority')
    require(len(fingerprint) == 16 and fingerprint != bytes(16), 'invalid API fingerprint')


def pack_elf(data, manifest=None):
    elf = ELFFile(io.BytesIO(data))
    require(elf.little_endian and elf['e_type'] == 'ET_EXEC', 'expected little-endian ET_EXEC')
    require(elf['e_machine'] in PROFILES, 'unsupported architecture')
    machine, width, absolute, relative = PROFILES[elf['e_machine']]
    require(elf.elfclass == width * 8, 'incorrect pointer width')
    if machine == 40:
        require(elf['e_flags'] & 0xFF000600 == 0x05000400, 'expected ARM EABI5 hard float')
    sections = [(index, section) for index, section in enumerate(elf.iter_sections())
                if section['sh_flags'] & 2 and section['sh_size']]
    require(sections, 'empty image')
    sections.sort(key=lambda item: item[1]['sh_addr'])
    base = sections[0][1]['sh_addr']
    end = max(s['sh_addr'] + s['sh_size'] for _, s in sections)
    require(base % PAGE == 0 and 0 < end - base <= MAX_IMAGE, 'invalid image extent')
    memory_size = end - base
    file_size = max(s['sh_addr'] + s['sh_size'] - base for _, s in sections if s['sh_type'] != 'SHT_NOBITS')
    writable = [s['sh_addr'] - base for _, s in sections if s['sh_flags'] & 1]
    boundary = width if manifest is not None and machine == 40 else PAGE
    rx_size = min(writable) if writable else (memory_size + boundary - 1) // boundary * boundary
    require(rx_size > 0 and rx_size % boundary == 0, 'invalid RX/RW boundary alignment')
    # The RX byte count is also the logical RW offset. Only POSIX and legacy
    # images need a page boundary; split ARM images retain ELF section alignment.
    file_size = max(file_size, rx_size)
    memory_size = max(memory_size, rx_size)
    # APMOD07 allocates RW separately on ARM, without requiring page padding.
    # Each half of the alignment word preserves one segment's ELF alignment.
    rw_alignment = max([width] + [s['sh_addralign'] for _, s in sections if s['sh_flags'] & 1])
    rx_alignment = max([width] + [s['sh_addralign'] for _, s in sections if not s['sh_flags'] & 1])
    image = bytearray(file_size)
    imports = []
    previous_end = base
    seen_bss = False
    section_ids = {i for i, _ in sections}
    for _, section in sections:
        address, size = section['sh_addr'], section['sh_size']
        import_section = re.fullmatch(
            r'\.ap_module_import_([0-9a-f]{16})\.([A-Za-z0-9_.]{1,63})', section.name)
        require(section.name in SECTIONS or import_section is not None, f'unsupported allocated section {section.name}')
        require(section.name != '.got' or machine == 62, 'ARM GOT is unsupported')
        require(section['sh_type'] in ('SHT_PROGBITS', 'SHT_NOBITS', 'SHT_ARM_EXIDX'), 'unsupported section type')
        require(not section['sh_flags'] & 0x400, 'TLS is unsupported')
        require(section['sh_addralign'] <= PAGE, 'excessive section alignment')
        require(address >= previous_end, 'overlapping sections')
        previous_end = address + size
        offset = address - base
        if section['sh_flags'] & 1:
            require(not section['sh_flags'] & 4 and offset >= rx_size, 'writable executable section')
            require((offset - rx_size) % max(1, section['sh_addralign']) == 0, 'misaligned RW section')
        else:
            require(offset + size <= rx_size, 'read-only section follows writable data')
        if section['sh_type'] != 'SHT_NOBITS':
            image[offset:offset + size] = section.data()
        else:
            seen_bss = True
        if import_section is not None:
            require(section['sh_type'] == 'SHT_PROGBITS' and section['sh_flags'] & 1 and size == 4,
                    'invalid import section')
            require(not seen_bss, 'import section follows BSS')
            imports.append((int(import_section[1], 16), offset, import_section[2]))
    entry = elf['e_entry'] - base
    code_entry = entry & ~1 if machine == 40 else entry
    require(any(s['sh_flags'] & 4 and s['sh_addr'] - base <= code_entry < s['sh_addr'] - base + s['sh_size']
                for _, s in sections), 'entry is outside executable code')
    require(machine != 40 or entry & 1, 'ARM entry must be Thumb')
    symbol_values = set()
    symbol_table = elf.get_section_by_name('.symtab')
    require(isinstance(symbol_table, SymbolTableSection), 'unstripped ELF symbol table required')
    for symbol in symbol_table.iter_symbols():
        require(not symbol.name or symbol['st_shndx'] != 'SHN_UNDEF', f'undefined symbol {symbol.name}')
        if symbol['st_shndx'] in section_ids:
            symbol_values.add(symbol['st_value'])
            import_symbol = re.fullmatch(
                r'__ap_module_import_([0-9a-f]{16})\.([A-Za-z0-9_.]{1,63})', symbol.name)
            if import_symbol is not None:
                import_section = elf.get_section(symbol['st_shndx'])
                require(symbol['st_size'] == 4 and import_section['sh_type'] == 'SHT_PROGBITS'
                        and import_section['sh_flags'] & 1, 'invalid import symbol')
                imports.append((int(import_symbol[1], 16), symbol['st_value'] - base, import_symbol[2]))
    fixups = []
    destinations = set()
    relocation_sections = 0
    for section in elf.iter_sections():
        if not isinstance(section, RelocationSection) or section['sh_info'] not in section_ids:
            continue
        relocation_sections += 1
        target = elf.get_section(section['sh_info'])
        symbols = elf.get_section(section['sh_link'])
        for reloc in section.iter_relocations():
            kind = reloc['r_info_type']
            if kind == 0:
                continue
            require(kind == absolute or kind in relative, f'unsupported relocation {kind} in {target.name}')
            symbol = symbols.get_symbol(reloc['r_info_sym'])
            require(symbol['st_shndx'] in section_ids, f'non-image relocation symbol {symbol.name}')
            offset = reloc['r_offset'] - base
            size = width if kind == absolute else 4
            require(target['sh_type'] != 'SHT_NOBITS'
                    and target['sh_addr'] <= base + offset
                    and base + offset + size <= target['sh_addr'] + target['sh_size'], 'fixup outside section')
            require(not destinations.intersection(range(offset, offset + size)), 'overlapping relocations')
            destinations.update(range(offset, offset + size))
            if kind != absolute:
                if machine == 40:
                    symbol_address = symbol['st_value'] & ~1
                    require(offset + size <= rx_size, 'instruction relocation outside RX image')
                    if kind in (87, 88):
                        require(symbol_address >= base + rx_size, 'BREL relocation target is not RW')
                    else:
                        require(symbol_address < base + rx_size, 'PC-relative relocation target is not RX')
                continue
            require(offset % width == 0, 'unaligned absolute relocation')
            value = int.from_bytes(image[offset:offset + width], 'little')
            require(base <= value <= end, 'absolute pointer outside image')
            target_offset = value - base
            require(not (manifest is not None and machine == 40 and writable and target_offset == rx_size and
                         not elf.get_section(symbol['st_shndx'])['sh_flags'] & 1),
                    'ambiguous one-past RX pointer at RW boundary')
            # Bit zero also occurs in ordinary byte pointers. Keep its value;
            # an even RX/RW boundary already distinguishes the two segments.
            require(not (machine == 40 and offset < rx_size and target_offset >= rx_size),
                    'ARM RX relocation targets RW')
            fixups.append((offset, target_offset))
            image[offset:offset + width] = target_offset.to_bytes(width, 'little')
    require(relocation_sections > 0, 'no retained relocations: link with --emit-relocs')
    # Static x86-64 linkers can create GOT slots without retained relocations.
    # Each slot must be the exact address of a defined image symbol. No lazy
    # binding, reserved .got.plt slots, imports or arbitrary constants allowed.
    for _, section in sections:
        if section.name != '.got':
            continue
        offset = section['sh_addr'] - base
        require(offset % width == 0 and section['sh_size'] % width == 0, 'unaligned GOT')
        for slot in range(offset, offset + section['sh_size'], width):
            require(not destinations.intersection(range(slot, slot + width)), 'relocated GOT slot')
            value = int.from_bytes(image[slot:slot + width], 'little')
            require(base <= value < end and value in symbol_values, 'GOT slot is not an internal symbol')
            fixups.append((slot, value - base))
            image[slot:slot + width] = (value - base).to_bytes(width, 'little')
    fixups.sort()
    imports.sort(key=lambda item: item[1])
    require(len(imports) <= MAX_IMPORTS, 'too many imports')
    hashes = set()
    previous_end = 0
    for binding_hash, offset, _ in imports:
        require(offset >= max(previous_end, rx_size) and offset % 4 == 0 and offset + 4 <= file_size,
                'invalid import offset')
        require(image[offset:offset + 4] == b'\xff' * 4, 'invalid import placeholder')
        require(not destinations.intersection(range(offset, offset + 4)), 'relocated import slot')
        require(binding_hash not in hashes, 'duplicate import hash')
        hashes.add(binding_hash)
        previous_end = offset + 4
    if manifest is not None:
        body = bytes(image) + b''.join(FIXUP.pack(offset) for offset, _ in fixups)
        names = bytearray()
        records = []
        for binding_hash, offset, name in imports:
            name_offset = len(names)
            names += name.encode('ascii') + b'\0'
            records.append((binding_hash, offset, name_offset))
        body += b''.join(IMPORT.pack(*item) for item in records) + names
        require(isinstance(manifest, dict), 'manifest must be an object')
        require(set(manifest) == {'abi_version', 'cpu_profile', 'stack_size', 'heap_size', 'fingerprint', 'thread_priority'},
                'invalid manifest fields')
        require(isinstance(manifest['fingerprint'], str), 'invalid API fingerprint')
        try:
            fingerprint = bytes.fromhex(manifest['fingerprint'])
        except ValueError as error:
            raise ValueError('invalid API fingerprint') from error
        abi, profile, stack = (manifest[key] for key in ('abi_version', 'cpu_profile', 'stack_size'))
        heap = manifest['heap_size']
        priority = manifest['thread_priority']
        validate_contract(machine, abi, profile, stack, fingerprint, heap, priority)
        fields = (7, machine, width, rw_alignment | rx_alignment << 16, file_size, memory_size, rx_size, entry, len(fixups),
                  abi, profile, stack, heap, len(imports), fingerprint, priority)
        header = RUNTIME_HEADER.pack(RUNTIME_MAGIC, *fields, 0)
        return RUNTIME_HEADER.pack(RUNTIME_MAGIC, *fields, zlib.crc32(header + body)) + body
    # Untagged containers remain restricted to the original relocation probes.
    require(not imports, 'binding imports require a runtime manifest')
    legacy_image = bytearray(image)
    for offset, _ in fixups:
        legacy_image[offset:offset + width] = bytes(width)
    body = bytes(legacy_image) + b''.join(LEGACY_FIXUP.pack(*f) for f in fixups)
    fields = (1, machine, width, PAGE, file_size, memory_size, rx_size, entry, len(fixups), 0, 0, 0)
    header = HEADER.pack(MAGIC, *fields)
    crc = zlib.crc32(header + body)
    return HEADER.pack(MAGIC, *fields[:-1], crc) + body


def unpack(data):
    require(len(data) >= HEADER.size, 'truncated header')
    contract = None
    import_count = 0
    runtime_magic = data[:8]
    if runtime_magic == RUNTIME_MAGIC:
        require(len(data) >= RUNTIME_HEADER.size, 'truncated header')
        header_size = RUNTIME_HEADER.size
        (magic, version, machine, width, alignment, file_size, memory_size, rx_size,
         entry, count, abi, profile, stack, heap, import_count, fp, priority, crc) = RUNTIME_HEADER.unpack_from(data)
        require(version == 7, 'invalid module version')
        validate_contract(machine, abi, profile, stack, fp, heap, priority)
        contract = dict(abi_version=abi, cpu_profile=profile, stack_size=stack, heap_size=heap, fingerprint=fp.hex(),
                        thread_priority=priority)
    else:
        header_size = HEADER.size
        magic, version, machine, width, alignment, file_size, memory_size, rx_size, entry, count, abi, flags, crc = \
            HEADER.unpack_from(data)
        require(magic == MAGIC and version == 1, 'invalid module version')
        require(abi == 0 and flags == 0, 'unsupported module profile')
    require((machine, width) in ((40, 4), (62, 8)), 'unsupported module target')
    compact = runtime_magic == RUNTIME_MAGIC
    if compact:
        rw_alignment, rx_alignment = alignment & 0xffff, alignment >> 16
        require(all(width <= value <= PAGE and value & (value - 1) == 0
                    for value in (rw_alignment, rx_alignment)), 'unsupported module alignment')
    else:
        require(alignment == PAGE, 'unsupported module alignment')
        rw_alignment = rx_alignment = PAGE
    require(0 < file_size <= memory_size <= MAX_IMAGE, 'invalid module sizes')
    boundary = width if compact and machine == 40 else PAGE
    require(0 < rx_size <= (memory_size + boundary - 1) // boundary * boundary and rx_size % boundary == 0,
            'invalid code boundary')
    require(entry < min(rx_size, file_size) and (machine != 40 or entry & 1), 'invalid entry')
    require(count <= file_size // width, 'too many relocations')
    require(import_count <= min(file_size // 4, MAX_IMPORTS), 'too many imports')
    if compact:
        require(rx_size <= file_size and rx_size <= memory_size, 'invalid split image sizes')
    fixup_record = FIXUP if compact else LEGACY_FIXUP
    import_record = IMPORT
    metadata_end = header_size + file_size + count * fixup_record.size + import_count * import_record.size
    require(len(data) >= metadata_end, 'incorrect file length')
    require(zlib.crc32(data[:header_size - 4] + bytes(4) + data[header_size:]) == crc, 'checksum mismatch')
    image = data[header_size:header_size + file_size]
    fixups = []
    previous_end = 0
    for index in range(count):
        record_offset = header_size + file_size + index * fixup_record.size
        if compact:
            offset, = FIXUP.unpack_from(data, record_offset)
            require(offset + width <= file_size, 'invalid relocation offset')
            value = int.from_bytes(image[offset:offset + width], 'little')
        else:
            offset, value = LEGACY_FIXUP.unpack_from(data, record_offset)
        require(previous_end <= offset and offset + width <= file_size, 'invalid relocation offset')
        require(offset % width == 0, 'unaligned relocation offset')
        require(value <= memory_size, 'invalid relocation value')
        if compact:
            require(not (machine == 40 and offset < rx_size and value >= rx_size),
                    'ARM RX relocation targets RW')
        else:
            require(image[offset:offset + width] == bytes(width), 'nonzero relocation slot')
        previous_end = offset + width
        fixups.append((offset, value))
    imports = []
    hashes = set()
    previous_end = rx_size
    fixup_index = 0
    names = data[metadata_end:]
    previous_name_end = 0
    for index in range(import_count):
        record_offset = header_size + file_size + count * fixup_record.size + index * import_record.size
        binding_hash, offset, name_offset = IMPORT.unpack_from(data, record_offset)
        require(name_offset == previous_name_end and name_offset < len(names), 'invalid import name offset')
        name_end = names.find(b'\0', name_offset)
        require(name_end > name_offset and name_end - name_offset <= 63, 'invalid import name')
        encoded_name = names[name_offset:name_end]
        require(re.fullmatch(rb'[A-Za-z0-9_.]+', encoded_name) is not None, 'invalid import name')
        name = encoded_name.decode('ascii')
        previous_name_end = name_end + 1
        require(previous_end <= offset and offset + 4 <= file_size and offset % 4 == 0, 'invalid import offset')
        require(image[offset:offset + 4] == b'\xff' * 4, 'invalid import placeholder')
        require(binding_hash not in hashes, 'duplicate import hash')
        while fixup_index < len(fixups) and fixups[fixup_index][0] + width <= offset:
            fixup_index += 1
        require(fixup_index == len(fixups) or fixups[fixup_index][0] >= offset + 4, 'relocated import slot')
        hashes.add(binding_hash)
        imports.append((binding_hash, offset, name))
        previous_end = offset + 4
    require(previous_name_end == len(names), 'incorrect import names length')
    return dict(machine=machine, width=width, image=image, memory_size=memory_size,
                rx_size=rx_size, rw_alignment=rw_alignment, rx_alignment=rx_alignment,
                entry=entry, fixups=fixups, imports=imports, contract=contract)


def relocate(data, base, rw_base=None):
    module = unpack(data)
    rx_alignment = module['rx_alignment'] if data[:8] == RUNTIME_MAGIC and module['machine'] == 40 else PAGE
    require(base >= 0 and base % rx_alignment == 0, 'unaligned load address')
    if rw_base is None:
        rw_base = base + module['rx_size']
    rw_alignment = module['rw_alignment']
    require(rw_base >= 0 and rw_base % rw_alignment == 0, 'unaligned RW load address')
    require(base + module['rx_size'] < 1 << (module['width'] * 8), 'load address overflow')
    require(rw_base + module['memory_size'] - module['rx_size'] < 1 << (module['width'] * 8),
            'load address overflow')
    require(module['machine'] != 62 or rw_base == base + module['rx_size'],
            'x86-64 requires contiguous RX/RW images')
    image = bytearray(module['image'])
    image.extend(bytes(module['memory_size'] - len(image)))
    for offset, value in module['fixups']:
        target_base = base if value < module['rx_size'] else rw_base - module['rx_size']
        image[offset:offset + module['width']] = (target_base + value).to_bytes(module['width'], 'little')
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--manifest', type=Path, help='SDK ABI/profile/stack/fingerprint JSON; omit only for legacy probes')
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text()) if args.manifest else None
    if args.manifest:
        require(isinstance(manifest, dict), 'manifest must be an object')
    args.output.write_bytes(pack_elf(args.elf.read_bytes(), manifest))


if __name__ == '__main__':
    main()
