#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Inspect a native module and optionally change its stack, initial heap or thread priority."""

import argparse
import io
import os
import stat
import struct
import tempfile
import zlib

from pathlib import Path

import pack

from elftools.common.exceptions import ELFError
from elftools.elf.elffile import ELFFile
from targets import CPU_PROFILES

MACHINES = {
    40: 'ARM',
    62: 'x86-64',
}


def resize_resources(data, stack_size=None, heap_size=None, thread_priority=None):
    module = pack.unpack(data)
    contract = module['contract']
    if contract is None:
        raise ValueError('cannot change resource sizes in a relocation probe without a runtime contract')
    stack_size = contract['stack_size'] if stack_size is None else stack_size
    heap_size = contract['heap_size'] if heap_size is None else heap_size
    thread_priority = contract['thread_priority'] if thread_priority is None else thread_priority
    pack.validate_contract(module['machine'], contract['abi_version'], contract['cpu_profile'],
                           stack_size, bytes.fromhex(contract['fingerprint']), heap_size, thread_priority)
    fields = list(pack.RUNTIME_HEADER.unpack_from(data))
    # APMOD07 fields 12/13 declare stack/heap; field 16 is native priority.
    fields[12:14] = stack_size, heap_size
    fields[16] = thread_priority
    fields[-1] = 0
    body = data[pack.RUNTIME_HEADER.size:]
    fields[-1] = zlib.crc32(pack.RUNTIME_HEADER.pack(*fields) + body)
    return pack.RUNTIME_HEADER.pack(*fields) + body


def replace_module(path, data):
    # Replace atomically, following a symlink to the requested module rather
    # than replacing the link itself. Keep the original file permissions.
    path = path.resolve()
    mode = stat.S_IMODE(path.stat().st_mode)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f'.{path.name}.', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.chmod(mode)
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def file_breakdown(data, module, elf_data=None):
    runtime = module['contract'] is not None
    compact = data[:8] == pack.RUNTIME_MAGIC
    sizes = {'Container header': pack.RUNTIME_HEADER.size if runtime else pack.HEADER.size}
    if elf_data is None:
        sizes['Image (code/data/padding)'] = len(module['image'])
    else:
        try:
            rebuilt = pack.pack_elf(elf_data, module['contract'])
        except ELFError as error:
            raise ValueError(f'invalid companion ELF: {error}') from error
        if rebuilt != data:
            raise ValueError('companion ELF does not match module; use the matching ELF or --no-elf')
        elf = ELFFile(io.BytesIO(elf_data))
        sizes.update({'Code': 0, 'Read-only data': 0, 'Read-only metadata': 0, 'Initialized RW data': 0})
        for section in elf.iter_sections():
            if not section['sh_flags'] & 2 or section['sh_type'] == 'SHT_NOBITS':
                continue
            if section['sh_flags'] & 4:
                category = 'Code'
            elif section['sh_flags'] & 1:
                category = 'Initialized RW data'
            elif section.name == '.ARM.exidx':
                category = 'Read-only metadata'
            else:
                category = 'Read-only data'
            sizes[category] += section['sh_size']
        section_size = sum(value for key, value in sizes.items() if key != 'Container header')
        sizes['Image padding'] = len(module['image']) - section_size
    sizes['Relocation table'] = len(module['fixups']) * (pack.FIXUP.size if compact else pack.LEGACY_FIXUP.size)
    sizes['Import table'] = len(module['imports']) * pack.IMPORT.size
    sizes['Import names'] = len(data) - sum(sizes.values())
    return sizes


def format_module(path, data, show_fixups=False, elf_data=None, elf_path=None):
    module = pack.unpack(data)
    magic = data[:8].rstrip(b'\0').decode('ascii')
    version = struct.unpack_from('<I', data, 8)[0]
    contract = module['contract']
    machine = module['machine']
    width = module['width']
    image_size = len(module['image'])
    memory_size = module['memory_size']
    rx_size = module['rx_size']
    rw_file_size = image_size - rx_size
    rw_memory_size = memory_size - rx_size
    rx_fixups = sum(offset < rx_size for offset, _ in module['fixups'])

    lines = [
        f'Module: {path}',
        f'Format: {magic} (version {version})',
        f'File size: {len(data)} bytes',
        f'Machine: {MACHINES.get(machine, "unknown")} ({machine})',
        f'Pointer width: {width * 8} bits',
    ]
    if contract is not None:
        profile = contract['cpu_profile']
        cpu = CPU_PROFILES[profile]
        lines += [
            f'Target profile: {cpu.target} ({profile})',
            f'CPU requirement: {cpu.cpu}' + (f' ({cpu.fpu}, hard float)' if cpu.fpu else ''),
            f'ABI version: {contract["abi_version"]}',
            f'API fingerprint: {contract["fingerprint"]}',
            f'Stack requirement: {contract["stack_size"]} bytes',
            f'Initial heap: {contract["heap_size"]} bytes',
            f'Thread priority: {contract["thread_priority"]}',
        ]
    else:
        lines.append('Runtime contract: none (relocation probe)')
    lines.append('File breakdown:')
    if elf_data is not None:
        lines.append(f'  Section sizes from verified ELF: {elf_path or "<provided>"}')
    else:
        lines.append('  Code/data/padding split unavailable without matching ELF.')
    lines += [f'  {label}: {size} bytes' for label, size in file_breakdown(data, module, elf_data).items()]
    lines.append(f'  Total: {len(data)} bytes')
    lines += [
        f'Initialized image: {image_size} bytes',
        f'Memory image: {memory_size} bytes ({memory_size - image_size} bytes zero-fill)',
        f'RX image: {rx_size} bytes',
        f'RW image: {rw_file_size} initialized, {rw_memory_size} memory '
        f'({rw_memory_size - rw_file_size} bytes zero-fill)',
        f'RX boundary: 0x{rx_size:08x}',
        f'RX alignment: {module["rx_alignment"]} bytes',
        f'RW alignment: {module["rw_alignment"]} bytes',
        f'Entry offset: 0x{module["entry"]:08x}',
        f'Relocations: {len(module["fixups"])} '
        f'({rx_fixups} RX, {len(module["fixups"]) - rx_fixups} RW)',
    ]
    if show_fixups and module['fixups']:
        lines.append('  SEG  OFFSET      TARGET')
        lines += [f'  {"RX" if offset < rx_size else "RW"}   0x{offset:08x}  0x{target:08x}'
                  for offset, target in module['fixups']]

    lines.append(f'Imports: {len(module["imports"])}')
    if module['imports']:
        lines.append('  HASH              SLOT        API')
        for binding_hash, offset, name in module['imports']:
            diagnostic_name = name if name is not None else '<not stored in this format>'
            lines.append(f'  {binding_hash:016x}  0x{offset:08x}  {diagnostic_name}')
    lines.append('Validation: OK')
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('module', type=Path)
    parser.add_argument('--fixups', action='store_true', help='list relocation offsets and targets')
    parser.add_argument('--stack-size', type=int, metavar='BYTES',
                        help='change declared module stack in place (256..16777216, excluding host-call margin)')
    parser.add_argument('--heap-size', type=int, metavar='BYTES',
                        help='change initial module heap in place (256..16777216, still limited by MOD_HEAP_MAX)')
    parser.add_argument('--thread-priority', type=int, metavar='PRIORITY',
                        help='change native thread priority in place (1..254, limited by HAL and MOD_THD_PRIO_MAX)')
    elf_options = parser.add_mutually_exclusive_group()
    elf_options.add_argument('--elf', type=Path, help='matching unstripped ELF (default: MODULE with .elf suffix)')
    elf_options.add_argument('--no-elf', action='store_true', help='inspect only the module, ignoring any companion ELF')
    args = parser.parse_args()
    try:
        data = args.module.read_bytes()
        original = data
        if args.stack_size is not None or args.heap_size is not None or args.thread_priority is not None:
            data = resize_resources(data, args.stack_size, args.heap_size, args.thread_priority)
        elf_path = args.elf or args.module.with_suffix('.elf')
        elf_data = None
        if not args.no_elf and (args.elf is not None or elf_path.is_file()):
            elf_data = elf_path.read_bytes()
        output = format_module(args.module, data, args.fixups, elf_data, elf_path)
        if data != original:
            replace_module(args.module, data)
    except (OSError, ValueError) as error:
        parser.exit(1, f'{parser.prog}: {args.module}: {error}\n')
    print(output)


if __name__ == '__main__':
    main()
