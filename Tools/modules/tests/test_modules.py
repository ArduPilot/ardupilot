# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Offline relocation and native execution tests for the prototype container."""

import ctypes
import io
import mmap
import os
import platform
import re
import struct
import subprocess
import sys
import zlib

from pathlib import Path

import pack
import pytest

from elftools.elf.elffile import ELFFile
from rust.build import check_rust_version

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / 'Tools/renode/tests'))
from test_peripherals import renode  # noqa: E402,F401


def build(tmp_path, target, base=0x10000, relocation=None, cpu=None):
    compiler = os.environ.get('RUSTC', 'rustc')
    version = subprocess.check_output([compiler, '--version'], text=True)
    check_rust_version(version)
    if relocation is None:
        relocation = 'ropi-rwpi' if target == 'thumbv7em-none-eabihf' else 'pic'
    output = tmp_path / f'{target}-{base:x}-{relocation}.elf'
    command = [compiler, str(HERE / 'rust/probe.rs'), '--edition=2021', '--target', target,
               '-C', 'opt-level=s', '-C', 'panic=abort', '-C', 'lto=fat',
               '-C', f'relocation-model={relocation}', '-o', str(output)]
    if cpu is not None:
        command += ['-C', 'target-cpu=' + cpu]
    arguments = [f'-T{HERE / "module.ld"}', '--emit-relocs', f'--defsym=MODULE_BASE={base}']
    if target == 'x86_64-unknown-linux-gnu':
        command += ['-C', 'link-arg=-nostdlib', '-C', 'link-arg=-static', '-C', 'link-arg=-no-pie']
        arguments = ['-Wl,' + arg for arg in arguments]
    for arg in arguments:
        command += ['-C', 'link-arg=' + arg]
    subprocess.run(command, check=True, capture_output=True)
    return output.read_bytes()


@pytest.fixture(params=['thumbv7em-none-eabihf', 'x86_64-unknown-linux-gnu'])
def module(request, tmp_path):
    return pack.pack_elf(build(tmp_path, request.param))


def test_link_at_two_bases(tmp_path):
    for target in ('thumbv7em-none-eabihf', 'x86_64-unknown-linux-gnu'):
        first = pack.pack_elf(build(tmp_path, target))
        second = pack.pack_elf(build(tmp_path, target, base=0x24123000))
        assert first == second
        module = pack.unpack(first)
        assert len(module['fixups']) >= 2  # function pointer table
        assert module['memory_size'] > len(module['image'])  # BSS


def checksum(data, header_size=pack.HEADER.size):
    data[header_size - 4:header_size] = bytes(4)
    data[header_size - 4:header_size] = zlib.crc32(data).to_bytes(4, 'little')
    return data


def runtime_manifest(target):
    return dict(abi_version=1, cpu_profile=1 if target == 'thumbv7em-none-eabihf' else 2,
                thread_priority=2 if target == 'thumbv7em-none-eabihf' else 1,
                stack_size=4096, heap_size=32768, fingerprint=bytes(range(1, 17)).hex())


def upgrade_frozen_container(data):
    # Keep the frozen old SDK machine code/import identities unchanged while
    # updating only the container envelope for the current loader contract.
    old = struct.Struct('<8s14I16sI')
    fields = list(old.unpack_from(data))
    assert fields[0] == b'APMOD06\0' and fields[1] == 6
    assert fields[-1] == zlib.crc32(data[:80] + bytes(4) + data[84:])
    fields[0], fields[1] = pack.RUNTIME_MAGIC, 7
    fields.insert(-1, 2 if fields[2] == 40 else 1)
    fields[-1] = 0
    body = data[old.size:]
    fields[-1] = zlib.crc32(pack.RUNTIME_HEADER.pack(*fields) + body)
    return pack.RUNTIME_HEADER.pack(*fields) + body


@pytest.mark.parametrize('target', ['thumbv7em-none-eabihf', 'x86_64-unknown-linux-gnu'])
def test_runtime_contract(tmp_path, target):
    manifest = runtime_manifest(target)
    elf = build(tmp_path, target)
    data = pack.pack_elf(elf, manifest)
    assert data[:8] == b'APMOD07\0'
    assert data == pack.pack_elf(build(tmp_path, target, base=0x24123000), manifest)
    assert pack.unpack(data)['contract'] == manifest
    # Tagging changes the compatibility envelope, not the image or relocation.
    assert pack.relocate(data, 0x24000000) == pack.relocate(pack.pack_elf(elf), 0x24000000)
    for size in (256, 257, 4095, 4097, pack.MAX_IMAGE):
        contract = {**manifest, 'stack_size': size}
        assert pack.unpack(pack.pack_elf(elf, contract))['contract'] == contract
    for offset, value in ((8, 1), (44, 0), (48, 3 - manifest['cpu_profile']),
                          (52, 0), (52, 255), (52, pack.MAX_IMAGE + 1), (52, 0xffffffff)):
        damaged = bytearray(data)
        damaged[offset:offset + 4] = value.to_bytes(4, 'little')
        with pytest.raises(ValueError):
            pack.unpack(checksum(damaged, pack.RUNTIME_HEADER.size))
    damaged = bytearray(data)
    damaged[64:80] = bytes(16)
    with pytest.raises(ValueError, match='fingerprint'):
        pack.unpack(checksum(damaged, pack.RUNTIME_HEADER.size))
    for length in (0, 55, 56, 75, len(data) - 1):
        with pytest.raises(ValueError):
            pack.unpack(data[:length])
    for key, value in (('abi_version', True), ('cpu_profile', 4), ('stack_size', 0), ('stack_size', True),
                       ('stack_size', 255), ('stack_size', pack.MAX_IMAGE + 1), ('fingerprint', '00' * 16)):
        with pytest.raises(ValueError):
            pack.pack_elf(elf, {**manifest, key: value})


@pytest.mark.parametrize('target', ['thumbv7em-none-eabihf', 'x86_64-unknown-linux-gnu'])
def test_compact_runtime_fixups(tmp_path, target):
    data = pack.pack_elf(build(tmp_path, target), runtime_manifest(target))
    info = pack.unpack(data)
    table = pack.RUNTIME_HEADER.size + len(info['image'])
    assert len(info['fixups']) >= 2
    for index, (offset, target_offset) in enumerate(info['fixups']):
        assert struct.unpack_from('<I', data, table + index * pack.FIXUP.size)[0] == offset
        assert int.from_bytes(info['image'][offset:offset + info['width']], 'little') == target_offset
    if info['machine'] == 40:
        assert any(target_offset & 1 for _, target_offset in info['fixups'])
        pack.relocate(data, 0x08000000, 0x20000000 + info['rw_alignment'])
        with pytest.raises(ValueError, match='unaligned RW'):
            pack.relocate(data, 0x08000000, 0x20000002)

    damaged = bytearray(data)
    damaged[table] ^= 1
    with pytest.raises(ValueError, match='checksum'):
        pack.unpack(damaged)

    damaged = bytearray(data)
    struct.pack_into('<I', damaged, table, info['fixups'][0][0] + 1)
    with pytest.raises(ValueError, match='relocation offset'):
        pack.unpack(checksum(damaged, pack.RUNTIME_HEADER.size))

    damaged = bytearray(data)
    first_offset = info['fixups'][0][0]
    damaged[pack.RUNTIME_HEADER.size + first_offset:
            pack.RUNTIME_HEADER.size + first_offset + info['width']] = \
        (info['memory_size'] + 2).to_bytes(info['width'], 'little')
    with pytest.raises(ValueError, match='relocation value'):
        pack.unpack(checksum(damaged, pack.RUNTIME_HEADER.size))

    if info['machine'] == 40:
        rx_offset = next(offset for offset, _ in info['fixups'] if offset < info['rx_size'])
        damaged = bytearray(data)
        damaged[pack.RUNTIME_HEADER.size + rx_offset:
                pack.RUNTIME_HEADER.size + rx_offset + info['width']] = \
            info['rx_size'].to_bytes(info['width'], 'little')
        with pytest.raises(ValueError, match='RX relocation targets RW'):
            pack.unpack(checksum(damaged, pack.RUNTIME_HEADER.size))


def test_truncation_and_corruption(module):
    for length in (0, 8, pack.HEADER.size - 1, pack.HEADER.size, len(module) - 1):
        with pytest.raises(ValueError):
            pack.unpack(module[:length])
    damaged = bytearray(module)
    damaged[pack.HEADER.size] ^= 1
    with pytest.raises(ValueError, match='checksum'):
        pack.unpack(damaged)


def test_invalid_fixup_even_with_valid_checksum(module):
    damaged = bytearray(module)
    module_info = pack.unpack(module)
    offset = pack.HEADER.size + len(module_info['image'])
    damaged[offset:offset + 4] = (len(module_info['image']) - 1).to_bytes(4, 'little')
    with pytest.raises(ValueError, match='relocation offset'):
        pack.unpack(checksum(damaged))
    damaged = bytearray(module)
    damaged[offset + 4:offset + 8] = (module_info['memory_size'] + 1).to_bytes(4, 'little')
    with pytest.raises(ValueError, match='relocation value'):
        pack.unpack(checksum(damaged))


def test_invalid_base(module):
    with pytest.raises(ValueError, match='unaligned'):
        pack.relocate(module, 3)
    with pytest.raises(ValueError, match='overflow'):
        pack.relocate(module, (1 << (8 * pack.unpack(module)['width'])) - pack.PAGE)


def test_static_arm_requires_instruction_relocations(tmp_path):
    elf = build(tmp_path, 'thumbv7em-none-eabihf', relocation='static')
    with pytest.raises(ValueError, match='unsupported relocation (47|48)'):
        pack.pack_elf(elf)


def test_non_relaxed_got_normalized(tmp_path):
    source = tmp_path / 'got.S'
    source.write_text('.globl start\n.text\nstart: movq obj@GOTPCREL(%rip), %rax\nret\n'
                      '.data\nobj: .quad 0\n')
    output = tmp_path / 'got.elf'
    subprocess.run(['gcc', '-nostdlib', '-static', '-no-pie', '-Wl,--no-relax,--emit-relocs',
                    '-Wl,-T' + str(HERE / 'module.ld'), str(source), '-o', str(output)],
                   check=True, capture_output=True)
    # GNU ld adds unused lazy-binding slots even to this static fixture. They
    # are deliberately unsupported; remove them only after proving rejection.
    with pytest.raises(ValueError, match='unsupported allocated section .got.plt'):
        pack.pack_elf(output.read_bytes())
    subprocess.run(['objcopy', '--remove-section=.got.plt', str(output)], check=True, capture_output=True)
    elf = output.read_bytes()
    data = pack.pack_elf(elf)
    info = pack.unpack(data)
    got = ELFFile(io.BytesIO(elf)).get_section_by_name('.got')
    assert info['fixups'] == [(got['sh_addr'] - 0x10000, 0x1000)]
    damaged = bytearray(elf)
    damaged[got['sh_offset']:got['sh_offset'] + 8] = (0xDEADBEEF).to_bytes(8, 'little')
    with pytest.raises(ValueError, match='GOT slot is not an internal symbol'):
        pack.pack_elf(damaged)


def test_native_execution_two_allocations(tmp_path):
    if platform.machine() != 'x86_64' or sys.platform != 'linux':
        pytest.skip('native executable test needs x86-64 Linux')
    data = pack.pack_elf(build(tmp_path, 'x86_64-unknown-linux-gnu'))
    info = pack.unpack(data)
    output_fn = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_size_t)
    sleep_fn = ctypes.CFUNCTYPE(None, ctypes.c_uint32)
    alloc_fn = ctypes.CFUNCTYPE(ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t)
    free_fn = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t)

    class Host(ctypes.Structure):
        _fields_ = [('output', output_fn), ('sleep', sleep_fn), ('allocate', alloc_fn), ('deallocate', free_fn)]

    messages, sleeps = [], []
    allocations = {}
    allocation_errors = []

    def allocate(size, alignment):
        raw = ctypes.create_string_buffer(size + alignment)
        address = (ctypes.addressof(raw) + alignment - 1) & ~(alignment - 1)
        allocations[address] = (raw, size, alignment)
        return address

    def deallocate(address, size, alignment):
        allocation = allocations.pop(address, None)
        if allocation is None or allocation[1:] != (size, alignment):
            allocation_errors.append((address, size, alignment))

    host = Host(output_fn(lambda p, n: messages.append(ctypes.string_at(p, n))),
                sleep_fn(sleeps.append), alloc_fn(allocate), free_fn(deallocate))
    exhausted_host = Host(host.output, host.sleep, alloc_fn(lambda size, alignment: None), host.deallocate)
    libc = ctypes.CDLL(None, use_errno=True)
    libc.mprotect.argtypes = (ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int)
    mappings = []
    try:
        for _ in range(2):
            size = (info['memory_size'] + pack.PAGE - 1) // pack.PAGE * pack.PAGE
            memory = mmap.mmap(-1, size, prot=mmap.PROT_READ | mmap.PROT_WRITE)
            mappings.append(memory)
            address = ctypes.addressof(ctypes.c_char.from_buffer(memory))
            memory.write(pack.relocate(data, address))
            assert libc.mprotect(address, info['rx_size'], mmap.PROT_READ | mmap.PROT_EXEC) == 0
            start = ctypes.CFUNCTYPE(ctypes.c_uint32, ctypes.POINTER(Host))(address + info['entry'])
            assert start(ctypes.byref(exhausted_host)) == 0xBAD0
            assert start(ctypes.byref(host)) == 31
            assert start(ctypes.byref(host)) == 26
            assert allocations == {}
        assert messages == [b'Native module probe'] * 4
        assert sleeps == [10] * 4
        assert allocation_errors == []
    finally:
        for memory in mappings:
            memory.close()


@pytest.mark.parametrize('cpu,ram,loader,module_cpu', [
    (cpu, ram, loader, 'cortex-m4')
    for cpu, ram in [('cortex-m7', 0x24000000), ('cortex-m4', 0x20000000)]
    for loader in ['python', 'cpp', 'cpp_runtime']
] + [(cpu, ram, 'cpp_runtime', 'cortex-m7')
     for cpu, ram in [('cortex-m7', 0x24000000), ('cortex-m4', 0x20000000)]])
def test_renode_two_bases(tmp_path, renode, cpu, ram, loader, module_cpu):  # noqa: F811
    manifest = runtime_manifest('thumbv7em-none-eabihf') if loader == 'cpp_runtime' else None
    if manifest is not None and module_cpu == 'cortex-m7':
        manifest['cpu_profile'] = 3
    data = pack.pack_elf(build(tmp_path, 'thumbv7em-none-eabihf', cpu=module_cpu), manifest)
    reject = module_cpu == 'cortex-m7' and cpu == 'cortex-m4'
    info = pack.unpack(data)
    first, second = ram + 0x4000, ram + 0xA000
    assert info['memory_size'] <= 0x6000
    platform_file = tmp_path / 'module.repl'
    platform_file.write_text(f'''
cpu: CPU.CortexM @ sysbus
    cpuType: "{cpu}"
    nvic: nvic
nvic: IRQControllers.NVIC @ sysbus 0xE000E000
    IRQ -> cpu@0
flash: Memory.MappedMemory @ sysbus 0x08000000
    size: 0x20000
ram: Memory.MappedMemory @ sysbus 0x{ram:x}
    size: 0x20000
''')
    linker = tmp_path / 'harness.ld'
    linker.write_text('ENTRY(reset)\nSECTIONS { . = 0x08000000; .vectors : { KEEP(*(.vectors)) } '
                      '.text : { *(.text*) *(.rodata*) } }')
    source = tmp_path / 'harness.S'
    loader_setup = 'bl load_images\n cmp r0, #1\n bne fault' if loader != 'python' else ''
    if reject:
        # Stop after admission: a rejected module must never reach its entry.
        loader_setup = f'''bl load_images
 cmp r0, #0
 bne fault
 ldr r0, =0x{ram + 0x110:x}
 ldr r1, =0x600df00d
 str r1, [r0]
 b .'''
    source.write_text(f'''
.syntax unified
.thumb
.section .vectors,"a"
.word 0x{ram + 0x20000:x}
.word reset
.rept 14
.word fault
.endr
.section .text,"ax"
.thumb_func
reset:
 ldr r0, =0xe000ed88
 ldr r1, =0x00f00000
 str r1, [r0]
 dsb
 isb
 {loader_setup}
 ldr r4, =0x{ram + 0x100:x}
 movs r0, #0
 str r0, [r4, #8]
 str r0, [r4, #12]
 str r0, [r4, #20]
 str r0, [r4, #24]
 ldr r9, =0x{first + info['rx_size']:x}
 ldr r0, =host
 ldr r1, =0x{first + info['entry']:x}
 blx r1
 str r0, [r4]
 ldr r9, =0x{second + info['rx_size']:x}
 ldr r0, =host
 ldr r1, =0x{second + info['entry']:x}
 blx r1
 str r0, [r4, #4]
 ldr r0, =0x600df00d
 str r0, [r4, #16]
 b .
.thumb_func
fault:
 ldr r0, =0x{ram + 0x110:x}
 ldr r1, =0xbad00001
 str r1, [r0]
 b .
.thumb_func
output:
 cmp r1, #19
 bne fault
 ldrb r0, [r0]
 cmp r0, #78
 bne fault
 ldr r2, =0x{ram + 0x100:x}
 ldr r0, [r2, #8]
 adds r0, #1
 str r0, [r2, #8]
 bx lr
.thumb_func
sleep:
 ldr r2, =0x{ram + 0x100:x}
 ldr r1, [r2, #12]
 adds r0, r1
 str r0, [r2, #12]
 bx lr
.thumb_func
allocate:
 cmp r0, #256
 bne fault
 cmp r1, #4
 bne fault
 ldr r2, =0x{ram + 0x100:x}
 ldr r0, [r2, #20]
 adds r0, #1
 str r0, [r2, #20]
 ldr r0, =0x{ram + 0x200:x}
 bx lr
.thumb_func
deallocate:
 ldr r3, =0x{ram + 0x200:x}
 cmp r0, r3
 bne fault
 cmp r1, #256
 bne fault
 cmp r2, #4
 bne fault
 ldr r2, =0x{ram + 0x100:x}
 ldr r0, [r2, #24]
 adds r0, #1
 str r0, [r2, #24]
 bx lr
.align 2
host:
 .word output
 .word sleep
 .word allocate
 .word deallocate
''')
    elf = tmp_path / 'harness.elf'
    inputs = [str(source)]
    if loader != 'python':
        loader_source = tmp_path / 'loader.cpp'
        if loader == 'cpp_runtime':
            board_profile = 3 if cpu == 'cortex-m7' else 1
            contract = (f'const AP_Module::Image::Compatibility expected{{40, 1, {board_profile}, 8192, '
                        '{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16}};')
            parse = f'image.parse(reinterpret_cast<const uint8_t *>(0x08010000), {len(data)}, 0x6000, expected)'
            load_first = (f'image.load_split(reinterpret_cast<void *>({first}), {info["rx_size"]}, '
                          f'reinterpret_cast<void *>({first + info["rx_size"]}), '
                          f'{0x6000 - info["rx_size"]})')
            load_second = (f'image.load_split(reinterpret_cast<void *>({second}), {info["rx_size"]}, '
                           f'reinterpret_cast<void *>({second + info["rx_size"]}), '
                           f'{0x6000 - info["rx_size"]})')
        else:
            contract = ''
            parse = f'image.parse_probe(reinterpret_cast<const uint8_t *>(0x08010000), {len(data)}, 0x6000, 40)'
            load_first = f'image.load(reinterpret_cast<void *>({first}), 0x6000)'
            load_second = f'image.load(reinterpret_cast<void *>({second}), 0x6000)'
        loaded = f'return {parse} && {load_first} && {load_second};'
        if reject:
            loaded = (f'if ({parse}) {{ return 1; }}\n'
                      '    return strcmp(image.parse_error(), "requires Cortex-M7 with double-precision FPU") != 0;')
        loader_source.write_text(f'''
#include <AP_Module/Image.h>
#include <string.h>
extern "C" int load_images()
{{
    AP_Module::Image image;
    {contract}
    {loaded}
}}
''')
        inputs += [str(loader_source), str(ROOT / 'libraries/AP_Module/Image.cpp'), str(ROOT / 'libraries/AP_Math/crc.cpp')]
    subprocess.run(['arm-none-eabi-g++', '-nostartfiles', '-mcpu=' + cpu, '-mthumb',
                    '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard', '-std=gnu++17', '-Os',
                    '-fno-exceptions', '-fno-rtti', '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
                    '-DAP_MODULE_ENABLED=1', f'-DAP_MODULE_LEGACY_PROBE_ENABLED={int(loader != "cpp_runtime")}',
                    '-DCONFIG_HAL_BOARD=99', '-I' + str(ROOT / 'libraries'),
                    '-T', str(linker), *inputs, '-o', str(elf)], check=True, capture_output=True)
    commands = f'mach create\nmachine LoadPlatformDescription @{platform_file}\nsysbus LoadELF @{elf}\n'
    if loader != 'python':
        container = tmp_path / 'probe.apmod'
        container.write_bytes(data)
        commands += f'sysbus LoadBinary @{container} 0x08010000\n'
    else:
        for base in (first, second):
            binary = tmp_path / f'{base:x}.bin'
            binary.write_bytes(pack.relocate(data, base))
            commands += f'sysbus LoadBinary @{binary} 0x{base:x}\n'
    commands += 'emulation RunFor "0.05"\n'
    for offset in range(0, 28, 4):
        commands += f'sysbus ReadDoubleWord 0x{ram + 0x100 + offset:x}\n'
    output = renode(commands)
    words = re.findall(r'^0x([0-9a-fA-F]{8})\s*$', output, re.M)
    expected = [0, 0, 0, 0, 0x600DF00D, 0, 0] if reject else [31, 31, 2, 20, 0x600DF00D, 2, 2]
    assert [int(word, 16) for word in words] == expected, output
