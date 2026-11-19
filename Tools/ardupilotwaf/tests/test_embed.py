# AP_FLAKE8_CLEAN
"""Tests for generated ROMFS compression and native-module prelinking."""

import importlib.util
import io
import re
import struct
import subprocess
import zlib

from pathlib import Path

import pytest

from elftools.elf.elffile import ELFFile

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("ap_romfs_embed", ROOT / "Tools/ardupilotwaf/embed.py")
embed = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(embed)


def make_apmod(fixups=None, imports=(), alignment=4, rx_alignment=4, rx_size=4096, profile=1):
    if fixups is None:
        fixups = ((4, 9), (rx_size, 0))
    file_size = rx_size + 4
    memory_size = rx_size + 8
    image = bytearray(file_size)
    destinations = []
    for destination, target in fixups:
        image[destination:destination + 4] = target.to_bytes(4, "little")
        destinations.append(struct.pack("<I", destination))
    import_records = []
    import_names = bytearray()
    for binding_hash, destination, name in imports:
        image[destination:destination + 4] = b"\xff" * 4
        import_records.append(embed.APMOD_IMPORT.pack(binding_hash, destination, len(import_names)))
        import_names += name.encode("ascii") + b"\0"
    fields = (7, 40, 4, alignment | rx_alignment << 16, file_size, memory_size, rx_size, 1, len(fixups),
              1, profile, 8192, 32768, len(imports), bytes.fromhex("00112233445566778899aabbccddeeff"), 2)
    body = bytes(image) + b"".join(destinations) + b"".join(import_records) + import_names
    header = embed.APMOD_HEADER.pack(embed.APMOD_MAGIC, *fields, 0)
    crc = zlib.crc32(header + body)
    return embed.APMOD_HEADER.pack(embed.APMOD_MAGIC, *fields, crc) + body


@pytest.mark.parametrize('alignment', [4, 16, 64, 4096])
def test_apmod_rw_alignment_and_odd_data_pointer(alignment):
    module = make_apmod(fixups=((4, 9), (4096, 4097)), alignment=alignment)
    machine, width, rx_size, fixups = embed.apmod_details(module, 'rust/odd.apmod')
    assert (machine, width, rx_size) == (40, 4, 4096)
    assert fixups[-1] == (4096, 4097)


@pytest.mark.parametrize('rx_size', [16, 32, 5556])
def test_compact_arm_rx_boundary(tmp_path, rx_size):
    contents = make_apmod(rx_size=rx_size)
    assert embed.apmod_details(contents, 'rust/compact.apmod')[2] == rx_size
    module = tmp_path / 'compact.apmod'
    module.write_bytes(contents)
    generated = generate(tmp_path, [('rust/compact.apmod', str(module))], prelink=True)
    assert 'ap_romfs_0' in generated and '.balign 4' in generated


def test_m7_profile_can_be_prelinked(tmp_path):
    contents = make_apmod(profile=3)
    assert embed.apmod_details(contents, 'modules/m7.apmod')[0] == 40
    module = tmp_path / 'm7.apmod'
    module.write_bytes(contents)
    generated = generate(tmp_path, [('modules/m7.apmod', str(module))], prelink=True)
    assert 'ap_romfs_0' in generated


@pytest.mark.parametrize('profile', [0, 2, 4])
def test_prelink_rejects_invalid_arm_profiles(profile):
    with pytest.raises(ValueError, match='invalid runtime contract'):
        embed.apmod_details(make_apmod(profile=profile), 'modules/wrong-cpu.apmod')


@pytest.mark.parametrize('priority', [0, 255, 0xffffffff])
def test_prelink_rejects_invalid_thread_priority(priority):
    contents = bytearray(make_apmod())
    struct.pack_into('<I', contents, 80, priority)
    with pytest.raises(ValueError, match='invalid runtime contract'):
        embed.apmod_details(replace_crc(contents), 'modules/wrong-priority.apmod')


def test_x86_rx_boundary_still_requires_page_alignment():
    contents = bytearray(make_apmod(rx_size=16))
    for offset, value in [(12, 62), (16, 8), (20, 8 | 8 << 16), (36, 0), (48, 2)]:
        struct.pack_into('<I', contents, offset, value)
    with pytest.raises(ValueError, match='invalid RX image'):
        embed.apmod_details(replace_crc(contents), 'rust/not-page-aligned.apmod')


@pytest.mark.parametrize('alignment', [0, 2, 24, 8192])
def test_invalid_apmod_rw_alignment(alignment):
    with pytest.raises(ValueError, match='invalid image sizes'):
        embed.apmod_details(make_apmod(alignment=alignment), 'rust/bad.apmod')


@pytest.mark.parametrize('alignment', [4, 8, 16, 64, 4096])
def test_prelink_aligns_rx_after_header(tmp_path, alignment):
    module = tmp_path / 'aligned.apmod'
    module.write_bytes(make_apmod(rx_alignment=alignment))
    generated = generate(tmp_path, [('rust/aligned.apmod', str(module))], prelink=True)
    assert '.balign %u' % alignment in generated
    padding = (-embed.APMOD_HEADER_SIZE) % alignment
    if padding:
        assert generated.index('.space %u' % padding) < generated.index('ap_romfs_0:')
    else:
        assert '.space ' not in generated


@pytest.mark.parametrize('script', [
    'common.ld', 'common_mixf.ld', 'common_extf.ld', 'common_extf_h730.ld', 'common_extf_h750.ld',
])
def test_linked_apmod_rx_alignment(tmp_path, script):
    """Input alignment must survive the firmware output section's layout."""
    output = io.BytesIO()
    output.write(b'#include <stdint.h>\n')
    # Deliberately leave preceding text/external data four bytes past an
    # aligned address. SUBALIGN(4) would silently misalign the module payloads.
    output.write(b'''asm(".section .vectors,\\"a\\"\\n.word 0, Reset_Handler\\n"
".section .app_descriptor,\\"a\\"\\n.global descriptor\\ndescriptor: .word 0x12345678\\n"
".text\\n.thumb\\n.global Reset_Handler\\n.type Reset_Handler,%function\\nReset_Handler: bx lr\\n.space 2\\n"
".section .extflash,\\"a\\"\\n.word 0x87654321\\n");
''')
    for index, alignment in enumerate((8, 16, 64, 4096)):
        contents = make_apmod(rx_alignment=alignment)
        _, _, rx_size, fixups = embed.apmod_details(contents, 'rust/aligned.apmod')
        embed.write_apmod_asm(output, contents, index, 'rust/aligned.apmod', fixups, rx_size)
    source = tmp_path / 'modules.cpp'
    source.write_bytes(output.getvalue())
    object_file = tmp_path / 'modules.o'
    subprocess.run(['arm-none-eabi-g++', '-c', '-mcpu=cortex-m4', '-mthumb', str(source), '-o', str(object_file)],
                   check=True, capture_output=True)
    linker_script = tmp_path / 'firmware.ld'
    linker_script.write_text('''MEMORY {
flash (rx) : ORIGIN = 0x08000000, LENGTH = 2M
ext_flash (rx) : ORIGIN = 0x90000000, LENGTH = 8M
default_flash (rx) : ORIGIN = 0x90000000, LENGTH = 8M
ram0 (rwx) : ORIGIN = 0x30000000, LENGTH = 256K
flashram (rwx) : ORIGIN = 0x24000000, LENGTH = 256K
dataram (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
instram (rwx) : ORIGIN = 0x00000000, LENGTH = 64K
}
__main_stack_size__ = 128;
__process_stack_size__ = 128;
INCLUDE "SCRIPT"
'''.replace('SCRIPT', str(ROOT / 'libraries/AP_HAL_ChibiOS/hwdef/common' / script)))
    executable = tmp_path / 'firmware.elf'
    subprocess.run(['arm-none-eabi-ld', '-T', str(linker_script), str(object_file), '-o', str(executable)],
                   check=True, capture_output=True)
    elf = ELFFile(io.BytesIO(executable.read_bytes()))
    symbols = {symbol.name: symbol['st_value'] for symbol in elf.get_section_by_name('.symtab').iter_symbols()}
    section = elf.get_section_by_name('.ap_romfs')
    for index, alignment in enumerate((8, 16, 64, 4096)):
        header_address = symbols['ap_romfs_%u' % index]
        code_address = header_address + embed.APMOD_HEADER_SIZE
        assert code_address % alignment == 0
        slot = header_address - section['sh_addr'] + embed.APMOD_HEADER_SIZE + 4
        assert struct.unpack_from('<I', section.data(), slot)[0] == code_address + 9
        assert symbols['descriptor'] < header_address
        if script == 'common_mixf.ld':
            assert symbols['__firmware_crc_ext_start__'] <= header_address
            assert symbols['__firmware_crc_ext_end__'] >= header_address + len(make_apmod())
        else:
            assert symbols['__firmware_crc_start__'] <= header_address
            assert symbols['__firmware_crc_end__'] >= header_address + len(make_apmod())


def replace_crc(module):
    module = bytearray(module)
    module[embed.APMOD_HEADER_SIZE - 4:embed.APMOD_HEADER_SIZE] = bytes(4)
    crc = zlib.crc32(module)
    module[embed.APMOD_HEADER_SIZE - 4:embed.APMOD_HEADER_SIZE] = struct.pack("<I", crc)
    return bytes(module)


def generate(tmp_path, entries, uncompressed=False, prelink=False):
    output = tmp_path / "ap_romfs_embedded.h"
    assert embed.create_embedded_h(output, entries, uncompressed, prelink)
    return output.read_text()


def table_size(generated, name):
    match = re.search(r'{ "%s", 0x([0-9a-f]{8}),' % re.escape(name), generated)
    assert match is not None
    return int(match.group(1), 16)


def test_mixed_compression_keeps_apmod_raw(tmp_path):
    text = tmp_path / "plain.txt"
    text.write_bytes(b"compress me " * 20)
    module = tmp_path / "probe.apmod"
    module.write_bytes(make_apmod())
    generated = generate(tmp_path, [("plain.txt", str(text)), ("rust/probe.apmod", str(module))])
    assert '"plain.txt"' in generated
    assert table_size(generated, "plain.txt") & ~embed.ROMFS_STORED_SIZE_MASK == 0
    assert table_size(generated, "rust/probe.apmod") == (embed.ROMFS_FLAG_UNCOMPRESSED | len(make_apmod()))


def test_global_uncompressed_preserves_logical_size(tmp_path):
    source = tmp_path / "source"
    source.write_bytes(b"abc")
    generated = generate(tmp_path, [("one.bin", str(source)), ("two.txt", str(source))], uncompressed=True)
    assert table_size(generated, "one.bin") == (embed.ROMFS_FLAG_UNCOMPRESSED | 3)
    assert table_size(generated, "two.txt") == (embed.ROMFS_FLAG_UNCOMPRESSED | 3)
    assert generated.count("{97,98,99,0};") == 2


def test_global_uncompressed_keeps_apmod_canonical(tmp_path):
    module = tmp_path / "probe.apmod"
    module.write_bytes(make_apmod())
    generated = generate(tmp_path, [("rust/probe.apmod", str(module))], uncompressed=True, prelink=True)
    assert ".pushsection .ap_romfs" not in generated
    assert table_size(generated, "rust/probe.apmod") == (embed.ROMFS_FLAG_UNCOMPRESSED | len(make_apmod()))


def test_arm_apmod_prelinks_only_rx_fixups(tmp_path):
    contents = make_apmod()
    assert embed.apmod_details(contents, "rust/probe.apmod")[3] == [(4, 9), (4096, 0)]
    module = tmp_path / "probe.apmod"
    module.write_bytes(contents)
    generated = generate(tmp_path, [("rust/probe.apmod", str(module))], prelink=True)
    assert '.pushsection .ap_romfs.0,\\"ax\\",%progbits' in generated
    assert '.word ap_romfs_0 + %u' % (embed.APMOD_HEADER_SIZE + 9) in generated
    assert generated.count('.word ap_romfs_0') == 1
    flags = embed.ROMFS_FLAG_UNCOMPRESSED | embed.ROMFS_FLAG_PRELINKED
    assert table_size(generated, "rust/probe.apmod") == (flags | len(contents))


def test_apmod_import_metadata_is_validated(tmp_path, capsys):
    contents = make_apmod(((4, 9),), ((0x12345678ABCDEF01, 4096, "hal.util"),))
    assert embed.apmod_details(contents, "rust/probe.apmod")[3] == [(4, 9)]
    module = bytearray(contents)
    module[-2] = ord("-")
    path = tmp_path / "bad-name.apmod"
    path.write_bytes(replace_crc(module))
    assert not embed.create_embedded_h(tmp_path / "generated.h", [("rust/bad-name.apmod", str(path))])
    assert "invalid import name" in capsys.readouterr().out


@pytest.mark.parametrize(("mutator", "error"), [
    (lambda data: b"NOTAPMOD" + data[8:], "expected APMOD07"),
    (lambda data: data[:-1], "truncated metadata"),
    (lambda data: data[:100] + bytes([data[100] ^ 1]) + data[101:], "checksum mismatch"),
])
def test_malformed_apmod_rejected(tmp_path, capsys, mutator, error):
    module = tmp_path / "bad.apmod"
    module.write_bytes(mutator(make_apmod()))
    output = tmp_path / "generated.h"
    assert not embed.create_embedded_h(output, [("rust/bad.apmod", str(module))])
    assert error in capsys.readouterr().out


def test_unaligned_apmod_fixup_rejected(tmp_path, capsys):
    module = bytearray(make_apmod())
    fixup = embed.APMOD_HEADER_SIZE + 4100
    module[fixup:fixup + 4] = struct.pack("<I", 2)
    path = tmp_path / "unaligned.apmod"
    path.write_bytes(replace_crc(module))
    assert not embed.create_embedded_h(tmp_path / "generated.h", [("rust/unaligned.apmod", str(path))],
                                       prelink_apmods=True)
    assert "invalid fixup" in capsys.readouterr().out


def test_rx_fixup_to_rw_rejected_for_prelink(tmp_path, capsys):
    module = tmp_path / "rx-to-rw.apmod"
    module.write_bytes(make_apmod(((4, 4096),)))
    assert not embed.create_embedded_h(tmp_path / "generated.h", [("rust/rx-to-rw.apmod", str(module))],
                                       prelink_apmods=True)
    assert "invalid fixup target" in capsys.readouterr().out


@pytest.mark.parametrize('uncompressed', [False, True])
def test_zero_length_file_preserved(tmp_path, uncompressed):
    source = tmp_path / "empty.bin"
    source.write_bytes(b"")
    output = tmp_path / "generated.h"
    assert embed.create_embedded_h(output, [("empty.bin", str(source))], uncompressed=uncompressed)
    generated = output.read_text()
    assert 'empty.bin' in generated
    assert '{,0}' not in generated


def test_stored_size_flag_bits_are_reserved(tmp_path, capsys, monkeypatch):
    source = tmp_path / "too-large.bin"
    source.write_bytes(b"abc")
    monkeypatch.setattr(embed, "ROMFS_STORED_SIZE_MASK", 2)
    assert not embed.create_embedded_h(tmp_path / "generated.h", [("too-large.bin", str(source))],
                                       uncompressed=True)
    assert "ROMFS file too-large.bin is too large" in capsys.readouterr().out
