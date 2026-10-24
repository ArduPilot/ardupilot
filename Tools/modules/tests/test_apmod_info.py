# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Account for every stored byte, using only verified ELF section boundaries."""

import io
import subprocess
import sys

from pathlib import Path

import apmod_info
import pack
import pytest

from elftools.elf.elffile import ELFFile
from test_modules import build as build_probe
from test_modules import upgrade_frozen_container

import build

ROOT = Path(__file__).resolve().parents[3]
TOOL = ROOT / 'Tools/modules/apmod_info.py'


@pytest.fixture(scope='module', params=list(build.TARGETS))
def image(tmp_path_factory, request):
    directory = tmp_path_factory.mktemp('apmod-info')
    path = build.build(ROOT / 'libraries/AP_Module/examples/simple_loop.rs', directory / 'loop.apmod',
                       request.param, 8192)
    return path


def test_exact_section_and_container_breakdown(image):
    data = image.read_bytes()
    elf_data = image.with_suffix('.elf').read_bytes()
    module = pack.unpack(data)
    sizes = apmod_info.file_breakdown(data, module, elf_data)
    elf = ELFFile(io.BytesIO(elf_data))
    assert sizes['Code'] == elf.get_section_by_name('.text')['sh_size']
    assert sizes['Read-only data'] == elf.get_section_by_name('.rodata')['sh_size']
    allocated = sum(section['sh_size'] for section in elf.iter_sections()
                    if section['sh_flags'] & 2 and section['sh_type'] != 'SHT_NOBITS')
    assert sizes['Image padding'] == len(module['image']) - allocated
    if module['machine'] == 40:
        assert sizes['Image padding'] < 32
    else:
        assert sizes['Image padding'] > 0
    assert sizes['Container header'] == 88
    assert sizes['Relocation table'] == 4 * len(module['fixups'])
    assert sizes['Import table'] == 16 * len(module['imports'])
    assert sizes['Import names'] == sum(len(name) + 1 for _, _, name in module['imports'])
    assert sum(sizes.values()) == len(data)
    output = apmod_info.format_module(image, data, elf_data=elf_data)
    assert f'  Code: {sizes["Code"]} bytes' in output
    assert f'  Image padding: {sizes["Image padding"]} bytes' in output
    assert f'  Total: {len(data)} bytes' in output


def test_no_elf_does_not_guess_code_or_padding(image):
    data = image.read_bytes()
    sizes = apmod_info.file_breakdown(data, pack.unpack(data))
    assert 'Code' not in sizes and 'Image padding' not in sizes
    assert sum(sizes.values()) == len(data)
    assert 'split unavailable without matching ELF' in apmod_info.format_module(image, data)


def test_wrong_or_invalid_elf_rejected(image):
    data = image.read_bytes()
    elf_data = bytearray(image.with_suffix('.elf').read_bytes())
    elf = ELFFile(io.BytesIO(elf_data))
    elf_data[elf.get_section_by_name('.text')['sh_offset']] ^= 1
    with pytest.raises(ValueError, match='ELF does not match module'):
        apmod_info.format_module(image, data, elf_data=elf_data)
    with pytest.raises(ValueError, match='invalid companion ELF'):
        apmod_info.format_module(image, data, elf_data=b'not ELF')


def test_cli_companion_selection(image, tmp_path):
    def run(*args):
        return subprocess.run([sys.executable, str(TOOL), str(image), *map(str, args)],
                              text=True, capture_output=True)

    automatic = run()
    assert automatic.returncode == 0, automatic.stderr
    assert 'Section sizes from verified ELF:' in automatic.stdout
    without = run('--no-elf')
    assert without.returncode == 0, without.stderr
    assert 'split unavailable without matching ELF' in without.stdout
    explicit = run('--elf', image.with_suffix('.elf'))
    assert explicit.returncode == 0 and explicit.stdout == automatic.stdout
    missing = run('--elf', tmp_path / 'missing.elf')
    assert missing.returncode == 1
    assert 'missing.elf' in missing.stderr


def test_cli_without_companion_and_with_stale_companion(image, tmp_path):
    module = tmp_path / 'copy.apmod'
    module.write_bytes(image.read_bytes())
    command = [sys.executable, str(TOOL), str(module)]
    missing = subprocess.run(command, text=True, capture_output=True)
    assert missing.returncode == 0
    assert 'split unavailable without matching ELF' in missing.stdout
    elf_data = bytearray(image.with_suffix('.elf').read_bytes())
    elf = ELFFile(io.BytesIO(elf_data))
    elf_data[elf.get_section_by_name('.text')['sh_offset']] ^= 1
    module.with_suffix('.elf').write_bytes(elf_data)
    stale = subprocess.run(command, text=True, capture_output=True)
    assert stale.returncode == 1
    assert 'ELF does not match module' in stale.stderr
    ignored = subprocess.run(command + ['--no-elf'], text=True, capture_output=True)
    assert ignored.returncode == 0


@pytest.mark.parametrize('target', list(build.TARGETS))
def test_relocation_probe_breakdown(tmp_path, target):
    elf_data = build_probe(tmp_path, target)
    data = pack.pack_elf(elf_data)
    sizes = apmod_info.file_breakdown(data, pack.unpack(data), elf_data)
    assert sizes['Container header'] == 56
    assert sizes['Import table'] == sizes['Import names'] == 0
    assert sum(sizes.values()) == len(data)


@pytest.mark.parametrize('target', ['thumbv7em', 'x86_64'])
def test_frozen_module_container_breakdown(target):
    data = bytes.fromhex((ROOT / f'libraries/AP_Module/tests/compat/{target}.apmod.hex').read_text())
    data = upgrade_frozen_container(data)
    sizes = apmod_info.file_breakdown(data, pack.unpack(data))
    assert sum(sizes.values()) == len(data)
    assert 'split unavailable' in apmod_info.format_module(Path('old.apmod'), data)


@pytest.mark.parametrize('sizes', [
    {'stack_size': 256}, {'heap_size': pack.MAX_IMAGE}, {'stack_size': 16384, 'heap_size': 32768},
    {'thread_priority': 1}, {'thread_priority': 254},
    {'stack_size': 4096, 'heap_size': 8192, 'thread_priority': 3},
])
def test_resize_resources_preserves_image_and_companion(image, sizes):
    original = image.read_bytes()
    before = pack.unpack(original)
    data = apmod_info.resize_resources(original, **sizes)
    after = pack.unpack(data)  # also verifies the updated CRC
    assert after['contract'] == {**before['contract'], **sizes}
    assert {k: v for k, v in after.items() if k != 'contract'} == {
        k: v for k, v in before.items() if k != 'contract'}
    assert data[pack.RUNTIME_HEADER.size:] == original[pack.RUNTIME_HEADER.size:]
    assert 'Validation: OK' in apmod_info.format_module(image, data, elf_data=image.with_suffix('.elf').read_bytes())


@pytest.mark.parametrize('option', ['stack_size', 'heap_size'])
@pytest.mark.parametrize('size', [-1, 0, 255, pack.MAX_IMAGE + 1, 1 << 64])
def test_invalid_resource_sizes_rejected(image, option, size):
    with pytest.raises(ValueError, match='invalid .* requirement'):
        apmod_info.resize_resources(image.read_bytes(), **{option: size})


@pytest.mark.parametrize('priority', [-1, 0, 255, 1 << 64, True, 1.5])
def test_invalid_thread_priority_rejected(image, priority):
    with pytest.raises(ValueError, match='invalid module thread priority'):
        apmod_info.resize_resources(image.read_bytes(), thread_priority=priority)


def test_cli_changes_resources_in_place(image, tmp_path):
    path = tmp_path / 'copy.apmod'
    path.write_bytes(image.read_bytes())
    path.chmod(0o640)
    path.with_suffix('.elf').write_bytes(image.with_suffix('.elf').read_bytes())
    result = subprocess.run([sys.executable, str(TOOL), str(path), '--stack-size', '16384', '--heap-size', '32768',
                             '--thread-priority', '3'],
                            text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    assert 'Stack requirement: 16384 bytes' in result.stdout
    assert 'Initial heap: 32768 bytes' in result.stdout
    assert 'Thread priority: 3' in result.stdout
    assert 'Section sizes from verified ELF' in result.stdout
    assert path.stat().st_mode & 0o777 == 0o640
    assert not list(tmp_path.glob('.copy.apmod.*'))
    contract = pack.unpack(path.read_bytes())['contract']
    assert contract['stack_size'] == 16384 and contract['heap_size'] == 32768
    assert contract['thread_priority'] == 3


@pytest.mark.parametrize('args', [
    ['--stack-size', '255'], ['--heap-size', str(pack.MAX_IMAGE + 1)],
    ['--heap-size', 'no'], ['--stack-size', '16384', '--elf', 'missing.elf'],
    ['--thread-priority', '0'], ['--thread-priority', '255'], ['--thread-priority', 'no'],
])
def test_cli_failure_does_not_modify_module(image, tmp_path, args):
    path = tmp_path / 'copy.apmod'
    original = image.read_bytes()
    path.write_bytes(original)
    result = subprocess.run([sys.executable, str(TOOL), str(path), *args], text=True, capture_output=True)
    assert result.returncode != 0
    assert path.read_bytes() == original


def test_cli_resize_rejects_stale_companion(image, tmp_path):
    path = tmp_path / 'copy.apmod'
    original = image.read_bytes()
    path.write_bytes(original)
    path.with_suffix('.elf').write_bytes(b'not an ELF')
    result = subprocess.run([sys.executable, str(TOOL), str(path), '--stack-size', '16384'], text=True, capture_output=True)
    assert result.returncode == 1
    assert 'invalid companion ELF' in result.stderr
    assert path.read_bytes() == original


def test_resize_resources_rejects_corruption(image):
    data = bytearray(image.read_bytes())
    data[-1] ^= 1
    with pytest.raises(ValueError):
        apmod_info.resize_resources(data, heap_size=8192)


def test_resize_resources_rejects_probe(tmp_path):
    data = pack.pack_elf(build_probe(tmp_path, 'thumbv7em-none-eabihf'))
    with pytest.raises(ValueError, match='relocation probe'):
        apmod_info.resize_resources(data, stack_size=8192)


def test_replace_module_preserves_symlink(image, tmp_path):
    target = tmp_path / 'target.apmod'
    target.write_bytes(image.read_bytes())
    link = tmp_path / 'link.apmod'
    link.symlink_to(target.name)
    updated = apmod_info.resize_resources(target.read_bytes(), heap_size=8192)
    apmod_info.replace_module(link, updated)
    assert link.is_symlink()
    assert target.read_bytes() == updated


def test_replace_failure_preserves_original_and_removes_temporary(image, tmp_path, monkeypatch):
    path = tmp_path / 'copy.apmod'
    original = image.read_bytes()
    path.write_bytes(original)

    def fail_replace(source, destination):
        raise OSError('simulated replacement failure')

    monkeypatch.setattr(Path, 'replace', fail_replace)
    with pytest.raises(OSError, match='replacement failure'):
        apmod_info.replace_module(path, apmod_info.resize_resources(original, heap_size=8192))
    assert path.read_bytes() == original
    assert list(tmp_path.iterdir()) == [path]
