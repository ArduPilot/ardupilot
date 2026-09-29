"""Tests for Tools/scripts/build_tests/pretty_diff_size.py.

The interesting part is binaries_are_identical(): it decides whether a board's
cell in the size table reads "*" (byte-identical) or "0" (same size, different
bytes), and it is fed a lower-cased binary name while the files on disk are
not always lower-case.
"""

import os

import pytest

from Tools.scripts.build_tests.pretty_diff_size import BINARY_TO_COLUMN
from Tools.scripts.build_tests.pretty_diff_size import binaries_are_identical
from Tools.scripts.build_tests.pretty_diff_size import find_binary


def write(directory, name, content=b"firmware"):
    path = os.path.join(directory, name)
    with open(path, "wb") as fh:
        fh.write(content)
    return path


@pytest.fixture
def builds(tmp_path):
    """two build directories, as the size job lays them out"""
    base = tmp_path / "base_branch_bin_no_versions"
    pr = tmp_path / "pr_bin_no_versions"
    base.mkdir()
    pr.mkdir()
    return str(base), str(pr)


def test_find_binary_exact(builds):
    base, _ = builds
    write(base, "arduplane.bin")
    assert find_binary(base, "arduplane.bin") == os.path.join(base, "arduplane.bin")


def test_find_binary_ignores_case(builds):
    """the caller lower-cases the name, the build writes AP_Periph.bin"""
    base, _ = builds
    write(base, "AP_Periph.bin")
    assert find_binary(base, "ap_periph.bin") == os.path.join(base, "AP_Periph.bin")


def test_find_binary_missing(builds):
    base, _ = builds
    assert find_binary(base, "arduplane.bin") is None


def test_find_binary_ignores_a_directory(builds):
    base, _ = builds
    os.mkdir(os.path.join(base, "arduplane"))
    assert find_binary(base, "arduplane") is None


def test_identical_bin(builds):
    base, pr = builds
    for d in builds:
        write(d, "arduplane.bin", b"same bytes")
    assert binaries_are_identical(base, "arduplane", pr) is True


def test_different_bin(builds):
    base, pr = builds
    write(base, "arduplane.bin", b"one")
    write(pr, "arduplane.bin", b"two")
    assert binaries_are_identical(base, "arduplane", pr) is False


def test_periph_bin_looked_up_in_lower_case(builds):
    """AP_Periph read as 'ap_periph' used to find no file and report changed"""
    base, pr = builds
    for d in builds:
        write(d, "AP_Periph.bin", b"same bytes")
        write(d, "AP_Periph", b"same elf")
    assert binaries_are_identical(base, "ap_periph", pr) is True


def test_bootloader_bin_looked_up_in_lower_case(builds):
    base, pr = builds
    for d in builds:
        write(d, "AP_Bootloader.bin", b"same bytes")
    assert binaries_are_identical(base, "ap_bootloader", pr) is True


def test_extensionless_elf(builds):
    """a Linux board writes a bare ELF: disco's bin/ has no .bin at all"""
    base, pr = builds
    for d in builds:
        write(d, "arduplane", b"same elf")
    assert binaries_are_identical(base, "arduplane", pr) is True


def test_bin_wins_over_the_elf(builds):
    """a differing .bin is the answer; the matching ELF must not override it"""
    base, pr = builds
    write(base, "arduplane.bin", b"one")
    write(pr, "arduplane.bin", b"two")
    for d in builds:
        write(d, "arduplane", b"same elf")
    assert binaries_are_identical(base, "arduplane", pr) is False


def test_nothing_to_compare(builds):
    base, pr = builds
    write(base, "arduplane.bin")
    assert binaries_are_identical(base, "arduplane", pr) is False


def test_differing_elf_is_stripped_before_deciding(builds, monkeypatch):
    """debug symbols alone are not a firmware change"""
    import Tools.scripts.build_tests.pretty_diff_size as pds

    base, pr = builds
    write(base, "arduplane.elf", b"one")
    write(pr, "arduplane.elf", b"two")
    calls = []
    monkeypatch.setattr(pds, "_stripped_equal",
                        lambda a, b, toolchain: calls.append(toolchain) or True)
    assert pds.binaries_are_identical(base, "arduplane", pr, "arm-linux-gnueabihf") is True
    assert calls == ["arm-linux-gnueabihf"]


def test_column_names_are_lower_case_keys():
    """print_table looks these up with name.lower(), so the keys must be lower"""
    assert all(key == key.lower() for key in BINARY_TO_COLUMN)
