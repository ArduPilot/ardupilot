# AP_FLAKE8_CLEAN
"""Post-link checks, including failures that --wrap alone cannot catch."""

import importlib
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import pytest


@pytest.fixture
def checker(monkeypatch):
    root = Path(__file__).resolve().parents[3]
    monkeypatch.syspath_prepend(str(root / "modules/waf"))
    monkeypatch.syspath_prepend(str(root))
    waf = importlib.import_module("Tools.ardupilotwaf.ardupilotwaf")
    env = waf.ConfigSet.ConfigSet()
    env.NM = [shutil.which("nm") or "nm"]
    env.vehicle_binary = True
    env.SIM_ENABLED = True
    env.CHECK_MALLOC_WRAPPING = True
    env.LINKFLAGS = ["-Wl,--wrap,malloc"]
    task = waf.check_elf_symbols(env=env)
    task.inputs = [SimpleNamespace(abspath=lambda: "firmware")]
    return waf, task


@pytest.mark.parametrize("flags", [["-Wl,--wrap,malloc"], ["-Wl,--wrap=malloc"],
                                   ["--wrap=malloc"], ["-Wl,--wrap", "-Wl,malloc"]])
def test_wrapped_malloc(checker, monkeypatch, flags):
    waf, task = checker
    task.env.LINKFLAGS = flags
    monkeypatch.setattr(waf.subprocess, "check_output", lambda *a, **kw:
                        "00001000 T __wrap_malloc\n         U malloc@GLIBC_2.2.5\n")
    task.run()


@pytest.mark.parametrize("symbols", ["", " U __wrap_malloc\n", "00001000 W __wrap_malloc\n"])
def test_wrapper_must_be_defined(checker, monkeypatch, symbols):
    waf, task = checker
    monkeypatch.setattr(waf.subprocess, "check_output", lambda *a, **kw: symbols)
    with pytest.raises(waf.Errors.WafError, match="Missing defined.*__wrap_malloc"):
        task.run()


@pytest.mark.parametrize("symbols", ["", " U _malloc\n", "00001000 W _malloc\n"])
def test_darwin_malloc_must_be_defined(checker, monkeypatch, symbols):
    waf, task = checker
    task.env.DEST_OS = "darwin"
    task.env.LINKFLAGS = []
    monkeypatch.setattr(waf.subprocess, "check_output", lambda *a, **kw: symbols)
    with pytest.raises(waf.Errors.WafError, match="Missing defined zero-filling malloc"):
        task.run()


def test_darwin_defined_malloc(checker, monkeypatch):
    waf, task = checker
    task.env.DEST_OS = "darwin"
    task.env.LINKFLAGS = []
    monkeypatch.setattr(waf.subprocess, "check_output", lambda *a, **kw: "0000000100001000 T _malloc\n")
    task.run()


@pytest.mark.parametrize("symbol", ["_malloc_r", "_malloc_r.constprop.0", "printf", "printf@@LIBC_1.0"])
def test_unwrapped_symbol_rejected_without_cxx_checks(checker, monkeypatch, symbol):
    waf, task = checker
    task.env.CHECK_MALLOC_WRAPPING = False
    task.env.SYMBOLS_BLACKLIST = ["_malloc_r", "printf"]
    monkeypatch.setattr(waf.subprocess, "check_output", lambda *a, **kw: "00001000 T %s\n" % symbol)
    with pytest.raises(waf.Errors.WafError, match="Disallowed unwrapped symbol"):
        task.run()


def test_wrapper_names_are_not_blacklisted(checker, monkeypatch):
    waf, task = checker
    task.env.CHECK_MALLOC_WRAPPING = False
    task.env.SYMBOLS_BLACKLIST = ["_malloc_r", "printf"]
    monkeypatch.setattr(waf.subprocess, "check_output", lambda *a, **kw:
                        "00001000 T __wrap__malloc_r\n00002000 T __wrap_printf\n")
    task.run()


def test_existing_cxx_blacklist(checker, monkeypatch):
    waf, task = checker
    task.env.CHECK_MALLOC_WRAPPING = False
    task.env.CHECK_SYMBOLS = True
    task.env.SIM_ENABLED = False
    monkeypatch.setattr(waf.subprocess, "check_output", lambda *a, **kw:
                        "00001000 T operator new(unsigned long)\n")
    with pytest.raises(waf.Errors.WafError, match="Disallowed symbol"):
        task.run()
    task.env.SIM_ENABLED = True
    task.run()


def compile_fixture(tmp_path, source, flags):
    if sys.platform == "darwin":
        pytest.skip("these link fixtures require GNU --wrap support")
    cc = shutil.which("cc")
    if not cc or not shutil.which("nm"):
        pytest.skip("C compiler and nm required")
    src = tmp_path / "fixture.c"
    elf = tmp_path / "fixture"
    src.write_text(source)
    subprocess.run([cc, "-O0", "-fno-builtin", str(src), "-o", str(elf)] + flags, check=True)
    return elf


@pytest.mark.parametrize("lto", [[], ["-flto"]])
def test_linked_but_unused_wrapper_is_not_enough(checker, tmp_path, lto):
    waf, task = checker
    source = """
#include <stdlib.h>
void *__wrap_malloc(size_t size) { return calloc(1, size); }
int main(void) { void *p = malloc(4); free(p); return 0; }
"""
    elf = compile_fixture(tmp_path, source, lto)
    task.inputs = [SimpleNamespace(abspath=lambda: str(elf))]
    task.env.LINKFLAGS = []
    assert " __wrap_malloc\n" in subprocess.check_output(task.env.NM + [str(elf)], text=True)
    with pytest.raises(waf.Errors.WafError, match="Missing malloc wrapping"):
        task.run()
    compile_fixture(tmp_path, source, lto + ["-Wl,--wrap,malloc"])
    task.env.LINKFLAGS = ["-Wl,--wrap,malloc"]
    task.run()


def test_same_object_reference_bypasses_linker_wrap(checker, tmp_path):
    waf, task = checker
    elf = compile_fixture(tmp_path, """
int _malloc_r(void) { return 42; }
int main(void) { return _malloc_r() != 42; }
""", ["-Wl,--wrap,_malloc_r"])
    # No __wrap__malloc_r exists, yet the linker succeeds: the call was resolved
    # within its input object. The post-link blacklist must catch this.
    subprocess.run([str(elf)], check=True)
    task.inputs = [SimpleNamespace(abspath=lambda: str(elf))]
    task.env.CHECK_MALLOC_WRAPPING = False
    task.env.SYMBOLS_BLACKLIST = ["_malloc_r"]
    with pytest.raises(waf.Errors.WafError, match="Disallowed unwrapped symbol.*_malloc_r"):
        task.run()
