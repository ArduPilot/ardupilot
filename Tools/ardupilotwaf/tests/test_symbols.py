# AP_FLAKE8_CLEAN
"""Post-link checks, including failures that --wrap alone cannot catch."""

import importlib
import json
import shutil
import subprocess
import sys

from pathlib import Path
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


@pytest.mark.parametrize("symbols", [" U __wrap_malloc\n", "00001000 W __wrap_malloc\n",
                                     " U malloc@GLIBC_2.2.5\n"])
def test_wrapper_must_be_defined(checker, monkeypatch, symbols):
    waf, task = checker
    monkeypatch.setattr(waf.subprocess, "check_output", lambda *a, **kw: symbols)
    with pytest.raises(waf.Errors.WafError, match="Missing defined.*__wrap_malloc"):
        task.run()


def test_no_malloc_reference(checker, monkeypatch):
    # e.g. AP_DAL_Standalone only allocates through operator new and calloc
    waf, task = checker
    monkeypatch.setattr(waf.subprocess, "check_output", lambda *a, **kw:
                        "00001000 T _Znwm\n                 U calloc@GLIBC_2.2.5\n")
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


@pytest.mark.parametrize("kind", ["b", "d", "g", "r", "s"])
def test_local_data_is_not_a_libc_function(checker, monkeypatch, kind):
    waf, task = checker
    task.env.CHECK_MALLOC_WRAPPING = False
    task.env.SYMBOLS_BLACKLIST = ["time"]
    monkeypatch.setattr(waf.subprocess, "check_output", lambda *a, **kw: "00001000 %s time\n" % kind)
    task.run()


@pytest.mark.parametrize("entry", ["00001000 t time", "00001000 W time", " U time", " w time",
                                   "00001000 i time", "00001000 D time"])
@pytest.mark.parametrize("data_first", [False, True])
def test_local_data_cannot_hide_a_forbidden_symbol(checker, monkeypatch, entry, data_first):
    waf, task = checker
    task.env.CHECK_MALLOC_WRAPPING = False
    task.env.SYMBOLS_BLACKLIST = ["time"]
    entries = [entry, "00002000 b time"]
    if data_first:
        entries.reverse()
    monkeypatch.setattr(waf.subprocess, "check_output", lambda *a, **kw: "\n".join(entries) + "\n")
    with pytest.raises(waf.Errors.WafError, match="Disallowed unwrapped symbol.*time"):
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


def test_linked_local_data_is_not_blacklisted(checker, tmp_path):
    _, task = checker
    elf = compile_fixture(tmp_path, """
static unsigned time;
int main(void) { return time; }
""", [])
    task.inputs = [SimpleNamespace(abspath=lambda: str(elf))]
    task.env.CHECK_MALLOC_WRAPPING = False
    task.env.SYMBOLS_BLACKLIST = ["time"]
    task.run()


@pytest.mark.parametrize("policy, value, error", [
    ("SYMBOLS_BLACKLIST", ["_malloc_r"], "Disallowed unwrapped symbol"),
    ("CHECK_MALLOC_WRAPPING", False, None),
    ("CHECK_SYMBOLS", True, None),
    ("vehicle_binary", False, None),
    ("SIM_ENABLED", True, None),
    ("LINKFLAGS", [], "Missing malloc wrapping"),
    ("DEST_OS", "darwin", "Missing defined zero-filling malloc"),
    ("NM", [shutil.which("nm") or "nm", "--defined-only"], None),
])
def test_incremental_policy_change(tmp_path, policy, value, error):
    # Run real Waf builds against the same ELF, changing only the check policy.
    elf = compile_fixture(tmp_path, """
#include <stdlib.h>
void *__wrap_malloc(size_t size) { return calloc(1, size); }
int _malloc_r(void) { return 42; }
int main(void) { void *p = malloc(4); free(p); return _malloc_r() != 42; }
""", ["-Wl,--wrap,malloc"])
    original_elf = elf.read_bytes()
    root = Path(__file__).resolve().parents[3]
    (tmp_path / "wscript").write_text(f"""
import json
import sys
sys.path.insert(0, {str(root)!r})
from Tools.ardupilotwaf import ardupilotwaf

top = '.'
out = 'build'

def configure(cfg):
    pass

def build(bld):
    for key, value in json.loads(bld.path.find_node('policy.json').read()).items():
        bld.env[key] = value
    generator = bld(name='symbols')
    generator.create_task('check_elf_symbols', src=bld.path.find_node('fixture'))
""")
    settings = {
        "NM": [shutil.which("nm")],
        "vehicle_binary": True,
        "SIM_ENABLED": False,
        "CHECK_SYMBOLS": False,
        "CHECK_MALLOC_WRAPPING": True,
        "LINKFLAGS": ["-Wl,--wrap,malloc"],
        "DEST_OS": "linux",
        "SYMBOLS_BLACKLIST": [],
    }
    config = tmp_path / "policy.json"
    config.write_text(json.dumps(settings))

    def run_waf(command):
        return subprocess.run([sys.executable, str(root / "modules/waf/waf-light"), command],
                              cwd=tmp_path, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    configured = run_waf("configure")
    assert configured.returncode == 0, configured.stdout
    first = run_waf("build")
    assert first.returncode == 0, first.stdout
    assert "checking symbols" in first.stdout
    unchanged = run_waf("build")
    assert unchanged.returncode == 0, unchanged.stdout
    assert "checking symbols" not in unchanged.stdout

    settings[policy] = value
    config.write_text(json.dumps(settings))
    changed = run_waf("build")
    assert "checking symbols" in changed.stdout
    if error:
        assert changed.returncode != 0, changed.stdout
        assert error in changed.stdout
    else:
        assert changed.returncode == 0, changed.stdout
    assert elf.read_bytes() == original_elf
