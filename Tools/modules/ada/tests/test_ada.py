# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""GNAT LLVM modules: split ARM data, retained checks, reload and runtime rejection."""

import os
import shutil
import subprocess
import sys

import pack
import pytest

from elftools.dwarf.callframe import FDE
from elftools.elf.elffile import ELFFile
from test_manager import test_chibios_runtime_manager as manager_probe
from test_math_runtime import chibios_probe
from test_runtime import ROOT
from test_runtime import Vehicle
from test_threads import renode  # noqa: F401

import build

NATIVE = 'x86_64-unknown-linux-gnu'
ARM = 'thumbv7em-none-eabihf'
DEMO = ROOT / 'libraries/AP_Module/examples/simple_loop.adb'
MATH_DECLARATIONS = '''   Counter : Integer := 41;
   pragma Volatile (Counter);
   F : Float := 3.5;
   D : Long_Float := 12.25;
   pragma Volatile (F);
   pragma Volatile (D);
   type Callback is access function (Value : Interfaces.C.int) return Interfaces.C.int
     with Convention => C;
   function Add_One (Value : Interfaces.C.int) return Interfaces.C.int with Convention => C;
   function Add_Two (Value : Interfaces.C.int) return Interfaces.C.int with Convention => C;
   Callbacks : constant array (0 .. 1) of Callback := (Add_One'Access, Add_Two'Access);'''
MATH_BODIES = '''   function Add_One (Value : Interfaces.C.int) return Interfaces.C.int is (Value + 1);
   function Add_Two (Value : Interfaces.C.int) return Interfaces.C.int is (Value + 2);
'''
MATH_STATEMENTS = '''
      Counter := Counter + 1;
      if Counter /= 42 or else F * 2.0 /= 7.0 or else D * 4.0 /= 49.0 then
         return -4;
      end if;
      if Callbacks (Counter mod 2) (Interfaces.C.int (Counter - 1)) /= 42 then
         return -4;
      end if;
      return Send_Text (Ctx, Info, "MATH ok");'''


@pytest.fixture
def gnat(monkeypatch):
    if shutil.which(os.environ.get('ADA', 'llvm-gnat1')) is None:
        pytest.skip('Requires GNAT LLVM; set ADA to llvm-gnat1')

    def select(target):
        name = 'ADA_ARM_INCLUDE_PATH' if target == ARM else 'ADA_NATIVE_INCLUDE_PATH'
        if name in os.environ:
            monkeypatch.setenv('ADA_INCLUDE_PATH', os.environ[name])
    return select


def probe_source(directory, declarations, statements, bodies=''):
    source = directory / 'probe.adb'
    source.with_suffix('.ads').write_text('''with AP_Module;
with Interfaces.C;
package Probe is
''' + declarations + '''
   function Run (Ctx : AP_Module.Context) return AP_Module.Status
     with Export, Convention => C, External_Name => "ap_module_ada_run";
end Probe;
''')
    source.write_text('''package body Probe is
   use AP_Module;
   use type Status;
''' + bodies + '''   function Run (Ctx : Context) return Status is
      Result : Status;
   begin
''' + statements + '''
   end Run;
end Probe;
''')
    return source


@pytest.mark.parametrize('target,cpu', [(NATIVE, 'x86-64'), (ARM, 'cortex-m4'), (ARM, 'cortex-m7')])
@pytest.mark.parametrize('debug', [False, True])
def test_ada_rebases(tmp_path, monkeypatch, gnat, target, cpu, debug):
    gnat(target)
    monkeypatch.setenv('RUSTC', '/no/rust/installation')
    first = build.build(DEMO, tmp_path / 'first.apmod', target, 8192, cpu=cpu, debug=debug)
    second = build.build(DEMO, tmp_path / 'second.apmod', target, 8192, base=0x24123000, cpu=cpu, debug=debug)
    assert first.read_bytes() == second.read_bytes()
    image = pack.unpack(first.read_bytes())
    assert image['contract']['stack_size'] == 8192
    assert image['contract']['heap_size'] == 4096
    assert {entry[2] for entry in image['imports']} == {'gcs.send_text'}
    assert not list(ROOT.glob('simple_loop.ali'))
    if debug:
        with first.with_suffix('.elf').open('rb') as stream:
            elf = ELFFile(stream)
            # C++ bridge CFI alone does not let GDB unwind the Ada caller.
            entry = elf.get_section_by_name('.symtab').get_symbol_by_name('ap_module_ada_run')[0]
            pc = entry['st_value'] & ~1 if target == ARM else entry['st_value']
            frames = elf.get_dwarf_info(relocate_dwarf_sections=False).CFI_entries()
            assert any(isinstance(frame, FDE) and frame['initial_location'] <= pc <
                       frame['initial_location'] + frame['address_range'] for frame in frames)
            assert not elf.get_section_by_name('.debug_frame')['sh_flags'] & 2


@pytest.mark.parametrize('target', [NATIVE, ARM])
def test_mutable_globals_rebase(tmp_path, gnat, target):
    gnat(target)
    source = probe_source(tmp_path, MATH_DECLARATIONS, MATH_STATEMENTS, MATH_BODIES)
    first = build.build(source, tmp_path / 'first.apmod', target, 8192)
    second = build.build(source, tmp_path / 'second.apmod', target, 8192, base=0x24123000)
    assert first.read_bytes() == second.read_bytes()
    image = pack.unpack(first.read_bytes())
    assert image['memory_size'] - image['rx_size'] >= 4
    assert len(image['fixups']) >= 2  # Retained read-only code-pointer table.


@pytest.mark.parametrize('level', ['0', '1', '2', '3', 'z'])
def test_ada_rejects_unaudited_optimization(tmp_path, level):
    with pytest.raises(ValueError, match='require --opt-level=s'):
        build.build(DEMO, tmp_path / 'bad.apmod', NATIVE, 8192, opt_level=level)


def test_missing_gnat(tmp_path, monkeypatch):
    monkeypatch.setenv('ADA', '/no/ada/compiler')
    with pytest.raises(ValueError, match='require GNAT LLVM'):
        build.build(DEMO, tmp_path / 'bad.apmod', NATIVE, 8192)


def test_missing_llvm_backend(tmp_path, monkeypatch):
    monkeypatch.setenv('ADA', sys.executable)
    monkeypatch.setenv('LLVM_LLC', '/no/llvm/backend')
    with pytest.raises(ValueError, match='require LLVM llc 21'):
        build.build(DEMO, tmp_path / 'bad.apmod', NATIVE, 8192)


def test_old_llvm_backend(tmp_path, monkeypatch):
    monkeypatch.setenv('ADA', sys.executable)
    monkeypatch.setenv('LLVM_LLC', sys.executable)
    monkeypatch.setattr(subprocess, 'check_output', lambda *args, **kwargs: 'LLVM version 20.1.0\n')
    with pytest.raises(ValueError, match='require LLVM llc 21'):
        build.build(DEMO, tmp_path / 'bad.apmod', NATIVE, 8192)


def test_rejects_rust_cfg(tmp_path):
    with pytest.raises(ValueError, match='Rust cfg options are not supported for Ada'):
        build.build(DEMO, tmp_path / 'bad.apmod', NATIVE, 8192, cfg=('test',))


@pytest.mark.parametrize('declarations,statements', [
    ('   task Worker;', '      return OK;'),
    ('', '      return OK; exception when others => return -4;'),
    ('   type Ptr is access Integer;\n   P : Ptr := new Integer;', '      return OK;'),
    ('   function Init return Integer with Import, Convention => C;\n   Value : Integer := Init;', '      return OK;'),
])
def test_unsupported_runtime_rejected(tmp_path, capfd, gnat, declarations, statements):
    gnat(NATIVE)
    source = probe_source(tmp_path, declarations, statements)
    output = tmp_path / 'bad.apmod'
    with pytest.raises(subprocess.CalledProcessError):
        build.build(source, output, NATIVE, 8192)
    assert 'restriction' in capfd.readouterr().err.lower()
    assert not output.exists()


def test_implicit_trampoline_rejected(tmp_path, capfd, gnat):
    gnat(NATIVE)
    source = probe_source(tmp_path, '', '''
      declare
         Value : Integer := 41;
         function Nested return Integer with Convention => C;
         function Nested return Integer is (Value);
         type Callback is access function return Integer with Convention => C;
         Fn : Callback := Nested'Access;
      begin
         Value := Fn.all;
      end;
      return OK;''')
    with pytest.raises(subprocess.CalledProcessError):
        build.build(source, tmp_path / 'bad.apmod', NATIVE, 8192)
    assert 'No_Implicit_Dynamic_Code' in capfd.readouterr().err


def test_unlinked_runtime_has_no_returning_stub(tmp_path, capfd, gnat):
    gnat(NATIVE)
    source = probe_source(tmp_path, '''
   procedure Missing_Runtime
     with Import, Convention => C, External_Name => "__gnat_missing_runtime_probe";''', '''
      Missing_Runtime;
      return OK;''')
    output = tmp_path / 'bad.apmod'
    with pytest.raises(subprocess.CalledProcessError):
        build.build(source, output, NATIVE, 8192)
    assert 'undefined symbol: __gnat_missing_runtime_probe' in capfd.readouterr().err
    assert not output.exists()


def test_ada_loop_sitl(tmp_path, gnat):
    gnat(NATIVE)
    image = build.build(DEMO, tmp_path / 'ada.apmod', NATIVE, 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {image.name: image.read_bytes()}, {})
    try:
        vehicle.start()
        vehicle.text('Ada tick')
        for generation in (1, 2):
            mark = len(vehicle.texts)
            vehicle.restart(generation)
            vehicle.text('Ada tick', after=mark)
        assert not any('returned -' in text or 'panic' in text for text in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()


def test_ada_global_reset_sitl(tmp_path, gnat):
    gnat(NATIVE)
    source = probe_source(tmp_path, '   Counter : Integer := 41;', '''
      Counter := Counter + 1;
      if Counter /= 42 then return -4; end if;
      Result := Send_Text (Ctx, Info, "Ada globals reset");
      if Result /= OK then return Result; end if;
      while not Stop_Requested (Ctx) loop
         Result := Sleep_Ms (Ctx, 10);
         if Result /= OK then return Result; end if;
      end loop;
      return Stopped;''')
    image = build.build(source, tmp_path / 'globals.apmod', NATIVE, 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {image.name: image.read_bytes()}, {})
    try:
        vehicle.start()
        vehicle.text('Ada globals reset')
        mark = len(vehicle.texts)
        vehicle.restart(1)
        vehicle.text('Ada globals reset', after=mark)
    finally:
        vehicle.stop()


@pytest.mark.parametrize('declarations,statements,check', [
    ('   Counter : Integer := Integer\'Last;\n   pragma Volatile (Counter);',
     '      Counter := Counter + 1;', 'CE_Overflow_Check'),
    ('   Counter : Integer := 100;\n   pragma Volatile (Counter);\n'
     '   subtype Small is Integer range 0 .. 10;\n   Value : Small := 0;',
     '      Value := Small (Counter);', 'CE_Range_Check'),
    ('   Counter : Integer := 3;\n   pragma Volatile (Counter);\n'
     '   Values : array (1 .. 2) of Integer := (1, 2);',
     '      Counter := Values (Counter);', 'CE_Index_Check'),
    ('   Counter : Integer := 0;\n   pragma Volatile (Counter);',
     '      Counter := 100 / Counter;', 'CE_Divide_By_Zero'),
    ('   Counter : Integer := 0;\n   pragma Volatile (Counter);',
     '      pragma Assert (Counter /= 0);', 'assertion failed'),
])
def test_ada_check_sitl(tmp_path, gnat, declarations, statements, check):
    gnat(NATIVE)
    source = probe_source(tmp_path, declarations, statements + '''
      return Send_Text (Ctx, Info, "ERROR runtime check missing");''')
    image = build.build(source, tmp_path / 'check.apmod', NATIVE, 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {image.name: image.read_bytes()}, {})
    try:
        vehicle.start()
        vehicle.text('Ada ' + check)
        vehicle.text('panic; reboot required')
        assert not any('ERROR runtime check missing' in text for text in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()


@pytest.mark.parametrize('cpu', ['cortex-m4', 'cortex-m7'])
def test_ada_globals_h7(tmp_path, renode, monkeypatch, gnat, cpu):  # noqa: F811
    gnat(ARM)
    source = probe_source(tmp_path, MATH_DECLARATIONS, MATH_STATEMENTS, MATH_BODIES)
    builder = build.build

    def image(unused, output, target, stack_size):
        return builder(source, output, target, stack_size, cpu=cpu)
    monkeypatch.setattr(build, 'build', image)
    chibios_probe(tmp_path, renode, 'ada', heap_limit=0)


def test_ada_check_h7(tmp_path, renode, monkeypatch, gnat):  # noqa: F811
    gnat(ARM)
    source = probe_source(tmp_path, '   Counter : Integer := Integer\'Last;\n   pragma Volatile (Counter);', '''
      Counter := Counter + 1;
      return Send_Text (Ctx, Info, "ERROR overflow check missing");''')
    builder = build.build

    def image(original, output, *args, **kwargs):
        return builder(source if output.name == 'end.apmod' else original, output, *args, **kwargs)
    monkeypatch.setattr(build, 'build', image)
    manager_probe(tmp_path, renode, monkeypatch, 'panic', 2)
