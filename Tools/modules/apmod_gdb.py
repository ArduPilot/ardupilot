# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""GDB: source this file, then use 'apmod help'. Requires Python and pyelftools."""

import os
import subprocess
import sys
import tempfile

from pathlib import Path
from types import SimpleNamespace

import gdb
import gdb.printing
import gdb.unwinder

sys.path.insert(0, str(Path(__file__).resolve().parent))
from apmod_debug import ModuleSymbols  # noqa: E402
from apmod_debug import quote  # noqa: E402


def native(expression):
    with gdb.with_parameter('language', 'c++'):
        return gdb.parse_and_eval(expression)


def module_info(pointer):
    module = pointer.dereference()
    return dict(pointer=int(pointer), name=module['_name'].string(), checksum=int(module['_checksum']),
                rx=int(module['_rx']), rw=int(module['_rw']))


def loaded_modules():
    if gdb.selected_thread() is None:
        return []
    runtime = native("'AP_Module::Runtime::_singleton'")
    if not int(runtime):
        return []
    slots = runtime.dereference()['_slots']
    result = []
    seen = set()
    while int(slots):
        if int(slots) in seen:
            raise ValueError('module registry contains a cycle')
        seen.add(int(slots))
        slot = slots.dereference()
        if int(slot['module']):
            result.append(module_info(slot['module']))
        slots = slot['next']
    return result


class InvokeUnwinder(gdb.unwinder.Unwinder):
    """Unwind the naked ARM gate's push {r9, lr}; it has no compiler CFI."""

    def __init__(self):
        super().__init__('apmod-invoke')

    def __call__(self, pending):
        if 'arm' not in pending.architecture().name():
            return None
        pc = pending.read_register('pc')
        block = gdb.block_for_pc(int(pc))
        while block and block.function is None:
            block = block.superblock
        if not block or not block.function.name.startswith('AP_Module::Module::invoke('):
            return None
        sp = pending.read_register('sp')
        # Once the four-byte push has executed, both words remain on the
        # stack until the final pop branches out of the gate.
        if int(pc) - block.start < 4:
            return None
        saved = bytes(gdb.selected_inferior().read_memory(int(sp), 8))
        r9, lr = (int.from_bytes(saved[i:i + 4], 'little') for i in (0, 4))
        result = pending.create_unwind_info(SimpleNamespace(sp=sp + 8, pc=pc))
        result.add_saved_register('sp', sp + 8)
        result.add_saved_register('pc', gdb.Value(lr & ~1).cast(pc.type))
        result.add_saved_register('lr', gdb.Value(lr).cast(pc.type))
        for name in ('r4', 'r5', 'r6', 'r7', 'r8', 'r10', 'r11'):
            result.add_saved_register(name, pending.read_register(name))
        result.add_saved_register('r9', gdb.Value(r9).cast(pc.type))
        return result


class Lifecycle(gdb.Breakpoint):
    def __init__(self, owner, unload):
        # The destructor can be inlined, even when a standalone symbol exists.
        # release() is the ownership gate and runs before any backing memory is freed.
        super().__init__('AP_Module::Module::release(AP_Module::Module*&)' if unload else
                         'AP_Module::Module::start()', internal=True)
        self.owner = owner
        self.unload = unload

    def stop(self):
        try:
            pointer = native('(AP_Module::Module *)module' if self.unload else 'this')
            if not int(pointer):
                return False
            info = module_info(pointer)
            if self.unload:
                self.owner.remove(info['pointer'])
            else:
                self.owner.load(info)
        except (gdb.error, OSError, ValueError) as error:
            gdb.write(f'apmod lifecycle error: {error}\n', gdb.STDERR)
            return True  # Never silently continue with potentially stale breakpoints.
        return False


class Apmod(gdb.Command):
    """Native module symbols. See 'apmod help' for commands."""

    def __init__(self):
        super().__init__('apmod', gdb.COMMAND_FILES)
        self.candidates = {}
        self.loaded = {}
        self.hooks = []
        self.printer = None
        self.temporary = tempfile.TemporaryDirectory(prefix='apmod-gdb-')

    def load(self, info):
        candidate = self.candidates.get(info['checksum'])
        if candidate is None or info['pointer'] in self.loaded:
            return
        target = Path(self.temporary.name) / f'{info["pointer"]:x}-{info["checksum"]:08x}.elf'
        target.write_bytes(candidate.debug_elf(info['rx'], info['rw']))
        previous = set(gdb.objfiles())
        gdb.execute(candidate.command(target, info['rx'], info['rw']), to_string=True)
        objfile = next(item for item in gdb.objfiles() if item not in previous)
        if self.printer and candidate.is_rust:
            gdb.printing.register_pretty_printer(objfile, self.printer, replace=True)
        self.loaded[info['pointer']] = (candidate, info, objfile)
        gdb.write(f'apmod: {info["name"]} RX=0x{info["rx"]:x} RW=0x{info["rw"]:x}\n')

    def remove(self, pointer):
        entry = self.loaded.get(pointer)
        if entry is None:
            return
        candidate, info, _ = entry
        ranges = [(address, address + size) for (name, _, size) in candidate.sections
                  for address in [candidate.addresses(info['rx'], info['rw'])[name]]]
        # Remove code breakpoints BEFORE Module's destructor frees executable
        # RAM. Otherwise a later GDB restore could corrupt a reused allocation.
        for breakpoint in gdb.breakpoints() or ():
            if breakpoint in self.hooks:
                continue
            if any(location.address is not None and any(lo <= location.address < hi for lo, hi in ranges)
                   for location in breakpoint.locations):
                gdb.write(f'apmod: removing breakpoint {breakpoint.number} before unloading {info["name"]}\n')
                breakpoint.delete()
        gdb.execute(f'remove-symbol-file -a 0x{info["rx"]:x}', to_string=True)
        del self.loaded[pointer]

    def printers(self, compiler):
        sysroot = Path(subprocess.check_output([compiler, '--print', 'sysroot'], text=True).strip())
        directory = sysroot / 'lib/rustlib/etc'
        sys.path.insert(0, str(directory))
        import gdb_lookup
        self.printer = gdb_lookup.printer
        for candidate, _, objfile in self.loaded.values():
            if candidate.is_rust:
                gdb.printing.register_pretty_printer(objfile, self.printer, replace=True)
        version = subprocess.check_output([compiler, '-vV'], text=True)
        commit = next(line.split(': ', 1)[1] for line in version.splitlines() if line.startswith('commit-hash: '))
        gdb.execute(f'set substitute-path /rustc/{commit} {quote(sysroot / "lib/rustlib/src/rust")}')
        gdb.write(f'apmod: Rust pretty-printers from {sysroot}\n')

    def invoke(self, argument, from_tty):
        self.dont_repeat()
        args = gdb.string_to_argv(argument)
        try:
            if args == ['list']:
                for info in loaded_modules():
                    gdb.write(f'{info["name"]} CRC={info["checksum"]:08x} RX=0x{info["rx"]:x} RW=0x{info["rw"]:x}\n')
            elif args and args[0] == 'symbols':
                try:
                    gdb.lookup_type('AP_Module::Module')
                    gdb.lookup_type('AP_Module::Runtime')
                except gdb.error as error:
                    raise ValueError('firmware ELF needs C++ debug information; rebuild with CXXFLAGS=-g') from error
                for path in args[1:]:
                    try:
                        candidate = ModuleSymbols(path)
                    except (OSError, ValueError) as error:
                        # A wildcard may include release or stale builds. One
                        # unusable companion must not hide every other module.
                        gdb.write(f'apmod: skipping {path}: {error}\n', gdb.STDERR)
                        continue
                    if candidate.is_rust and self.printer is None:
                        try:
                            self.printers(os.environ.get('RUSTC', 'rustc'))
                        except (OSError, subprocess.CalledProcessError, ImportError, ValueError) as error:
                            gdb.write(f'apmod: Rust printers unavailable: {error}\n', gdb.STDERR)
                    self.candidates[candidate.checksum] = candidate
                if not self.hooks:
                    self.hooks = [Lifecycle(self, False), Lifecycle(self, True)]
                for info in loaded_modules():
                    self.load(info)
                unmatched = set(self.candidates) - {info['checksum'] for info in loaded_modules()}
                for checksum in unmatched:
                    gdb.write(f'apmod: waiting for matching module CRC={checksum:08x}\n')
            elif args and args[0] == 'printers' and len(args) <= 2:
                self.printers(args[1] if len(args) == 2 else os.environ.get('RUSTC', 'rustc'))
            elif args == ['clear']:
                for pointer in list(self.loaded):
                    self.remove(pointer)
                for hook in self.hooks:
                    hook.delete()
                self.hooks = []
                self.candidates.clear()
            else:
                gdb.write('apmod list                  list loaded module addresses and CRCs\n'
                          'apmod symbols FILE.apmod... load matching ELFs; follow starts/unloads\n'
                          'apmod printers [RUSTC]      load matching Rust pretty-printers\n'
                          'apmod clear                 remove module breakpoints, symbols and hooks\n')
        except (gdb.error, OSError, ValueError, subprocess.CalledProcessError, ImportError) as error:
            raise gdb.GdbError(f'apmod: {error}') from error


if not hasattr(gdb, '_apmod_helper'):
    gdb._apmod_helper = Apmod()
gdb.unwinder.register_unwinder(None, InvokeUnwinder(), replace=True)
