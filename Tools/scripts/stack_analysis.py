#!/usr/bin/env python3
'''
Static stack usage analysis for ArduPilot ChibiOS firmware

Combines:
 - the call graph and frame sizes from GCC's -fcallgraph-info=su (.ci files)
 - GCC's IPA call graph dumps (-fdump-ipa-cgraph), which give the static
   type and vtable slot of each virtual call
 - DWARF debug info, which gives the class hierarchy and the method in each
   vtable slot, so virtual calls resolve to the possible overrides
 - disassembly of the ELF, for functions with no call graph info (ChibiOS
   kernel, newlib, assembly) and to find Functor callbacks given to the
   scheduler and device buses

Build with:

  SU="-fstack-usage -fcallgraph-info=su"
  CFLAGS="$SU" CXXFLAGS="$SU -fdump-ipa-cgraph" LINKFLAGS="$SU -fdump-ipa-cgraph -save-temps" \\
      ./waf configure --board X -g
  ./waf plane

then:

  Tools/scripts/stack_analysis.py build/X --elf build/X/bin/arduplane [--threads threads.txt]

The worst case for each thread is the deepest path through the call
graph from its entry point. Each function in a recursive cycle is counted
at most once on a path, deeper recursion is not bounded. Recursive cycles
too large to enumerate paths through count every function in them. Calls
that cannot be resolved are counted and can be listed with --unresolved.
Shared telemetry methods retain the receiver type for calls on this.
Suppression rules can depend on an ancestor and a chain of intermediate
methods, so unrelated callers keep their ordinary call graph. Contexts
are shown on paths; separate contexts can count a function more than once.

The result is an estimate. It can be too high, as the call graph includes
paths that can't happen at runtime (see stack_analysis_suppressions.txt),
and too low because of these known gaps:
 - unresolved indirect calls, mostly C function pointers and calls hidden
   in macros
 - virtual calls through a secondary base class can miss the override
 - Functor callbacks registered with the scheduler and device buses are
   mostly not found yet, so timer, io and bus thread callbacks are missing
 - interrupt and exception frames pushed onto a thread's stack are not
   included
 - stacks of threads created at runtime are not known, so only threads
   with statically allocated stacks are checked
 - only the vehicle setup()/loop() and the code of their callers are
   analysed for the main thread, not other code run during startup by
   functions called through pointers

AP_FLAKE8_CLEAN
'''

import argparse
import glob
import json
import os
import re
import subprocess
import sys

INDIRECT = '__indirect_call'

# Shared telemetry methods call virtual methods on their own receiver.
# Keep that receiver's type when a derived handler calls these methods.
RECEIVER_BASES = ('AP_RCTelemetry',)

# thread name -> regex on demangled entry function(s)
THREAD_ENTRIES = [
    ('main', r'^AP_Vehicle::(setup|loop)\(\)$|^(setup|loop)\(\)$'),
    ('timer', r'^ChibiOS::Scheduler::_timer_thread\('),
    ('rcout', r'^ChibiOS::Scheduler::_rcout_thread\('),
    ('rcin', r'^ChibiOS::Scheduler::_rcin_thread\('),
    ('io', r'^ChibiOS::Scheduler::_io_thread\('),
    ('storage', r'^ChibiOS::Scheduler::_storage_thread\('),
    ('monitor', r'^ChibiOS::Scheduler::_monitor_thread\('),
    ('UART', r'^ChibiOS::UARTDriver::uart_thread\('),
    ('UART_RX', r'^ChibiOS::UARTDriver::uart_rx_thread\('),
    ('bus', r'^ChibiOS::DeviceBus::bus_thread\('),
    ('IOMCU', r'^void Functor<void>::method_wrapper<AP_IOMCU, &AP_IOMCU::thread_main>'),
    ('log_io', r'^void Functor<void>::method_wrapper<AP_Logger, &AP_Logger::io_thread>'),
    ('FTP', r'^void Functor<void>::method_wrapper<GCS_FTP, &GCS_FTP::worker>'),
    ('dronecan', r'^void Functor<void>::method_wrapper<AP_DroneCAN, &AP_DroneCAN::loop>'),
    ('mount_calc_po', r'^void Functor<void>::method_wrapper<AP_Mount, '),
    ('idle', r'^__idle_thread$'),
]

# callers of a thread's entry points, outermost first. Their frames stay
# on the stack below the entry points, and their other calls are analysed
THREAD_PREFIX = {
    'main': [r'^main$', r'^HAL_ChibiOS::run\(', r'^main_loop\(\)$'],
    'functor': [r'^ChibiOS::Scheduler::thread_create_trampoline\('],
}

# map names from @SYS/threads.txt onto the entries above
THREAD_NAME_MAP = [
    (r'^Ardu|^AP_Periph$|^main$', 'main'),
    (r'^(OTG\d|UART\d+)$', 'UART'),
    (r'^(I2C|SPI)\d+$', 'bus'),
    (r'^dronecan_\d+$', 'dronecan'),
]

# Functor calls in these functions call callbacks from these task tables
TASK_TABLES = [
    (r'^AP_Scheduler::run\(', r'scheduler_tasks$'),
]

# Functor calls in these functions call callbacks passed to these functions
REGISTRATIONS = [
    (r'^ChibiOS::Scheduler::_run_timers\(', r'::register_timer_process\('),
    (r'^ChibiOS::Scheduler::_run_io\(', r'::register_io_process\('),
    (r'^ChibiOS::DeviceBus::bus_thread\(', r'::register_periodic_callback\('),
    (r'^ChibiOS::Scheduler::thread_create_trampoline\(', r'::thread_create\('),
]

# macros that hide the called method
MACRO_CALLS = {
    'DEV_PRINTF': 'printf',
    'WITH_SEMAPHORE': 'take_blocking',
    'GCS_SEND_TEXT': 'send_text',
}

# statically allocated thread working areas
THREAD_WORKING_AREAS = {
    'timer': '_timer_thread_wa',
    'rcout': '_rcout_thread_wa',
    'rcin': '_rcin_thread_wa',
    'io': '_io_thread_wa',
    'storage': '_storage_thread_wa',
    'monitor': '_monitor_thread_wa',
}

# ChibiOS structures that share a thread's working area with its stack
THREAD_STRUCTS = ('ch_thread', 'port_intctx', 'port_extctx')

# calls into these are not followed
DEFAULT_CUT = [r'^AP_HAL::panic\(', r'^chSysHalt$']

KNOWN_ROOTS = ('libraries/', 'modules/', 'Tools/', 'ArduPlane/', 'ArduCopter/', 'Rover/',
               'ArduSub/', 'AntennaTracker/', 'Blimp/')


def strip_partition(title):
    '''remove the file or ltrans partition prefix GCC adds to local symbols'''
    if ':' in title:
        prefix, sym = title.rsplit(':', 1)
        if '/' in prefix or re.search(r'\.(o|c|cpp|cc)$', prefix):
            return sym
    return title


def base_symbol(title):
    '''symbol without partition prefix or GCC clone suffixes'''
    return re.sub(r'\.(constprop|isra|part|lto_priv|cold)\.\d+', '', strip_partition(title))


def normalise_path(path):
    '''map a path from a .ci label onto the source tree'''
    path = path.replace('\\', '/')
    best = None
    for r in KNOWN_ROOTS:
        idx = path.rfind(r)
        if idx != -1 and (best is None or idx < best):
            best = idx
    return path[best:] if best is not None else path


def tu_of(fname):
    '''translation unit key shared by a .ci file and its cgraph dump'''
    base = os.path.basename(fname)
    if '.ltrans' in base or '.wpa.' in base:
        return 'LTO'
    m = re.match(r'(.*\.(?:cpp|c|cc|S))(?:\.\d+)?\.(?:ci|\d+i\.cgraph)$', fname)
    return m.group(1) if m else fname


def demangle_list(cxxfilt, names):
    try:
        out = subprocess.run([cxxfilt], input='\n'.join(names), capture_output=True,
                             text=True, check=True).stdout.splitlines()
        if len(out) == len(names):
            return dict(zip(names, out))
    except (OSError, subprocess.CalledProcessError):
        pass
    return {n: n for n in names}


def short_name(demangled):
    '''unqualified name of a demangled function'''
    depth = 0
    for i, c in enumerate(demangled):
        if c == '<':
            depth += 1
        elif c == '>':
            depth -= 1
        elif c == '(' and depth == 0:
            demangled = demangled[:i]
            break
    m = re.search(r'([~\w]+|operator\S+)\s*$', demangled)
    return m.group(1) if m else demangled


def type_name(t):
    '''unqualified class name from a GCC dump type string'''
    t = re.sub(r'\b(const|volatile|struct|class|union)\s+', '', t).strip()
    depth = 0
    last = 0
    for i, c in enumerate(t):
        if c == '<':
            depth += 1
        elif c == '>':
            depth -= 1
        elif c == ':' and depth == 0 and t[i:i+2] == '::':
            last = i + 2
    return t[last:]


class Func:
    __slots__ = ('title', 'name', 'names', 'loc', 'size', 'dynamic', 'edges', 'tu', 'origin', 'context')

    def __init__(self, title, name, loc, size, dynamic, tu, origin):
        self.title = title
        self.name = name
        self.names = set()
        self.loc = loc
        self.size = size
        self.dynamic = dynamic
        self.edges = []
        self.tu = tu
        self.origin = origin
        self.context = ''


class Program:
    '''everything known about the firmware'''

    def __init__(self, cxxfilt):
        self.cxxfilt = cxxfilt
        self.funcs = {}
        self.by_symbol = {}
        self.demangled = {}
        self.poly_by_func = {}
        self.poly_by_tu = {}
        self.elf_addr = {}
        self.elf_names_at = {}
        self.elf_func_addr = {}
        self.elf_frames = {}
        self.elf_dynamic = set()
        self.elf_indirect = {}
        self.elf_calls = {}
        self.elf_returns = {}
        self.consts = {}
        self.data_syms = []
        self.sections = []
        self.sym_values = {}

    def load_ci(self, files):
        '''call graph and frame sizes from -fcallgraph-info=su'''
        node_re = re.compile(r'^node: \{ title: "([^"]*)" label: "([^"]*)"')
        edge_re = re.compile(r'^edge: \{ sourcename: "([^"]*)" targetname: "([^"]*)" label: "([^"]*)"')
        for fname in files:
            tu = tu_of(fname)
            edges = []
            local = {}
            with open(fname) as f:
                for line in f:
                    m = node_re.match(line)
                    if m:
                        fn = self.add_ci_node(m.group(1), m.group(2), tu)
                        if fn is not None:
                            local[fn.title] = fn
                        continue
                    m = edge_re.match(line)
                    if m:
                        edges.append(m.groups())
            for src, dst, loc in edges:
                fn = local.get(src)
                if fn is not None:
                    fn.edges.append((dst, normalise_path(loc)))

    def add_ci_node(self, title, label, tu):
        parts = label.split('\\n')
        if len(parts) < 3:
            return None
        m = re.match(r'(\d+) bytes \(([a-z,]+)\)', parts[2])
        if not m or title in self.funcs:
            return None
        dynamic = 'dynamic' in m.group(2) and 'bounded' not in m.group(2)
        f = Func(title, parts[0], parts[1], int(m.group(1)), dynamic, tu, 'ci')
        self.funcs[title] = f
        self.by_symbol.setdefault(base_symbol(title), []).append(f)
        return f

    def load_cgraph_dumps(self, files):
        '''static type and vtable slot of virtual calls from -fdump-ipa-cgraph'''
        node_re = re.compile(r'^(\S+)/\d+ \(')
        poly_re = re.compile(r'^\s+Polymorphic indirect call of type (.*?) token:(\d+)')
        for fname in files:
            tu = tu_of(fname)
            cur = None
            with open(fname, errors='replace') as f:
                for line in f:
                    m = node_re.match(line)
                    if m:
                        cur = base_symbol(m.group(1))
                        continue
                    m = poly_re.match(line)
                    if m and cur is not None:
                        key = (type_name(m.group(1)), int(m.group(2)))
                        self.poly_by_func.setdefault((tu, cur), set()).add(key)
                        self.poly_by_tu.setdefault(tu, set()).add(key)

    def load_elf(self, elf_file):
        '''symbols, initialised data and disassembly of the firmware'''
        from elftools.elf.elffile import ELFFile
        with open(elf_file, 'rb') as f:
            elf = ELFFile(f)
            for s in elf.get_section_by_name('.symtab').iter_symbols():
                self.sym_values.setdefault(s.name, (s['st_value'], s['st_size']))
                if s['st_info']['type'] == 'STT_FUNC' and s['st_value'] != 0:
                    self.elf_addr.setdefault(s['st_value'] & ~1, s.name)
                    self.elf_names_at.setdefault(s['st_value'] & ~1, []).append(s.name)
                elif s['st_info']['type'] == 'STT_OBJECT' and s['st_size'] > 0:
                    self.data_syms.append((s.name, s['st_value'], s['st_size']))
            for sec in elf.iter_sections():
                if sec['sh_addr'] and sec['sh_type'] == 'SHT_PROGBITS':
                    self.sections.append((sec['sh_addr'], sec.data()))
        self.disassemble(elf_file)

    def read_data(self, addr, size):
        for base, data in self.sections:
            if base <= addr and addr + size <= base + len(data):
                return data[addr-base:addr-base+size]
        return None

    def disassemble(self, elf_file):
        '''frame sizes, direct calls and constants for every function in the ELF'''
        out = subprocess.run(['arm-none-eabi-objdump', '-d', '--no-show-raw-insn', elf_file],
                             capture_output=True, text=True).stdout
        func = None
        regs = {}
        insns = []

        def constant_return():
            # Only accept an unconditional, two-instruction constant return.
            if func is not None and len(insns) == 2 and re.fullmatch(r'bx\s+lr', insns[1]):
                m = re.fullmatch(r'movs?(?:\.w)?\s+r0,\s*#(\d+)', insns[0])
                if m:
                    self.elf_returns[func] = int(m.group(1))

        for line in out.splitlines():
            m = re.match(r'^([0-9a-f]+) <(.+)>:$', line)
            if m:
                constant_return()
                func = m.group(2)
                insns = []
                self.elf_func_addr[func] = int(m.group(1), 16)
                self.elf_frames[func] = 0
                self.elf_indirect[func] = 0
                self.elf_calls[func] = set()
                self.consts[func] = set()
                regs = {}
                continue
            if func is None or '\t' not in line:
                continue
            insn = line.split('\t', 1)[1]
            if len(insns) < 3:
                insns.append(insn.split(';', 1)[0].strip())
            m = re.match(r'(push|stmdb)(?:\.w)?\s+(?:sp!,\s*)?\{([^}]*)\}', insn)
            if m:
                self.elf_frames[func] += 4 * len(m.group(2).split(','))
                continue
            m = re.match(r'vpush\s+\{([^}]*)\}', insn)
            if m:
                n = 0
                rlist = [r.strip() for r in m.group(1).split(',')]
                for r in rlist:
                    if '-' in r:
                        a, b = r.split('-')
                        n += int(b[1:]) - int(a[1:]) + 1
                    else:
                        n += 1
                self.elf_frames[func] += 4 * n * (2 if rlist[0].startswith('d') else 1)
                continue
            m = re.match(r'sub(?:w|\.w)?\s+sp,\s*(?:sp,\s*)?#(\d+)', insn)
            if m:
                self.elf_frames[func] += int(m.group(1))
                continue
            # pre-indexed stores that move sp, e.g. strd ip, lr, [sp, #-16]!
            m = re.match(r'(?:str|strd|vstr)\S*\s+.*\[sp,\s*#-(\d+)\]!', insn)
            if m:
                self.elf_frames[func] += int(m.group(1))
                continue
            if re.match(r'sub(?:w|\.w|s)?\s+sp,\s*(?:sp,\s*)?(?:r\d+|ip|lr)\b', insn):
                # variable sized frame
                self.elf_dynamic.add(func)
                continue
            if re.match(r'(?:blx|bx)\s+(?:r\d+|ip)\b', insn) or re.match(r'(?:ldr|mov)\S*\s+pc,', insn):
                # call or tail call through a register
                self.elf_indirect[func] += 1
                continue
            m = re.match(r'(bl|blx|b(?:eq|ne|cs|hs|cc|lo|mi|pl|vs|vc|hi|ls|ge|lt|gt|le|al)?(?:\.w|\.n)?'
                         r'|cbn?z\s+\S+,)\s+[0-9a-f]+ <([^>+]+)(?:\+0x[0-9a-f]+)?>', insn)
            if m:
                if m.group(1) in ('bl', 'blx') or m.group(2) != func:
                    # calls, and branches into other functions
                    self.elf_calls[func].add(m.group(2))
                continue
            m = re.match(r'\.word\s+0x([0-9a-f]+)', insn)
            if m:
                self.consts[func].add(int(m.group(1), 16))
                continue
            m = re.match(r'movw\s+(r\d+|ip|lr),\s*#(\d+)', insn)
            if m:
                regs[m.group(1)] = int(m.group(2))
                continue
            m = re.match(r'movt\s+(r\d+|ip|lr),\s*#(\d+)', insn)
            if m and m.group(1) in regs:
                self.consts[func].add(regs.pop(m.group(1)) | (int(m.group(2)) << 16))
        constant_return()

    def remove_unlinked(self):
        '''drop functions with call graph info that the linker discarded'''
        if not self.sym_values:
            return 0
        removed = 0
        for sym in list(self.by_symbol.keys()):
            fs = self.by_symbol[sym]
            keep = [f for f in fs if f.origin != 'ci' or sym in self.sym_values or
                    strip_partition(f.title) in self.sym_values]
            if len(keep) != len(fs):
                removed += len(fs) - len(keep)
                for f in fs:
                    if f not in keep:
                        self.funcs.pop(f.title, None)
                if keep:
                    self.by_symbol[sym] = keep
                else:
                    del self.by_symbol[sym]
        return removed

    def add_elf_funcs(self):
        '''add functions that have no call graph info'''
        for name, frame in self.elf_frames.items():
            if base_symbol(name) in self.by_symbol:
                continue
            # another name for a function that has call graph info,
            # e.g. the C1/C2 constructor aliases
            addr = self.elf_func_addr.get(name)
            alias = None
            for other in self.elf_names_at.get(addr, []):
                if other != name and base_symbol(other) in self.by_symbol:
                    alias = self.by_symbol[base_symbol(other)]
                    break
            if alias is not None:
                self.by_symbol[base_symbol(name)] = alias
                continue
            f = Func(name, name, 'elf', frame, name in self.elf_dynamic, 'ELF', 'elf')
            for c in self.elf_calls.get(name, []):
                f.edges.append((c, 'elf'))
            for i in range(self.elf_indirect.get(name, 0)):
                f.edges.append((INDIRECT, 'elf:%s:%u' % (name, i)))
            self.funcs[name] = f
            self.by_symbol.setdefault(base_symbol(name), []).append(f)

    def demangle_all(self):
        self.demangled = demangle_list(self.cxxfilt, sorted(self.by_symbol.keys()))
        # unqualified function names, for matching against call sites. A
        # function can have several names when symbols are aliases
        for f in self.all_funcs():
            f.names = set()
        for sym, fs in self.by_symbol.items():
            name = short_name(self.demangled.get(sym, sym))
            for f in fs:
                f.names.add(name)
                if base_symbol(f.title) == sym:
                    f.name = name

    def demangle(self, f):
        return self.demangled.get(f.title, self.demangled.get(base_symbol(f.title), f.title))

    def describe(self, f):
        return self.demangle(f) + (' [%s]' % f.context if f.context else '')

    def all_funcs(self):
        seen = set()
        result = []
        for fs in self.by_symbol.values():
            for f in fs:
                if f.title not in seen:
                    seen.add(f.title)
                    result.append(f)
        return result


class ClassModel:
    '''class hierarchy and vtable slots from DWARF'''

    def __init__(self):
        self.bases = {}
        self.slots = {}
        self.by_unqual = {}
        self.derived = {}
        self.struct_sizes = {}
        self.cu_names = set()

    def load(self, elf_file, cache):
        st = os.stat(elf_file)
        key = '%u:%u' % (st.st_size, st.st_mtime_ns)
        if cache and os.path.exists(cache):
            with open(cache) as f:
                d = json.load(f)
            if d.get('key') == key:
                self.bases = d['bases']
                self.slots = {c: {int(k): v for k, v in s.items()} for c, s in d['slots'].items()}
                self.struct_sizes = d.get('struct_sizes', {})
                self.cu_names = set(d.get('cu_names', []))
                self.index()
                return
        self.parse(elf_file)
        if cache:
            with open(cache, 'w') as f:
                json.dump({'key': key, 'bases': self.bases, 'slots': self.slots,
                           'struct_sizes': self.struct_sizes, 'cu_names': sorted(self.cu_names)}, f)
        self.index()

    def parse(self, elf_file):
        from elftools.elf.elffile import ELFFile

        def qname(die):
            parts = []
            d = die
            while d is not None and d.tag in ('DW_TAG_class_type', 'DW_TAG_structure_type', 'DW_TAG_namespace'):
                n = d.attributes.get('DW_AT_name')
                parts.append(n.value.decode(errors='replace') if n else '?')
                d = d.get_parent()
            return '::'.join(reversed(parts))

        with open(elf_file, 'rb') as f:
            dw = ELFFile(f).get_dwarf_info()
            for cu in dw.iter_CUs():
                n = cu.get_top_DIE().attributes.get('DW_AT_name')
                if n is not None:
                    self.cu_names.add(normalise_path(n.value.decode(errors='replace')))
                for die in cu.iter_DIEs():
                    if die.tag not in ('DW_TAG_class_type', 'DW_TAG_structure_type'):
                        continue
                    if 'DW_AT_declaration' in die.attributes:
                        continue
                    q = qname(die)
                    if q in THREAD_STRUCTS and 'DW_AT_byte_size' in die.attributes:
                        self.struct_sizes[q] = die.attributes['DW_AT_byte_size'].value
                    bases = self.bases.setdefault(q, [])
                    slots = self.slots.setdefault(q, {})
                    for c in die.iter_children():
                        if c.tag == 'DW_TAG_inheritance':
                            b = qname(c.get_DIE_from_attribute('DW_AT_type'))
                            if b not in bases:
                                bases.append(b)
                        elif c.tag == 'DW_TAG_subprogram' and 'DW_AT_vtable_elem_location' in c.attributes:
                            loc = c.attributes['DW_AT_vtable_elem_location'].value
                            ln = c.attributes.get('DW_AT_linkage_name')
                            if isinstance(loc, list) and len(loc) == 2 and ln is not None:
                                slots[loc[1]] = ln.value.decode(errors='replace')

    def index(self):
        for c, bases in self.bases.items():
            self.by_unqual.setdefault(type_name(c), []).append(c)
            for b in bases:
                self.derived.setdefault(b, []).append(c)

    def slot_method(self, cls, slot):
        '''method in a vtable slot, inherited from the primary base if not overridden'''
        seen = set()
        while cls is not None and cls not in seen:
            seen.add(cls)
            s = self.slots.get(cls, {})
            if slot in s:
                return s[slot]
            bases = self.bases.get(cls, [])
            cls = bases[0] if bases else None
        return None

    def targets(self, tname, slot):
        '''linkage names of all methods a call on type tname, slot could reach'''
        result = set()
        todo = list(self.by_unqual.get(tname, []))
        seen = set()
        while todo:
            c = todo.pop()
            if c in seen:
                continue
            seen.add(c)
            m = self.slot_method(c, slot)
            if m:
                result.add(m)
            todo.extend(self.derived.get(c, []))
        return result

    def is_base(self, derived, base):
        '''whether base is derived itself or one of its base classes'''
        todo = [derived]
        seen = set()
        while todo:
            c = todo.pop()
            if c == base:
                return True
            if c not in seen:
                seen.add(c)
                todo.extend(self.bases.get(c, []))
        return False


class Analyser:
    def __init__(self, prog, classes, srcroot, builddir, cut, functors, suppressions=()):
        self.p = prog
        self.suppressions = [(scope, re.compile(a), re.compile(b), leaf,
                              re.compile(via) if via else None, re.compile(through) if through else None)
                             for scope, a, b, leaf, via, through in suppressions]
        self.suppressed = {}
        self.classes = classes
        self.srcroot = srcroot
        self.builddir = builddir
        self.cut = [re.compile(c) for c in cut]
        self.functors = functors
        self.lines = {}
        self.site_info = {}
        self.poly_cache = {}
        self.dispatchers = []
        self.setup_callbacks()
        self.build()

    def source_line(self, path, lineno):
        key = (path, lineno)
        if key not in self.lines:
            text = None
            for root in (self.srcroot, self.builddir):
                p = os.path.join(root, path)
                if os.path.exists(p):
                    with open(p, errors='replace') as f:
                        lines = f.readlines()
                    text = lines[lineno-1] if lineno <= len(lines) else None
                    break
            self.lines[key] = text
        return self.lines[key]

    def site_name(self, loc):
        '''name of the function or method called at loc'''
        m = re.match(r'(.*):(\d+):(\d+)$', loc)
        if not m:
            return None
        line = self.source_line(m.group(1), int(m.group(2)))
        if line is None:
            return None
        col = int(m.group(3)) - 1
        # GCC gives either the end of the called name or the start of the
        # call expression
        m2 = re.search(r'([A-Za-z_~]\w*)\s*$', line[:col])
        if not m2 or not re.match(r'\s*\(', line[col:]):
            m2 = re.match(r'\s*(?:[A-Za-z_]\w*(?:\[[^\]]*\])?\s*(?:->|\.|::)\s*)*([A-Za-z_~]\w*)\s*\(', line[col:])
        if not m2:
            return None
        return MACRO_CALLS.get(m2.group(1), m2.group(1))

    def own_receiver(self, loc, cls):
        '''only bare member calls, this-> calls and qualified base calls
        prove that the caller and callee have the same receiver'''
        m = re.match(r'(.*):(\d+):(\d+)$', loc)
        if not m:
            return False
        line = self.source_line(m.group(1), int(m.group(2)))
        if line is None:
            return False
        col = int(m.group(3)) - 1
        call = re.search(r'([A-Za-z_~]\w*)\s*$', line[:col])
        if not call or not re.match(r'\s*\(', line[col:]):
            call = re.match(r'([A-Za-z_~]\w*)\s*\(', line[col:])
            if not call:
                return False
            before = line[:col].rstrip()
        else:
            before = line[:call.start()].rstrip()
        if before.endswith('->'):
            return re.search(r'\bthis\s*->$', before) is not None
        if before.endswith('.'):
            return False
        if before.endswith('::'):
            qualifier = re.search(r'([\w:]+)::$', before)
            if qualifier is None:
                return False
            receiver = before[:qualifier.start()].rstrip()
            if receiver.endswith('.') or (receiver.endswith('->') and
                                          re.search(r'\bthis\s*->$', receiver) is None):
                return False
            return self.classes.is_base(cls, qualifier.group(1))
        return True

    def owner(self, f):
        name = self.p.demangle(f).split('(', 1)[0]
        cls = name.rsplit('::', 1)[0] if '::' in name else None
        return cls if cls in self.classes.bases else None

    def contextual(self, f, suffix, context):
        g = Func(f.title + suffix, f.name, f.loc, f.size, f.dynamic, f.tu, f.origin)
        g.names = f.names
        g.context = context
        self.p.demangled[g.title] = self.p.demangle(f)
        self.unresolved[g.title] = set(self.unresolved.get(f.title, ()))
        self.callback_succ[g.title] = set(self.callback_succ.get(f.title, ()))
        return g

    def add_receiver_contexts(self, cut):
        '''specialise shared methods only across calls on the same object;
        calls through other objects or without source information stay broad'''
        originals = list(self.funcs)
        owners = {f.title: self.owner(f) for f in originals}
        shared = {f.title for f in originals if owners[f.title] in RECEIVER_BASES}
        clones = {}
        todo = []

        def receiver_node(f, cls):
            key = (f.title, cls)
            if key not in clones:
                g = self.contextual(f, '#receiver=' + cls, 'this=' + cls)
                clones[key] = g
                todo.append((f, g, cls))
            return clones[key]

        def successors(f, cls):
            out = {}
            for dst, loc in f.edges:
                ts, why = self.targets(f, dst, loc)
                own = self.own_receiver(loc, owners[f.title])
                for t in ts:
                    if t.title in cut:
                        continue
                    owner = owners.get(t.title)
                    if own and why == 'virtual' and f.title in shared and owner is not None:
                        if not (self.classes.is_base(cls, owner) or self.classes.is_base(owner, cls)):
                            continue
                    if own and t.title in shared and self.classes.is_base(cls, owner):
                        t = receiver_node(t, cls)
                    out[t.title] = t
            return list(out.values())

        for f in originals:
            cls = owners[f.title]
            if cls is not None and any(self.classes.is_base(cls, base) for base in RECEIVER_BASES):
                self.raw_succ[f.title] = successors(f, cls)
        while todo:
            f, g, cls = todo.pop()
            self.raw_succ[g.title] = successors(f, cls)
        self.funcs.extend(clones.values())

    def remove_initialised_statustext(self):
        '''send_textv only services startup text if vehicle_initialised is
        false. Keep the branch unless every linked implementation returns true'''
        predicates = [f for f in self.funcs if f.name == 'vehicle_initialised']
        if not predicates or not all(self.p.elf_returns.get(strip_partition(f.title)) == 1 for f in predicates):
            return

        def guarded(loc):
            m = re.match(r'(.*):(\d+):(\d+)$', loc)
            if not m:
                return False
            path, lineno = m.group(1), int(m.group(2))
            line = self.source_line(path, lineno)
            previous = self.source_line(path, lineno - 1) if lineno > 1 else None
            return (line is not None and previous is not None and
                    re.fullmatch(r'\s*service_statustext\(\);\s*', line) is not None and
                    re.fullmatch(r'\s*if\s*\(!vehicle_initialised\(\)\)\s*\{\s*', previous) is not None)

        for f in self.funcs:
            if self.p.demangle(f).startswith('GCS::send_textv('):
                calls = [loc for dst, loc in f.edges
                         if any(self.p.demangle(t).startswith('GCS::service_statustext(')
                                for t in self.targets(f, dst, loc)[0])]
                if calls and all(guarded(loc) for loc in calls):
                    self.raw_succ[f.title] = [t for t in self.raw_succ[f.title]
                                              if not self.p.demangle(t).startswith('GCS::service_statustext(')]

    def poly_funcs(self, key):
        if key not in self.poly_cache:
            fs = []
            for ln in self.classes.targets(*key):
                fs.extend(self.p.by_symbol.get(ln, []))
            self.poly_cache[key] = fs
        return self.poly_cache[key]

    def wrappers_in(self, func_names):
        '''Functor method_wrappers whose address is used by these functions'''
        result = set()
        for fn in func_names:
            for c in self.p.consts.get(fn, ()):
                name = self.p.elf_addr.get(c & ~1)
                if name is not None and 'method_wrapper' in name:
                    result.add(name)
        return result

    def setup_callbacks(self):
        '''find the callbacks Functor calls in dispatcher functions can reach'''
        if not self.p.consts:
            return
        callbacks = {}
        data_dem = demangle_list(self.p.cxxfilt, [n for n, _, _ in self.p.data_syms])
        for dispatcher, table in TASK_TABLES:
            r = re.compile(table)
            targets = set()
            for name, addr, size in self.p.data_syms:
                if not r.search(data_dem.get(name, name)):
                    continue
                data = self.p.read_data(addr, size)
                if data is not None:
                    for i in range(0, len(data) - 3, 4):
                        n = self.p.elf_addr.get(int.from_bytes(data[i:i+4], 'little') & ~1)
                        if n is not None and 'method_wrapper' in n:
                            targets.add(n)
                else:
                    # filled in at runtime, look at the code that refers to it
                    users = [fn for fn, cs in self.p.consts.items() if any(addr <= c < addr + size for c in cs)]
                    targets |= self.wrappers_in(users)
            callbacks[dispatcher] = targets
        names = sorted(self.p.elf_frames.keys())
        dem = demangle_list(self.p.cxxfilt, names)
        for dispatcher, registrar in REGISTRATIONS:
            r = re.compile(registrar)
            regs = set(n for n in names if r.search(dem.get(n, n)))
            users = [fn for fn, calls in self.p.elf_calls.items() if calls & regs]
            callbacks.setdefault(dispatcher, set()).update(self.wrappers_in(users))
        self.dispatchers = [(re.compile(d), t) for d, t in callbacks.items()]

    def resolve_indirect(self, f, loc):
        if loc.startswith('elf:'):
            return [], 'elf-indirect'
        name = self.site_name(loc)
        if name is None:
            return [], 'no-name'
        if name == '_method':
            dem = self.p.demangle(f)
            for r, targets in self.dispatchers:
                if r.search(dem):
                    fs = [x for t in targets for x in self.p.by_symbol.get(base_symbol(t), [])]
                    return fs, 'callback'
            if self.functors == 'all':
                return [x for x in self.p.all_funcs() if 'method_wrapper' in x.title], 'functor-all'
            return [], 'functor'
        keys = self.p.poly_by_func.get((f.tu, base_symbol(f.title)), set())
        for scope in (keys, self.p.poly_by_tu.get(f.tu, set())):
            found = []
            for key in scope:
                found.extend(x for x in self.poly_funcs(key) if name in x.names)
            if found:
                return found, 'virtual'
        return [], 'unresolved'

    def targets(self, f, dst, loc):
        '''functions a call can reach, and a reason when there are none'''
        if dst == INDIRECT:
            fs, how = self.resolve_indirect(f, loc)
            self.site_info[(f.title, loc)] = (how, len(fs))
            return fs, how
        sym = base_symbol(dst)
        if f.origin == 'ci':
            # apply the linker's --wrap renaming to calls from before the
            # link. Calls in the ELF already have their final targets
            if sym.startswith('__real_'):
                sym = sym[len('__real_'):]
            elif '__wrap_' + sym in self.p.by_symbol:
                return self.p.by_symbol['__wrap_' + sym], 'direct'
        g = self.p.funcs.get(dst)
        if g is not None:
            return [g], 'direct'
        fs = self.p.by_symbol.get(sym, [])
        if not fs:
            # complete and base object constructors and destructors are aliases
            alt = re.sub(r'([CD])([12])E', lambda m: m.group(1) + ('2' if m.group(2) == '1' else '1') + 'E', sym)
            fs = self.p.by_symbol.get(alt, [])
        return fs, 'direct' if fs else 'missing %s' % dst

    def build(self):
        '''resolve every call edge once'''
        funcs = self.p.all_funcs()
        self.funcs = funcs
        cut = set(f.title for f in funcs if any(r.search(self.p.demangle(f)) for r in self.cut))
        self.raw_succ = {}
        self.unresolved = {}
        self.callback_succ = {}
        for f in funcs:
            out = {}
            unres = set()
            callbacks = set()
            for dst, loc in f.edges:
                ts, why = self.targets(f, dst, loc)
                if not ts:
                    unres.add((loc, why))
                if why == 'callback':
                    callbacks.update(t.title for t in ts)
                for t in ts:
                    if t.title not in cut:
                        out[t.title] = t
            self.raw_succ[f.title] = list(out.values())
            self.unresolved[f.title] = unres
            self.callback_succ[f.title] = callbacks
        self.add_receiver_contexts(cut)
        self.remove_initialised_statustext()
        # edges each suppression removes
        self.supp_edges = []
        dem = {f.title: self.p.demangle(f) for f in funcs}
        for scope, a, b, leaf, via, through in self.suppressions:
            callers = [f for f in funcs if a.search(dem[f.title])]
            edges = set()
            for f in callers:
                for t in self.raw_succ[f.title]:
                    if b.search(dem[t.title]):
                        edges.add((f.title, t.title))
            self.supp_edges.append(edges)
        self.variants = {}
        self.leaf_nodes = {}

    def leaf(self, f):
        '''a copy of f that keeps its frame but calls nothing'''
        if f.title not in self.leaf_nodes:
            g = Func(f.title + '#leaf', f.name, f.loc, f.size, f.dynamic, f.tu, f.origin)
            g.names = f.names
            g.context = f.context
            self.p.demangled[g.title] = self.p.demangle(f)
            self.leaf_nodes[f.title] = g
            self.unresolved[g.title] = set()
        return self.leaf_nodes[f.title]

    def applies(self, i, ids):
        '''does suppression i apply to a thread known by these names'''
        scope = self.suppressions[i][0]
        if scope is None:
            return True
        negate, names = scope
        match = bool(ids & names)
        return not match if negate else match

    def variant(self, name, key):
        '''call graph with the suppressions that apply to a thread, matched
        by its name or its category (e.g. UART1 or UART)'''
        ids = {name, key}
        active = frozenset(i for i in range(len(self.suppressions)) if self.applies(i, ids))
        if active not in self.variants:
            self.variants[active] = Variant(self, active)
        return self.variants[active]


class Variant:
    '''worst case depths for one set of active suppressions'''

    def __init__(self, a, active):
        self.a = a
        removed = set()
        leafed = set()
        for i in active:
            if a.suppressions[i][4] is not None:
                continue
            if a.suppressions[i][3]:
                leafed |= a.supp_edges[i]
            else:
                removed |= a.supp_edges[i]
            if a.supp_edges[i]:
                a.suppressed[i] = a.suppressed.get(i, 0) + len(a.supp_edges[i])
        self.succ = {}
        extra = []
        for title, ts in a.raw_succ.items():
            out = []
            for t in ts:
                if (title, t.title) in removed:
                    continue
                if (title, t.title) in leafed:
                    t = a.leaf(t)
                    if t.title not in self.succ:
                        self.succ[t.title] = []
                        extra.append(t)
                out.append(t)
            self.succ[title] = out
        funcs = a.funcs + extra
        funcs = self.contextual_suppressions(funcs, active)
        self.compute(funcs)

    def contextual_suppressions(self, funcs, active):
        '''carry ancestor conditions down calls, optionally only through a
        specified chain. Other callers retain their unsuppressed graph'''
        rules = [i for i in active if self.a.suppressions[i][4] is not None]
        if not rules:
            return funcs
        original = dict(self.succ)
        starts = {f.title: frozenset(i for i in rules if self.a.suppressions[i][4].search(self.a.p.demangle(f)))
                  for f in funcs}
        nodes = {(f.title, frozenset()): f for f in funcs}
        todo = [(f, f, frozenset()) for f in funcs]
        while todo:
            f, g, inherited = todo.pop()
            live = inherited | starts[f.title]
            out = []
            for t in original[f.title]:
                matched = [i for i in live if (f.title, t.title) in self.a.supp_edges[i]]
                for i in matched:
                    self.a.suppressed[i] = self.a.suppressed.get(i, 0) + 1
                if any(not self.a.suppressions[i][3] for i in matched):
                    continue
                if matched:
                    t = self.a.leaf(t)
                    self.succ[t.title] = []
                    nodes.setdefault((t.title, frozenset()), t)
                else:
                    carry = frozenset(i for i in live if self.a.suppressions[i][5] is None or
                                      self.a.suppressions[i][5].search(self.a.p.demangle(t)))
                    key = (t.title, carry)
                    if key not in nodes:
                        context = 'via=' + ','.join(str(i + 1) for i in sorted(carry))
                        h = self.a.contextual(t, '#' + context, ', '.join(s for s in (t.context, context) if s))
                        nodes[key] = h
                        todo.append((t, h, carry))
                    t = nodes[key]
                out.append(t)
            self.succ[g.title] = out
        return list(nodes.values())

    def compute(self, funcs):
        # Tarjan's algorithm, iterative. Components are produced callees first
        index = {}
        low = {}
        onstack = set()
        stack = []
        comps = []
        counter = 0
        for root in funcs:
            if root.title in index:
                continue
            work = [(root, iter(self.succ[root.title]))]
            index[root.title] = low[root.title] = counter
            counter += 1
            stack.append(root)
            onstack.add(root.title)
            while work:
                v, it = work[-1]
                advanced = False
                for w in it:
                    if w.title not in index:
                        index[w.title] = low[w.title] = counter
                        counter += 1
                        stack.append(w)
                        onstack.add(w.title)
                        work.append((w, iter(self.succ[w.title])))
                        advanced = True
                        break
                    elif w.title in onstack:
                        low[v.title] = min(low[v.title], index[w.title])
                if advanced:
                    continue
                work.pop()
                if work:
                    u = work[-1][0]
                    low[u.title] = min(low[u.title], low[v.title])
                if low[v.title] == index[v.title]:
                    members = []
                    while True:
                        w = stack.pop()
                        onstack.discard(w.title)
                        members.append(w)
                        if w is v:
                            break
                    comps.append(members)

        # longest path over the condensed graph, components are callees first
        self.depth = {}
        self.next = {}
        self.recursive = {}
        self.cycle_path = {}
        self.cycles = []
        for members in comps:
            titles = set(m.title for m in members)
            rec = len(members) > 1 or any(t.title == members[0].title for t in self.succ[members[0].title])
            best_exit = (0, None)
            for m in members:
                for t in self.succ[m.title]:
                    if t.title not in titles and self.depth[t.title] > best_exit[0]:
                        best_exit = (self.depth[t.title], t)
            if rec:
                cid = len(self.cycles)
                self.cycles.append(members)
                for m in members:
                    self.recursive[m.title] = cid
                if not self.simple_paths(members, titles):
                    # too big to enumerate paths. A path through the
                    # component can't use more than every function in it once
                    total = sum(m.size for m in members) + best_exit[0]
                    for m in members:
                        self.depth[m.title] = total
                        self.next[m.title] = best_exit[1]
            else:
                m = members[0]
                self.depth[m.title] = m.size + best_exit[0]
                self.next[m.title] = best_exit[1]

    def simple_paths(self, members, titles, max_size=16, max_steps=200000):
        '''exact depth from each function in a recursive component, as the
        deepest path that calls each function in it at most once and then
        leaves it. Deeper recursion is not bounded. Returns False if the
        component is too big to enumerate'''
        if len(members) > max_size:
            return False
        inside = {m.title: [t for t in self.succ[m.title] if t.title in titles] for m in members}
        exits = {}
        for m in members:
            best = (0, None)
            for t in self.succ[m.title]:
                if t.title not in titles and self.depth[t.title] > best[0]:
                    best = (self.depth[t.title], t)
            exits[m.title] = best
        results = {}
        steps = 0
        for start in members:
            best = (-1, None, None)
            work = [(start, start.size, [start], iter(inside[start.title]))]
            on_path = {start.title}
            d = start.size + exits[start.title][0]
            best = (d, [start], exits[start.title][1])
            while work:
                v, acc, path, it = work[-1]
                advanced = False
                for w in it:
                    steps += 1
                    if steps > max_steps:
                        return False
                    if w.title in on_path:
                        continue
                    acc2 = acc + w.size
                    path2 = path + [w]
                    d = acc2 + exits[w.title][0]
                    if d > best[0]:
                        best = (d, path2, exits[w.title][1])
                    on_path.add(w.title)
                    work.append((w, acc2, path2, iter(inside[w.title])))
                    advanced = True
                    break
                if not advanced:
                    work.pop()
                    on_path.discard(v.title)
            results[start.title] = best
        for m in members:
            d, path, exit_target = results[m.title]
            self.depth[m.title] = d
            self.cycle_path[m.title] = (path, exit_target)
            self.next[m.title] = exit_target
        return True

    def worst(self, f):
        '''return (depth, path) of the deepest call chain from f'''
        path = []
        x = f
        while x is not None and len(path) < 500:
            if x.title in self.cycle_path:
                members, exit_target = self.cycle_path[x.title]
                path.extend(members)
                path.append('<recursion in %u functions, each counted once>' %
                            len(self.cycles[self.recursive[x.title]]))
                x = exit_target
                continue
            path.append(x)
            if x.title in self.recursive:
                members = self.cycles[self.recursive[x.title]]
                path.append('<recursion in %u functions including %s, too large to enumerate paths: '
                            'all %u bytes of their frames are counted, %u not shown here>' %
                            (len(members), x.name, sum(m.size for m in members),
                             sum(m.size for m in members) - x.size))
            x = self.next.get(x.title)
        return self.depth[f.title], path

    def worst_skipping(self, f, skip):
        '''worst() of f, not following its calls to the titles in skip'''
        if not skip or f.title in self.recursive or f.title in self.cycle_path:
            return self.worst(f)
        best, path = 0, []
        for t in self.succ[f.title]:
            if t.title not in skip and self.depth[t.title] > best:
                best, path = self.worst(t)
        return f.size + best, [f] + path

    def reachable(self, roots, skip=None):
        '''unresolved calls, dynamic frames and recursion below roots, not
        following calls listed in skip (caller title -> callee titles)'''
        seen = set()
        todo = list(roots)
        unresolved = set()
        dynamic = []
        cycles = set()
        while todo:
            x = todo.pop()
            if x.title in seen:
                continue
            seen.add(x.title)
            if x.dynamic:
                dynamic.append(x)
            if x.title in self.recursive:
                cycles.add(self.recursive[x.title])
            unresolved |= self.a.unresolved[x.title]
            omit = skip.get(x.title, ()) if skip else ()
            todo.extend(t for t in self.succ[x.title] if t.title not in omit)
        return unresolved, dynamic, cycles


def parse_threads_txt(fname):
    threads = []
    with open(fname) as f:
        for line in f:
            m = re.match(r'^(\S+)\s+PRI=\s*\d+\s+sp=\S+\s+STACK=\s*(\d+)/\s*(\d+)', line)
            if m:
                threads.append((m.group(1), int(m.group(3)), int(m.group(3)) - int(m.group(2))))
    return threads


def load_suppressions(fname):
    '''lines of "[threads] caller regex -> callee regex [leaf]  # reason" for
    call edges known to be impossible. The optional thread list limits the
    threads it applies to, "[!main]" means every thread except main. With
    "leaf" the callee's own frame is kept but its calls are not followed.
    "via ancestor regex" limits a rule to calls below matching ancestors;
    "through regex" limits propagation to the listed intermediate methods'''
    result = []
    with open(fname) as f:
        for n, line in enumerate(f, 1):
            line = line.split('#', 1)[0].strip()
            if not line:
                continue
            scope = None
            leaf = False
            if line.endswith(' leaf'):
                leaf = True
                line = line[:-len(' leaf')].rstrip()
            via = None
            through = None
            if ' via ' in line:
                line, via = line.split(' via ', 1)
                if ' through ' in via:
                    via, through = via.split(' through ', 1)
                if not via.strip() or (through is not None and not through.strip()):
                    sys.exit('%s:%u: empty ancestor or intermediate regex' % (fname, n))
            m = re.match(r'^\[(!?)([^\]]+)\]\s*(.*)$', line)
            if m:
                scope = (m.group(1) == '!', set(k.strip() for k in m.group(2).split(',')))
                line = m.group(3)
            if '->' not in line:
                sys.exit('%s:%u: expected "caller -> callee"' % (fname, n))
            a, b = line.split('->', 1)
            result.append((scope, a.strip(), b.strip(), leaf, via, through))
    return result


def context_size(classes, builddir):
    '''stack used by a preempted thread's saved contexts, as in ChibiOS
    PORT_WA_CTX_SIZE. Interrupt handlers run on their own stack, so the
    PORT_INT_REQUIRED_STACK part of a working area is spare. None if the
    sizes aren't in the DWARF'''
    sizes = classes.struct_sizes
    if not all(n in sizes for n in THREAD_STRUCTS):
        return None
    simplified = False
    hwdef_h = os.path.join(builddir, 'hwdef.h')
    if os.path.exists(hwdef_h):
        with open(hwdef_h) as f:
            simplified = re.search(r'^#define CORTEX_SIMPLIFIED_PRIORITY\s+(TRUE|1)\b', f.read(), re.M) is not None
    return sizes['port_intctx'] + sizes['port_extctx'] * (1 if simplified else 2)


def static_allocations(prog, classes, ctx):
    '''usable stack sizes of threads with statically allocated stacks'''
    result = {}
    if ctx is None:
        return result
    # ChibiOS places the thread structure at the top of the working
    # area, aligned to the stack alignment
    overhead = ((classes.struct_sizes['ch_thread'] + 7) & ~7) + ctx
    for key, sym in THREAD_WORKING_AREAS.items():
        v = prog.sym_values.get(sym)
        if v is not None and v[1] > overhead:
            result[key] = v[1] - overhead
    base = prog.sym_values.get('__main_thread_stack_base__')
    end = prog.sym_values.get('__main_thread_stack_end__')
    if base is not None and end is not None and end[0] - base[0] > ctx:
        result['main'] = end[0] - base[0] - ctx
    return result


def thread_key(name):
    for pattern, key in THREAD_NAME_MAP:
        if re.match(pattern, name):
            return key
    return name


def main():
    parser = argparse.ArgumentParser(description='static stack usage analysis for ChibiOS firmware')
    parser.add_argument('builddir', help='board build directory, e.g. build/CubeOrange')
    parser.add_argument('--elf', help='firmware ELF built with -g, for virtual calls, callbacks and '
                        'functions without call graph info')
    parser.add_argument('--srcroot', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
    parser.add_argument('--threads', help='@SYS/threads.txt from the vehicle, for stack sizes and runtime use')
    parser.add_argument('--functors', choices=['none', 'all'], default='none',
                        help='resolve unknown Functor calls to all method wrappers (an upper bound)')
    parser.add_argument('--cut', action='append', default=None,
                        help='regex of functions whose calls are not followed (default: panic paths)')
    parser.add_argument('--path', action='append', default=[], help='show the deepest path for these threads')
    parser.add_argument('--unresolved', type=int, default=0, help='list this many unresolved calls per thread')
    parser.add_argument('--recursion', action='store_true', help='list recursive cycles reached by threads')
    default_suppressions = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'stack_analysis_suppressions.txt')
    parser.add_argument('--suppressions', default=default_suppressions,
                        help='file of call edges known to be impossible')
    parser.add_argument('--check', action='store_true',
                        help='exit with an error if a thread can exceed its stack')
    parser.add_argument('--margin', type=int, default=0, help='required free stack in bytes for --check')
    parser.add_argument('--cxxfilt', default='arm-none-eabi-c++filt')
    args = parser.parse_args()

    ci_files = glob.glob(os.path.join(args.builddir, '**', '*.ci'), recursive=True)
    if not ci_files:
        sys.exit('no .ci files found below %s, see --help for build flags' % args.builddir)
    dump_files = glob.glob(os.path.join(args.builddir, '**', '*i.cgraph'), recursive=True)

    prog = Program(args.cxxfilt)
    classes = ClassModel()
    if args.elf:
        classes.load(args.elf, args.elf + '.classes.json')
    if classes.cu_names:
        # only use call graph info for sources linked into this firmware
        def linked(fname):
            tu = tu_of(fname)
            return tu == 'LTO' or normalise_path(tu) in classes.cu_names
        skipped = [f for f in ci_files if not linked(f)]
        ci_files = [f for f in ci_files if linked(f)]
        dump_files = [f for f in dump_files if linked(f)]
        if skipped:
            print('ignoring %u .ci files for sources not linked into the firmware' % len(skipped))
    prog.load_ci(ci_files)
    prog.load_cgraph_dumps(dump_files)
    if args.elf:
        prog.load_elf(args.elf)
        removed = prog.remove_unlinked()
        if removed:
            print('ignoring %u functions not in the firmware' % removed)
        prog.add_elf_funcs()
    prog.demangle_all()

    ctx = context_size(classes, args.builddir) if args.elf else None
    if ctx is not None:
        # the context __port_switch saves is part of ctx
        for f in prog.all_funcs():
            if f.name == '__port_switch':
                f.size = 0
    suppressions = load_suppressions(args.suppressions) if os.path.exists(args.suppressions) else []
    a = Analyser(prog, classes, os.path.abspath(args.srcroot), os.path.abspath(args.builddir),
                 args.cut if args.cut is not None else DEFAULT_CUT, args.functors, suppressions)
    allocs = static_allocations(prog, classes, ctx)
    if ctx is not None:
        print('stack sizes exclude %u bytes for saved contexts' % ctx)

    entries = {}
    for key, pattern in THREAD_ENTRIES:
        r = re.compile(pattern)
        matches = [f for f in prog.all_funcs() if r.search(prog.demangle(f))]
        if matches:
            entries[key] = matches

    how = {}
    for h, n in a.site_info.values():
        how[h] = how.get(h, 0) + 1
    allf = prog.all_funcs()
    print('%u functions (%u from ELF only), %u .ci files, %u cgraph dumps' %
          (len(allf), sum(1 for f in allf if f.origin == 'elf'), len(ci_files), len(dump_files)))
    print('indirect call sites: %u (%s)' % (len(a.site_info), ' '.join('%s=%u' % kv for kv in sorted(how.items()))))
    missing = sum(1 for us in a.unresolved.values() for loc, why in us if why.startswith('missing'))
    print('direct calls to functions not found: %u' % missing)

    rows = []
    if args.threads:
        for name, total, used in parse_threads_txt(args.threads):
            key = thread_key(name)
            if ctx is not None and key != 'ISR':
                total -= ctx
            rows.append((name, key, total, used))
    else:
        rows = [(k, k, None, None) for k in entries]
    for key in allocs:
        if not any(k == key for _, k, _, _ in rows):
            rows.append((key, key, None, None))

    prefix = {}
    for key, patterns in THREAD_PREFIX.items():
        for pattern in patterns:
            r = re.compile(pattern)
            matches = [f for f in prog.all_funcs() if r.search(prog.demangle(f))]
            if matches:
                prefix.setdefault(key, []).append(max(matches, key=lambda f: f.size))

    print('%-14s %6s %7s %7s %7s %8s %s' % ('thread', 'alloc', 'runtime', 'static', 'margin',
                                            'unresolv', 'entry'))
    failures = []
    incomplete = []
    checked = 0
    for name, key, total, used in rows:
        if key == 'ISR':
            # interrupts run on their own stack, which isn't analysed
            print('%-14s %6s %7s %7s %7s %8s %s' % (name, total, used, '', '', '', 'interrupt stack, not analysed'))
            continue
        if key in allocs:
            total = allocs[key]
        roots = entries.get(key)
        if roots is None:
            print('%-14s %6s %7s %7s %7s %8s %s' % (name, total or '', used if used is not None else '', '?', '', '',
                                                    'no entry point known'))
            if total is not None:
                incomplete.append('%s: no entry point found' % name)
            continue
        v = a.variant(name, key)
        f = max(roots, key=lambda r: v.depth[r.title])
        depth, path = v.worst(f)
        pkey = 'functor' if 'method_wrapper' in prog.demangle(f) else key
        levels = prefix.get(pkey, [])
        depth += sum(lv.size for lv in levels)
        path = levels + path
        # the trampoline calls this thread's entry, not other threads' callbacks
        skip = {lv.title: a.callback_succ[lv.title] for lv in levels} if pkey == 'functor' else {}
        # callers of the entry points make other calls of their own
        above = 0
        for i, lv in enumerate(levels):
            d, p = v.worst_skipping(lv, skip.get(lv.title))
            if above + d > depth:
                depth = above + d
                path = levels[:i] + p
            above += lv.size
        unresolved, dynamic, cycles = v.reachable(roots + levels, skip)
        margin = '' if total is None else str(total - depth)
        if total is not None:
            checked += 1
            if depth + args.margin > total:
                failures.append((name, total, depth, path))
        flags = []
        if dynamic:
            flags.append('dynamic frames')
        if cycles:
            flags.append('recursion')
        flagstr = ' (%s)' % ', '.join(flags) if flags else ''
        print('%-14s %6s %7s %7u %7s %8u %s%s' % (name, total or '', used if used is not None else '',
                                                  depth, margin, len(unresolved), prog.demangle(f)[:48], flagstr))
        if name in args.path or key in args.path:
            for p in path:
                if isinstance(p, str):
                    print('        %s' % p)
                else:
                    print('    %6u %s  %s' % (p.size, prog.describe(p)[:140], p.loc))
        if args.unresolved:
            for loc, why in sorted(unresolved)[:args.unresolved]:
                print('        %s: %s' % (why, loc))
        if args.recursion:
            for c in sorted(cycles):
                members = v.cycles[c]
                print('        recursion: %u functions, %u bytes: %s' %
                      (len(members), sum(m.size for m in members),
                       ', '.join(prog.demangle(m)[:40] for m in members[:6])))

    for i, (scope, caller, callee, leaf, via, through) in enumerate(suppressions):
        if i not in a.suppressed:
            print('unused suppression: %s -> %s' % (caller, callee))

    if args.check:
        if not args.elf:
            incomplete.append('--check needs --elf for stack sizes')
        elif not allocs:
            incomplete.append('no thread stack sizes found in the ELF')
        for key, sym in THREAD_WORKING_AREAS.items():
            if sym in prog.sym_values and key not in allocs:
                incomplete.append('%s: stack size unknown, %s missing from DWARF' % (key, '/'.join(THREAD_STRUCTS)))
        # unresolved calls and dynamic frames are reported, but only missing
        # stack sizes or entry points make the check incomplete
        print('checked %u threads with known stack sizes, %u can overflow' % (checked, len(failures)))
        for msg in incomplete:
            print('INCOMPLETE: %s' % msg)
        for name, total, depth, path in failures:
            print('STACK OVERFLOW: %s needs up to %u bytes of its %u byte stack, deepest path:' % (name, depth, total))
            for p in path:
                if isinstance(p, str):
                    print('           %s' % p)
                else:
                    print('    %6u %s' % (p.size, prog.describe(p)[:140]))
        if failures or incomplete:
            sys.exit(1)


if __name__ == '__main__':
    main()
