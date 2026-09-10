#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
Statistical PC profiler for the RP2350 ArduPilot port.

Samples the Cortex-M33 DWT Program Counter Sample Register (PCSR, 0xE000101C)
over an existing OpenOCD TCL RPC connection while the target runs at full
speed. Reading PCSR does not halt the core, so sampling is non-intrusive and
representative of real timing - unlike halt/read/resume profilers.

Each sample's PC is attributed to a function using the ELF symbol table, then
functions are ranked by sample count (a proxy for exclusive CPU time). Samples
are bucketed by memory region so XIP-flash-resident hot code (the code that
thrashes the 16 KB XIP cache) is separated from code already relocated to SRAM.

The main output is a ranked table plus a set of ready-to-paste lines for
libraries/AP_HAL_ChibiOS/hwdef/common/rp2350_ramfunc2_registry.txt covering the
hottest XIP-resident functions not already relocated.

Prerequisites:
  - OpenOCD already running and attached to the target, exposing its TCL RPC
    port (the project's invocation uses tcl_port 50001).
  - The matching unstripped ELF (build/<board>/bin/arducopter), ideally a
    --debug build so source file/line attribution works.
  - arm-none-eabi-nm and arm-none-eabi-c++filt on PATH (or pass --nm/--cppfilt).

Example:
  Tools/debug/rp2350_pc_profiler.py --elf build/Laurel/bin/arducopter \\
      --tcl-port 50001 --samples 30000

Sampling core1 (where the EKF/PID/attitude dispatch runs) needs OpenOCD to
select that core first; pass --target-select rp2350.cm1 (the Raspberry Pi
OpenOCD rp2350.cfg names the cores rp2350.cm0/rp2350.cm1; run 'targets' in the
OpenOCD telnet console to confirm the names for your build).
"""

import argparse
import os
import re
import socket
import subprocess
import sys
import time
from bisect import bisect_right

# Cortex-M33 debug registers.
DEMCR = 0xE000EDFC       # Debug Exception and Monitor Control Register
DEMCR_TRCENA = 1 << 24   # enables DWT/ITM, required before PCSR is valid
DWT_PCSR = 0xE000101C    # Program Counter Sample Register
PCSR_INVALID = 0xFFFFFFFF  # returned when no valid sample is available

# RP2350 address map (only the ranges we categorise).
XIP_BASE = 0x10000000
XIP_END = 0x20000000
SRAM_BASE = 0x20000000
SRAM_END = 0x20090000
ROM_BASE = 0x00000000
ROM_END = 0x00008000

TCL_TERM = b'\x1a'  # OpenOCD TCL RPC command/response terminator (Ctrl-Z)


class OpenOCDError(Exception):
    pass


class OpenOCDTcl:
    """Minimal OpenOCD TCL RPC client (command terminated by 0x1a)."""

    def __init__(self, host, port, timeout=5.0):
        try:
            self.sock = socket.create_connection((host, port), timeout=timeout)
        except OSError as e:
            raise OpenOCDError(
                "could not connect to OpenOCD TCL RPC at %s:%d (%s). Is OpenOCD "
                "running with a 'tcl_port'?" % (host, port, e))
        self.sock.settimeout(timeout)
        self.buf = b''

    def cmd(self, command):
        self.sock.sendall(command.encode() + TCL_TERM)
        while TCL_TERM not in self.buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise OpenOCDError("OpenOCD closed the connection")
            self.buf += chunk
        resp, _, self.buf = self.buf.partition(TCL_TERM)
        return resp.decode(errors='replace').strip()

    def read_word(self, addr):
        # 'mdw' prints e.g. "0xe000101c: deadbeef"; grab the trailing hex word.
        resp = self.cmd("mdw 0x%08x" % addr)
        m = re.search(r':\s*([0-9a-fA-F]{8})', resp)
        if not m:
            raise OpenOCDError("unexpected mdw response: %r" % resp)
        return int(m.group(1), 16)

    def write_word(self, addr, value):
        self.cmd("mww 0x%08x 0x%08x" % (addr, value))

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


class SymbolTable:
    """Function address ranges parsed from arm-none-eabi-nm."""

    def __init__(self, elf, nm='arm-none-eabi-nm', cppfilt='arm-none-eabi-c++filt',
                 repo_root=None):
        self.repo_root = repo_root
        self.starts = []     # sorted function start addresses
        self.entries = []    # parallel: (start, end, mangled, file)
        self._parse(elf, nm)
        self._demangle(cppfilt)

    def _parse(self, elf, nm):
        try:
            out = subprocess.check_output(
                [nm, '--print-size', '--numeric-sort', '--line-numbers', elf],
                text=True, stderr=subprocess.DEVNULL)
        except (OSError, subprocess.CalledProcessError) as e:
            raise OpenOCDError("failed to run nm on %s: %s" % (elf, e))

        raw = []  # (addr, size_or_None, name, file)
        for line in out.splitlines():
            # "addr [size] type name[\tfile:line]"
            parts = line.split('\t', 1)
            fileinfo = parts[1].strip() if len(parts) > 1 else ''
            cols = parts[0].split()
            if len(cols) == 4:
                addr_s, size_s, typ, name = cols
                size = int(size_s, 16)
            elif len(cols) == 3:
                addr_s, typ, name = cols
                size = None
            else:
                continue
            if typ not in 'tTwW':  # functions only
                continue
            raw.append((int(addr_s, 16), size, name, self._relpath(fileinfo)))

        raw.sort(key=lambda r: r[0])
        for i, (addr, size, name, fil) in enumerate(raw):
            if size:
                end = addr + size
            elif i + 1 < len(raw):
                end = raw[i + 1][0]   # fall back to gap until next symbol
            else:
                end = addr + 4
            self.starts.append(addr)
            self.entries.append((addr, end, name, fil))

    def _relpath(self, fileinfo):
        if not fileinfo or ':' not in fileinfo:
            return ''
        path = fileinfo.rsplit(':', 1)[0]
        if self.repo_root and os.path.isabs(path):
            try:
                return os.path.relpath(path, self.repo_root)
            except ValueError:
                return path
        return path

    def _demangle(self, cppfilt):
        names = [e[2] for e in self.entries]
        try:
            proc = subprocess.run([cppfilt], input='\n'.join(names),
                                  text=True, capture_output=True, check=True)
            self.demangled = proc.stdout.splitlines()
        except (OSError, subprocess.CalledProcessError):
            self.demangled = names  # degrade gracefully to mangled names
        if len(self.demangled) != len(self.entries):
            self.demangled = names

    def lookup(self, pc):
        """Return index of the function containing pc, or None."""
        i = bisect_right(self.starts, pc) - 1
        if i < 0:
            return None
        start, end, _, _ = self.entries[i]
        if start <= pc < end:
            return i
        return None


def region_of(pc):
    if pc == PCSR_INVALID:
        return 'invalid'
    if XIP_BASE <= pc < XIP_END:
        return 'xip'
    if SRAM_BASE <= pc < SRAM_END:
        return 'sram'
    if ROM_BASE <= pc < ROM_END:
        return 'rom'
    return 'other'


def strip_signature(demangled):
    """'Foo::bar(int, float)' -> 'Foo::bar' to match the registry format."""
    return re.sub(r'\(.*', '', demangled).strip()


def load_registry(path):
    """Return the set of demangled symbols already in the RAMFUNC2 registry."""
    symbols = set()
    if not os.path.exists(path):
        return symbols
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or '|' not in line:
                continue
            symbols.add(line.split('|', 1)[1].strip())
    return symbols


def sample(ocd, n_samples, duration, progress=True):
    """Collect PC samples; stop at n_samples or after duration seconds."""
    samples = []
    t0 = time.time()
    last = t0
    while True:
        samples.append(ocd.read_word(DWT_PCSR))
        now = time.time()
        if n_samples and len(samples) >= n_samples:
            break
        if duration and (now - t0) >= duration:
            break
        if progress and (now - last) >= 1.0:
            last = now
            sys.stderr.write("\r  sampled %d (%.0f/s)..." %
                             (len(samples), len(samples) / (now - t0)))
            sys.stderr.flush()
    if progress:
        sys.stderr.write("\r" + " " * 50 + "\r")
    return samples, time.time() - t0


def tally(samples, syms):
    """Attribute samples to regions and functions, keeping per-function PC histograms."""
    total = len(samples)
    region_counts = {}
    func_counts = {}      # idx -> total count
    func_pc_hist = {}     # idx -> {pc: count}
    unknown = 0
    for pc in samples:
        r = region_of(pc)
        region_counts[r] = region_counts.get(r, 0) + 1
        if pc == PCSR_INVALID:
            continue
        idx = syms.lookup(pc)
        if idx is None:
            unknown += 1
            continue
        func_counts[idx] = func_counts.get(idx, 0) + 1
        h = func_pc_hist.setdefault(idx, {})
        h[pc] = h.get(pc, 0) + 1
    valid = total - region_counts.get('invalid', 0)
    return region_counts, func_counts, func_pc_hist, unknown, valid, total


def build_report(t, elapsed, syms, registry, top, threshold):
    region_counts, func_counts, func_pc_hist, unknown, valid, total = t

    lines = []
    lines.append("# RP2350 PC profile")
    lines.append("")
    lines.append("samples: %d over %.1fs (%.0f samples/s)" %
                 (total, elapsed, total / elapsed if elapsed else 0))
    lines.append("")
    lines.append("## Region breakdown")
    for region in ('xip', 'sram', 'rom', 'other', 'invalid'):
        c = region_counts.get(region, 0)
        lines.append("  %-8s %8d  %5.1f%%" % (region, c, 100.0 * c / total if total else 0))
    lines.append("  %-8s %8d  (samples landing outside any known function)" %
                 ('no-sym', unknown))
    lines.append("")
    lines.append("Note: 'xip' = code running from flash via the 16 KB XIP cache "
                 "(relocation target);")
    lines.append("      'sram' = code already resident in SRAM (e.g. RAMFUNC2).")
    lines.append("")

    ranked = sorted(func_counts.items(), key=lambda kv: kv[1], reverse=True)

    def fmt_rows(items):
        rows = []
        for idx, c in items:
            start, _, _, fil = syms.entries[idx]
            name = strip_signature(syms.demangled[idx])
            pct = 100.0 * c / valid if valid else 0
            rows.append("  %5.1f%% %6d  %-5s 0x%08x  %s" %
                        (pct, c, region_of(start), start, name))
        return rows

    lines.append("## Top %d functions (by exclusive samples, valid only)" % top)
    lines.extend(fmt_rows(ranked[:top]))
    lines.append("")

    # Registry suggestions: hottest XIP-resident functions not already relocated.
    suggestions = []
    for idx, c in ranked:
        start, _, _, fil = syms.entries[idx]
        if region_of(start) != 'xip':
            continue
        pct = 100.0 * c / valid if valid else 0
        if pct < threshold:
            break
        name = strip_signature(syms.demangled[idx])
        if name in registry:
            continue
        suggestions.append((pct, fil, name))

    lines.append("## RAMFUNC2 registry suggestions (XIP-resident, >= %.1f%%, not yet relocated)" % threshold)
    if not suggestions:
        lines.append("  (none above threshold - hot path may already be in SRAM)")
    else:
        lines.append("# paste into rp2350_ramfunc2_registry.txt (path|symbol):")
        for pct, fil, name in suggestions:
            if not fil:
                lines.append("# %.1f%% (no source path from nm) %s" % (pct, name))
            else:
                lines.append("%s|%s    # %.1f%%" % (fil, name, pct))
    lines.append("")
    return "\n".join(lines)


# Region bases for the on-chip PROFc1 dump tags (see rp2350_pc_sampler.cpp).
# The F base is the app's XIP text start, which moves with FLASH_RESERVE_START_KB,
# so it is read back from the ELF rather than assumed.
PROFC1_TAG_BASE = {'F': 0x10010000, 'S': 0x20000000, 'C': 0x20080000, 'o': 0}


def elf_flash_base(elf, nm='arm-none-eabi-nm', default=0x10010000):
    """Address of __vectors_base__, the XIP text start the F tag is relative to."""
    try:
        out = subprocess.check_output([nm, elf], text=True, stderr=subprocess.DEVNULL)
    except (OSError, subprocess.CalledProcessError):
        return default
    for line in out.splitlines():
        cols = line.split()
        if len(cols) == 3 and cols[2] == '__vectors_base__':
            return int(cols[0], 16)
    return default


PROFC1_TOKEN = re.compile(r'([FSCo])([0-9a-fA-F]+):(\d+)')
PROFC1_N = re.compile(r'\bn=(\d+)')
PROFC1_CORE = re.compile(r'PROF\s*c(\d)')


def parse_profc1_text(text, flash_base=None):
    """Parse pasted 'PROFc1' MAVLink lines into ({address: count}, total_n).

    Only lines mentioning PROF are considered, so diagnostic lines like 'C1:...'
    cannot be misread as tokens. The firmware hash accumulates since boot, so
    across several pasted cycles we keep the max count per address and max n."""
    bases = dict(PROFC1_TAG_BASE)
    if flash_base is not None:
        bases['F'] = flash_base
    hist = {}
    total = 0
    for line in text.splitlines():
        if 'PROF' not in line:
            continue
        m = PROFC1_N.search(line)
        if m:
            total = max(total, int(m.group(1)))
        for tag, off, cnt in PROFC1_TOKEN.findall(line):
            addr = bases[tag] + int(off, 16)
            cnt = int(cnt)
            if cnt > hist.get(addr, 0):
                hist[addr] = cnt
    return hist, total


def build_histogram_report(hist, total_n, syms, registry, top, core=1):
    """Aggregate the emitted top-PC histogram by function and rank it."""
    emitted = sum(hist.values()) or 1
    func_count = {}
    unknown = 0
    for addr, c in hist.items():
        idx = syms.lookup(addr)
        if idx is None:
            unknown += c
            continue
        func_count[idx] = func_count.get(idx, 0) + c
    ranked = sorted(func_count.items(), key=lambda kv: kv[1], reverse=True)
    denom = total_n if total_n else emitted

    L = []
    L.append("# RP2350 PROFc%d histogram attribution (core%d)" % (core, core))
    L.append("")
    L.append("n (all samples): %d   emitted top-PC sum: %d   distinct PCs: %d   no-symbol: %d"
             % (total_n, emitted, len(hist), unknown))
    L.append("")
    L.append("## Hot functions (from the emitted top PCs, aggregated)")
    L.append("   %tot  %emit     cnt  region  address      size  function")
    for idx, c in ranked[:top]:
        start, end, _, _ = syms.entries[idx]
        name = strip_signature(syms.demangled[idx])
        note = "  [RAMFUNC2]" if name in registry else ""
        L.append("  %5.1f%% %5.1f%% %7d  %-5s 0x%08x %5d  %s%s"
                 % (100.0 * c / denom, 100.0 * c / emitted, c,
                    region_of(start), start, end - start, name, note))
    L.append("")
    L.append("  %%tot = share of all core%d samples (n); %%emit = share of emitted top PCs." % core)
    L.append("  [RAMFUNC2] = already resident in SRAM.")
    L.append("")

    L.append("## RAMFUNC2 candidates (xip-resident, not yet relocated)")
    sugg = []
    for idx, c in ranked:
        start, end, _, fil = syms.entries[idx]
        if region_of(start) != 'xip':
            continue
        name = strip_signature(syms.demangled[idx])
        if name in registry:
            continue
        sugg.append((c, fil, name, end - start))
    if not sugg:
        L.append("  (none - the emitted hot PCs are already in SRAM)")
    else:
        L.append("# paste into rp2350_ramfunc2_registry.txt (path|symbol):")
        for c, fil, name, size in sugg:
            pct = 100.0 * c / denom
            if fil:
                L.append("%s|%s    # %.1f%% of n, %d B" % (fil, name, pct, size))
            else:
                L.append("# %.1f%% of n, %d B (no source path) %s" % (pct, size, name))
    L.append("")
    return "\n".join(L)


def addr2line_map(addr2line_bin, elf, addrs, repo_root):
    """Map instruction addresses to 'relpath:line' via addr2line (stdin-fed)."""
    uniq = sorted(set(addrs))
    if not uniq:
        return {}
    try:
        proc = subprocess.run([addr2line_bin, '-e', elf],
                              input='\n'.join('0x%x' % a for a in uniq),
                              text=True, capture_output=True, check=True)
    except (OSError, subprocess.CalledProcessError):
        return {}
    result = {}
    for a, raw in zip(uniq, proc.stdout.splitlines()):
        loc = raw.strip().split(' (')[0]  # drop ' (discriminator N)'
        if ':' in loc:
            path, _, lineno = loc.rpartition(':')
        else:
            path, lineno = loc, '0'
        if repo_root and os.path.isabs(path):
            try:
                path = os.path.relpath(path, repo_root)
            except ValueError:
                pass
        result[a] = '%s:%s' % (path, lineno)
    return result


def hot_kernel_section(t, syms, addr2line_bin, elf, repo_root,
                       n_funcs, kernel_pct, top_lines=5):
    """Per-function intra-function hotness: how concentrated are the samples?

    Reveals functions whose samples cluster in a small kernel inside a much
    larger body - relocating those whole to SRAM wastes space, so they are
    better split (relocate hot blocks only / -freorder-blocks-and-partition).
    """
    _, func_counts, func_pc_hist, _, valid, _ = t
    ranked = sorted(func_counts.items(), key=lambda kv: kv[1], reverse=True)
    selected = [(idx, c) for idx, c in ranked
                if region_of(syms.entries[idx][0]) in ('xip', 'sram')][:n_funcs]

    want = []
    for idx, _ in selected:
        want.extend(func_pc_hist.get(idx, {}).keys())
    loc = addr2line_map(addr2line_bin, elf, want, repo_root)

    lines = []
    lines.append("## Hot-kernel analysis (top %d XIP/SRAM functions, %.0f%% coverage)"
                 % (len(selected), kernel_pct))
    lines.append("# kernel = distinct 8-byte cache lines covering %.0f%% of a function's "
                 "samples." % kernel_pct)
    lines.append("# small kernel in a large function => whole-function SRAM relocation "
                 "wastes space (split candidate).")
    lines.append("")
    for idx, c in selected:
        start, end, _, _ = syms.entries[idx]
        size = end - start
        name = strip_signature(syms.demangled[idx])
        region = region_of(start)
        pct = 100.0 * c / valid if valid else 0
        hist = func_pc_hist.get(idx, {})

        line_hist = {}
        for pc, n in hist.items():
            ln = pc & ~7  # 8-byte cache line
            line_hist[ln] = line_hist.get(ln, 0) + n
        need = c * kernel_pct / 100.0
        acc = 0
        kernel_lines = 0
        for v in sorted(line_hist.values(), reverse=True):
            acc += v
            kernel_lines += 1
            if acc >= need:
                break
        kernel_bytes = kernel_lines * 8
        conc = (kernel_bytes / size) if size else 1.0
        split = size >= 256 and conc <= 0.5
        verdict = "SPLIT CANDIDATE" if split else "relocate whole"

        lines.append("  %-44s %5.1f%% %6d samp  size %5dB  [%s]" %
                     (name[:44], pct, c, size, region))
        lines.append("     kernel %dB (%.0f%% of function) covers %.0f%% of samples -> %s" %
                     (kernel_bytes, 100.0 * conc, kernel_pct, verdict))
        src_counts = {}
        for pc, n in hist.items():
            s = loc.get(pc, '?')
            src_counts[s] = src_counts.get(s, 0) + n
        for s, n in sorted(src_counts.items(), key=lambda kv: kv[1], reverse=True)[:top_lines]:
            lines.append("        %6d  %s" % (n, s))
        lines.append("")
    return "\n".join(lines)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_root = os.path.abspath(os.path.join(here, '..', '..'))
    default_registry = os.path.join(
        default_root, 'libraries/AP_HAL_ChibiOS/hwdef/common/rp2350_ramfunc2_registry.txt')

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--elf', default=os.path.join(default_root, 'build/Laurel/bin/arducopter'),
                    help='ELF with symbols (default: build/Laurel/bin/arducopter)')
    ap.add_argument('--host', default='localhost', help='OpenOCD host')
    ap.add_argument('--tcl-port', type=int, default=50001,
                    help='OpenOCD TCL RPC port (project default 50001)')
    ap.add_argument('--samples', type=int, default=20000,
                    help='number of PC samples to collect (0 = use --duration)')
    ap.add_argument('--duration', type=float, default=0.0,
                    help='sample for this many seconds instead of a fixed count')
    ap.add_argument('--top', type=int, default=30, help='functions to list')
    ap.add_argument('--threshold', type=float, default=1.0,
                    help='minimum %% of valid samples for a registry suggestion')
    ap.add_argument('--by-line', type=int, default=0, metavar='N',
                    help='also show intra-function hot-kernel analysis for the top N '
                         'XIP/SRAM functions (0 = off)')
    ap.add_argument('--kernel-pct', type=float, default=90.0,
                    help='%% of a function''s samples that define its hot kernel')
    ap.add_argument('--addr2line', default='arm-none-eabi-addr2line')
    ap.add_argument('--target-select', default=None,
                    help="OpenOCD target to select first, e.g. rp2350.cm1 for core1")
    ap.add_argument('--registry', default=default_registry,
                    help='RAMFUNC2 registry to diff suggestions against')
    ap.add_argument('--repo-root', default=default_root,
                    help='repo root for making source paths relative')
    ap.add_argument('--nm', default='arm-none-eabi-nm')
    ap.add_argument('--cppfilt', default='arm-none-eabi-c++filt')
    ap.add_argument('--out', default=None, help='also write the report to this file')
    ap.add_argument('--raw-out', default=None, help='write raw sampled PCs (hex, one per line)')
    ap.add_argument('--histogram', default=None, metavar='FILE',
                    help="attribute a pasted on-chip PROFc1 dump ('-' for stdin) "
                         "instead of SWD sampling")
    args = ap.parse_args()

    if not os.path.exists(args.elf):
        sys.exit("ELF not found: %s (build the board first, or pass --elf)" % args.elf)

    sys.stderr.write("loading symbols from %s ...\n" % args.elf)
    syms = SymbolTable(args.elf, nm=args.nm, cppfilt=args.cppfilt, repo_root=args.repo_root)
    registry = load_registry(args.registry)
    sys.stderr.write("  %d functions, %d already in registry\n" %
                     (len(syms.entries), len(registry)))

    if args.histogram is not None:
        text = sys.stdin.read() if args.histogram == '-' else open(args.histogram).read()
        hist, total_n = parse_profc1_text(text, elf_flash_base(args.elf, args.nm))
        if not hist:
            sys.exit("no PROFc1 tokens found in input")
        m = PROFC1_CORE.search(text)
        core = int(m.group(1)) if m else 1
        report = build_histogram_report(hist, total_n, syms, registry, args.top, core)
        print(report)
        if args.out:
            with open(args.out, 'w') as f:
                f.write(report + "\n")
            sys.stderr.write("wrote %s\n" % args.out)
        return

    ocd = OpenOCDTcl(args.host, args.tcl_port)
    try:
        if args.target_select:
            ocd.cmd("targets %s" % args.target_select)
        # Ensure DWT is enabled so PCSR returns valid samples.
        demcr = ocd.read_word(DEMCR)
        if not (demcr & DEMCR_TRCENA):
            ocd.write_word(DEMCR, demcr | DEMCR_TRCENA)
            sys.stderr.write("enabled DEMCR.TRCENA for PCSR sampling\n")

        n = args.samples if not args.duration else 0
        sys.stderr.write("sampling DWT_PCSR (target must be running)...\n")
        samples, elapsed = sample(ocd, n, args.duration)
    finally:
        ocd.close()

    invalid = sum(1 for pc in samples if pc == PCSR_INVALID)
    if samples and invalid == len(samples):
        sys.stderr.write("WARNING: every sample was invalid (0xFFFFFFFF). The "
                         "target is probably halted, or PCSR is unavailable on "
                         "the selected core. Resume the target and retry.\n")

    if args.raw_out:
        with open(args.raw_out, 'w') as f:
            f.write('\n'.join("0x%08x" % pc for pc in samples))

    t = tally(samples, syms)
    report = build_report(t, elapsed, syms, registry, args.top, args.threshold)
    if args.by_line:
        report += "\n" + hot_kernel_section(t, syms, args.addr2line, args.elf,
                                            args.repo_root, args.by_line, args.kernel_pct)
    print(report)
    if args.out:
        with open(args.out, 'w') as f:
            f.write(report + "\n")
        sys.stderr.write("wrote %s\n" % args.out)


if __name__ == '__main__':
    try:
        main()
    except OpenOCDError as e:
        sys.exit("error: %s" % e)
    except KeyboardInterrupt:
        sys.exit("\ninterrupted")
