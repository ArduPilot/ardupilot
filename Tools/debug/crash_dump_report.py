#!/usr/bin/env python3

"""
Non-interactive crash dump analysis tool for ArduPilot.

Reads a crash_dump.bin file using GDB and CrashDebug, emitting a
human-readable report to stdout.

Usage:
    python3 crash_dump_report.py ELF_FILE CRASH_DUMP_FILE

Requires:
    - arm-none-eabi-gdb
    - CrashDebug binary (from modules/CrashDebug/bins/)

Copyright ArduPilot Project 2026
Released under GNU GPL version 3 or later
"""

import argparse
import os
import struct
import subprocess
import sys
import tempfile

# crash dump signature bytes
CRASHDUMP_SIG = b'cC'

# register names in dump order (version 3.0)
INT_REG_NAMES = [
    'R0', 'R1', 'R2', 'R3',
    'R4', 'R5', 'R6', 'R7',
    'R8', 'R9', 'R10', 'R11',
    'R12', 'SP', 'LR', 'PC',
    'xPSR', 'MSP', 'PSP', 'exceptionPSR',
]

FP_REG_NAMES = [f'S{i}' for i in range(32)] + ['FPSCR']

# Cortex-M fault status register addresses
FAULT_REGS = {
    'CFSR':   0xE000ED28,  # Configurable Fault Status Register
    'HFSR':   0xE000ED2C,  # HardFault Status Register
    'DFSR':   0xE000ED30,  # Debug Fault Status Register
    'MMFAR':  0xE000ED34,  # MemManage Fault Address Register
    'BFAR':   0xE000ED38,  # BusFault Address Register
    'AFSR':   0xE000ED3C,  # Auxiliary Fault Status Register
    'ICSR':   0xE000ED04,  # Interrupt Control and State Register
}

# CFSR bit definitions
CFSR_BITS = {
    # MemManage (bits 0-7)
    0:  'IACCVIOL  - Instruction access violation',
    1:  'DACCVIOL  - Data access violation',
    3:  'MUNSTKERR - MemManage fault on unstacking for return from exception',
    4:  'MSTKERR   - MemManage fault on stacking for exception entry',
    5:  'MLSPERR   - MemManage fault during FP lazy state preservation',
    7:  'MMARVALID - MMFAR holds a valid address',
    # BusFault (bits 8-15)
    8:  'IBUSERR   - Instruction bus error',
    9:  'PRECISERR - Precise data bus error',
    10: 'IMPRECISERR - Imprecise data bus error',
    11: 'UNSTKERR  - BusFault on unstacking for return from exception',
    12: 'STKERR    - BusFault on stacking for exception entry',
    13: 'LSPERR    - BusFault during FP lazy state preservation',
    15: 'BFARVALID - BFAR holds a valid address',
    # UsageFault (bits 16-31)
    16: 'UNDEFINSTR - Undefined instruction',
    17: 'INVSTATE  - Invalid state (e.g. executing ARM instruction in Thumb mode)',
    18: 'INVPC     - Invalid PC load',
    19: 'NOCP      - No coprocessor',
    20: 'STKOF     - Stack overflow (ARMv8-M)',
    24: 'UNALIGNED - Unaligned access',
    25: 'DIVBYZERO - Divide by zero',
}

HFSR_BITS = {
    1:  'VECTTBL - BusFault on vector table read',
    30: 'FORCED  - Forced HardFault (escalated from configurable fault)',
    31: 'DEBUGEVT - Debug event triggered HardFault',
}

# ChibiOS thread state names (tstate_t values from chschd.h)
CHIBI_THREAD_STATES = {
    0:  'READY',
    1:  'CURRENT',
    2:  'WTSTART',
    3:  'SUSPENDED',
    4:  'QUEUED',
    5:  'WTSEM',
    6:  'WTMTX',
    7:  'WTCOND',
    8:  'SLEEPING',
    9:  'WTEXIT',
    10: 'WTOREVT',
    11: 'WTANDEVT',
    12: 'SNDMSGQ',
    13: 'SNDMSG',
    14: 'WTMSG',
    15: 'FINAL',
}

# Cortex-M EXC_RETURN patterns loaded into LR by hardware on exception entry
EXC_RETURN_PATTERNS = {
    0xFFFFFFF1: 'Return to Handler mode, MSP, no FPU',
    0xFFFFFFF9: 'Return to Thread mode, MSP, no FPU',
    0xFFFFFFFD: 'Return to Thread mode, PSP, no FPU',
    0xFFFFFFE1: 'Return to Handler mode, MSP, FPU active',
    0xFFFFFFE9: 'Return to Thread mode, MSP, FPU active',
    0xFFFFFFED: 'Return to Thread mode, PSP, FPU active',
}


def resolve_addr2line(elf_file, addr):
    """Resolve an address to (function_name, source_location) via addr2line.

    Strips the Thumb mode bit (bit 0) before lookup.
    Returns (None, None) on failure or unresolvable address.
    """
    addr_clean = addr & ~1
    if addr_clean > 0xF0000000:
        return None, None
    try:
        result = subprocess.run(
            ['arm-none-eabi-addr2line', '-e', elf_file, '-f', '-C', '-s',
             f'{addr_clean:#010x}'],
            capture_output=True, text=True, timeout=10,
        )
        lines = result.stdout.strip().splitlines()
        if len(lines) >= 2:
            func = lines[0].strip()
            loc = lines[1].strip()
            if func and func not in ('??', '??()'):
                resolved_loc = loc if loc and loc not in ('??:0', '??:?') else None
                return func, resolved_loc
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass
    return None, None


def find_thread_working_areas(elf_file):
    """Locate ChibiOS thread working-area symbols in the ELF.

    Returns a dict mapping symbol_name -> start_address (int), sorted by
    address.  Includes static _thread_wa symbols and the ChibiOS main thread
    (ch0.mainthread), whose stack lives between __main_thread_stack_base__ and
    __main_thread_stack_end__.
    """
    result = {}
    nm_syms = {}
    try:
        out = subprocess.run(
            ['arm-none-eabi-nm', '--defined-only', elf_file],
            capture_output=True, text=True, timeout=15,
        )
        for line in out.stdout.splitlines():
            parts = line.split()
            if len(parts) >= 3:
                try:
                    nm_syms[parts[2]] = int(parts[0], 16)
                except ValueError:
                    pass
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    for name, addr in nm_syms.items():
        if '_thread_wa' in name:
            result[name] = addr

    # The ArduPilot/ChibiOS main thread is not a _thread_wa static array;
    # its thread_t is embedded in ch0.mainthread and its stack occupies the
    # __main_thread_stack_base__ region.  Use None as the wa_addr sentinel so
    # the caller knows to use a GDB expression rather than a dump-region lookup.
    if 'ch0' in nm_syms and '__main_thread_stack_base__' in nm_syms:
        result['ch0.mainthread'] = nm_syms['__main_thread_stack_base__']

    return dict(sorted(result.items(), key=lambda kv: kv[1]))


def find_crashdebug():
    """Locate the CrashDebug binary."""
    script_dir = os.path.dirname(os.path.realpath(__file__))
    base = os.path.join(script_dir, '..', '..', 'modules', 'CrashDebug', 'bins')

    if sys.platform.startswith('linux'):
        path = os.path.join(base, 'lin64', 'CrashDebug')
    elif sys.platform == 'darwin':
        path = os.path.join(base, 'osx64', 'CrashDebug')
    elif sys.platform == 'win32':
        path = os.path.join(base, 'win32', 'CrashDebug.exe')
    else:
        return None

    path = os.path.normpath(path)
    if os.path.isfile(path):
        return path
    return None


def validate_crashdump(path):
    """Read and validate the crash dump header, return parsed info."""
    with open(path, 'rb') as f:
        data = f.read()

    if len(data) < 4:
        print(f"Error: crash dump too small ({len(data)} bytes)", file=sys.stderr)
        sys.exit(1)

    if data[0:2] != CRASHDUMP_SIG:
        print(f"Error: invalid crash dump signature (got 0x{data[0]:02x} 0x{data[1]:02x}, "
              f"expected 0x63 0x43)", file=sys.stderr)
        sys.exit(1)

    version_major = data[2]
    version_minor = data[3]

    if len(data) < 8:
        print("Error: crash dump truncated", file=sys.stderr)
        sys.exit(1)

    flags = struct.unpack_from('<I', data, 4)[0]
    has_fpu = bool(flags & 1)

    # parse integer registers
    int_reg_count = 20 if version_major >= 3 else 18
    reg_offset = 8
    needed = reg_offset + int_reg_count * 4
    if len(data) < needed:
        print(f"Error: crash dump truncated (need {needed} bytes for registers, "
              f"have {len(data)})", file=sys.stderr)
        sys.exit(1)

    int_regs = {}
    for i in range(int_reg_count):
        val = struct.unpack_from('<I', data, reg_offset + i * 4)[0]
        if i < len(INT_REG_NAMES):
            int_regs[INT_REG_NAMES[i]] = val

    offset = reg_offset + int_reg_count * 4

    # parse FP registers if present
    fp_regs = {}
    if has_fpu:
        fp_needed = offset + 33 * 4
        if len(data) >= fp_needed:
            for i in range(33):
                val = struct.unpack_from('<I', data, offset + i * 4)[0]
                fp_regs[FP_REG_NAMES[i]] = val
            offset += 33 * 4

    # parse memory regions
    regions = []
    while offset + 8 <= len(data):
        start_addr = struct.unpack_from('<I', data, offset)[0]
        end_addr = struct.unpack_from('<I', data, offset + 4)[0]
        if start_addr == 0xFFFFFFFF:
            break
        if start_addr == 0xACCE55ED:
            regions.append(('STACK_OVERFLOW_SENTINEL', 0, 0))
            break
        size = end_addr - start_addr
        offset += 8
        if offset + size > len(data):
            regions.append((start_addr, end_addr, min(size, len(data) - offset)))
            break
        regions.append((start_addr, end_addr, size))
        offset += size

    return {
        'version': (version_major, version_minor),
        'flags': flags,
        'has_fpu': has_fpu,
        'int_regs': int_regs,
        'fp_regs': fp_regs,
        'regions': regions,
        'size': len(data),
    }


def decode_fault_type(xpsr):
    """Decode the exception number from xPSR."""
    exception_num = xpsr & 0x1FF
    fault_names = {
        0: 'Thread mode (no exception)',
        2: 'NMI',
        3: 'HardFault',
        4: 'MemManage',
        5: 'BusFault',
        6: 'UsageFault',
        11: 'SVCall',
        12: 'Debug Monitor',
        14: 'PendSV',
        15: 'SysTick',
    }
    if exception_num in fault_names:
        return fault_names[exception_num]
    if exception_num >= 16:
        return f'IRQ{exception_num - 16} (External Interrupt)'
    return f'Reserved (exception #{exception_num})'


def build_gdb_script(int_regs):
    """Build the GDB commands for extracting crash information."""
    commands = []
    commands.append('set pagination off')
    commands.append('set print pretty on')
    commands.append('set print array on')
    commands.append('set print demangle on')
    commands.append('set width 0')

    # register dump
    commands.append('echo ===GDB_REGISTERS_START===\\n')
    commands.append('info registers')
    commands.append('echo ===GDB_REGISTERS_END===\\n')

    # resolve address-bearing registers to symbols
    # use "info symbol" for symbol+offset and "info line" for source location
    commands.append('echo ===GDB_SYMBOLS_START===\\n')
    for reg_name in ['PC', 'LR', 'R0', 'R1', 'R2', 'R3', 'R4', 'R5',
                     'R6', 'R7', 'R8', 'R9', 'R10', 'R11', 'R12']:
        addr = int_regs.get(reg_name)
        if addr is None:
            continue
        # EXC_RETURN values in LR are not code addresses; skip symbol lookup
        if reg_name == 'LR' and addr in EXC_RETURN_PATTERNS:
            continue
        commands.append(f'echo {reg_name}_SYM=\\n')
        commands.append(f'info symbol {addr:#010x}')
        if reg_name in ('PC', 'LR'):
            commands.append(f'echo {reg_name}_LINE=\\n')
            commands.append(f'info line *{addr:#010x}')
    commands.append('echo ===GDB_SYMBOLS_END===\\n')

    # backtrace of current thread
    commands.append('echo ===GDB_BACKTRACE_START===\\n')
    commands.append('bt full')
    commands.append('echo ===GDB_BACKTRACE_END===\\n')

    # thread info
    commands.append('echo ===GDB_THREADS_START===\\n')
    commands.append('info threads')
    commands.append('echo ===GDB_THREADS_END===\\n')

    # backtrace of all threads
    commands.append('echo ===GDB_THREAD_BACKTRACE_START===\\n')
    commands.append('thread apply all bt full')
    commands.append('echo ===GDB_THREAD_BACKTRACE_END===\\n')

    # stack dump around SP
    commands.append('echo ===GDB_STACK_START===\\n')
    commands.append('x/32xw $sp')
    commands.append('echo ===GDB_STACK_END===\\n')

    # Fault status registers last: reading unmapped peripheral addresses can
    # abort the GDB script, so all critical sections must run before this.
    commands.append('echo ===GDB_FAULT_REGS_START===\\n')
    for name, addr in FAULT_REGS.items():
        commands.append(f'echo {name}=\\n')
        commands.append(f'x/1xw {addr:#010x}')
    commands.append('echo ===GDB_FAULT_REGS_END===\\n')

    commands.append('quit')
    return '\n'.join(commands)


def _build_chibi_thread_gdb_script(thread_t_addr, sym_name):
    """Build a minimal GDB script to get info and bt for one ChibiOS thread.

    Each thread runs in its own GDB session so a GDB internal error on one
    thread cannot prevent the others from being processed.

    thread_t_addr may be an int (for static _thread_wa threads) or None, in
    which case the GDB expression '&ch0.mainthread' is used.
    """
    if thread_t_addr is None:
        tp_expr = '(thread_t *)(&ch0.mainthread)'
    else:
        tp_expr = f'(thread_t *){thread_t_addr:#010x}'
    cmds = [
        'set pagination off',
        'set print frame-arguments none',
        'set width 0',
        'echo CHIBI_THREAD_START\\n',
        f'set $tp = {tp_expr}',
        'printf "CHIBI_THREAD_ADDR=0x%08x\\n", (unsigned int)$tp',
        f'echo CHIBI_THREAD_WA={sym_name}\\n',
        # Verify name pointer is in flash/RAM range to avoid dereferencing fill
        'if ($tp->name >= 0x08000000 && $tp->name < 0x40000000)',
        'printf "CHIBI_THREAD_NAME=%s\\n", $tp->name',
        'else',
        'echo CHIBI_THREAD_NAME=\\n',
        'end',
        'printf "CHIBI_THREAD_STATE=%d\\n", (int)$tp->state',
        'printf "CHIBI_THREAD_PRIO=%d\\n", (int)$tp->hdr.pqueue.prio',
        'printf "CHIBI_THREAD_WABASE=0x%08x\\n", (unsigned int)$tp->wabase',
        'printf "CHIBI_THREAD_CTX_SP=0x%08x\\n", (unsigned int)$tp->ctx.sp',
        'echo CHIBI_THREAD_BT_START\\n',
        'if ((int)$tp->state != 1)',
        # ctx.sp must point into RAM; fill pattern (0x55555555) or uninitialised
        # values would cause GDB to abort on dereference.
        'if ($tp->ctx.sp >= 0x20000000 && $tp->ctx.sp < 0x40000000)',
        'set $ictx = $tp->ctx.sp',
        'set $r4  = $ictx->r4',
        'set $r5  = $ictx->r5',
        'set $r6  = $ictx->r6',
        'set $r7  = $ictx->r7',
        'set $r8  = $ictx->r8',
        'set $r9  = $ictx->r9',
        'set $r10 = $ictx->r10',
        'set $r11 = $ictx->r11',
        'set $pc  = (unsigned int)$ictx->lr',
        'set $sp  = (unsigned int)($ictx + 1)',
        'bt',
        'else',
        'echo (ctx.sp not in valid RAM range - stack not captured)\\n',
        'end',
        'end',
        'echo CHIBI_THREAD_BT_END\\n',
        'echo CHIBI_THREAD_END\\n',
        'quit',
    ]
    return '\n'.join(cmds)


def run_gdb_chibi_threads(elf_file, dump_file, crashdebug_exe, thread_areas):
    """Run a separate GDB session per ChibiOS thread and collect results.

    Returns a concatenated string in the same format as the inline CHIBI
    section so parse_chibi_threads() can consume it unchanged.
    """
    combined = '===GDB_CHIBI_THREADS_START===\n'

    for sym_name, (wa_addr, thread_t_addr) in thread_areas.items():
        script = _build_chibi_thread_gdb_script(thread_t_addr, sym_name)

        with tempfile.NamedTemporaryFile(mode='w', suffix='.gdb', delete=False) as f:
            f.write(script)
            script_path = f.name

        try:
            target_cmd = (f'target remote | {crashdebug_exe} '
                          f'--elf {elf_file} --dump {dump_file}')
            cmd = [
                'arm-none-eabi-gdb', '-nx', '--batch', '--quiet', elf_file,
                '-ex', 'set target-charset ASCII',
                '-ex', target_cmd,
                '-x', script_path,
            ]
            result = subprocess.run(
                cmd, capture_output=True, text=True,
                encoding='utf-8', errors='replace', timeout=30,
            )
            out = result.stdout
            start = out.find('CHIBI_THREAD_START')
            end = out.find('CHIBI_THREAD_END')
            if start >= 0 and end >= 0:
                combined += out[start:end + len('CHIBI_THREAD_END')] + '\n'
            elif start >= 0:
                # GDB crashed before CHIBI_THREAD_END - close it out
                combined += out[start:] + '\nCHIBI_THREAD_END\n'
        except (subprocess.TimeoutExpired, FileNotFoundError):
            combined += f'CHIBI_THREAD_START\nCHIBI_THREAD_WA={sym_name}\n'
            combined += 'CHIBI_THREAD_NAME=\nCHIBI_THREAD_STATE=-1\n'
            combined += 'CHIBI_THREAD_BT_START\n(GDB timed out or not found)\n'
            combined += 'CHIBI_THREAD_BT_END\nCHIBI_THREAD_END\n'
        finally:
            os.unlink(script_path)

    combined += '===GDB_CHIBI_THREADS_END===\n'
    return combined


def run_gdb(elf_file, dump_file, crashdebug_exe, int_regs):
    """Run GDB in batch mode and capture output."""
    script = build_gdb_script(int_regs)

    with tempfile.NamedTemporaryFile(mode='w', suffix='.gdb', delete=False) as f:
        f.write(script)
        script_path = f.name

    try:
        target_cmd = (f'target remote | {crashdebug_exe} '
                      f'--elf {elf_file} --dump {dump_file}')
        cmd = [
            'arm-none-eabi-gdb',
            '-nx',
            '--batch',
            '--quiet',
            elf_file,
            '-ex', 'set target-charset ASCII',
            '-ex', target_cmd,
            '-x', script_path,
        ]

        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            encoding='utf-8',
            errors='replace',
            timeout=120,
        )
        return result.stdout, result.stderr, result.returncode
    except FileNotFoundError:
        print("Error: arm-none-eabi-gdb not found. Is the ARM toolchain installed?",
              file=sys.stderr)
        sys.exit(1)
    except subprocess.TimeoutExpired:
        print("Error: GDB timed out after 120 seconds", file=sys.stderr)
        sys.exit(1)
    finally:
        os.unlink(script_path)


def extract_section(output, start_marker, end_marker, allow_partial=False):
    """Extract text between markers from GDB output.

    If allow_partial is True and the end marker is absent (e.g. GDB crashed
    before writing it), return whatever follows the start marker.
    """
    start = output.find(start_marker)
    if start == -1:
        return None
    end = output.find(end_marker, start)
    if end == -1:
        if allow_partial:
            return output[start + len(start_marker):].strip()
        return None
    return output[start + len(start_marker):end].strip()


def parse_gdb_fault_regs(section):
    """Parse fault register values from GDB output."""
    if section is None:
        return {}
    regs = {}
    current_name = None
    for line in section.splitlines():
        line = line.strip()
        if line.endswith('='):
            current_name = line[:-1]
        elif current_name and ':' in line:
            # line like "0xe000ed28:   0x00000000"
            parts = line.split()
            if len(parts) >= 2:
                try:
                    regs[current_name] = int(parts[1], 16)
                except ValueError:
                    pass
            current_name = None
    return regs


def parse_gdb_symbols(section):
    """Parse symbol resolution results from GDB output.

    Returns a dict mapping register names to dicts with keys:
        'symbol': e.g. "AP_InertialSensor::update() + 124"
        'line':   e.g. "Line 456 of \"AP_InertialSensor.cpp\" ..."  (PC/LR only)
    """
    if section is None:
        return {}
    results = {}
    current_key = None
    current_field = None  # 'symbol' or 'line'
    for line in section.splitlines():
        line = line.strip()
        if line.endswith('_SYM='):
            reg_name = line[:-5]
            current_key = reg_name
            current_field = 'symbol'
            results.setdefault(reg_name, {})
        elif line.endswith('_LINE='):
            reg_name = line[:-6]
            current_key = reg_name
            current_field = 'line'
            results.setdefault(reg_name, {})
        elif current_key and current_field:
            # GDB outputs "No symbol matches ..." for unknown addresses
            if 'No symbol' in line or 'No line' in line:
                current_field = None
                continue
            results[current_key][current_field] = line
            current_field = None
    return results


def parse_chibi_threads(section):
    """Parse ChibiOS thread info from the GDB registry-walk output.

    Returns a list of dicts, each with keys:
        CHIBI_THREAD_ADDR, CHIBI_THREAD_NAME, CHIBI_THREAD_STATE,
        CHIBI_THREAD_PRIO, CHIBI_THREAD_WABASE, CHIBI_THREAD_CTX_SP,
        'bt' (list of backtrace lines)
    """
    if not section:
        return []

    threads = []
    current = None
    in_bt = False
    bt_lines = []

    for line in section.splitlines():
        line = line.rstrip()
        stripped = line.strip()
        if stripped == 'CHIBI_THREAD_START':
            current = {}
            in_bt = False
            bt_lines = []
        elif stripped == 'CHIBI_THREAD_END':
            if current is not None:
                current['bt'] = bt_lines
                threads.append(current)
                current = None
                bt_lines = []
        elif stripped == 'CHIBI_THREAD_BT_START':
            in_bt = True
        elif stripped == 'CHIBI_THREAD_BT_END':
            in_bt = False
        elif in_bt:
            # Collect backtrace; skip blank and GDB error lines
            if stripped and not stripped.startswith('Cannot access') \
                    and not stripped.startswith('warning:'):
                bt_lines.append(stripped)
        elif current is not None and '=' in stripped and not in_bt:
            key, _, val = stripped.partition('=')
            if key.startswith('CHIBI_THREAD_'):
                current[key] = val
    return threads


def format_reg_with_symbol(name, value, symbols):
    """Format a register value with its symbolic name if available."""
    sym_info = symbols.get(name, {})
    sym = sym_info.get('symbol', '')
    line = sym_info.get('line', '')

    result = f'{name:>3}: {value:#010x}'
    if sym:
        # "info symbol" output looks like:
        #   "func_name + offset in section .text of /path/to/elf"
        # trim the " in section .text of /path" part for readability
        display = sym
        in_section = display.find(' in section ')
        if in_section != -1:
            display = display[:in_section]
        result += f'  <{display}>'
    if line:
        # "info line" output looks like:
        #   "Line 123 of \"file.cpp\" starts at address 0x... and ends at 0x..."
        # extract just the "Line N of "file.cpp"" part
        ends_at = line.find(' starts at address')
        if ends_at != -1:
            display = line[:ends_at]
        else:
            display = line
        # clean up escaped quotes from GDB
        display = display.replace('\\"', '"')
        result += f'\n{"":>18}{display}'
    return result


def decode_cfsr(val):
    """Decode CFSR register bits."""
    lines = []
    for bit, desc in sorted(CFSR_BITS.items()):
        if val & (1 << bit):
            lines.append(f'    bit {bit:2d}: {desc}')
    return lines


def decode_hfsr(val):
    """Decode HFSR register bits."""
    lines = []
    for bit, desc in sorted(HFSR_BITS.items()):
        if val & (1 << bit):
            lines.append(f'    bit {bit:2d}: {desc}')
    return lines


def print_report(dump_info, gdb_stdout, gdb_stderr, elf_file=None, chibi_stdout=None):
    """Print the formatted crash dump report."""
    print('=' * 72)
    print('ArduPilot Crash Dump Report')
    print('=' * 72)

    # dump file info
    ver = dump_info['version']
    print(f'\nDump format version: {ver[0]}.{ver[1]}')
    print(f'Dump size: {dump_info["size"]} bytes')
    print(f'FPU state saved: {"yes" if dump_info["has_fpu"] else "no"}')

    # fault type from xPSR
    int_regs = dump_info['int_regs']
    if 'exceptionPSR' in int_regs:
        fault_type = decode_fault_type(int_regs['exceptionPSR'])
        exc_num = int_regs['exceptionPSR'] & 0x1FF
        print(f'Exception type: {fault_type} (exception #{exc_num})')
    elif 'xPSR' in int_regs:
        fault_type = decode_fault_type(int_regs['xPSR'])
        exc_num = int_regs['xPSR'] & 0x1FF
        print(f'Exception type: {fault_type} (exception #{exc_num})')

    # check for stack overflow sentinel
    for region in dump_info['regions']:
        if region[0] == 'STACK_OVERFLOW_SENTINEL':
            print('\n*** WARNING: CrashCatcher stack overflow detected! ***')
            print('*** The crash dump itself may be corrupt. ***')

    # resolve register addresses to symbols
    sym_section = extract_section(
        gdb_stdout, '===GDB_SYMBOLS_START===', '===GDB_SYMBOLS_END===')
    symbols = parse_gdb_symbols(sym_section)

    # registers from dump with symbolic names
    print(f'\n{"Registers":=^72}')
    print(f'  {format_reg_with_symbol("PC", int_regs.get("PC", 0), symbols)}')
    print(f'  {format_reg_with_symbol("LR", int_regs.get("LR", 0), symbols)}')
    lr_addr = int_regs.get('LR', 0)
    lr_indent = ' ' * 20
    if lr_addr in EXC_RETURN_PATTERNS:
        print(f'{lr_indent}EXC_RETURN: {EXC_RETURN_PATTERNS[lr_addr]}')
    elif not symbols.get('LR', {}).get('symbol') and elf_file:
        func, loc = resolve_addr2line(elf_file, lr_addr)
        if func:
            loc_str = f' at {loc}' if loc else ''
            print(f'{lr_indent}(in function: {func}{loc_str})')
    print(f'  SP:  {int_regs.get("SP", 0):#010x}    '
          f'MSP: {int_regs.get("MSP", 0):#010x}    '
          f'PSP: {int_regs.get("PSP", 0):#010x}')
    # print R0-R12: those with symbols get their own line,
    # the rest are grouped compactly
    printed = set()
    for i in range(13):
        name = f'R{i}'
        if symbols.get(name, {}).get('symbol'):
            print(f'  {format_reg_with_symbol(name, int_regs.get(name, 0), symbols)}')
            printed.add(i)
    # print remaining in groups of 4
    non_sym = [i for i in range(13) if i not in printed]
    for start in range(0, len(non_sym), 4):
        group = non_sym[start:start + 4]
        parts = [f'R{i:>2}: {int_regs.get(f"R{i}", 0):#010x}' for i in group]
        print(f'  {"    ".join(parts)}')
    print(f'  xPSR: {int_regs.get("xPSR", 0):#010x}    '
          f'exceptionPSR: {int_regs.get("exceptionPSR", 0):#010x}')

    # fault status registers (from GDB/memory)
    fault_regs_section = extract_section(
        gdb_stdout, '===GDB_FAULT_REGS_START===', '===GDB_FAULT_REGS_END===')
    fault_regs = parse_gdb_fault_regs(fault_regs_section)

    if fault_regs:
        print(f'\n{"Fault Status Registers":=^72}')
        for name in ['CFSR', 'HFSR', 'DFSR', 'MMFAR', 'BFAR', 'AFSR', 'ICSR']:
            if name in fault_regs:
                print(f'  {name:6s}: {fault_regs[name]:#010x}')
                if name == 'CFSR' and fault_regs[name]:
                    for line in decode_cfsr(fault_regs[name]):
                        print(line)
                elif name == 'HFSR' and fault_regs[name]:
                    for line in decode_hfsr(fault_regs[name]):
                        print(line)
    else:
        print(f'\n{"Fault Status Registers":=^72}')
        print('  (not available in crash dump - SCB registers were not captured)')

    # backtrace from GDB
    bt_section = extract_section(
        gdb_stdout, '===GDB_BACKTRACE_START===', '===GDB_BACKTRACE_END===')
    print(f'\n{"Backtrace":=^72}')
    if bt_section:
        print(bt_section)
    else:
        print('  (GDB backtrace not available)')

    # threads from GDB
    threads_section = extract_section(
        gdb_stdout, '===GDB_THREADS_START===', '===GDB_THREADS_END===')
    if threads_section:
        print(f'\n{"Threads":=^72}')
        print(threads_section)

    # backtrace of all threads
    thread_bt_section = extract_section(
        gdb_stdout, '===GDB_THREAD_BACKTRACE_START===', '===GDB_THREAD_BACKTRACE_END===')
    if thread_bt_section:
        print(f'\n{"Backtrace (all threads)":=^72}')
        print(thread_bt_section)

    # GDB register view (may contain symbolic names)
    regs_section = extract_section(
        gdb_stdout, '===GDB_REGISTERS_START===', '===GDB_REGISTERS_END===')
    if regs_section:
        print(f'\n{"Registers (GDB view)":=^72}')
        print(regs_section)

    # stack dump
    stack_section = extract_section(
        gdb_stdout, '===GDB_STACK_START===', '===GDB_STACK_END===')
    if stack_section:
        print(f'\n{"Stack Dump (32 words from SP)":=^72}')
        print(stack_section)

    # ChibiOS thread registry
    # ChibiOS thread data comes from per-thread GDB sessions (chibi_stdout),
    # not from the main GDB run.
    chibi_section = extract_section(
        chibi_stdout or '', '===GDB_CHIBI_THREADS_START===',
        '===GDB_CHIBI_THREADS_END===', allow_partial=True)
    chibi_threads = parse_chibi_threads(chibi_section)
    if chibi_threads:
        print(f'\n{"ChibiOS Threads":=^72}')
        for t in chibi_threads:
            addr = t.get('CHIBI_THREAD_ADDR', '?')
            wa_sym = t.get('CHIBI_THREAD_WA', '')
            raw_name = t.get('CHIBI_THREAD_NAME', '')
            # Keep only ASCII printable chars (32-126); filters out both
            # non-printable bytes and UTF-8 replacement chars (U+FFFD) that
            # arise when the name pointer points to arbitrary memory.
            name = ''.join(c for c in raw_name if 32 <= ord(c) <= 126)[:32]
            try:
                state_num = int(t.get('CHIBI_THREAD_STATE', '-1'))
            except ValueError:
                state_num = -1
            state_str = CHIBI_THREAD_STATES.get(state_num, f'state {state_num}')
            prio = t.get('CHIBI_THREAD_PRIO', '?')
            wabase = t.get('CHIBI_THREAD_WABASE', '?')
            ctx_sp = t.get('CHIBI_THREAD_CTX_SP', '?')

            name_display = f'"{name}"' if name else '(unnamed)'
            wa_display = f'  [{wa_sym}]' if wa_sym else ''
            print(f'\n  Thread {addr}{wa_display}  {name_display}  prio={prio}  [{state_str}]')
            print(f'    wabase={wabase}  ctx.sp={ctx_sp}', end='')

            # detect stack overflow (ctx.sp < wabase)
            try:
                if int(ctx_sp, 16) < int(wabase, 16):
                    print('  *** STACK OVERFLOW ***', end='')
            except ValueError:
                pass
            print()

            bt = t.get('bt', [])
            if state_num == 1:  # CH_STATE_CURRENT
                print('    (thread was active at crash; registers shown above)')
            elif bt:
                print('    Backtrace:')
                for btline in bt:
                    print(f'      {btline}')
            else:
                print('    (no backtrace available)')

    # memory regions summary
    print(f'\n{"Memory Regions in Dump":=^72}')
    total_mem = 0
    for region in dump_info['regions']:
        if region[0] == 'STACK_OVERFLOW_SENTINEL':
            print('  [STACK OVERFLOW SENTINEL]')
            continue
        start, end, size = region
        total_mem += size
        print(f'  {start:#010x} - {end:#010x}  ({size:>7d} bytes)')
    print(f'  Total memory captured: {total_mem} bytes')

    # FP registers if present
    if dump_info['fp_regs']:
        print(f'\n{"Floating Point Registers":=^72}')
        fp = dump_info['fp_regs']
        # detect ChibiOS stack fill pattern (lazy FPU stacking)
        fill_pattern = 0x55555555
        s_regs = [fp.get(f'S{i}', 0) for i in range(32)]
        fill_count = sum(1 for v in s_regs if v == fill_pattern)
        if fill_count > 24:
            print('  FP registers contain ChibiOS stack fill pattern (0x55555555)')
            print('  This indicates lazy FPU context save - the faulting code was')
            print('  not using floating point, so the FP register slots on the')
            print('  stack were never written by hardware.')
            non_fill = {f'S{i}': v for i, v in enumerate(s_regs) if v != fill_pattern}
            if non_fill:
                print('  Non-fill registers:')
                for name, raw in non_fill.items():
                    fval = struct.unpack('<f', struct.pack('<I', raw))[0]
                    print(f'    {name}: {raw:#010x} ({fval:.6g})')
        else:
            for i in range(0, 32, 4):
                parts = []
                for j in range(4):
                    name = f'S{i+j}'
                    if name in fp:
                        raw = fp[name]
                        fval = struct.unpack('<f', struct.pack('<I', raw))[0]
                        parts.append(f'{name:>3}: {raw:#010x} ({fval:>12.6g})')
                print(f'  {"  ".join(parts)}')
        if 'FPSCR' in fp:
            print(f'  FPSCR: {fp["FPSCR"]:#010x}')

    # any GDB errors worth noting
    if gdb_stderr:
        # filter out common noise
        errors = []
        for line in gdb_stderr.splitlines():
            line = line.strip()
            if not line:
                continue
            # skip common harmless warnings
            if 'warning: No executable has been specified' in line:
                continue
            if 'Try "help target"' in line:
                continue
            errors.append(line)
        if errors:
            print(f'\n{"GDB Warnings/Errors":=^72}')
            for line in errors:
                print(f'  {line}')

    print(f'\n{"=" * 72}')
    print('Note: crash dumps contain only a subset of RAM at the time of')
    print('the fault. Some memory reads and backtraces may be incomplete.')
    print(f'{"=" * 72}')


def main():
    parser = argparse.ArgumentParser(
        description='Generate a non-interactive crash dump report for ArduPilot',
    )
    parser.add_argument('elf_file', help='ELF file matching the firmware that crashed')
    parser.add_argument('dump_file', help='crash_dump.bin file')
    parser.add_argument('--gdb', default='arm-none-eabi-gdb',
                        help='path to arm-none-eabi-gdb (default: from PATH)')
    parser.add_argument('--crashdebug', default=None,
                        help='path to CrashDebug binary (default: auto-detect)')

    args = parser.parse_args()

    if not os.path.isfile(args.elf_file):
        print(f"Error: ELF file not found: {args.elf_file}", file=sys.stderr)
        sys.exit(1)

    if not os.path.isfile(args.dump_file):
        print(f"Error: crash dump file not found: {args.dump_file}", file=sys.stderr)
        sys.exit(1)

    crashdebug = args.crashdebug or find_crashdebug()
    if crashdebug is None or not os.path.isfile(crashdebug):
        print("Error: CrashDebug binary not found. Ensure modules/CrashDebug "
              "is checked out.", file=sys.stderr)
        sys.exit(1)

    # validate and parse the dump file directly
    dump_info = validate_crashdump(args.dump_file)

    thread_was = find_thread_working_areas(args.elf_file)

    # In ChibiOS, thread_t lives at the TOP (high address) of the working
    # area, not at the base.  The dump captures the used stack as one region
    # starting at wa_addr, with the thread_t struct immediately above it as a
    # separate small region.  Identify thread_t_addr as the end address of the
    # large stack region that begins at wa_addr.
    region_end = {r[0]: r[1] for r in dump_info['regions'] if isinstance(r[0], int)}
    thread_areas = {}
    for sym, wa in thread_was.items():
        if sym == 'ch0.mainthread':
            # Main thread: thread_t is ch0.mainthread (embedded in ch_os_instance,
            # not at the top of a _thread_wa array).  Use the sentinel None so
            # run_gdb_chibi_threads uses a GDB expression instead of a number.
            thread_areas[sym] = (wa, None)
        else:
            thread_areas[sym] = (wa, region_end.get(wa, wa))

    # run GDB for symbolic information (main crash context)
    gdb_stdout, gdb_stderr, gdb_rc = run_gdb(
        args.elf_file, args.dump_file, crashdebug, dump_info['int_regs'])

    # run per-thread GDB sessions for ChibiOS thread backtraces; each thread
    # runs in its own session so a GDB internal error on one thread does not
    # prevent the others from being processed.
    chibi_stdout = run_gdb_chibi_threads(
        args.elf_file, args.dump_file, crashdebug, thread_areas)

    print_report(dump_info, gdb_stdout, gdb_stderr, elf_file=args.elf_file,
                 chibi_stdout=chibi_stdout)


if __name__ == '__main__':
    main()
