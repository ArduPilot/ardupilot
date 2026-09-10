#!/usr/bin/env python3
"""
rp2350_ramfunc2_sizes.py -- annotate rp2350_ramfunc2_registry.txt with symbol sizes.

Usage:
    python3 Tools/scripts/rp2350_ramfunc2_sizes.py [ELF]

ELF defaults to build/Laurel/bin/arducopter.

For each active (non-commented) registry line the script looks up the symbol
in the built binary using arm-none-eabi-nm and appends the size as a trailing
comment: "  # 252 B (flash)" or "  # 252 B (sram)".

Pass --update to rewrite the registry file in-place; otherwise prints to stdout.
"""

import argparse
import subprocess
import re
import sys
from pathlib import Path

REGISTRY = Path(__file__).resolve().parent.parent.parent / \
    "libraries/AP_HAL_ChibiOS/hwdef/common/rp2350_ramfunc2_registry.txt"

NM = "arm-none-eabi-nm"
XIP_BASE  = 0x10000000   # flash window on RP2350
SRAM_BASE = 0x20000000


def load_symbols(elf: Path):
    """Return dict: demangled_name_no_args -> (size, addr)."""
    try:
        raw = subprocess.check_output(
            [NM, "--print-size", "--size-sort", str(elf)],
            stderr=subprocess.DEVNULL
        ).decode()
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        sys.exit(f"nm failed: {e}")

    mangled = {}  # mangled -> (size, addr)
    for line in raw.splitlines():
        parts = line.split()
        if len(parts) < 4:
            continue
        addr_s, size_s, typ, sym = parts[0], parts[1], parts[2], parts[3]
        if typ.upper() not in ("T", "W"):
            continue
        try:
            mangled[sym] = (int(size_s, 16), int(addr_s, 16))
        except ValueError:
            pass

    # demangle all at once
    try:
        demangled_out = subprocess.check_output(
            ["c++filt"] + list(mangled.keys()),
            stderr=subprocess.DEVNULL
        ).decode().splitlines()
    except (subprocess.CalledProcessError, FileNotFoundError):
        demangled_out = list(mangled.keys())

    result = {}  # demangled_no_args -> (size, addr)
    for sym, dem in zip(mangled.keys(), demangled_out):
        size, addr = mangled[sym]
        # strip argument list and spaces
        dem_stripped = re.sub(r'\(.*', '', dem).replace(' ', '')
        if dem_stripped not in result or result[dem_stripped][0] < size:
            result[dem_stripped] = (size, addr)
    return result


def annotate(registry: Path, symbols: dict, update: bool):
    lines = registry.read_text(encoding='utf-8').splitlines(keepends=True)
    out = []
    total_sram = 0
    total_flash_saved = 0

    for line in lines:
        stripped = line.rstrip('\n')
        if stripped.lstrip().startswith('#') or '|' not in stripped:
            out.append(line)
            continue

        parts = stripped.split('|')
        sym_name = parts[1].strip().replace(' ', '')

        # strip old size comment if present
        sym_name_clean = re.sub(r'\s*#.*$', '', sym_name)
        path_col = parts[0]
        if len(parts) > 2:
            extra = parts[2]
        else:
            extra = ''

        info = symbols.get(sym_name_clean)
        if info:
            size, addr = info
            if addr >= SRAM_BASE:
                loc = "sram"
                total_sram += size
            elif addr >= XIP_BASE:
                loc = "flash"
                total_flash_saved += size
            else:
                loc = "?"

            comment = f"  # {size} B ({loc})"
            if extra:
                new_line = f"{path_col}|{sym_name_clean}|{size}\n"
            else:
                new_line = f"{path_col}|{sym_name_clean}{comment}\n"
            out.append(new_line)
        else:
            out.append(line)
            print(f"  WARNING: symbol not found in binary: {sym_name_clean}", file=sys.stderr)

    result = ''.join(out)

    if update:
        registry.write_text(result, encoding='utf-8')
        print(f"Updated {registry}")
    else:
        print(result, end='')

    print("\nSummary:", file=sys.stderr)
    print(f"  Functions confirmed in SRAM:  {total_sram:,} B ({total_sram/1024:.1f} KB)", file=sys.stderr)
    print(f"  Functions confirmed in flash: {total_flash_saved:,} B (need to add to registry?)", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('elf', nargs='?', default='build/Laurel/bin/arducopter',
                    help='ELF binary (default: build/Laurel/bin/arducopter)')
    ap.add_argument('--update', action='store_true',
                    help='Rewrite registry file in-place with size annotations')
    args = ap.parse_args()

    elf = Path(args.elf)
    if not elf.exists():
        sys.exit(f"ELF not found: {elf}")

    symbols = load_symbols(elf)
    annotate(REGISTRY, symbols, args.update)


if __name__ == '__main__':
    main()
