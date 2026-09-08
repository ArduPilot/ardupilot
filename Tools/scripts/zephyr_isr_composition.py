#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Per-vector interrupt rates from g_ap_isr_vec over SWD - never halts.

Requires CONFIG_TRACING=y + CONFIG_TRACING_USER=y + CONFIG_AP_ISR_COUNT=y
(the per-vector table lives beside the existing total counters in Util.cpp).
Reads the 256-entry table twice across a window and prints the delta as
interrupts/second per vector, so boot-time noise is excluded.

WHY: at 258 Hz (2026-08-08) the remaining CPU tax is switch/IRQ RATE -
pendsv 7.5% + sys_clock_isr 6% + isr_wrapper 2.3% + lpspi_isr 2.5%, all
already in ITCM. Attacking rate needs the composition: which vectors, how
often. Totals alone cannot say whether SysTick, LPSPI, LPI2C-per-byte or
LPUART dominates.

Vector numbering (ARMv7-M): 15 = SysTick, 16+n = external IRQ n. External
names come from the IRQn enum in the NXP device header, parsed at runtime -
never hand-copy an IRQ table.

USAGE
    Tools/scripts/zephyr_isr_composition.py [seconds] [--elf PATH]
"""
import argparse
import re
import subprocess
import sys
import time

from pyocd.core.helpers import ConnectHelper

DEFAULT_ELF = 'build/mr_vmu_rt1176/zephyr_build/zephyr/zephyr.elf'
NM_CANDIDATES = ['arm-zephyr-eabi-nm', 'arm-none-eabi-nm', 'nm']
NXP_HEADER = ('modules/zephyr/modules/hal/nxp/mcux/mcux-sdk-ng/devices/RT/'
              'RT1170/MIMXRT1176/MIMXRT1176_cm7.h')
N_VEC = 256
SYSTEM_NAMES = {2: 'NMI', 3: 'HardFault', 4: 'MemManage', 5: 'BusFault',
                6: 'UsageFault', 11: 'SVCall', 14: 'PendSV', 15: 'SysTick'}


def find_symbol(elf, symbol):
    for nm in NM_CANDIDATES:
        try:
            out = subprocess.run([nm, elf], capture_output=True, text=True,
                                 check=True).stdout
        except (FileNotFoundError, subprocess.CalledProcessError):
            continue
        for line in out.splitlines():
            parts = line.split()
            if len(parts) == 3 and parts[2] == symbol:
                return int(parts[0], 16)
    return None


def irq_names():
    """IRQ number -> name from the NXP device header's IRQn enum."""
    names = {}
    try:
        with open(NXP_HEADER, 'r', encoding='utf-8') as f:
            text = f.read()
    except OSError:
        return names
    for m in re.finditer(r'^\s*(\w+)_IRQn\s*=\s*(\d+)\s*,', text, re.M):
        # first definition wins; the enum also contains negative system ones
        names.setdefault(int(m.group(2)), m.group(1))
    return names


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('seconds', nargs='?', type=float, default=15.0)
    ap.add_argument('--elf', default=DEFAULT_ELF)
    args = ap.parse_args()

    addr = find_symbol(args.elf, 'g_ap_isr_vec')
    if addr is None:
        print('g_ap_isr_vec not in %s - is the diagnostic build flashed?'
              % args.elf)
        return 1

    session = ConnectHelper.session_with_chosen_probe(
        target_override='mimxrt1170_cm7', connect_mode='attach',
        options={'frequency': 10000000})
    if session is None:
        print('no probe')
        return 1
    session.open()
    tgt = session.target
    first = tgt.read_memory_block32(addr, N_VEC)
    t0 = time.time()
    time.sleep(args.seconds)
    second = tgt.read_memory_block32(addr, N_VEC)
    dt = time.time() - t0
    session.close()

    ext = irq_names()
    rows = []
    total = 0
    for v in range(N_VEC):
        d = second[v] - first[v]
        if d <= 0:
            continue
        total += d
        if v >= 16:
            name = 'IRQ%d %s' % (v - 16, ext.get(v - 16, '?'))
        else:
            name = SYSTEM_NAMES.get(v, 'exc%d' % v)
        rows.append((d, name))
    rows.sort(reverse=True)

    print('window %.1f s   total %d interrupts  (%.0f /s)'
          % (dt, total, total / dt))
    for d, name in rows:
        print('  %8.0f /s  %5.1f%%  %s' % (d / dt, 100.0 * d / total, name))
    if not rows:
        print('ALL ZERO - tooling symptom first: wrong ELF, or the '
              'diagnostic Kconfigs did not reach the AP-side objects '
              '(see libraries/AP_HAL_Zephyr/ARCHITECTURAL.md on build dependency edges)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
