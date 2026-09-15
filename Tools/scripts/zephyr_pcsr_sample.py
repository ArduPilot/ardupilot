#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Statistical profiler via DWT_PCSR over SWD - never halts the core.

DWT_PCSR (0xE000101D... base 0xE000101C, "Program Counter Sample Register",
ARMv7-M C1.8.8) returns the address of a recently executed instruction each
time the debugger reads it. Reading it costs the target nothing and works
while the core runs, so unlike halt-based PC sampling it does not stall the
flight loop or kill the USB CDC link. 0xFFFFFFFF means the core was halted,
sleeping (WFI/WFE), or the read raced a state where no sample was available.

WHY THIS EXISTS (2026-08-08): per-phase wall durations said
EKF3::UpdateFilter costs 6.15 ms/call against a 2.5 ms whole-loop budget for
400 Hz. Wall time cannot distinguish "main's cycles are being stolen by
higher-priority threads/ISRs" from "this code executes slowly (XIP fetch
stalls)". PC samples answer both at once:
  - WHICH code the CPU is really in (symbol histogram), and
  - WHERE it executes from (ITCM 0x0xxxxxxx vs external NOR XIP 0x30xxxxxx),
    which is the difference between ~1 cycle and ~100+ cycle fetches.

Needs >=2000 samples before trusting any line of the output - a 4-sample run
once named PWM_Init as a hang location and that claim was never reproduced.

USAGE
    Tools/scripts/zephyr_pcsr_sample.py [seconds] [--elf PATH] [--top N]
"""
import argparse
import bisect
import subprocess
import sys
import time

from collections import Counter

from pyocd.core.helpers import ConnectHelper

# valid_aps=[0] REQUIRED on mr_vmu_rt1176 (2026-08-08): pyocd's default
# discovery walks AHB-AP#1 - the dormant CM4's debug port - which answers
# WAIT forever and wedges the whole DAP until the target is reset. AP0 (CM7)
# only. Clearing a wedged DAP needs no hands: zephyr_pin_reset.py pulses
# nRST through the probe with zero DAP transactions.


DWT_PCSR = 0xE000101C
DEFAULT_ELF = 'build/mr_vmu_rt1176/zephyr_build/zephyr/zephyr.elf'
NM_CANDIDATES = ['arm-zephyr-eabi-nm', 'arm-none-eabi-nm', 'nm']

# mr_vmu_rt1176 (i.MX RT1176 CM7) executable regions, from the board DTS and
# IMXRT1170RM rev3 ch.2 (system memory map):
#   ITCM   0x0000_0000  FlexRAM bank-mapped, single-cycle
#   DTCM   0x2000_0000  (data, PCs here would be a bug)
#   OCRAM  0x2020_0000+ on-chip RAM
#   FLEXSPI1 XIP 0x3000_0000  external serial NOR - the slow one
REGIONS = (
    (0x00000000, 0x000FFFFF, 'ITCM'),
    (0x20000000, 0x201FFFFF, 'DTCM'),
    (0x20200000, 0x2FFFFFFF, 'OCRAM'),
    (0x30000000, 0x3FFFFFFF, 'XIP-NOR'),
)


def region_of(pc):
    for lo, hi, name in REGIONS:
        if lo <= pc <= hi:
            return name
    return 'other'


def load_symbols(elf):
    """Sorted (addr, name) list of function symbols for bisect lookup."""
    out = None
    for nm in NM_CANDIDATES:
        try:
            out = subprocess.run([nm, '-C', elf], capture_output=True,
                                 text=True, check=True).stdout
            break
        except (FileNotFoundError, subprocess.CalledProcessError):
            continue
    if out is None:
        sys.exit('no working nm among %s' % (NM_CANDIDATES,))
    syms = []
    for line in out.splitlines():
        parts = line.split(None, 2)
        # t/T/w/W: code symbols only
        if len(parts) == 3 and parts[1] in 'tTwW':
            syms.append((int(parts[0], 16), parts[2]))
    syms.sort()
    return syms


def resolve(syms, addrs, pc):
    i = bisect.bisect_right(addrs, pc) - 1
    if i < 0:
        return '?0x%08x' % pc
    return syms[i][1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('seconds', nargs='?', type=float, default=15.0)
    ap.add_argument('--elf', default=DEFAULT_ELF)
    ap.add_argument('--top', type=int, default=25)
    args = ap.parse_args()

    syms = load_symbols(args.elf)
    addrs = [a for a, _ in syms]

    session = ConnectHelper.session_with_chosen_probe(
        target_override='cortex_m', connect_mode='attach',
        options={'frequency': 4000000, 'valid_aps': [0]})
    if session is None:
        sys.exit('no probe')
    session.open()
    tgt = session.target

    pcs = Counter()
    invalid = 0
    n = 0
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        try:
            pc = tgt.read32(DWT_PCSR)
        except Exception:  # noqa: BLE001
            continue
        n += 1
        if pc == 0xFFFFFFFF:
            invalid += 1
            continue
        pcs[pc] += 1
    session.close()

    valid = n - invalid
    print('%d reads in %.1f s (%.0f Hz), %d valid PC samples, %d invalid/sleep'
          % (n, args.seconds, n / args.seconds, valid, invalid))
    if valid < 2000:
        print('WARNING: <2000 samples - treat every line below as anecdote')
    if not valid:
        return 1

    by_region = Counter()
    by_sym = Counter()
    sym_region = {}
    for pc, c in pcs.items():
        r = region_of(pc)
        by_region[r] += c
        s = resolve(syms, addrs, pc)
        by_sym[s] += c
        sym_region[s] = r

    print('\n--- by memory region (where the CPU fetches from) ---')
    for r, c in by_region.most_common():
        print('  %-8s %6d  %5.1f%%' % (r, c, 100.0 * c / valid))

    print('\n--- top %d symbols ---' % args.top)
    for s, c in by_sym.most_common(args.top):
        print('  %6.2f%%  %-8s %s' % (100.0 * c / valid, sym_region[s], s))
    return 0


if __name__ == '__main__':
    sys.exit(main())
