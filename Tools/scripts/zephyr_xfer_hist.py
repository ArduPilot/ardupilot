#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Read the SPI transfer-duration histogram out of g_ap_prof over SWD.

REQUIRES CONFIG_AP_CHAIN_PROFILE=y, WHICH IS NO LONGER THE DEFAULT
Set it in libraries/AP_HAL_Zephyr/zephyr/prj.mr_vmu_rt1176.conf and rebuild.
Confirm it took with:

    grep AP_CHAIN_PROFILE build/<board>/zephyr_build/ardupilot_prj_autogen.conf

(that merge happens at BUILD time, not configure time - see libraries/AP_HAL_Zephyr/README.md).

AND NOTE HOW IT FAILS, because it does not fail loudly: g_ap_prof is DEFINED
unconditionally in Scheduler.cpp, but only WRITTEN when CONFIG_AP_CHAIN_PROFILE
is set. So against a =n build the symbol still resolves, every read succeeds,
and this script reports 0.0 Hz and 0 callbacks - numbers that look exactly like
a stalled board rather than absent instrumentation. On 2026-08-07 a genuinely
stalled board and a mis-built one had to be told apart the hard way; do not
repeat that. If every counter reads zero, suspect the BUILD before the board.

WHY THIS EXISTS
The blocker on mr_vmu_rt1176 is that a 5.6-byte SPI transfer costs
294 us against ~7 us of wire time. 294 us is a MEAN (total SPI_XFER wall time /
transfer count) and a mean cannot separate the two remaining explanations,
which call for opposite fixes:

  tight distribution   -> the cost really is per-transfer, so it is inside the
                          driver or the hardware
  long tail / bimodal  -> most transfers are fast and a few are enormous, so the
                          cost is QUEUEING. AP_PHASE_SPI_XFER counts WALL time,
                          so a bus thread that is ready but not scheduled has
                          its stall billed as transfer time.

This reports the distribution, so one run decides it. That matters because the
unexplained 22.7-41.3 Hz run-to-run loop-rate spread currently makes any A/B
comparison untrustworthy.

The core is NEVER halted - read_memory_block32 returns while the CPU runs, so
this does not perturb timing and does not kill the USB CDC link the way
halt-based PC sampling does.

USAGE
    Tools/scripts/zephyr_xfer_hist.py [seconds] [--elf PATH]

The g_ap_prof address is re-derived from the ELF on every run. Do not pass a
remembered address: a stale one reads unmapped memory as zeros, which looks
exactly like a wedged board and has cost this project real time. An all-zero
reading is a TOOLING symptom first.
"""
import argparse
import subprocess
import sys
import time

from pyocd.core.helpers import ConnectHelper

# valid_aps=[0] REQUIRED on mr_vmu_rt1176 (2026-08-08): pyocd's default
# discovery walks AHB-AP#1 - the dormant CM4's debug port - which answers
# WAIT forever and wedges the whole DAP until the target is reset. AP0 (CM7)
# only. Clearing a wedged DAP needs no hands: zephyr_pin_reset.py pulses
# nRST through the probe with zero DAP transactions.


# Layout mirrors libraries/AP_HAL_Zephyr/chain_profile.h. Keep in step with it.
MAX_BUSES = 6
I_LOOP_COUNT = 1
I_BUSCB_COUNT = 2
I_XFER_COUNT = 3 + 2 * MAX_BUSES
I_XFER_BYTES = I_XFER_COUNT + 1
I_CYC_LO = I_XFER_BYTES + 1
I_CYC_HI = I_CYC_LO + 1
I_CYC_MIN = I_CYC_HI + 1
I_CYC_MAX = I_CYC_MIN + 1
I_HZ = I_CYC_MAX + 1
I_HIST0 = I_HZ + 1
HIST_N = 24
# Per-phase durations. Order must match enum ap_phase_t in chain_profile.h.
PHASES = ['OTHER', 'bus_cb', 'spi_xfer', 'read_fifo', 'wait_sample',
          'ins_update', 'ekf3_update', 'ahrs_update', 'sched_tasks']
PHASE_N = len(PHASES)
I_PH_COUNT0 = I_HIST0 + HIST_N
I_PH_CYC0 = I_PH_COUNT0 + PHASE_N
# Per-scheduler-task: 3 words each - count, total microseconds, name pointer.
MAX_TASKS = 64
I_TASK0 = I_PH_CYC0 + 2 * PHASE_N
N_WORDS = I_TASK0 + 3 * MAX_TASKS

# Reference costs for the ratio column. ONLY spi_xfer has a defensible one:
# wire time for the measured ~8 byte payload at the configured clock.
#
# There is deliberately NO ekf3_update entry. An earlier version of this script
# compared it against CPUInfo's 17.9 us "EKF benchmark" and printed 476x, which
# is apples-to-oranges - that benchmark is a small synthetic matrix op, not a
# full AP_NavEKF3::UpdateFilter with 24-state covariance work. Quoting it as
# "EKF3 is 476x too slow" would be exactly the kind of inherited-but-unchecked
# number this project has been bitten by. Compare EKF3 against AP_HAL_ChibiOS
# on real hardware instead.
EXPECTED_US = {'spi_xfer': 7.0}

DEFAULT_ELF = 'build/mr_vmu_rt1176/zephyr_build/zephyr/zephyr.elf'
NM_CANDIDATES = ['arm-zephyr-eabi-nm', 'arm-none-eabi-nm', 'nm']


def find_symbol(elf, symbol):
    """Resolve a symbol address from the ELF. Never hardcode this."""
    for nm in NM_CANDIDATES:
        try:
            out = subprocess.run([nm, elf], capture_output=True, text=True, check=True).stdout
        except (FileNotFoundError, subprocess.CalledProcessError):
            continue
        for line in out.splitlines():
            parts = line.split()
            if len(parts) == 3 and parts[2] == symbol:
                return int(parts[0], 16)
    return None


def read_block(target, addr):
    return target.read_memory_block32(addr, N_WORDS)


def u64(words, lo_index):
    return words[lo_index] | (words[lo_index + 1] << 32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('seconds', nargs='?', type=float, default=10.0)
    ap.add_argument('--elf', default=DEFAULT_ELF)
    args = ap.parse_args()

    addr = find_symbol(args.elf, 'g_ap_prof')
    if addr is None:
        print('could not resolve g_ap_prof in %s' % args.elf)
        print('(is CONFIG_AP_CHAIN_PROFILE=y, and was the build successful?)')
        return 1
    print('g_ap_prof @ 0x%08x  (from %s)' % (addr, args.elf))

    session = ConnectHelper.session_with_chosen_probe(
        target_override='cortex_m', connect_mode='attach',
        options={'frequency': 4000000, 'valid_aps': [0]})
    if session is None:
        print('no probe')
        return 1
    session.open()
    target = session.target

    first = read_block(target, addr)
    t0 = time.time()
    time.sleep(args.seconds)
    second = read_block(target, addr)
    elapsed = time.time() - t0

    hz = second[I_HZ] or first[I_HZ]
    if hz == 0:
        print('cycle rate not published yet - no SPI transfer has completed.')
        print('ALL-ZERO IS A TOOLING SYMPTOM FIRST: check the ELF matches the '
              'firmware actually flashed.')
        return 1

    def cyc_to_us(c):
        return 1e6 * c / hz

    d_xfers = second[I_XFER_COUNT] - first[I_XFER_COUNT]
    d_bytes = second[I_XFER_BYTES] - first[I_XFER_BYTES]
    d_cycles = u64(second, I_CYC_LO) - u64(first, I_CYC_LO)
    d_loops = second[I_LOOP_COUNT] - first[I_LOOP_COUNT]
    d_cbs = second[I_BUSCB_COUNT] - first[I_BUSCB_COUNT]

    print('\nwindow %.1f s   cycle counter %.0f MHz' % (elapsed, hz / 1e6))
    print('main loop      %8.1f Hz' % (d_loops / elapsed))
    print('bus callbacks  %8.1f /s' % (d_cbs / elapsed))
    print('SPI transfers  %8.1f /s   (%d in window)' % (d_xfers / elapsed, d_xfers))
    if d_xfers:
        print('mean payload   %8.1f bytes' % (d_bytes / d_xfers))
        print('MEAN duration  %8.1f us   <-- the 294 us figure' % cyc_to_us(d_cycles / d_xfers))
    print('min duration   %8.1f us   (since boot)' % cyc_to_us(second[I_CYC_MIN]))
    print('max duration   %8.1f us   (since boot)' % cyc_to_us(second[I_CYC_MAX]))

    print('\nDURATION DISTRIBUTION (delta over the window)')
    deltas = [second[I_HIST0 + i] - first[I_HIST0 + i] for i in range(HIST_N)]
    total = sum(deltas)
    if total == 0:
        print('  no transfers completed in the window')
        return 0
    peak = max(deltas)
    shown = 0
    for i, count in enumerate(deltas):
        if count == 0:
            continue
        shown += 1
        lo_us = cyc_to_us(1 << i)
        hi_us = cyc_to_us((1 << (i + 1)) - 1)
        bar = '#' * max(1, int(40 * count / peak))
        print('  %9.1f - %9.1f us  %8d  %5.1f%%  %s'
              % (lo_us, hi_us, count, 100.0 * count / total, bar))

    # The verdict. Concentration in one or two adjacent buckets means the cost is
    # real per-transfer work; a spread across many means the bus thread is
    # waiting for CPU, not for the wire.
    ordered = sorted(range(HIST_N), key=lambda i: deltas[i], reverse=True)
    top2 = sum(deltas[i] for i in ordered[:2])
    print('\n  %d populated buckets; top 2 hold %.1f%% of transfers'
          % (shown, 100.0 * top2 / total))
    if top2 / total > 0.9 and shown <= 3:
        print('  => TIGHT. Cost is per-transfer: look inside the driver/hardware.')
    else:
        print('  => SPREAD/BIMODAL. Consistent with QUEUEING delay being billed as')
        print('     transfer time. All SPI bus threads and the 1 kHz timer thread')
        print('     share one Zephyr priority level, and equal-priority preemptible')
        print('     threads do not preempt each other.')

    # Per-phase mean duration. This is the number the occupancy histogram could
    # never give, and where the ~2000x read_AHRS ratio should show up directly.
    print('\nPER-PHASE MEAN DURATION (delta over the window)')
    print('  %-12s %10s %12s %12s %10s' % ('phase', 'calls', 'calls/s', 'mean', 'vs expected'))
    any_phase = False
    for p in range(PHASE_N):
        dn = second[I_PH_COUNT0 + p] - first[I_PH_COUNT0 + p]
        if dn <= 0:
            continue
        any_phase = True
        dc = u64(second, I_PH_CYC0 + 2 * p) - u64(first, I_PH_CYC0 + 2 * p)
        mean_us = cyc_to_us(dc / dn)
        exp = EXPECTED_US.get(PHASES[p])
        ratio = ('%.0fx' % (mean_us / exp)) if exp else '-'
        unit = ('%.1f us' % mean_us) if mean_us < 1000 else ('%.2f ms' % (mean_us / 1000))
        print('  %-12s %10d %12.1f %12s %10s' % (PHASES[p], dn, dn / elapsed, unit, ratio))
    if not any_phase:
        print('  (none - per-phase duration slots are new; is this firmware current?)')

    report_tasks(target, first, second, elapsed)
    return 0


def read_cstring(target, addr, limit=64):
    """Follow a task-name pointer over SWD.

    Read in small chunks rather than one fixed-size block. A single 48-byte read
    fails outright if the string sits near the end of a mapped region, and that
    silently turned one task into 'task[3]' while every other name resolved.
    Note ITCM is mapped at address 0 on this SoC, so a low address is not by
    itself evidence of a bad pointer - only reject the clearly-null range.
    """
    if not addr or addr < 0x100:
        return None
    out = []
    step = 16
    while len(out) < limit:
        try:
            raw = target.read_memory_block8(addr + len(out), step)
        except Exception:  # noqa: BLE001
            break
        for b in raw:
            if b == 0:
                return ''.join(out) if out else None
            if b < 32 or b > 126:
                return ''.join(out) if out else None
            out.append(chr(b))
    return ''.join(out) if out else None


def report_tasks(target, first, second, elapsed):
    """Attribute AP_PHASE_SCHED_TASKS across the scheduler task table.

    sched_tasks was measured at 26.11 ms of a 27.7 ms loop; AP_AHRS explained
    15.68 ms of it and the rest was unattributed. This is that remainder.
    """
    rows = []
    for t in range(MAX_TASKS):
        b = I_TASK0 + 3 * t
        dn = second[b] - first[b]
        if dn <= 0:
            continue
        dus = second[b + 1] - first[b + 1]
        name = read_cstring(target, second[b + 2]) or ('task[%d]' % t)
        rows.append((dus, dn, name, t))

    if not rows:
        print('\nPER-TASK: no data (needs the AP_Scheduler capture; is this firmware current?)')
        return

    total_us = sum(r[0] for r in rows)
    print('\nPER-SCHEDULER-TASK, by total time (window %.1f s)' % elapsed)
    print('  %-34s %8s %9s %10s %8s' % ('task', 'calls', 'calls/s', 'mean', 'share'))
    for dus, dn, name, _t in sorted(rows, reverse=True)[:18]:
        mean_us = dus / dn
        unit = ('%.1f us' % mean_us) if mean_us < 1000 else ('%.2f ms' % (mean_us / 1000))
        print('  %-34s %8d %9.1f %10s %7.1f%%'
              % (name[:34], dn, dn / elapsed, unit, 100.0 * dus / total_us))
    print('  %-34s %8s %9s %10s %8s'
          % ('TOTAL', '', '', '%.2f ms/loop' % (total_us / 1000.0 / max(1, elapsed) / 36.0), ''))
    print('  (%d tasks ran; total task time %.1f ms/s = %.1f%% of a core)'
          % (len(rows), total_us / elapsed / 1000.0, 100.0 * total_us / elapsed / 1e6))


if __name__ == '__main__':
    sys.exit(main())
