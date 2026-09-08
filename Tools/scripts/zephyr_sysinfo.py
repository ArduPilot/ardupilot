#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Read @SYS/threads.txt and @SYS/tasks.txt off the board over SWD.

MAVFTP does not work on mr_vmu_rt1176 - it has never successfully fetched these
files over MAVLink. The firmware therefore renders the same text that
AP_Filesystem_Sys would serve into a global buffer (`g_ap_sysinfo`, populated by
ap_sysinfo_capture() every 2 s from the io thread), and this reads that buffer
straight out of RAM.

Reading over SWD rather than printing to the console is deliberate: printk()
goes through the BLOCKING mcux_lpuart_poll_out() at ~87 us/byte, so dumping a
few KB would stall the flight loop for hundreds of milliseconds and corrupt the
very measurement being taken. A memory read costs the target nothing.

`threads.txt` needs ./waf configure --enable-stats for the CPU LOAD% column
(CONFIG_THREAD_RUNTIME_STATS). Without it the threads are listed with stack
figures but no load.

Usage: Tools/scripts/zephyr_sysinfo.py [--elf PATH] [--outdir DIR]

Writes ./threads.txt and ./tasks.txt in the repo root - stable names matching the
@SYS files they mirror, so git shows the diff from run to run.
"""
import argparse
import os
import subprocess
import sys
import time

from pyocd.core.helpers import ConnectHelper

# valid_aps=[0] REQUIRED on mr_vmu_rt1176 (2026-08-08): pyocd's default
# discovery walks AHB-AP#1 - the dormant CM4's debug port - which answers
# WAIT forever and wedges the whole DAP until the target is reset. AP0 (CM7)
# only. Clearing a wedged DAP needs no hands: zephyr_pin_reset.py pulses
# nRST through the probe with zero DAP transactions.


DEFAULT_ELF = 'build/mr_vmu_rt1176/zephyr_build/zephyr/zephyr.elf'
NM_CANDIDATES = ['arm-zephyr-eabi-nm', 'arm-none-eabi-nm', 'nm']
MAX_LEN = 8192


def symbols(elf, wanted):
    """Resolve several symbols in one nm pass. Never hardcode these addresses:
    a stale one reads unmapped memory as zeros and looks like a wedged board."""
    found = {}
    for nm in NM_CANDIDATES:
        try:
            out = subprocess.run([nm, elf], capture_output=True, text=True, check=True).stdout
        except (FileNotFoundError, subprocess.CalledProcessError):
            continue
        for line in out.splitlines():
            parts = line.split()
            if len(parts) == 3 and parts[2] in wanted:
                found[parts[2]] = int(parts[0], 16)
        if found:
            return found
    return found


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--elf', default=DEFAULT_ELF)
    ap.add_argument('--wait', type=float, default=0.0,
                    help='seconds to wait for a fresh capture (seq change)')
    # Captures are EVIDENCE for a paid HAL contract - they belong in the repo,
    # not in /tmp. Written on every successful read to STABLE filenames matching
    # the @SYS names they mirror: git then gives a diff across runs, which dated
    # filenames do not.
    ap.add_argument('--outdir', default=None,
                    help='directory for threads.txt / tasks.txt (default: repo root)')
    args = ap.parse_args()

    want = {'g_ap_sysinfo', 'g_ap_sysinfo_len', 'g_ap_sysinfo_seq'}
    sym = symbols(args.elf, want)
    missing = want - set(sym)
    if missing:
        print('could not resolve %s in %s' % (', '.join(sorted(missing)), args.elf))
        print('(is CONFIG_AP_CHAIN_PROFILE=y and is this ELF the running build?)')
        return 1

    session = ConnectHelper.session_with_chosen_probe(
        target_override='cortex_m', connect_mode='attach',
        options={'frequency': 4000000, 'valid_aps': [0]})
    if session is None:
        print('no probe')
        return 1
    session.open()
    target = session.target

    def seq():
        return target.read_memory_block32(sym['g_ap_sysinfo_seq'], 1)[0]

    if args.wait > 0:
        start = seq()
        deadline = time.time() + args.wait
        while time.time() < deadline and seq() == start:
            time.sleep(0.25)

    # Read seq either side so a capture landing mid-read is detectable rather
    # than silently producing a torn buffer.
    seq_before = seq()
    length = target.read_memory_block32(sym['g_ap_sysinfo_len'], 1)[0]
    if length == 0:
        print('buffer empty - ap_sysinfo_capture() has not run yet.')
        print('It fires every 2 s from the io thread once the scheduler is up;')
        print('AP_Scheduler::task_info() also self-enables on its first call, so')
        print('the first capture carries only headers. Retry in a few seconds.')
        return 1
    if length > MAX_LEN:
        print('implausible length %d - stale ELF or wrong symbol.' % length)
        return 1

    raw = target.read_memory_block8(sym['g_ap_sysinfo'], length)
    seq_after = seq()

    text = bytes(raw).decode('ascii', errors='replace')
    print(text)
    print('---- %d bytes, capture seq %d ----' % (length, seq_after))

    torn = seq_after != seq_before
    if torn:
        print('WARNING: a new capture landed while reading (seq %d -> %d).'
              % (seq_before, seq_after))
        print('The text above may be torn. Re-run.')

    outdir = args.outdir
    if outdir is None:
        outdir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    # thread_info() emits a "ThreadsV2" header and task_info() a "TasksV2" one,
    # in that order, so split on the TasksV2 marker rather than a byte offset.
    marker = 'TasksV2'
    idx = text.find(marker)
    if idx >= 0:
        parts = {'threads.txt': text[:idx], 'tasks.txt': text[idx:]}
    else:
        # task_info() self-enables on its first call, so an early capture can be
        # threads-only. Do not invent an empty tasks.txt in that case.
        parts = {'threads.txt': text}
        print('\nNOTE: no TasksV2 section yet - task_info() self-enables on its '
              'first call. Re-run in a few seconds for tasks.txt.')

    def header_for(name):
        return '\n'.join([
            '# @SYS/%s captured from mr_vmu_rt1176 over SWD.' % name,
            '#',
            '# Read straight out of the g_ap_sysinfo buffer, NOT via MAVFTP - MAVFTP',
            '# has never successfully served these files on this board. The firmware',
            '# renders them with ap_sysinfo_capture() every 2 s on the io thread.',
            '# Not printed to the console: printk() blocks in mcux_lpuart_poll_out()',
            '# at ~87 us/byte and would stall the very loop being measured.',
            '#',
            '# CPU LOAD% requires ./waf configure --enable-stats.' if name == 'threads.txt'
            else '# Includes per-task MIN/MAX/avg - AP_Scheduler::task_info().',
            '# Captured %s, capture seq %d.' % (time.strftime('%Y-%m-%d %H:%M:%S'), seq_after),
            '# ELF: %s' % args.elf,
        ] + (['# WARNING: seq changed during the read - this capture may be TORN.']
             if torn else []))

    for name, body in parts.items():
        path = os.path.join(outdir, name)
        try:
            with open(path, 'w') as fh:
                fh.write(header_for(name) + '\n\n' + body.rstrip() + '\n')
            print('Evidence written to %s' % path)
        except OSError as e:
            print('FAILED to write %s: %s' % (path, e))
            return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
