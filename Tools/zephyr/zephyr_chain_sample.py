#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""High-rate, NON-INTRUSIVE sampler for g_ap_prof over SWD.

One pyocd session, read_memory_block32 in a tight loop. The core is never
halted (verified: read32 returns while the CPU runs), so this does not perturb
timing and does not kill the USB CDC link the way halt-based PC sampling does.

Each sample grabs all four words in ONE transaction, so main-phase, bus-phase
and both counters come from the same instant and can be correlated.
"""
import sys
import time

from collections import Counter

from pyocd.core.helpers import ConnectHelper

# valid_aps=[0] REQUIRED on mr_vmu_rt1176 (2026-08-08): pyocd's default
# discovery walks AHB-AP#1 - the dormant CM4's debug port - which answers
# WAIT forever and wedges the whole DAP until the target is reset. AP0 (CM7)
# only. Clearing a wedged DAP needs no hands: zephyr_pin_reset.py pulses
# nRST through the probe with zero DAP transactions.


ADDR = int(sys.argv[1], 0) if len(sys.argv) > 1 else 0x20212a54
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

NAMES = ['OTHER', 'bus_cb', 'spi_xfer', 'read_fifo',
         'wait_sample', 'ins_update', 'ekf3_update', 'ahrs_update']


def name(i):
    return NAMES[i] if i < len(NAMES) else '?%d' % i


session = ConnectHelper.session_with_chosen_probe(
    target_override='cortex_m', connect_mode='attach',
    options={'frequency': 4000000, 'valid_aps': [0]})
if session is None:
    print('no probe')
    sys.exit(1)
session.open()
tgt = session.target

main_c, bus_c = Counter(), Counter()
first = last = None
n = 0
t0 = time.time()
while time.time() - t0 < SECS:
    try:
        v = tgt.read_memory_block32(ADDR, 15)
    except Exception:  # noqa: BLE001
        continue
    # layout: [0]=main phase [1]=loop cnt [2]=buscb total
    #         [3..8]=per-bus phase  [9..14]=per-bus counters
    main_c[v[0]] += 1
    for b in range(6):
        if v[3 + b]:
            bus_c[v[3 + b]] += 1
    if first is None:
        first = (v[1], v[2], time.time(), list(v[9:15]))
    last = (v[1], v[2], time.time(), list(v[9:15]))
    n += 1
session.close()

print('samples: %d over %.1fs (%.0f Hz sampling)' % (n, SECS, n / SECS))
if not n:
    sys.exit(0)

print('--- MAIN thread occupancy ---')
for k, c in main_c.most_common():
    print('  %-12s %5d (%5.1f%%)' % (name(k), c, c * 100.0 / n))
print('--- BUS thread occupancy ---')
for k, c in bus_c.most_common():
    print('  %-12s %5d (%5.1f%%)' % (name(k), c, c * 100.0 / n))

if first and last and last[2] > first[2]:
    dt = last[2] - first[2]
    dl = last[0] - first[0]
    db = last[1] - first[1]
    print('--- counters over %.1fs ---' % dt)
    print('  main loop iterations : %6d  -> %.1f Hz' % (dl, dl / dt))
    print('  bus callbacks        : %6d  -> %.1f Hz' % (db, db / dt))
    if dl:
        print('  bus_cb per loop      : %.1f' % (db / float(dl)))
    for b in range(6):
        d = last[3][b] - first[3][b]
        if d > 0:
            print('    bus %d              : %6d  -> %.1f Hz' % (b, d, d / dt))
