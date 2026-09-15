#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Measure gaps in USB CDC traffic - an SWD-free liveness check.

Why: the loop-rate metric is read over SWD from g_ap_prof. When that metric
reports 0 Hz there are two possible explanations - the board is stalled, or the
measurement is lying - and no amount of staring at the same counter can tell
them apart. This is a second instrument that shares nothing with the first: no
debug probe, no ELF symbol lookup, no target memory read. It just listens to
the bytes the board emits.

MAVLink streams continuously once booted, so a board that stalls for seconds
CANNOT emit bytes across the stall. If SWD-reported stalls line up with silent
gaps here, the board really is stopping. If traffic is smooth while SWD reports
0 Hz, the metric is at fault.

Caveat worth knowing when reading the output: attaching a reader also DRAINS
the CDC buffer. If the firmware blocks writing to a full, unread CDC endpoint,
then merely running this can change the behaviour it is measuring. A board that
is healthy with a reader attached and stalled without one is itself a finding,
not a contradiction.

Usage: zephyr_cdc_gaps.py [seconds] [gap_threshold_s]
"""
import glob
import sys
import time

import serial

PORT_GLOB = '/dev/serial/by-id/usb-ArduPilot_MR-VMU-RT1176_*-if00'


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
    gap_thresh = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

    ports = glob.glob(PORT_GLOB)
    if not ports:
        print('cdc_gaps: no CDC port matching %s' % PORT_GLOB)
        return 1
    port = ports[0]
    try:
        ser = serial.Serial(port, 115200, timeout=0.05)
    except serial.SerialException as e:
        print('cdc_gaps: cannot open %s: %s' % (port, e))
        return 1

    print('# port=%s duration=%.0fs gap_threshold=%.1fs' % (port, duration, gap_thresh))
    t_start = time.time()
    last_rx = t_start
    total = 0
    gaps = []
    while time.time() - t_start < duration:
        data = ser.read(4096)
        now = time.time()
        if data:
            gap = now - last_rx
            if gap >= gap_thresh:
                gaps.append((last_rx - t_start, gap))
                print('  GAP %6.1fs -> %6.1fs  (%.1fs of silence)'
                      % (last_rx - t_start, now - t_start, gap))
                sys.stdout.flush()
            total += len(data)
            last_rx = now
    ser.close()

    elapsed = time.time() - t_start
    silent = sum(g for _, g in gaps)
    print('\n# bytes=%d  mean=%.0f B/s over %.0fs' % (total, total / elapsed, elapsed))
    print('# gaps >=%.1fs: %d, totalling %.1fs (%.0f%% of the window silent)'
          % (gap_thresh, len(gaps), silent, 100.0 * silent / elapsed))
    if gaps:
        print('# VERDICT: traffic is NOT continuous - the board really does stop.')
    else:
        print('# VERDICT: traffic is continuous - no stall visible from outside.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
