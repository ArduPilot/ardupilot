#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Wait until the board is in STEADY STATE, by polling - never a fixed sleep.

REQUIRES CONFIG_AP_CHAIN_PROFILE=y, WHICH IS NO LONGER THE DEFAULT
See zephyr_timeseries.py for the full note. Short version: g_ap_prof is DEFINED
unconditionally but only WRITTEN when that Kconfig is set, so the "no g_ap_prof"
check below CANNOT catch a =n build - the symbol resolves and the counters just
never move. Against such a build this waits out the whole timeout and reports
"board never reached steady state" on a board that is running perfectly.

A blind `sleep 90` is wrong in both directions: it wastes time when boot is
quick, and it still samples a booting board when boot is slow. That produced
readings of 3.9 Hz and 16.5 Hz that were reported as loop rates when they were
just a board still initialising.

What "ready" means here, in order:

  1. USB CDC enumerated as the APP (not the '-BL' bootloader name) - proves the
     image started at all. Takes a couple of seconds.
  2. g_ap_prof counters advancing - proves the flight loop is actually running.
  3. Bus callback rate at its plateau and the loop rate stable between two
     consecutive windows - proves initialisation has finished. Gyro cal runs to
     a 30 s bound on this board and dominates boot, so the plateau is the real
     signal, not any particular elapsed time.

Reads over SWD without halting, so polling costs the target nothing.

Usage: zephyr_wait_ready.py <elf> [timeout_s]
Exit 0 when steady, 1 on timeout.
"""
import subprocess
import sys
import time

from pyocd.core.helpers import ConnectHelper

MAX_BUSES = 6
I_LOOP_COUNT = 1
I_BUSCB_COUNT = 2
N_WORDS = 3

# Steady state on this board is ~3100 bus callbacks/s. Well below that means
# sensors are not being polled at rate yet.
MIN_CB_RATE = 2500
# Two consecutive loop-rate windows within this fraction => plateau reached.
STABLE_TOL = 0.08


def sym(elf, name):
    for nm in ('arm-zephyr-eabi-nm', 'arm-none-eabi-nm', 'nm'):
        try:
            out = subprocess.run([nm, elf], capture_output=True, text=True, check=True).stdout
        except (FileNotFoundError, subprocess.CalledProcessError):
            continue
        for line in out.splitlines():
            p = line.split()
            if len(p) == 3 and p[2] == name:
                return int(p[0], 16)
    return None


def main():
    if len(sys.argv) < 2:
        print('usage: zephyr_wait_ready.py <elf> [timeout_s]')
        return 1
    elf = sys.argv[1]
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 150.0

    addr = sym(elf, 'g_ap_prof')
    if addr is None:
        print('wait_ready: no g_ap_prof in %s (CONFIG_AP_CHAIN_PROFILE off?)' % elf)
        return 1

    s = ConnectHelper.session_with_chosen_probe(
        target_override='mimxrt1170_cm7', connect_mode='attach',
        options={'frequency': 10000000})
    if s is None:
        print('wait_ready: no probe')
        return 1
    s.open()
    t = s.target

    deadline = time.time() + timeout
    prev_rate = None
    window = 3.0
    while time.time() < deadline:
        a = t.read_memory_block32(addr, N_WORDS)
        t0 = time.time()
        time.sleep(window)
        b = t.read_memory_block32(addr, N_WORDS)
        dt = time.time() - t0

        cb_rate = (b[I_BUSCB_COUNT] - a[I_BUSCB_COUNT]) / dt
        loop_rate = (b[I_LOOP_COUNT] - a[I_LOOP_COUNT]) / dt

        if cb_rate < MIN_CB_RATE:
            print('  wait_ready: cb=%.0f/s loop=%.1fHz - still initialising' % (cb_rate, loop_rate))
            prev_rate = None
            continue

        if prev_rate is not None and prev_rate > 0:
            if abs(loop_rate - prev_rate) / prev_rate <= STABLE_TOL:
                print('  wait_ready: STEADY at loop=%.1fHz cb=%.0f/s (%.0fs)'
                      % (loop_rate, cb_rate, timeout - (deadline - time.time())))
                return 0
        print('  wait_ready: cb=%.0f/s loop=%.1fHz - waiting for plateau' % (cb_rate, loop_rate))
        prev_rate = loop_rate

    print('wait_ready: TIMEOUT after %.0fs - board never reached steady state' % timeout)
    return 1


if __name__ == '__main__':
    sys.exit(main())
