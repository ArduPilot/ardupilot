#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Record loop rate

NOTE 2026-08-08: valid_aps=[0] is REQUIRED on mr_vmu_rt1176 - pyocd's full
discovery walks AHB-AP#1 (the dormant CM4's port), which answers WAIT forever
and wedges the whole DAP until target power-off. Restricting to AP0 (CM7)
avoids it entirely. This cost a full evening to find.
 and bus callback rate as a TIME SERIES. Observe only.

WORKS WITH OR WITHOUT CONFIG_AP_CHAIN_PROFILE, with reduced output when off:

  =y   g_ap_prof      loop rate AND bus callback rate, so stalls can be
                      distinguished from a merely slow loop.
  =n   g_ap_loop_count  loop rate only. That counter is UNGATED in
                      Scheduler.cpp, so a shipping build stays measurable.
                      Bus callbacks print as '-', never as 0.

Enable the full version in libraries/AP_HAL_Zephyr/zephyr/prj.mr_vmu_rt1176.conf
and confirm it took with:

    grep AP_CHAIN_PROFILE build/<board>/zephyr_build/ardupilot_prj_autogen.conf

That merge happens at BUILD time, not configure time (see libraries/AP_HAL_Zephyr/README.md) - AND waf
has no dependency edge from AP sources to the generated autoconf.h, so after
flipping this option you MUST wipe build/<board> or the old objects are silently
reused. Measured 2026-08-07: a supposedly =n image still contained
ap_prof_cycles and had a byte-identical .itcm to the =y build, which made an
overhead comparison between the two meaningless without either build failing.
Check the ELF, not the .conf:

    arm-none-eabi-nm zephyr.elf | grep -c ap_prof_cycles   # 0 means really off

Why this exists: every A/B measurement on this board was a short window (8 s)
taken after a fixed wait, on the assumption that the board settles to a steady
rate and stays there. On 2026-08-07 a readiness poll showed that assumption is
false for at least one build - the loop rate swung between 0 Hz and 79 Hz on a
multi-second period, and both the main loop and the SPI bus thread stopped
together (bus callbacks ~50/s against a normal ~3100/s), which is a stall
rather than a slowdown.

Against a signal like that an 8 s sample is a lottery ticket, so a difference
between two arms means nothing until the shape of the signal is known. This
script takes NO view on what "ready" means and discards nothing - it just
records, so the oscillation is visible instead of being averaged away.

Reads over SWD without halting, so observing costs the target nothing.

Usage: zephyr_timeseries.py <elf> [duration_s] [bin_s]
"""
import subprocess
import sys
import time

from pyocd.core.helpers import ConnectHelper

I_LOOP_COUNT = 1
I_BUSCB_COUNT = 2
N_WORDS = 3

# A stall, not a slow patch: the bus thread runs ~3100 callbacks/s when healthy,
# so anything near zero means it is blocked rather than merely behind.
STALL_CB = 200


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
        print('usage: zephyr_timeseries.py <elf> [duration_s] [bin_s]')
        return 1
    elf = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0
    binw = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0

    """
    Two instruments, in order of preference:

      g_ap_prof        loop count AND bus callback count. Needs
                       CONFIG_AP_CHAIN_PROFILE=y.
      g_ap_loop_count  loop count only, but UNGATED (Scheduler.cpp) - it is
                       incremented once per main-loop iteration regardless of
                       any Kconfig, so loop rate stays measurable on a shipping
                       build. Bus callback rate is simply unavailable.

    Falling back matters because a =n build has no g_ap_prof at all, and
    without this the only route to a loop rate was deriving one from
    @SYS/tasks.txt percentages - an indirect calculation that has already
    produced wrong answers in this project.
    """
    addr = sym(elf, 'g_ap_prof')
    have_buscb = addr is not None
    if not have_buscb:
        addr = sym(elf, 'g_ap_loop_count')
        if addr is None:
            print('timeseries: neither g_ap_prof nor g_ap_loop_count in %s' % elf)
            return 1
        print('# NOTE: no g_ap_prof (CONFIG_AP_CHAIN_PROFILE=n) - using')
        print('#       g_ap_loop_count. Loop rate is real; bus callbacks read 0')
        print('#       because they are not counted, NOT because the bus is idle.')

    s = ConnectHelper.session_with_chosen_probe(
        target_override='cortex_m', connect_mode='attach',
        options={'frequency': 4000000, 'valid_aps': [0]})
    if s is None:
        print('timeseries: no probe')
        return 1
    s.open()
    t = s.target

    print('# elf=%s duration=%.0fs bin=%.1fs' % (elf, duration, binw))
    print('#  t(s)   loop(Hz)   buscb(/s)')
    loops = []
    t_start = time.time()
    nw = N_WORDS if have_buscb else 1
    prev = t.read_memory_block32(addr, nw)
    t_prev = time.time()
    while time.time() - t_start < duration:
        time.sleep(binw)
        cur = t.read_memory_block32(addr, nw)
        now = time.time()
        dt = now - t_prev
        if have_buscb:
            loop_rate = (cur[I_LOOP_COUNT] - prev[I_LOOP_COUNT]) / dt
            cb_rate = (cur[I_BUSCB_COUNT] - prev[I_BUSCB_COUNT]) / dt
        else:
            loop_rate = (cur[0] - prev[0]) / dt
            cb_rate = float('nan')      # not counted - do NOT print it as 0
        # Stall detection needs the bus counter. Without it, say nothing rather
        # than inventing a verdict from a number we do not have.
        flag = ('  <-- STALL' if cb_rate < STALL_CB else '') if have_buscb else ''
        cbtxt = ('%11.0f' % cb_rate) if have_buscb else '          -'
        print('%7.1f %10.1f %s%s' % (now - t_start, loop_rate, cbtxt, flag))
        sys.stdout.flush()
        loops.append((loop_rate, cb_rate))
        prev, t_prev = cur, now

    if not loops:
        return 1
    lr = sorted(x[0] for x in loops)
    n = len(lr)
    print('\n# bins=%d  min=%.1f  p50=%.1f  max=%.1f Hz' % (n, lr[0], lr[n // 2], lr[-1]))
    if have_buscb:
        stalls = sum(1 for x in loops if x[1] < STALL_CB)
        print('# stalled bins: %d/%d (%.0f%%)' % (stalls, n, 100.0 * stalls / n))
    else:
        print('# stalled bins: UNAVAILABLE - needs the bus counter from')
        print('#                CONFIG_AP_CHAIN_PROFILE=y. A 0.0 Hz bin below is')
        print('#                a real stalled loop, not missing instrumentation.')
    # The point of the whole exercise: if max is far above p50 the signal is not
    # a steady rate, and any single short sample of it is unreliable by itself.
    if lr[-1] > 1.5 * max(lr[n // 2], 1e-6):
        print('# VERDICT: NOT steady - spread is too wide for a single short')
        print('#          sample to characterise. Compare distributions, not points.')
    else:
        print('# VERDICT: steady enough that a single short sample is meaningful.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
