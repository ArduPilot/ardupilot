#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Read the last fatal-error record off mr_vmu_rt1176 over SWD.

k_sys_fatal_error_handler() (AP_HAL_Zephyr/zephyr/src/ap_fault_handler.c) stashes
reason/pc/lr/CFSR into globals BEFORE it tries to print. That matters because it
prints via a BLOCKING poll_out on a console that frequently does not drain, so
the handler can wedge mid-message with the reason unrecoverable from registers.

Zephyr reason codes (include/zephyr/fatal_types.h):
  0 CPU exception   1 unhandled interrupt   2 stack overflow
  3 kernel oops     4 kernel panic
"""
import subprocess
import sys

from pyocd.core.helpers import ConnectHelper

ELF = 'build/mr_vmu_rt1176/zephyr_build/zephyr/zephyr.elf'
REASONS = {0: 'CPU exception', 1: 'unhandled interrupt', 2: 'STACK OVERFLOW',
           3: 'kernel oops', 4: 'kernel panic'}
WANT = ['g_ap_fatal_count', 'g_ap_fatal_reason', 'g_ap_fatal_pc',
        'g_ap_fatal_lr', 'g_ap_fatal_cfsr']


def main():
    elf = sys.argv[1] if len(sys.argv) > 1 else ELF
    out = subprocess.run(['arm-none-eabi-nm', elf], capture_output=True, text=True).stdout
    sym = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) == 3 and p[2] in WANT:
            sym[p[2]] = int(p[0], 16)
    missing = set(WANT) - set(sym)
    if missing:
        print('missing symbols: %s (old firmware?)' % ', '.join(sorted(missing)))
        return 1

    s = ConnectHelper.session_with_chosen_probe(
        target_override='mimxrt1170_cm7', connect_mode='attach',
        options={'frequency': 10000000})
    if s is None:
        print('no probe')
        return 1
    s.open()
    t = s.target
    v = {k: t.read_memory_block32(a, 1)[0] for k, a in sym.items()}

    if v['g_ap_fatal_count'] == 0:
        print('No fatal error recorded (count=0). The board did not panic.')
        return 0

    reason = v['g_ap_fatal_reason']
    print('FATAL ERRORS: %d' % v['g_ap_fatal_count'])
    print('  reason = %d (%s)' % (reason, REASONS.get(reason, 'unknown')))
    print('  pc     = 0x%08x' % v['g_ap_fatal_pc'])
    print('  lr     = 0x%08x' % v['g_ap_fatal_lr'])
    print('  CFSR   = 0x%08x%s' % (v['g_ap_fatal_cfsr'],
                                   '  (zero => software panic, not a bus/mem fault)'
                                   if v['g_ap_fatal_cfsr'] == 0 else ''))
    for name, addr in (('pc', v['g_ap_fatal_pc']), ('lr', v['g_ap_fatal_lr'])):
        if addr:
            r = subprocess.run(['arm-none-eabi-addr2line', '-f', '-C', '-e', elf, hex(addr)],
                               capture_output=True, text=True).stdout.strip()
            print('  %s -> %s' % (name, r.replace('\n', ' @ ')))
    return 0


if __name__ == '__main__':
    sys.exit(main())
