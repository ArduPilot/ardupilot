#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
'''
Read @SYS/threads.txt and @SYS/tasks.txt off a running board over SWD, through
a Black Magic Probe and arm-none-eabi-gdb.

The firmware renders both files into RAM every 2 s (ap_sysinfo_capture() in
AP_HAL_Zephyr/Util.cpp) and publishes them as g_threads_txt / g_tasks_txt
(pointer + length into g_ap_sysinfo). This attaches, reads them, and detaches,
with nothing printed on any port - the reason this exists rather than a
console dump. The core IS halted from attach to detach - measured at up to
~0.6 s per read through a Black Magic Probe, gdb start-up included, printed
at the end; a Renode target is paused for the whole read - so it refuses a
board whose g_ap_soft_armed is set unless --force is given: 0.6 s at 400 Hz
is ~240 missed loops and, on some boards, the watchdog. It is the CubeOrange
counterpart of zephyr_sysinfo.py, which reads
the same buffer with pyocd on the RT1176 (MCU-Link).

    Tools/zephyr/zephyr_sysinfo_gdb.py --tag after-ladder
    Tools/zephyr/zephyr_sysinfo_gdb.py --elf build/CubeOrangeZephyr/zephyr_build/zephyr/zephyr.elf --outdir .

Capture discipline (maintainer, 2026-08-08): the first read after boot is not
kept - this reads twice and keeps the second; a --enable-stats build is the
one whose threads.txt carries per-thread load. The header line of each output
file records the ELF's git hash and the capture sequence number so captures
are never compared blind.

The ELF must be the one on the board: symbol addresses come from it. The
sequence counter is ODD while the firmware is rendering a capture and EVEN
when the buffer is whole; it is read before and after the dump and the read
is retried if it moved or was odd, so a capture landing mid-read, or a halt
landing mid-capture, is never kept.
'''

import argparse
import os
import re
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_ELF = os.path.join(ROOT, 'build', 'CubeOrangeZephyr', 'zephyr_build', 'zephyr', 'zephyr.elf')
DEFAULT_GDB = '/opt/gcc-arm-none-eabi-10-2020-q4-major/bin/arm-none-eabi-gdb'


def find_bmp():
    import glob
    ports = sorted(glob.glob('/dev/serial/by-id/usb-Black_Magic_Debug*-if00'))
    return ports[0] if ports else None


HALT_MS = []   # attach-to-detach wall time of every gdb session, an upper bound on the halt


def gdb_batch(gdb, bmp, elf, script, timeout=90):
    '''Run one attach/script/detach session; return gdb's full output.

    The full output is kept and returned on purpose: a helper that greps
    "matched" out of it turns a failed attach into silence that reads as
    success (bench notes, 2026-09-10).'''
    args = [gdb, '-q', '-batch', '-nx',
            '-ex', 'set confirm off', '-ex', 'set pagination off']
    if bmp.startswith('/dev/'):
        # a Black Magic Probe: scan the SWD line and attach to the first core
        args += ['-ex', 'target extended-remote %s' % bmp,
                 '-ex', 'monitor swdp_scan', '-ex', 'attach 1']
    else:
        # a plain GDB stub, e.g. Renode's (run.py --gdb exposes one): no probe,
        # no scan, the target is already attached on connect
        args += ['-ex', 'target remote %s' % bmp]
    args += ['-ex', 'set mem inaccessible-by-default off', '-x', script, '-ex', 'detach', elf]
    t0 = time.time()
    out = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    HALT_MS.append(int((time.time() - t0) * 1000))
    text = out.stdout + out.stderr
    if 'scan failed' in text or 'No usable targets' in text or 'Available Targets:\n\n' in text:
        raise RuntimeError('SWD attach failed:\n' + text)
    return text


def elf_has_symbol(elf, name):
    try:
        out = subprocess.run(['nm', elf], capture_output=True, text=True, timeout=60).stdout
    except Exception:  # noqa: BLE001
        return False
    return re.search(r' [A-Za-z] %s$' % re.escape(name), out, re.M) is not None


def read_once(gdb, bmp, elf, tmpdir, force, has_armed_flag):
    threads_path = os.path.join(tmpdir, 'threads.bin')
    tasks_path = os.path.join(tmpdir, 'tasks.bin')
    script = os.path.join(tmpdir, 'read.gdb')
    # One session: the armed flag and seq first, the 8 KB dump only if the board
    # is disarmed (or --force) - so a refused read halts the core for as little
    # as the attach itself costs. seq odd = the firmware is mid-capture.
    with open(script, 'w') as f:
        f.write('printf "ARMED=%%u\\n", %s\n' % ('g_ap_soft_armed' if has_armed_flag else '0'))
        f.write('printf "SEQ0=%u\\n", g_ap_sysinfo_seq\n')
        f.write('if (%s) == 0 || %d\n' % ('g_ap_soft_armed' if has_armed_flag else '0', 1 if force else 0))
        f.write('  printf "TLEN=%u KLEN=%u\\n", g_threads_txt_len, g_tasks_txt_len\n')
        f.write('  dump binary memory %s g_threads_txt (g_threads_txt + g_threads_txt_len)\n' % threads_path)
        f.write('  dump binary memory %s g_tasks_txt (g_tasks_txt + g_tasks_txt_len)\n' % tasks_path)
        f.write('  printf "SEQ1=%u\\n", g_ap_sysinfo_seq\n')
        f.write('end\n')
    text = gdb_batch(gdb, bmp, elf, script)
    ma = re.search(r'ARMED=(\d+)', text)
    m0 = re.search(r'SEQ0=(\d+)', text)
    if not (ma and m0):
        raise RuntimeError('could not read the capture globals - is this the ELF on the board?\n' + text)
    if int(ma.group(1)) and not force:
        raise RuntimeError('the board is ARMED (g_ap_soft_armed=1): refusing to halt it for a read; '
                           '--force overrides, at the cost of missed loops while halted')
    m1 = re.search(r'SEQ1=(\d+)', text)
    ml = re.search(r'TLEN=(\d+) KLEN=(\d+)', text)
    if not (m1 and ml):
        raise RuntimeError('could not read the capture globals - is this the ELF on the board?\n' + text)
    seq0, seq1 = int(m0.group(1)), int(m1.group(1))
    tlen, klen = int(ml.group(1)), int(ml.group(2))
    if seq0 == 0:
        raise RuntimeError('g_ap_sysinfo_seq is 0: the io thread has not rendered a capture yet')
    if seq0 != seq1 or (seq0 & 1):
        return None, seq1  # a capture landed mid-read, or was in progress; caller retries
    with open(threads_path, 'rb') as f:
        threads = f.read().decode('utf-8', 'replace')
    with open(tasks_path, 'rb') as f:
        tasks = f.read().decode('utf-8', 'replace')
    if len(threads) != tlen or len(tasks) != klen:
        raise RuntimeError('dump length mismatch: threads %d/%d tasks %d/%d' % (len(threads), tlen, len(tasks), klen))
    return {'threads.txt': threads, 'tasks.txt': tasks}, seq1


def elf_git(elf):
    try:
        out = subprocess.run(['strings', elf], capture_output=True, text=True, timeout=60).stdout
    except Exception:  # noqa: BLE001
        return 'unknown'
    m = re.search(r'ArduCopter V[\d.]+-?\w* \(([0-9a-f]{8})\)', out) or re.search(r'\(([0-9a-f]{8})\)', out)
    return m.group(1) if m else 'unknown'


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--elf', default=DEFAULT_ELF)
    ap.add_argument('--gdb', default=DEFAULT_GDB)
    ap.add_argument('--bmp', '--target', dest='bmp', default=None,
                    help='Black Magic Probe port (/dev/...; default: the first *-if00 found) '
                         'or a plain GDB stub as host:port, e.g. Renode\'s')
    ap.add_argument('--tag', default='capture')
    ap.add_argument('--outdir', default='.')
    ap.add_argument('--keep-first', action='store_true')
    ap.add_argument('--retries', type=int, default=4)
    ap.add_argument('--force', action='store_true',
                    help='read even if the board reports armed (each read halts the core)')
    args = ap.parse_args()

    bmp = args.bmp or find_bmp()
    if not bmp:
        print('no Black Magic Probe found under /dev/serial/by-id')
        return 1
    print('target: %s' % bmp, flush=True)
    if not os.path.isfile(args.elf):
        print('no ELF at %s' % args.elf)
        return 1
    git = elf_git(args.elf)
    has_armed_flag = elf_has_symbol(args.elf, 'g_ap_soft_armed')
    if not has_armed_flag:
        print('WARNING: this ELF has no g_ap_soft_armed (firmware before 2026-09-12): '
              'cannot tell whether the board is armed before halting it', flush=True)
    reads = 1 if args.keep_first else 2
    result = None
    seq = None
    with tempfile.TemporaryDirectory() as tmpdir:
        for r in range(reads):
            for attempt in range(args.retries):
                result, seq = read_once(args.gdb, bmp, args.elf, tmpdir, args.force, has_armed_flag)
                if result is not None:
                    break
                print('capture moved or in progress during the read (seq now %s), retrying' % seq, flush=True)
                time.sleep(0.5)
            if result is None:
                print('could not get a stable read in %d attempts' % args.retries)
                return 1
            print('read %d of %d: seq %d, threads.txt %d bytes, tasks.txt %d bytes%s' % (
                r + 1, reads, seq, len(result['threads.txt']), len(result['tasks.txt']),
                '' if r == reads - 1 else ' (discarded)'), flush=True)
            if r < reads - 1:
                time.sleep(2.5)  # the next capture is at most 2 s away
    os.makedirs(args.outdir, exist_ok=True)
    for name, body in result.items():
        out = os.path.join(args.outdir, '%s-%s-%s.txt' % (name.replace('.txt', ''), git, args.tag))
        with open(out, 'w') as f:
            f.write('# %s read over SWD (Black Magic Probe) %s, ELF git %s, capture seq %d, read %d of %d kept, tag %s\n'
                    % (name, time.strftime('%Y-%m-%d %H:%M:%S UTC', time.gmtime()), git, seq, reads, reads, args.tag))
            f.write(body)
        print('wrote %s' % out)
    print('core halted for at most %s ms per session (attach to detach, host-side)' % '/'.join(str(x) for x in HALT_MS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
