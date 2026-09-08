#!/usr/bin/env python3
'''
Boot an AP_HAL_Zephyr firmware under Renode and wait for a MAVLink heartbeat.

Zephyr boards have no Renode platform of their own. They do not need one: a
Zephyr board built for the same silicon as a ChibiOS board runs on that board's
generated platform, because run.py builds the platform from the MCU in
hwdef.dat and not from anything ChibiOS-specific. CubeOrangeZephyr is the same
H743 as CubeOrange, so:

    Tools/renode/zephyr_boot_check.py

is CubeOrangeZephyr's firmware on CubeOrange's platform. Exits 0 once a
HEARTBEAT arrives on the emulated UART, 1 on timeout.

Deliberately not named test_*: Tools/renode/tests is collected by pytest in CI,
and this needs a Zephyr toolchain and a built ELF that CI does not have.
'''

import argparse
import os
import socket
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# MAVLink v2 framing: 0xFD, len, incompat, compat, seq, sysid, compid, msgid[3]
MAVLINK2_MAGIC = 0xFD
MSGID_HEARTBEAT = 0


def find_heartbeat(buf):
    '''True when buf holds a complete-looking v2 HEARTBEAT frame.'''
    for i in range(len(buf) - 10):
        if buf[i] != MAVLINK2_MAGIC:
            continue
        msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
        if msgid == MSGID_HEARTBEAT:
            return True
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--board', default='CubeOrangeZephyr',
                    help='Zephyr board whose firmware to boot')
    ap.add_argument('--platform', default='CubeOrange',
                    help='ChibiOS board whose Renode platform to use; must be '
                         'the same MCU as --board')
    ap.add_argument('--elf', help='override the firmware ELF path')
    ap.add_argument('--renode', default='build/renode/renode')
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--port', type=int, default=5762,
                    help='emulated serial port to listen on')
    ap.add_argument('--monitor-port', type=int, default=5811)
    args = ap.parse_args()

    elf = args.elf or os.path.join(
        ROOT, 'build', args.board, 'zephyr_build', 'zephyr', 'zephyr.elf')
    if not os.path.isfile(elf):
        print('no firmware at %s - build it first:' % elf)
        print('    ./waf configure --board=%s && ./waf copter' % args.board)
        return 1

    cmd = [sys.executable, os.path.join(ROOT, 'Tools', 'renode', 'run.py'),
           args.platform, '--elf', elf, '--renode', args.renode,
           '--no-xterm', '--port', str(args.monitor_port), '--exec', 'start']
    print('booting %s on the %s platform' % (os.path.basename(elf), args.platform))
    proc = subprocess.Popen(cmd, cwd=ROOT,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    deadline = time.time() + args.timeout
    buf = b''
    try:
        sock = None
        while time.time() < deadline:
            if proc.poll() is not None:
                print('renode exited early with %d' % proc.returncode)
                return 1
            if sock is None:
                try:
                    sock = socket.create_connection(('127.0.0.1', args.port),
                                                    timeout=5)
                    sock.settimeout(5)
                except OSError:
                    time.sleep(2)          # machine still being assembled
                    continue
            try:
                chunk = sock.recv(512)
            except socket.timeout:
                continue
            except OSError:
                sock = None
                continue
            if not chunk:
                sock = None
                continue
            buf += chunk
            if find_heartbeat(buf):
                print('HEARTBEAT after %.0fs, %d bytes'
                      % (args.timeout - (deadline - time.time()), len(buf)))
                return 0
            buf = buf[-4096:]
        print('no heartbeat within %.0fs (%d bytes seen)' % (args.timeout, len(buf)))
        return 1
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()


if __name__ == '__main__':
    sys.exit(main())
