#!/usr/bin/env python3
'''
Fetch @SYS/threads.txt, @SYS/tasks.txt and @SYS/mem.txt over MAVFTP.

    Tools/scripts/zephyr_sysfiles_ftp.py /dev/serial/by-id/usb-ArduPilot_CubeOrange*-if00 --tag before
    Tools/scripts/zephyr_sysfiles_ftp.py tcp:127.0.0.1:5762 --tag renode

Capture discipline from the bench notes (maintainer, 2026-08-08): the first
read after boot is discarded and the second kept, because the first carries
boot-time load; a MAVFTP @SYS transfer often fails on the first attempt, so
each file is retried up to four times; and a --enable-stats build is the one
whose threads.txt carries per-thread load, so the header records the
firmware's git hash so nobody compares a stats build against a plain one.

Output: <outdir>/<file>-<board>-<git>-<tag>.txt with a provenance header.
Both HALs emit the same ThreadsV2/TasksV2 formats, so a ChibiOS capture and a
Zephyr capture compare column for column.
'''
import argparse
import os
import sys
import time

from pymavlink import mavftp
from pymavlink import mavutil

SYS_FILES = ('threads.txt', 'tasks.txt', 'mem.txt')


def fetch(ftp, name, attempts, tmp):
    for i in range(attempts):
        if os.path.exists(tmp):
            os.unlink(tmp)
        ret = ftp.cmd_get(['@SYS/' + name, tmp])
        if not getattr(ret, 'error_code', 1):
            ret = ftp.process_ftp_reply('get', timeout=60)
        if getattr(ret, 'error_code', 1) == 0 and os.path.exists(tmp) and os.path.getsize(tmp) > 0:
            with open(tmp, 'rb') as f:
                data = f.read()
            os.unlink(tmp)
            return data.decode('utf-8', 'replace')
        print('  %s: attempt %d/%d failed (%s)' % (
            name, i + 1, attempts, getattr(ret, 'error_code', ret)), flush=True)
        time.sleep(1.5)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('link', help='serial device or tcp:host:port')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--tag', default='capture')
    ap.add_argument('--outdir', default='.')
    ap.add_argument('--attempts', type=int, default=4)
    ap.add_argument('--keep-first', action='store_true',
                    help='keep the first read (default discards it and keeps the second)')
    args = ap.parse_args()

    conn = mavutil.mavlink_connection(args.link, baud=args.baud)
    hb = conn.wait_heartbeat(timeout=30)
    if hb is None:
        print('no heartbeat on %s' % args.link)
        return 1
    # firmware identity for the header
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                               mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
    git = 'unknown'
    t0 = time.time()
    while time.time() - t0 < 5:
        m = conn.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=1)
        if m is not None:
            git = bytes(m.flight_custom_version).decode('ascii', 'ignore').strip('\x00') or git
            break
    ftp = mavftp.MAVFTP(conn, target_system=conn.target_system, target_component=conn.target_component)
    os.makedirs(args.outdir, exist_ok=True)
    # pymavlink stages every download at one fixed path (/tmp/temp_mavftp_file),
    # shared by every MAVFTP client on the machine; two captures at once would
    # write over each other. Stage under --outdir, per process.
    ftp.temp_filename = os.path.join(args.outdir, '.mavftp-%d.tmp' % os.getpid())

    reads = 1 if args.keep_first else 2
    per_read = [dict() for _ in range(reads)]
    for r in range(reads):
        print('read %d of %d%s' % (r + 1, reads, '' if r == reads - 1 else ' (will be discarded)'), flush=True)
        for name in SYS_FILES:
            # mem.txt is not served by every firmware (absent on AP_HAL_Zephyr
            # builds before 2026-09-12); one attempt is enough to find out.
            attempts = 1 if name == 'mem.txt' else args.attempts
            text = fetch(ftp, name, attempts, os.path.join(args.outdir, '.sysfile-%d.tmp' % os.getpid()))
            if text is None:
                if name == 'mem.txt':
                    why = 'not on this firmware'
                else:
                    why = 'FAILED after %d attempts' % attempts
                print('  %s: %s' % (name, why))
                continue
            per_read[r][name] = text
            print('  %s: %d bytes' % (name, len(text)))
        if r < reads - 1:
            time.sleep(2.0)

    # Keep the LAST read that succeeded for each file, and say which one that
    # was: a second read that failed must not be reported as "read 2 of 2 kept"
    # over the first read's text.
    kept = {}
    for name in SYS_FILES:
        for r in reversed(range(reads)):
            if name in per_read[r]:
                kept[name] = (r + 1, per_read[r][name])
                break
    if not kept:
        print('nothing fetched')
        return 1
    rc = 0
    for name, (which, text) in kept.items():
        if which != reads:
            print('  %s: only read %d of %d succeeded - keeping it, but it is the boot-time read' % (name, which, reads))
            rc = 2
        out = os.path.join(args.outdir, '%s-%s-%s.txt' % (name.replace('.txt', ''), git, args.tag))
        with open(out, 'w') as f:
            f.write('# captured %s over MAVFTP from %s, firmware git %s, read %d of %d kept, tag %s\n'
                    % (time.strftime('%Y-%m-%d %H:%M:%S UTC', time.gmtime()), args.link, git, which, reads, args.tag))
            f.write(text)
        print('wrote %s' % out)
    return rc


if __name__ == '__main__':
    sys.exit(main())
