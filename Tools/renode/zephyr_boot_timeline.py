#!/usr/bin/env python3
'''
When does this board's IMU actually start publishing, on its own clock?

The INS rate estimate gets its fast convergence window only while
millis64() < 30000 and the vehicle is disarmed
(AP_InertialSensor_Backend.cpp:70-73). CubeOrangeZephyr's three rate estimates
behave as if their first publish came well after that - but that figure is an
inference from running the rate filter backwards, because flight logs start at
arming and nothing observes the boot itself.

This observes it. It starts Renode, attaches to the emulated UART at once,
asks the board for all data streams, and timestamps every STATUSTEXT, the first
HEARTBEAT and the first IMU message in both wall clock and the board's own
SYSTEM_TIME.time_boot_ms - which is exactly the millis() the window uses.

    Tools/renode/zephyr_boot_timeline.py --elf build/CubeOrangeZephyr/zephyr_build/zephyr/zephyr.elf
    Tools/renode/zephyr_boot_timeline.py --elf build/CubeOrange/bin/arducopter   # the reference

Not named test_*: needs a built ELF and a Renode that CI does not have.

Note the first IMU message here is an upper bound on the first publish: it can
only arrive after MAVLink is up and the stream request has been honoured. The
STATUSTEXT timeline is the part that says where the boot time goes.
'''

import argparse
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from process_utils import terminate_process_group  # noqa: E402
from pymavlink import mavutil  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def kill_stray_renodes():
    '''Stop any Renode left over from an earlier run.

    A survivor keeps the UART port and the next run attaches to it instead of
    its own emulator. Matched on /proc/PID/exe and cmdline, never on pgrep -f,
    which matches the calling shell.'''
    victims = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open('/proc/%s/cmdline' % entry, 'rb') as f:
                if b'build/renode' not in f.read():
                    continue
            if 'renode' not in os.path.basename(
                    os.readlink('/proc/%s/exe' % entry)).lower():
                continue
        except OSError:
            continue
        victims.append(int(entry))
    for pid in victims:
        try:
            os.kill(pid, 15)
        except OSError:
            pass
    if victims:
        time.sleep(4)
        for pid in victims:
            try:
                os.kill(pid, 9)
            except OSError:
                pass


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--elf', required=True)
    ap.add_argument('--platform', default='CubeOrange')
    ap.add_argument('--renode', default='build/renode/renode')
    ap.add_argument('--uart-port', type=int, default=5762)
    ap.add_argument('--monitor-port', type=int, default=5811)
    ap.add_argument('--seconds', type=float, default=1200.0,
                    help='wall-clock budget')
    ap.add_argument('--until-boot-ms', type=int, default=70000,
                    help='stop once the board reports this much uptime')
    args = ap.parse_args()

    kill_stray_renodes()

    cmd = [sys.executable, os.path.join(ROOT, 'Tools', 'renode', 'run.py'),
           args.platform, '--elf', args.elf, '--renode', args.renode,
           '--no-xterm', '--port', str(args.monitor_port),
           '--uart-port', str(args.uart_port), '--exec', 'start']
    proc = subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.DEVNULL,
                            stderr=subprocess.STDOUT,
                            start_new_session=(os.name == 'posix'))
    t_launch = time.time()
    print('launched Renode with %s' % os.path.basename(args.elf), flush=True)

    texts = 0
    last_boot_ms = None
    first_imu = None
    first_hb = None
    try:
        conn = None
        last_request = 0.0
        deadline = t_launch + args.seconds
        while time.time() < deadline:
            if proc.poll() is not None:
                print('renode exited early (%d)' % proc.returncode)
                break
            if conn is None:
                try:
                    conn = mavutil.mavlink_connection(
                        'tcp:127.0.0.1:%d' % args.uart_port)
                except (OSError, ConnectionError):
                    # Renode has not opened the UART socket yet.
                    time.sleep(2)
                    continue
            if conn.target_system and last_boot_ms is None \
                    and time.time() - last_request > 10:
                # Nothing streams RAW_IMU or SYSTEM_TIME to a GCS that has not
                # asked, so without this the board looks silent however
                # healthy it is. 0 = MAV_DATA_STREAM_ALL. Re-sent every 10 s
                # until SYSTEM_TIME arrives: a single request sent the moment
                # the first heartbeat lands can reach the board before its GCS
                # link is ready to act on it, and the ChibiOS reference then
                # sits silent for the whole budget.
                conn.mav.request_data_stream_send(
                    conn.target_system, conn.target_component, 0, 4, 1)
                last_request = time.time()
            msg = conn.recv_match(blocking=True, timeout=5)
            if msg is None:
                continue
            wall = time.time() - t_launch
            mt = msg.get_type()
            if mt == 'SYSTEM_TIME':
                last_boot_ms = msg.time_boot_ms
            elif mt == 'STATUSTEXT':
                texts += 1
                print('%7.1fs wall  boot_ms=%-8s  %s'
                      % (wall, last_boot_ms, msg.text.strip()), flush=True)
            elif mt == 'HEARTBEAT' and first_hb is None:
                first_hb = (wall, last_boot_ms)
                print('%7.1fs wall  boot_ms=%-8s  <first HEARTBEAT>'
                      % (wall, last_boot_ms), flush=True)
            elif mt in ('RAW_IMU', 'SCALED_IMU2', 'SCALED_IMU3') \
                    and first_imu is None:
                first_imu = (wall, last_boot_ms, mt)
                print('%7.1fs wall  boot_ms=%-8s  <first %s: IMU is publishing>'
                      % (wall, last_boot_ms, mt), flush=True)
            if last_boot_ms is not None and last_boot_ms >= args.until_boot_ms:
                print('reached %d ms of board uptime, stopping'
                      % args.until_boot_ms, flush=True)
                break
    finally:
        try:
            terminate_process_group(proc)
        except (OSError, ProcessLookupError):
            # Already gone, or never had a process group of its own.
            pass

    print('\n=== %s ===' % os.path.basename(args.elf))
    print('first HEARTBEAT   : wall %s  boot_ms %s' % (
        first_hb and '%.1f s' % first_hb[0], first_hb and first_hb[1]))
    print('first IMU message : wall %s  boot_ms %s  (%s)' % (
        first_imu and '%.1f s' % first_imu[0], first_imu and first_imu[1],
        first_imu and first_imu[2]))
    print('last board uptime : %s ms' % last_boot_ms)
    if first_imu and first_imu[1] is not None:
        print('IMU first seen at %d ms of board uptime -> %s' % (
            first_imu[1],
            'AFTER the 30 s window: no fast convergence'
            if first_imu[1] >= 30000 else 'inside the 30 s window'))
    print('%d STATUSTEXTs' % texts)
    return 0


if __name__ == '__main__':
    sys.exit(main())
