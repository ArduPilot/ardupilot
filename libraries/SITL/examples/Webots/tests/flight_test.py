#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""
Fly ArduCopter SITL against mock_webots.py and check the result.

This exercises the whole SIM_Webots.cpp path -- socket handshake, JSON framing,
time stepping, rotor-speed reporting -- without needing Webots installed.  It is
not a substitute for running the real simulator, but it catches the kind of
breakage that silently stopped this backend from working.

    ./flight_test.py                       # mock, default quadx profile
    ./flight_test.py --profile quadx-legacy
    ./flight_test.py --lockstep            # controller blocks for the servo frame

Or against the real simulator, which also exercises the world file, the
controller binary and Webots' own physics:

    WEBOTS_HOME=/usr/local/webots ./flight_test.py --webots webots_quadX.wbt

Needs pymavlink and numpy, and an arducopter binary:

    ./waf configure --board sitl && ./waf copter
"""

import argparse
import math
import os
import subprocess
import sys
import tempfile
import time

from mock_webots import PROFILES
from mock_webots import G
from pymavlink import mavutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..', '..'))
BINARY = os.path.join(ROOT, 'build', 'sitl', 'bin', 'arducopter')

# Arming checks and the auto-disarm timer are off because this harness talks
# MAVLink on wall-clock time while SITL runs on Webots' clock; EKF3 is forced on
# so the estimator is actually exercised rather than bypassed.
TEST_DEFAULTS = """
ARMING_SKIPCHK 127
DISARM_DELAY 0
AHRS_EKF_TYPE 3
EK3_ENABLE 1
"""

# what a healthy flight looks like, from the recalibrated webots_quadX.wbt
LIMITS = {
    'tilt_rms_deg': 0.5,
    'yawrate_rms_dps': 1.0,
    'alt_rms_err_m': 0.1,
    'step_overshoot_m': 2.0,
    'step_final_err_m': 0.5,
}

# how far the reported hover rotor speed may be from the profile's
HOVER_RPM_TOLERANCE = 0.10


def expected_hover_rpm(profile):
    """Rotor speed at which four rotors carry the profile's weight."""
    p = PROFILES[profile]
    omega = math.sqrt(p['mass'] * G / 4 / p['kT'])
    return omega * 60 / (2 * math.pi)


def wait_for(m, predicate, types, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type=types, blocking=True, timeout=2)
        if msg is not None and predicate(msg):
            return True
    return False


def start_simulator(args, workdir):
    """Start either the mock or real Webots.  Returns (process, logfile)."""
    log = open(os.path.join(workdir, 'sim.log'), 'w')

    if args.webots:
        env = dict(os.environ)
        env.setdefault('WEBOTS_HOME', '/usr/local/webots')
        proc = subprocess.Popen(
            ['webots', '--batch', '--mode=realtime', '--no-rendering',
             '--stdout', '--stderr', '--minimize', 'worlds/' + args.webots],
            cwd=os.path.join(HERE, '..'), env=env,
            stdout=log, stderr=subprocess.STDOUT)
        # a first run downloads the EXTERNPROTO assets, which is slow
        time.sleep(args.webots_startup)
        return proc, log

    cmd = [sys.executable, os.path.join(HERE, 'mock_webots.py'),
           '--proto', 'json-tcp', '--profile', args.profile,
           '--port', str(args.port), '--max-speed', '1.0']
    if args.lockstep:
        cmd.append('--lockstep')
    if args.expect_no_rpm:
        cmd.append('--no-rpm')
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
    time.sleep(1.0)
    return proc, log


def run(args, workdir):
    port = args.port
    mock, mock_log = start_simulator(args, workdir)

    defaults = os.path.join(workdir, 'test.parm')
    open(defaults, 'w').write(TEST_DEFAULTS)

    sitl_log = open(os.path.join(workdir, 'sitl.log'), 'w')
    sitl = subprocess.Popen(
        [BINARY,
         '--model', 'webots-quad:127.0.0.1:%d' % port,
         '--home', '-35.363261,149.165230,0,0',
         '--defaults', ','.join([
             os.path.join(ROOT, 'Tools/autotest/default_params/copter.parm'),
             os.path.join(HERE, '..', 'quadX.parm'),
             defaults]),
         '--serial0', 'tcp:0', '-I', str(args.instance)],
        cwd=workdir, stdout=sitl_log, stderr=subprocess.STDOUT)

    res = {}
    try:
        m = None
        for _ in range(60):
            try:
                m = mavutil.mavlink_connection('tcp:127.0.0.1:%d' % (5760 + 10 * args.instance))
                break
            except OSError:
                time.sleep(0.5)
        if m is None:
            res['error'] = 'no mavlink connection'
            return res

        m.wait_heartbeat(timeout=30)
        print('heartbeat ok', flush=True)
        m.mav.request_data_stream_send(m.target_system, m.target_component,
                                       mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                226, 100000, 0, 0, 0, 0, 0)

        need = (mavutil.mavlink.EKF_ATTITUDE | mavutil.mavlink.EKF_VELOCITY_HORIZ |
                mavutil.mavlink.EKF_POS_HORIZ_REL | mavutil.mavlink.EKF_PRED_POS_HORIZ_REL)
        # real Webots runs slower than wall-clock and SITL's clock follows it,
        # so give the estimator more wall time to reach the same sim time
        print('waiting for EKF...', flush=True)
        res['ekf_ready'] = wait_for(m, lambda x: x.flags & need == need,
                                    'EKF_STATUS_REPORT',
                                    150 if args.webots else 60)
        print('ekf_ready=%s' % res['ekf_ready'], flush=True)

        m.set_mode_apm('GUIDED')
        time.sleep(1)

        # arm and take off; retry because the two clocks are not synchronised
        peak = 0.0
        reached = False
        last_cmd = 0.0
        t0 = time.time()
        rpm = []
        while time.time() - t0 < 90 and not reached:
            if time.time() - last_cmd > 3.0:
                m.mav.command_long_send(m.target_system, m.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                        0, 1, 0, 0, 0, 0, 0, 0)
                time.sleep(0.4)
                m.mav.command_long_send(m.target_system, m.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                        0, 0, 0, 0, 0, 0, 10)
                last_cmd = time.time()
            msg = m.recv_match(type=['GLOBAL_POSITION_INT', 'RPM'],
                               blocking=True, timeout=2)
            if msg is None:
                continue
            if msg.get_type() == 'RPM':
                rpm.append(msg.rpm1)
            else:
                peak = max(peak, msg.relative_alt / 1000.0)
                reached = peak > 9.5
        res['reached_10m'] = reached
        print('takeoff reached=%s peak=%.2f' % (reached, peak), flush=True)
        if not reached:
            return res

        # let the climb settle, then score 20 s of hold
        time.sleep(12)
        t0 = time.time()
        tilt, yawrate, alts = [], [], []
        while time.time() - t0 < 20:
            msg = m.recv_match(type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'RPM'],
                               blocking=True, timeout=3)
            if msg is None:
                continue
            if msg.get_type() == 'ATTITUDE':
                tilt.append(math.degrees(math.hypot(msg.roll, msg.pitch)))
                yawrate.append(math.degrees(abs(msg.yawspeed)))
            elif msg.get_type() == 'RPM':
                rpm.append(msg.rpm1)
            else:
                alts.append(msg.relative_alt / 1000.0)

        res['tilt_rms_deg'] = round(math.sqrt(sum(x * x for x in tilt) / len(tilt)), 3)
        res['yawrate_rms_dps'] = round(math.sqrt(sum(x * x for x in yawrate) / len(yawrate)), 3)
        mean = sum(alts) / len(alts)
        res['alt_rms_err_m'] = round(math.sqrt(sum((a - mean) ** 2 for a in alts) / len(alts)), 4)
        res['hover_rpm'] = round(sum(rpm[-20:]) / max(1, len(rpm[-20:])), 1) if rpm else 0.0

        # Webots does not necessarily keep up with wall-clock time (about 0.62x
        # real time for webots_quadX.wbt here), and SITL takes its clock from
        # the simulator, so scale the manoeuvre budget by the measured ratio.
        a = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        wall_a = time.time()
        t_probe = time.time()
        while time.time() - t_probe < 8:
            m.recv_match(blocking=True, timeout=1)
        b = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        ratio = 1.0
        if a is not None and b is not None and time.time() > wall_a:
            ratio = ((b.time_boot_ms - a.time_boot_ms) / 1000.0) / (time.time() - wall_a)
        res['sim_per_wall'] = round(ratio, 3)
        print('sim/wall=%.3f, step budget %.0f s' % (ratio, 40.0 / max(0.05, ratio)), flush=True)
        budget = 40.0 / max(0.05, ratio)

        # 20 m step north
        m.mav.set_position_target_local_ned_send(
            0, m.target_system, m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000,
            20, 0, -10, 0, 0, 0, 0, 0, 0, 0, 0)
        t0 = time.time()
        north = []
        last_cmd = time.time()
        while time.time() - t0 < budget:
            if time.time() - last_cmd > 2.0:
                m.mav.set_position_target_local_ned_send(
                    0, m.target_system, m.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000,
                    20, 0, -10, 0, 0, 0, 0, 0, 0, 0, 0)
                last_cmd = time.time()
            msg = m.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=3)
            if msg is not None:
                north.append(msg.x)
        if north:
            res['step_overshoot_m'] = round(max(0.0, max(north) - 20.0), 2)
            res['step_final_err_m'] = round(abs(north[-1] - 20.0), 2)
    finally:
        for p in (sitl, mock):
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        mock_log.close()
        sitl_log.close()
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', default='quadx', choices=['quadx', 'quadx-legacy'])
    ap.add_argument('--port', type=int, default=5577)
    ap.add_argument('--lockstep', action='store_true',
                    help='mock blocks for the servo frame instead of re-sending sensors')
    ap.add_argument('--webots', metavar='WORLD',
                    help='run the real simulator on this world instead of the mock, '
                         'e.g. webots_quadX.wbt')
    ap.add_argument('--webots-startup', type=float, default=25.0,
                    help='seconds to let Webots load the world before connecting')
    ap.add_argument('--keep', action='store_true', help='keep the log directory')
    ap.add_argument('--instance', type=int, default=0,
                    help='SITL instance, so the test can run beside another SITL: '
                    'MAVLink on TCP 5760 + 10 * INSTANCE')
    ap.add_argument('--expect-no-rpm', action='store_true',
                    help='the simulator sends no "rpm" key, like an older controller '
                    '(the mock is told to leave it out): check that none is reported '
                    'instead of checking hover rpm')
    args = ap.parse_args()

    if not os.path.exists(BINARY):
        print('no arducopter binary at %s\n'
              'build one with: ./waf configure --board sitl && ./waf copter' % BINARY)
        return 2

    workdir = tempfile.mkdtemp(prefix='webots-flight-test-')
    print('logs: %s' % workdir, flush=True)
    try:
        res = run(args, workdir)
    finally:
        if args.keep:
            print('logs in %s' % workdir)

    print()
    for k, v in res.items():
        print('%-22s %s' % (k, v))

    if not res.get('reached_10m'):
        print('\nFAIL: did not reach takeoff altitude')
        return 1

    failures = ['%s = %s, limit %s' % (k, res[k], lim) for k, lim in LIMITS.items()
                if k in res and res[k] > lim]

    # the rotor speeds the simulator reports must reach AP_RPM
    hover_rpm = res.get('hover_rpm', 0.0)
    if args.expect_no_rpm:
        if hover_rpm != 0.0:
            failures.append('hover_rpm = %s, expected none' % hover_rpm)
    else:
        want = expected_hover_rpm(args.profile)
        if abs(hover_rpm - want) > HOVER_RPM_TOLERANCE * want:
            failures.append('hover_rpm = %s, expected %.0f +- %.0f%%'
                            % (hover_rpm, want, 100 * HOVER_RPM_TOLERANCE))

    if failures:
        print()
        for f in failures:
            print('FAIL: ' + f)
        return 1

    print('\nPASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
