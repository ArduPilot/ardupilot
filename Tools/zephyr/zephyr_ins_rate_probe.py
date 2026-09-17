#!/usr/bin/env python3
'''
Compare two ArduPilot dataflash logs for the INS sample-rate fault that made
CubeOrangeZephyr fly away under Renode. The mechanism is described in
libraries/AP_HAL_Zephyr/ARCHITECTURAL.md, "Boot time is a correctness
property".

    Tools/zephyr/zephyr_ins_rate_probe.py zephyr.BIN chibios.BIN

Four checks, each printing its own verdict:

1. IMU.GHz slope per instance. The EKF's dt is 1/_gyro_raw_sample_rates[i]
   (AP_InertialSensor_Backend.cpp:330), an estimate re-fitted once a second by
   _update_sensor_rate() (:82-109). While millis64() < 30000 and disarmed it
   may move ~20 %/s; after that at most 0.1 %/s. An instance whose GHz moves at
   exactly +/-0.1 %/s for a whole flight never got its fast window, and its
   starting value back-extrapolates to the compiled-in nominal (8000 for the
   Invensense v1 driver with fast sampling, 9000 for v2). One that has
   converged moves an order of magnitude slower.

2. AP clock against GPS time. Regressing TimeUS on GPS.GMS measures the guest
   clock directly. A slope away from 1.0 would shut the 30 s window early and
   give the same symptom as a late boot - so this has to be exact before the
   late-boot explanation can be believed.

3. Accelerometer scale, binned by tilt over the whole log. A pure scale error
   holds its ratio to G at every tilt. Averaging a "pre-arm window" instead
   catches different flight phases in the two logs and produced a false 1.15 %
   finding once; do not do that.

4. VIBE and sample-to-sample accel jumps, for context. Elevated on the faulty
   log, but as a consequence of the wrong dt feeding the filters, not a cause.
AP_FLAKE8_CLEAN
'''

import math
import statistics
import sys

from pymavlink import mavutil

G = 9.80665
CONVERGE_RAIL_PCT_PER_S = 0.1
NOMINALS = (8000.0, 9000.0, 16000.0, 1125.0, 1000.0)


def load(path):
    m = mavutil.mavlink_connection(path)
    imu, ghz, att, gps, vibe = {}, {}, [], [], []
    while True:
        msg = m.recv_match(type=['IMU', 'ATT', 'GPS', 'VIBE'])
        if msg is None:
            break
        t = msg.get_type()
        if t == 'IMU':
            i = getattr(msg, 'I', 0)
            imu.setdefault(i, []).append(
                (msg.TimeUS * 1e-6, msg.AccX, msg.AccY, msg.AccZ))
            ghz.setdefault(i, []).append((msg.TimeUS * 1e-6, msg.GHz))
        elif t == 'ATT':
            att.append((msg.TimeUS * 1e-6, msg.Roll, msg.Pitch))
        elif t == 'GPS':
            if getattr(msg, 'Status', 0) >= 3 and getattr(msg, 'GMS', 0) > 0:
                gps.append((msg.TimeUS * 1e-6, msg.GMS * 1e-3))
        elif t == 'VIBE':
            vibe.append((msg.VibeX, msg.VibeY, msg.VibeZ))
    return imu, ghz, att, gps, vibe


def check_rate_slope(label, ghz):
    print('--- 1. IMU.GHz slope (%s) ---' % label)
    for i in sorted(ghz):
        s = ghz[i]
        dt = s[-1][0] - s[0][0]
        if dt <= 0 or not s[0][1]:
            continue
        pct = 100.0 * (s[-1][1] / s[0][1] - 1.0) / dt
        railed = abs(abs(pct) - CONVERGE_RAIL_PCT_PER_S) < 0.02
        # A railed lane that still sits near a compiled-in nominal never had
        # its fast window. One that has already walked far from every nominal
        # got part of the window and is finishing the walk at the post-window
        # rate - the state ChibiOS's lanes 1/2 are in on this rig too.
        near_nominal = any(abs(s[0][1] / nom - 1.0) < 0.15 for nom in NOMINALS)
        if railed and near_nominal:
            verdict = 'ON THE 0.1 %/s RAIL from a nominal - never got a fast window'
        elif railed:
            verdict = 'on the 0.1 %/s rail, far from any nominal - partly converged, window was cut short'
        else:
            verdict = 'converged or converging'
        print('  IMU%d  GHz %8.1f -> %8.1f over %5.1f s = %+.4f %%/s   %s'
              % (i, s[0][1], s[-1][1], dt, pct, verdict))
        if railed and pct:
            for nom in NOMINALS:
                secs = math.log(s[0][1] / nom) / math.log(1 + pct / 100.0)
                if -5 < secs < 400:
                    print('        back-extrapolates to nominal %6.0f Hz at '
                          '%.1f s before the log starts' % (nom, secs))


def check_clock(label, gps):
    print('--- 2. AP clock vs GPS time (%s) ---' % label)
    if len(gps) < 20:
        print('  only %d usable GPS fixes' % len(gps))
        return
    t0, g0 = gps[0]
    xs = [p[0] - t0 for p in gps]
    ys = [p[1] - g0 for p in gps]
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    slope = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    print('  %d fixes over %.1f s: GPS seconds per AP second = %.6f  '
          '(AP clock %+.4f %% vs truth)'
          % (n, xs[-1], slope, 100 * (1 / slope - 1)))


def check_accel_scale(label, imu, att):
    print('--- 3. |accel| by tilt, IMU0 (%s) ---' % label)
    rows = imu.get(0, [])
    if not rows or not att:
        print('  no data')
        return

    def tilt_at(t):
        a = min(att, key=lambda x: abs(x[0] - t))
        c = math.cos(math.radians(a[1])) * math.cos(math.radians(a[2]))
        return math.degrees(math.acos(max(-1.0, min(1.0, c))))

    for lo, hi in ((0, 5), (5, 10), (10, 20), (20, 40)):
        sel = [r for r in rows if lo <= tilt_at(r[0]) < hi]
        if len(sel) < 20:
            continue
        mg = [math.sqrt(r[1] ** 2 + r[2] ** 2 + r[3] ** 2) for r in sel]
        print('  tilt %2d-%2d deg  n=%-5d |a|/G = %.5f'
              % (lo, hi, len(sel), statistics.mean(mg) / G))


def check_noise(label, imu, vibe):
    print('--- 4. vibration and sample jumps (%s) ---' % label)
    if vibe:
        print('  VIBE mean  X %.4f  Y %.4f  Z %.4f' % tuple(
            statistics.mean(v[k] for v in vibe) for k in range(3)))
    rows = imu.get(0, [])
    d = sorted(math.sqrt(sum((rows[i + 1][k] - rows[i][k]) ** 2
                             for k in (1, 2, 3)))
               for i in range(len(rows) - 1))
    if d:
        n = len(d)
        print('  |d(accel)| median %.4f  p99 %.4f  ratio %.1f'
              % (d[n // 2], d[int(0.99 * n)],
                 d[int(0.99 * n)] / d[n // 2] if d[n // 2] else float('nan')))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    for path in sys.argv[1:]:
        imu, ghz, att, gps, vibe = load(path)
        print('===== %s =====' % path)
        check_rate_slope(path, ghz)
        check_clock(path, gps)
        check_accel_scale(path, imu, att)
        check_noise(path, imu, vibe)
        print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
