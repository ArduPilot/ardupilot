#!/usr/bin/env python3
"""Everything the mavlink-and-crash-analysis skill wants, from one pass over each log.

    Tools/scripts/zephyr_crash_report_data.py flight_zephyr.BIN ZEPHYR flight_chibios.BIN CHIBIOS

Every field is checked against the message's fieldnames before use."""
import collections
import math
import statistics
import sys

from pymavlink import mavutil


def ts(us):  # log TimeUS -> [HH:MM:SS.mmm]
    s = us / 1e6
    h = int(s // 3600)
    m = int((s % 3600) // 60)
    sec = s % 60
    return '[%02d:%02d:%06.3f]' % (h, m, sec)


def has(msg, *f):
    return all(x in msg.get_fieldnames() for x in f)


def collect(path):
    """One pass over the log. Returns every series the report sections need."""
    m = mavutil.mavlink_connection(path)
    d = {
        'ver': None, 'msgs': [], 'errs': [], 'evs': [], 'modes': [],
        'att': [], 'ctun': [], 'vibe': collections.defaultdict(list), 'clip': {},
        'rcou': [], 'bat': [],
        'xkf4': collections.defaultdict(list), 'xkf3': collections.defaultdict(list),
        'mag': [], 'pm': [], 'gps': [], 'pos': [],
        't_first': None, 't_last': None, 'fieldsets': {},
    }
    while True:
        msg = m.recv_match()
        if msg is None:
            break
        t = msg.get_type()
        if t not in d['fieldsets']:
            d['fieldsets'][t] = msg.get_fieldnames()
        tu = getattr(msg, 'TimeUS', None)
        if tu is not None:
            if d['t_first'] is None:
                d['t_first'] = tu
            d['t_last'] = tu
        if t == 'VER' and d['ver'] is None and has(msg, 'FWS'):
            d['ver'] = msg.FWS
        elif t == 'MSG' and has(msg, 'Message'):
            d['msgs'].append((tu, msg.Message))
            if d['ver'] is None and ('ArduCopter' in msg.Message or 'Copter' in msg.Message):
                d['ver'] = msg.Message
        elif t == 'ERR' and has(msg, 'Subsys', 'ECode'):
            d['errs'].append((tu, msg.Subsys, msg.ECode))
        elif t == 'EV' and has(msg, 'Id'):
            d['evs'].append((tu, msg.Id))
        elif t == 'MODE' and has(msg, 'Mode', 'ModeNum'):
            d['modes'].append((tu, msg.Mode, msg.ModeNum))
        elif t == 'ATT' and has(msg, 'Roll', 'DesRoll', 'Pitch', 'DesPitch', 'Yaw', 'DesYaw'):
            d['att'].append((tu, msg.Roll, msg.DesRoll, msg.Pitch, msg.DesPitch))
        elif t == 'CTUN' and has(msg, 'Alt', 'DAlt', 'CRt', 'DCRt'):
            d['ctun'].append((tu, msg.Alt, msg.DAlt, msg.CRt, msg.DCRt))
        elif t == 'VIBE' and has(msg, 'VibeX', 'VibeY', 'VibeZ'):
            inst = msg.IMU if has(msg, 'IMU') else 0
            d['vibe'][inst].append((msg.VibeX, msg.VibeY, msg.VibeZ))
            if has(msg, 'Clip'):
                d['clip'][inst] = msg.Clip
        elif t == 'RCOU' and has(msg, 'C1', 'C2', 'C3', 'C4'):
            d['rcou'].append((tu, msg.C1, msg.C2, msg.C3, msg.C4))
        elif t == 'BAT' and has(msg, 'Volt', 'Curr'):
            d['bat'].append((tu, msg.Volt, msg.Curr,
                             msg.Res if has(msg, 'Res') else None,
                             msg.VoltR if has(msg, 'VoltR') else None))
        elif t == 'XKF4' and has(msg, 'SV', 'SP', 'SH', 'SM', 'FS', 'SS', 'GPS', 'PI'):
            c = msg.C if has(msg, 'C') else 0
            d['xkf4'][c].append((tu, msg.SV, msg.SP, msg.SH, msg.SM,
                                 msg.FS, msg.SS, msg.GPS, msg.PI))
        elif t == 'XKF3' and has(msg, 'IVN', 'IVE', 'IVD', 'IPN', 'IPE', 'IPD',
                                 'IMX', 'IMY', 'IMZ'):
            c = msg.C if has(msg, 'C') else 0
            d['xkf3'][c].append((tu, msg.IVN, msg.IVE, msg.IVD,
                                 msg.IPN, msg.IPE, msg.IPD,
                                 msg.IMX, msg.IMY, msg.IMZ))
        elif t == 'MAG' and has(msg, 'MagX', 'MagY', 'MagZ') and (not has(msg, 'I') or msg.I == 0):
            d['mag'].append((tu, math.sqrt(msg.MagX**2 + msg.MagY**2 + msg.MagZ**2)))
        elif t == 'PM' and has(msg, 'LR', 'NLon', 'MaxT', 'Mem', 'Load'):
            d['pm'].append((tu, msg.LR, msg.NLon, msg.MaxT, msg.Mem, msg.Load,
                            msg.Ex if has(msg, 'Ex') else None,
                            msg.ErC if has(msg, 'ErC') else None,
                            msg.InE if has(msg, 'InE') else None,  # codespell:ignore
                            msg.SPIC if has(msg, 'SPIC') else None,
                            msg.I2CC if has(msg, 'I2CC') else None))
        elif t == 'GPS' and has(msg, 'Status', 'Lat', 'Lng', 'Spd') and (not has(msg, 'I') or msg.I == 0):
            d['gps'].append((tu, msg.Status, msg.Lat, msg.Lng, msg.Spd,
                             msg.NSats if has(msg, 'NSats') else None))
        elif t == 'POS' and has(msg, 'Lat', 'Lng', 'Alt'):
            d['pos'].append((tu, msg.Lat, msg.Lng, msg.Alt))
    return d


def report_header(d, path, label):
    print('=' * 78)
    print('%s   %s' % (label, path))
    print('=' * 78)
    print('version           : %s' % d['ver'])
    print('log span          : %s -> %s  (%.1f s of TimeUS)' % (
        ts(d['t_first']), ts(d['t_last']), (d['t_last'] - d['t_first']) / 1e6))
    print('VIBE fieldnames   : %s' % d['fieldsets'].get('VIBE'))
    print('PM fieldnames     : %s' % d['fieldsets'].get('PM'))
    print('BAT fieldnames    : %s' % d['fieldsets'].get('BAT'))


def report_event_horizon(d):
    print('\n-- 1. EVENT HORIZON (log TimeUS) --')
    print('  ERR: %d records' % len(d['errs']))
    for tu, sub, code in d['errs'][:20]:
        print('    %s ERR Subsys=%d ECode=%d' % (ts(tu), sub, code))
    print('  EV: %d records' % len(d['evs']))
    for tu, i in d['evs'][:24]:
        print('    %s EV Id=%d' % (ts(tu), i))
    print('  MODE changes: %d' % len(d['modes']))
    for tu, mode, num in d['modes'][:16]:
        print('    %s MODE %s (%d)' % (ts(tu), mode, num))
    print('  MSG: %d records; those mentioning EKF/failsafe/vibr/glitch/yaw/land/desync:'
          % len(d['msgs']))
    keys = ('ekf', 'failsafe', 'vibra', 'glitch', 'yaw', 'land', 'desync',
            'error', 'fault', 'variance', 'arm')
    for tu, s in d['msgs']:
        sl = s.lower()
        if any(k in sl for k in keys):
            print('    %s %s' % (ts(tu), s))


def report_control_tracking(d):
    print('\n-- 2. CONTROL TRACKING --')
    att = d['att']
    if att:
        er = sorted(abs(a[1] - a[2]) for a in att)
        ep = sorted(abs(a[3] - a[4]) for a in att)
        n = len(er)
        print('  ATT roll  |actual-desired| deg: median %.2f  p95 %.2f  max %.2f  at %s' % (
            er[n // 2], er[int(.95 * n)], er[-1],
            ts(max(att, key=lambda a: abs(a[1] - a[2]))[0])))
        print('  ATT pitch |actual-desired| deg: median %.2f  p95 %.2f  max %.2f  at %s' % (
            ep[n // 2], ep[int(.95 * n)], ep[-1],
            ts(max(att, key=lambda a: abs(a[3] - a[4]))[0])))
    ctun = d['ctun']
    if ctun:
        ea = sorted(abs(c[1] - c[2]) for c in ctun)
        n = len(ea)
        crt = [c[3] for c in ctun]
        print('  CTUN |Alt-DAlt| m: median %.2f  p95 %.2f  max %.2f  at %s' % (
            ea[n // 2], ea[int(.95 * n)], ea[-1],
            ts(max(ctun, key=lambda c: abs(c[1] - c[2]))[0])))
        print('  CTUN climb rate cm/s: median %.1f  sd %.1f  min %.1f  max %.1f' % (
            statistics.median(crt), statistics.pstdev(crt), min(crt), max(crt)))
        highest = max(ctun, key=lambda c: c[1])
        print('  CTUN max Alt %.2f m at %s' % (highest[1], ts(highest[0])))


def report_vibration(d):
    print('\n-- 3. VIBRATION (per VIBE.IMU instance, m/s^2) --')
    for inst in sorted(d['vibe']):
        v = d['vibe'][inst]
        print('  IMU%d n=%d  mean X %.3f Y %.3f Z %.3f | max X %.2f Y %.2f Z %.2f | Clip=%s' % (
            inst, len(v),
            statistics.mean(r[0] for r in v),
            statistics.mean(r[1] for r in v),
            statistics.mean(r[2] for r in v),
            max(r[0] for r in v), max(r[1] for r in v), max(r[2] for r in v),
            d['clip'].get(inst, 'n/a')))


def report_power(d):
    print('\n-- 4. POWER --')
    bat = d['bat']
    if not bat:
        print('  no BAT records - cannot assess power')
        return
    v = [b[1] for b in bat]
    c = [b[2] for b in bat]
    res = [b[3] for b in bat if b[3] is not None]
    vr = [b[4] for b in bat if b[4] is not None]
    print('  BAT n=%d Volt min %.2f max %.2f V | Curr min %.2f max %.2f A | Res %s | VoltR %s' % (
        len(bat), min(v), max(v), min(c), max(c),
        ('%.4f..%.4f ohm' % (min(res), max(res))) if res else 'n/a',
        ('%.2f..%.2f V' % (min(vr), max(vr))) if vr else 'n/a'))
    print('  min Volt at %s' % ts(min(bat, key=lambda b: b[1])[0]))


def report_motors(d):
    print('\n-- 2A. MOTOR OUTPUTS (RCOU C1-C4, PWM) --')
    rcou = d['rcou']
    if not rcou:
        print('  no RCOU records - cannot assess motors')
        return
    for k in range(4):
        col = [r[k + 1] for r in rcou]
        print('  C%d min %d max %d mean %.0f' % (k + 1, min(col), max(col), statistics.mean(col)))
    sp = [(r[0], max(r[1:5]) - min(r[1:5])) for r in rcou]
    worst = max(sp, key=lambda x: x[1])
    print('  max motor spread (max-min PWM) %d at %s; samples with spread>600: %d of %d' % (
        worst[1], ts(worst[0]), sum(1 for x in sp if x[1] > 600), len(sp)))


def report_ekf(d):
    print('\n-- 2C. EKF3 TEST RATIOS (XKF4, per core; >1.0 = innovation failed its gate) --')
    for c in sorted(d['xkf4']):
        rows = d['xkf4'][c]

        def stat(i, rows=rows):
            col = [r[i] for r in rows]
            return statistics.median(col), max(col), ts(max(rows, key=lambda r: r[i])[0])

        sv, sp_, sh, sm = stat(1), stat(2), stat(3), stat(4)
        over = sum(1 for r in rows if r[2] > 1.0)
        first_over = ts(next((r[0] for r in rows if r[2] > 1.0), rows[0][0])) if over else 'n/a'
        print('  core %d n=%d  SV med %.3f max %.3f'
              ' | SP med %.3f max %.3f (>1.0 in %d rows, first at %s)'
              ' | SH med %.3f max %.3f | SM med %.3f max %.3f | PI %s' % (
                  c, len(rows), sv[0], sv[1], sp_[0], sp_[1], over, first_over,
                  sh[0], sh[1], sm[0], sm[1], sorted({r[8] for r in rows})))
        print('           FS values %s | SS values %s | GPS status values %s' % (
            sorted({r[5] for r in rows}),
            sorted({r[6] for r in rows}),
            sorted({r[7] for r in rows})))
    print('  XKF3 raw innovations, core 0:')
    if 0 in d['xkf3']:
        rows = d['xkf3'][0]
        for name, i in (('IVN', 1), ('IVE', 2), ('IVD', 3),
                        ('IPN', 4), ('IPE', 5), ('IPD', 6),
                        ('IMX', 7), ('IMY', 8), ('IMZ', 9)):
            col = [r[i] for r in rows]
            print('    %s min %+.3f max %+.3f' % (name, min(col), max(col)), end='')
        print()


def report_compass_vs_current(d):
    print('\n-- 2C. COMPASS vs CURRENT --')
    mag, bat = d['mag'], d['bat']
    if not (mag and bat and len(bat) > 5):
        print('  insufficient MAG/BAT data')
        return
    # pair |mag| with nearest BAT.Curr, correlate
    bi = 0
    pairs = []
    for tu, mm in mag:
        while bi + 1 < len(bat) and bat[bi + 1][0] <= tu:
            bi += 1
        pairs.append((mm, bat[bi][2]))
    xs = [p[1] for p in pairs]
    ys = [p[0] for p in pairs]
    if not (statistics.pstdev(xs) > 0 and statistics.pstdev(ys) > 0):
        print('  current or field constant - correlation undefined (Curr sd %.3f)'
              % statistics.pstdev(xs))
        return
    mx, my = statistics.mean(xs), statistics.mean(ys)
    r = (sum((x - mx) * (y - my) for x, y in pairs)
         / (len(pairs) * statistics.pstdev(xs) * statistics.pstdev(ys)))
    print('  |MAG| vs BAT.Curr: r = %+.3f over %d pairs (|r|>0.7 = EMI suspect);'
          ' |MAG| mean %.1f sd %.1f' % (r, len(pairs), my, statistics.pstdev(ys)))


def report_scheduler(d):
    print('\n-- 2E. SCHEDULER (PM) --')
    pm = d['pm']
    if not pm:
        return
    lr = [p[1] for p in pm]
    nl = [p[2] for p in pm]
    mt = [p[3] for p in pm]
    mem = [p[4] for p in pm]
    ld = [p[5] for p in pm]
    ex = [p[6] for p in pm if p[6] is not None]
    erc = [p[7] for p in pm if p[7] is not None]
    internal_err = [p[8] for p in pm if p[8] is not None]
    spic = [p[9] for p in pm if p[9] is not None]
    i2cc = [p[10] for p in pm if p[10] is not None]
    period = 1e6 / statistics.median(lr) if statistics.median(lr) else float('nan')
    print('  n=%d  LR %s Hz (period %.0f us)'
          ' | MaxT med %.0f max %.0f us (= %.0f%% of period) at %s'
          ' | NLon total %d | Load med %.0f max %.0f /1000 | Mem min %d' % (
              len(pm), sorted(set(lr)), period,
              statistics.median(mt), max(mt), 100 * max(mt) / period,
              ts(max(pm, key=lambda p: p[3])[0]),
              sum(nl), statistics.median(ld), max(ld), min(mem)))
    print('  Ex %s | ErC %s | InE %s | SPIC/msg %s | I2CC/msg %s' % (  # codespell:ignore
        ('max %d' % max(ex)) if ex else 'n/a',
        ('max %d' % max(erc)) if erc else 'n/a',
        ('max 0x%x' % max(internal_err)) if internal_err else 'n/a',
        ('med %.0f' % statistics.median(spic)) if spic else 'n/a',
        ('med %.0f' % statistics.median(i2cc)) if i2cc else 'n/a'))


def report_acceptance(d):
    print('\n-- ACCEPTANCE: landing offset from first 3D-fix position --')
    if not d['gps']:
        print()
        return
    fix = [g for g in d['gps'] if g[1] >= 3]
    if fix:
        # pymavlink's DFReader already applies the 1e7 multiplier: Lat/Lng
        # arrive in degrees. Dividing again once produced a false "0.0 m".
        lat0, lng0 = fix[0][2], fix[0][3]
        lat1, lng1 = fix[-1][2], fix[-1][3]
        dn = (lat1 - lat0) * 111320.0
        de = (lng1 - lng0) * 111320.0 * math.cos(math.radians(lat0))
        print('  first fix %s  last fix %s  offset %.1f m (N %+.1f E %+.1f)'
              ' | max GPS Spd %.2f m/s | NSats %s' % (
                  ts(fix[0][0]), ts(fix[-1][0]), math.hypot(dn, de), dn, de,
                  max(g[4] for g in fix),
                  sorted({g[5] for g in fix if g[5] is not None})))
    print()


def analyse(path, label):
    d = collect(path)
    report_header(d, path, label)
    report_event_horizon(d)
    report_control_tracking(d)
    report_vibration(d)
    report_power(d)
    report_motors(d)
    report_ekf(d)
    report_compass_vs_current(d)
    report_scheduler(d)
    report_acceptance(d)


for i in range(1, len(sys.argv), 2):
    analyse(sys.argv[i], sys.argv[i + 1])
