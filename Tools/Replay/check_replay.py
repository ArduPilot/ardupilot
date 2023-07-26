#!/usr/bin/env python

'''
check that replay produced identical results
'''

from __future__ import print_function

def check_log(logfile, progress=print, ekf2_only=False, ekf3_only=False, verbose=False, accuracy=0.0, ignores=set()):
    '''check replay log for matching output'''
    from pymavlink import mavutil
    progress("Processing log %s" % logfile)
    failure = 0
    errors = 0
    count = 0
    base_count = 0
    counts = {}
    base_counts = {}

    mlog = mavutil.mavlink_connection(logfile)

    ek2_list = ['NKF1','NKF2','NKF3','NKF4','NKF5','NKF0','NKQ', 'NKY0', 'NKY1']
    ek3_list = ['XKF1','XKF2','XKF3','XKF4','XKF0','XKFS','XKQ','XKFD','XKV1','XKV2','XKY0','XKY1']
    
    if ekf2_only:
        mlist = ek2_list
    elif ekf3_only:
        mlist = ek3_list
    else:
        mlist = ek2_list + ek3_list

    base = {}
    for m in mlist:
        base[m] = {}

    while True:
        m = mlog.recv_match(type=mlist)
        if m is None:
            break
        if not hasattr(m,'C'):
            continue
        mtype = m.get_type()
        if not mtype in counts:
            counts[mtype] = 0
            base_counts[mtype] = 0
        core = m.C
        if core < 100:
            base[mtype][core] = m
            base_count += 1
            base_counts[mtype] += 1
            continue
        mb = base[mtype][core-100]
        count += 1
        counts[mtype] += 1
        mismatch = False
        for f in m._fieldnames:
            if f == 'C':
                continue
            if "%s.%s" % (mtype, f) in ignores:
                continue
            v1 = getattr(m,f)
            v2 = getattr(mb,f)
            ok = v1 == v2
            if not ok and accuracy > 0:
                avg = (v1+v2)*0.5
                margin = accuracy*0.01*avg
                if abs(v1-v2) <= abs(margin):
                    ok = True
            if not ok:
                mismatch = True
                errors += 1
                progress("Mismatch in field %s.%s: %s %s" % (mtype, f, str(v1), str(v2)))
        if mismatch:
            progress(mb)
            progress(m)
    progress("Processed %u/%u messages, %u errors" % (count, base_count, errors))
    if verbose:
        for mtype in counts.keys():
            progress("%s %u/%u %d" % (mtype, counts[mtype], base_counts[mtype], base_counts[mtype]-counts[mtype]))
    count_delta = abs(count - base_count)
    if count == 0 or count_delta > 100:
        progress("count=%u count_delta=%u" % (count, count_delta))
        failure += 1
    if failure != 0 or errors != 0:
        return False
    return True

if __name__ == '__main__':
    import sys
    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--ekf2-only", action='store_true', help="only check EKF2")
    parser.add_argument("--ekf3-only", action='store_true', help="only check EKF3")
    parser.add_argument("--verbose", action='store_true', help="verbose output")
    parser.add_argument("--accuracy", type=float, default=0.0, help="accuracy percentage for match")
    parser.add_argument("--ignore-field", action='append', default=[], help="ignore message field when comparing")
    parser.add_argument("logs", metavar="LOG", nargs="+")

    args = parser.parse_args()

    failed = False
    for filename in args.logs:
        if not check_log(filename, print, args.ekf2_only, args.ekf3_only, args.verbose, accuracy=args.accuracy, ignores=args.ignore_field):
            failed = True

    if failed:
        print("FAILED")
        sys.exit(1)
    print("Passed")
    sys.exit(0)
