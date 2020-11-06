#!/usr/bin/env python

'''
check that replay produced identical results
'''

import time
import sys

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--condition", default=None, help="condition for packets")
parser.add_argument("--ekf2-only", action='store_true', help="only check EKF2")
parser.add_argument("--ekf3-only", action='store_true', help="only check EKF3")
parser.add_argument("--verbose", action='store_true', help="verbose output")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil

failure = 0
errors = 0

def check_log(logfile):
    '''check replay log for matching output'''
    print("Processing log %s" % filename)
    count = 0
    base_count = 0
    counts = {}
    base_counts = {}
    global errors, failure

    mlog = mavutil.mavlink_connection(filename)

    ek2_list = ['NKF1','NKF2','NKF3','NKF4','NKF5','NKF0','NKQ']
    ek3_list = ['XKF1','XKF2','XKF3','XKF4','XKF0','XKFS','XKQ','XKFD','XKV1','XKV2']
    
    if args.ekf2_only:
        mlist = ek2_list
    elif args.ekf3_only:
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
            v1 = getattr(m,f)
            v2 = getattr(mb,f)
            if v1 != v2:
                mismatch = True
                errors += 1
                print("Mismatch in field %s.%s: %s %s" % (mtype, f, str(v1), str(v2)))
        if mismatch:
            print(mb)
            print(m)
    print("Processed %u/%u messages, %u errors" % (count, base_count, errors))
    if args.verbose:
        for mtype in counts.keys():
            print("%s %u/%u %d" % (mtype, counts[mtype], base_counts[mtype], base_counts[mtype]-counts[mtype]))
    if count == 0 or count != base_count:
        failure += 1

for filename in args.logs:
    check_log(filename)

if failure != 0 or errors != 0:
    print("FAILED: %u %u" % (failure, errors))
    sys.exit(1)
print("Passed")
sys.exit(0)
