#!/usr/bin/env python3

from argparse import ArgumentParser
parser = ArgumentParser()
parser.add_argument("log")

args = parser.parse_args()

from pymavlink import mavutil

def process(logfile):
    '''look for duplicate raw gyro samples'''
    mlog = mavutil.mavlink_connection(logfile)

    last_gyr = {}
    dup_count = {}
    total_dup = {}
    while True:
        m = mlog.recv_match(type='GYR')
        if m is None:
            break
        if not m.I in last_gyr:
            last_gyr[m.I] = m
            dup_count[m.I] = 0
            total_dup[m.I] = 0
            continue
        axes = 0
        if last_gyr[m.I].GyrX == m.GyrX and abs(m.GyrX) >= 1:
            axes |= 1
        if last_gyr[m.I].GyrY == m.GyrY and abs(m.GyrY) >= 1:
            axes |= 2
        if last_gyr[m.I].GyrX == m.GyrZ and abs(m.GyrZ) >= 1:
            axes |= 4
        if axes != 0:
            if dup_count[m.I] == 0:
                print("%s" % str(last_gyr[m.I]))
            dup_count[m.I] += 1
            total_dup[m.I] += 1
            print("%s dup=%u axes=%u" % (str(m), dup_count[m.I], axes))
        else:
            dup_count[m.I] = 0
        last_gyr[m.I] = m
    print(total_dup)

process(args.log)

