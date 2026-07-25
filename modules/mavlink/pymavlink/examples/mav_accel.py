#!/usr/bin/env python3

'''
show accel calibration for a set of logs
'''
from argparse import ArgumentParser
parser = ArgumentParser()
parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--planner", action='store_true', help="use planner file format")
parser.add_argument("--robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil


def process(logfile):
    '''display accel cal for a log file'''
    mlog = mavutil.mavlink_connection(filename,
                                      planner_format=args.planner,
                                      notimestamps=args.notimestamps,
                                      robust_parsing=args.robust)

    m = mlog.recv_match(type='SENSOR_OFFSETS')
    if m is not None:
        z_sensor = (m.accel_cal_z - 9.805) * (4096/9.81)
        print("accel cal %5.2f %5.2f %5.2f %6u  %s" % (
            m.accel_cal_x, m.accel_cal_y, m.accel_cal_z,
            z_sensor,
            logfile))


total = 0.0
for filename in args.logs:
    process(filename)
