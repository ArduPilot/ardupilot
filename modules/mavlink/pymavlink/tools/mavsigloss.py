#!/usr/bin/env python3

'''
show times when signal is lost
'''
import time

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--planner", action='store_true', help="use planner file format")
parser.add_argument("--robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_argument("--deltat", type=float, default=1.0, help="loss threshold in seconds")
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--types", default=None, help="types of messages (comma separated)")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil


def sigloss(logfile):
    '''work out signal loss times for a log file'''
    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename,
                                      planner_format=args.planner,
                                      notimestamps=args.notimestamps,
                                      robust_parsing=args.robust)

    last_t = 0

    types = args.types
    if types is not None:
        types = types.split(',')

    while True:
        m = mlog.recv_match(condition=args.condition)
        if m is None:
            return
        if types is not None and m.get_type() not in types:
            continue
        if args.notimestamps:
            if not 'usec' in m._fieldnames:
                continue
            t = m.usec / 1.0e6
        else:
            t = m._timestamp
        if last_t != 0:
            if t - last_t > args.deltat:
                print("Sig lost for %.1fs at %s" % (t-last_t, time.asctime(time.localtime(t))))
        last_t = t

total = 0.0
for filename in args.logs:
    sigloss(filename)
