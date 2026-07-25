#!/usr/bin/env python3

'''
search a set of log files for a condition
'''
from pymavlink import mavutil

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--condition", default=None, help="conditional check on log")
parser.add_argument("--types", default=None, help="message types to look for (comma separated)")
parser.add_argument("--stop", action='store_true', help="stop when message type found")
parser.add_argument("--stopcondition", action='store_true', help="stop when condition met")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

def mavsearch(filename):
    print("Loading %s ..." % filename)
    mlog = mavutil.mavlink_connection(filename)
    if args.types is not None:
        types = args.types.split(',')
    else:
        types = None
    while True:
        m = mlog.recv_match(type=types)
        if m is None:
            break
        if mlog.check_condition(args.condition):
            print(m)
            if args.stopcondition:
                break
        if args.stop:
            break


for f in args.logs:
    mavsearch(f)
