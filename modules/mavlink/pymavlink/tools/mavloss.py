#!/usr/bin/env python3

'''
show MAVLink packet loss
'''
from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--planner", action='store_true', help="use planner file format")
parser.add_argument("--robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_argument("--condition", default=None, help="condition for packets")
parser.add_argument("--dialect", default="ardupilotmega", help="MAVLink dialect")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil


def mavloss(logfile):
    '''work out signal loss times for a log file'''
    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename,
                                      planner_format=args.planner,
                                      notimestamps=args.notimestamps,
                                      dialect=args.dialect,
                                      robust_parsing=args.robust)

    # Track the reasons for MAVLink parsing errors and print them all out at the end.
    reason_ids = set()
    reasons = []

    while True:
        m = mlog.recv_match(condition=args.condition)

        # Stop parsing the file once we've reached the end
        if m is None:
            break

        # Save the parsing failure reason for this message if it's a new one
        if m.get_type() == 'BAD_DATA':
            reason_id = ''.join(m.reason.split(' ')[0:3])
            if reason_id not in reason_ids:
                reason_ids.add(reason_id)
                reasons.append(m.reason)

    # Print out the final packet loss results
    print("%u packets, %u lost %.1f%%" % (
            mlog.mav_count, mlog.mav_loss, mlog.packet_loss()))

    # Also print out the reasons why losses occurred
    if len(reasons) > 0:
        print("Packet loss at least partially attributed to the following:")
        for r in reasons:
            print("  * " + r)

for filename in args.logs:
    mavloss(filename)
