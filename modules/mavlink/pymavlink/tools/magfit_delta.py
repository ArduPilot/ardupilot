#!/usr/bin/env python3

'''
fit best estimate of magnetometer offsets using the algorithm from
Bill Premerlani
'''

import sys

# command line option handling
from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--verbose", action='store_true', default=False, help="verbose offset output")
parser.add_argument("--gain", type=float, default=0.01, help="algorithm gain")
parser.add_argument("--noise", type=float, default=0, help="noise to add")
parser.add_argument("--max-change", type=float, default=10, help="max step change")
parser.add_argument("--min-diff", type=float, default=50, help="min mag vector delta")
parser.add_argument("--history", type=int, default=20, help="how many points to keep")
parser.add_argument("--repeat", type=int, default=1, help="number of repeats through the data")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink.rotmat import Vector3


def noise():
    '''a noise vector'''
    from random import gauss
    v = Vector3(gauss(0, 1), gauss(0, 1), gauss(0, 1))
    v.normalize()
    return v * args.noise

def find_offsets(data, ofs):
    '''find mag offsets by applying Bills "offsets revisited" algorithm
       on the data

       This is an implementation of the algorithm from:
          http://gentlenav.googlecode.com/files/MagnetometerOffsetNullingRevisited.pdf
       '''

    # a limit on the maximum change in each step
    max_change = args.max_change

    # the gain factor for the algorithm
    gain = args.gain

    data2 = []
    for d in data:
        d = d.copy() + noise()
        d.x = float(int(d.x + 0.5))
        d.y = float(int(d.y + 0.5))
        d.z = float(int(d.z + 0.5))
        data2.append(d)
    data = data2

    history_idx = 0
    mag_history = data[0:args.history]

    for i in range(args.history, len(data)):
        B1 = mag_history[history_idx] + ofs
        B2 = data[i] + ofs

        diff = B2 - B1
        diff_length = diff.length()
        if diff_length <= args.min_diff:
            # the mag vector hasn't changed enough - we don't get any
            # information from this
            history_idx = (history_idx+1) % args.history
            continue

        mag_history[history_idx] = data[i]
        history_idx = (history_idx+1) % args.history

        # equation 6 of Bills paper
        delta = diff * (gain * (B2.length() - B1.length()) / diff_length)

        # limit the change from any one reading. This is to prevent
        # single crazy readings from throwing off the offsets for a long
        # time
        delta_length = delta.length()
        if max_change != 0 and delta_length > max_change:
            delta *= max_change / delta_length

        # set the new offsets
        ofs = ofs - delta

        if args.verbose:
            print(ofs)
    return ofs


def magfit(logfile):
    '''find best magnetometer offset fit to a log file'''

    print("Processing log %s" % filename)

    # open the log file
    mlog = mavutil.mavlink_connection(filename, notimestamps=args.notimestamps)

    data = []
    mag = None
    offsets = Vector3(0,0,0)

    # now gather all the data
    while True:
        # get the next MAVLink message in the log
        m = mlog.recv_match(condition=args.condition)
        if m is None:
            break
        if m.get_type() == "SENSOR_OFFSETS":
            # update offsets that were used during this flight
            offsets = Vector3(m.mag_ofs_x, m.mag_ofs_y, m.mag_ofs_z)
        if m.get_type() == "RAW_IMU" and offsets is not None:
            # extract one mag vector, removing the offsets that were
            # used during that flight to get the raw sensor values
            mag = Vector3(m.xmag, m.ymag, m.zmag) - offsets
            data.append(mag)

    print("Extracted %u data points" % len(data))
    print("Current offsets: %s" % offsets)

    # run the fitting algorithm
    ofs = offsets
    ofs = Vector3(0,0,0)
    for r in range(args.repeat):
        ofs = find_offsets(data, ofs)
        print('Loop %u offsets %s' % (r, ofs))
        sys.stdout.flush()
    print("New offsets: %s" % ofs)

total = 0.0
for filename in args.logs:
    magfit(filename)
