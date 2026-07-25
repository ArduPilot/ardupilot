#!/usr/bin/env python3

'''
work out total flight time for a mavlink log
'''
import time
import glob

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--condition", default=None, help="condition for packets")
parser.add_argument("--groundspeed", type=float, default=3.0, help="groundspeed threshold")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink.mavextra import distance_two


def flight_time(logfile):
    '''work out flight time for a log file'''
    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename)

    in_air = False
    start_time = 0.0
    total_time = 0.0
    total_dist = 0.0
    t = None
    last_msg = None
    last_time_usec = None

    while True:
        m = mlog.recv_match(type=['GPS','GPS_RAW_INT'], condition=args.condition)
        if m is None:
            if in_air:
                total_time += time.mktime(t) - start_time
            if total_time > 0:
                print("Flight time : %u:%02u" % (int(total_time)/60, int(total_time)%60))
            return (total_time, total_dist)
        if m.get_type() == 'GPS_RAW_INT':
            groundspeed = m.vel*0.01
            status = m.fix_type
            time_usec = m.time_usec
        else:
            groundspeed = m.Spd
            status = m.Status
            time_usec = m.TimeUS
        if status < 3:
            continue
        t = time.localtime(m._timestamp)
        if groundspeed > args.groundspeed and not in_air:
            print("In air at %s (percent %.0f%% groundspeed %.1f)" % (time.asctime(t), mlog.percent, groundspeed))
            in_air = True
            start_time = time.mktime(t)
        elif groundspeed < args.groundspeed and in_air:
            print("On ground at %s (percent %.1f%% groundspeed %.1f  time=%.1f seconds)" % (
                time.asctime(t), mlog.percent, groundspeed, time.mktime(t) - start_time))
            in_air = False
            total_time += time.mktime(t) - start_time

        if last_msg is None or time_usec > last_time_usec or time_usec+30e6 < last_time_usec:
            if last_msg is not None:
                total_dist += distance_two(last_msg, m)
            last_msg = m
            last_time_usec = time_usec
    return (total_time, total_dist)

total_time = 0.0
total_dist = 0.0
for filename in args.logs:
    for f in glob.glob(filename):
        (ftime, fdist) = flight_time(f)
        total_time += ftime
        total_dist += fdist

print("Total time in air: %u:%02u" % (int(total_time)//60, int(total_time)%60))
print("Total distance travelled: %.1f meters" % total_dist)
