#!/usr/bin/env python3

'''
show changes in flight modes
'''
import datetime
import os
import time

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil


def flight_modes(logfile):
    '''show flight modes for a log file'''
    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename)

    mode = ""
    previous_mode = ""
    mode_start_timestamp = -1
    time_in_mode = {}
    previous_percent = -1
    seconds_per_percent = -1

    filesize = os.path.getsize(filename)

    while True:
        m = mlog.recv_match(type=['SYS_STATUS','HEARTBEAT','MODE'],
                            condition='MAV.flightmode!="%s"' % mlog.flightmode)
        if m is None:
            break
        print('%s MAV.flightmode=%-12s (MAV.timestamp=%u %u%%)' % (
            time.asctime(time.localtime(m._timestamp)),
            mlog.flightmode,
            m._timestamp, mlog.percent))

        mode = mlog.flightmode
        if (mode not in time_in_mode):
            time_in_mode[mode] = 0

        if (mode_start_timestamp == -1):
            mode_start_timestamp = m._timestamp
        elif (previous_mode != "" and previous_mode != mode):
            time_in_mode[previous_mode] = time_in_mode[previous_mode] + (m._timestamp - mode_start_timestamp)

            #figure out how many seconds per percentage point so I can
            #calculate how many seconds for the final mode
            if (seconds_per_percent == -1 and previous_percent != -1
                    and previous_percent != mlog.percent):
                seconds_per_percent = (m._timestamp - mode_start_timestamp) / (mlog.percent - previous_percent)

            mode_start_timestamp = m._timestamp

        previous_mode = mode
        previous_percent = mlog.percent

    #put a whitespace line before the per-mode report
    print()
    print("Time per mode:")

    #need to get the time in the final mode
    if (seconds_per_percent != -1):
        seconds_remaining = (100.0 - previous_percent) * seconds_per_percent

        time_in_mode[previous_mode] = time_in_mode[previous_mode] + seconds_remaining

        total_flight_time = 0
        for key, value in time_in_mode.items():
            total_flight_time = total_flight_time + value

        for key, value in time_in_mode.items():
            print('%-12s %s %.2f%%' % (key, str(datetime.timedelta(seconds=int(value))), (value / total_flight_time) * 100.0))
    else:
        #can't print time in mode if only one mode during flight
        print(previous_mode, " 100% of flight time")

for filename in args.logs:
    flight_modes(filename)
