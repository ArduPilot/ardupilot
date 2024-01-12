#!/usr/bin/env python

"""
Parses a log file and shows how the solution status changed over time

AP_FLAKE8_CLEAN

"""

from __future__ import print_function

import optparse
import sys
import time

from pymavlink import mavutil


class SolutionStatusChange(object):
    def __init__(self, master):
        self.master = master

    def progress(self, text):
        '''emit text with possible timestamps etc'''
        print("%u: %s" % (time.time(), text))

    def run(self):

        self.progress("Creating connection")
        self.conn = mavutil.mavlink_connection(master)

        bit_descriptions = {
            "attitude": 0,
            "horiz_vel": 1,
            "vert_vel": 2,
            "horiz_pos_rel": 3,
            "horiz_pos_abs": 4,
            "vert_pos": 5,
            "terrain_alt": 6,
            "const_pos_mode": 7,
            "pred_horiz_pos_rel": 8,
            "pred_horiz_pos_abs": 9,
            "takeoff_detected": 10,
            "takeoff": 11,
            "touchdown": 12,
            "using_gps": 13,
            "gps_glitching": 14,
            "gps_quality_good": 15,
            "initalized": 16,
        }

        desired_type = "XKF4"
        old_message_per_core = {}
        while True:
            m = self.conn.recv_match(type=desired_type)
            if m.C != 0:
                continue
            if m is None:
                break
            if m.C not in old_message_per_core:
                old_message_per_core[m.C] = m
                continue
            current = old_message_per_core[m.C]
            if m.SS == current.SS:
                continue
            line = ""
            for (name, bit) in bit_descriptions.items():
                old_bit_set = current.SS & (1 << bit)
                new_bit_set = m.SS & (1 << bit)
                if new_bit_set and not old_bit_set:
                    line += " +%s" % name
                elif not new_bit_set and old_bit_set:
                    line += " -%s" % name

            old_message_per_core[m.C] = m

            timestamp = getattr(m, '_timestamp', 0.0)
            formatted_timestamp = "%s.%02u" % (
                time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp)),
                int(timestamp * 100.0) % 100)

            print("%s: C=%u %s" % (formatted_timestamp, m.C, line))
            current = m


if __name__ == '__main__':
    parser = optparse.OptionParser("solution-status-change.py [options]")

    (opts, args) = parser.parse_args()

    if len(args) < 1:
        parser.print_help()
        sys.exit(1)

    master = args[0]

    tester = SolutionStatusChange(master)
    tester.run()
