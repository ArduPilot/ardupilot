#!/usr/bin/env python3

"""
Parses a log file and shows how the SENSOR_STATUS flags changed over time

AP_FLAKE8_CLEAN

"""

import optparse
import sys
import time

from pymavlink import mavutil


class SYS_STATUS_Change(object):
    def __init__(self, master):
        self.master = master

    def progress(self, text):
        '''emit text with possible timestamps etc'''
        print("%u: %s" % (time.time(), text))

    def bit_description(self, bit_number):
        if 1 << bit_number not in mavutil.mavlink.enums["MAV_SYS_STATUS_SENSOR"]:
            return "UNKNOWN_BIT[%u]" % bit_number

        name = mavutil.mavlink.enums["MAV_SYS_STATUS_SENSOR"][1 << bit_number].name
        # return name with common prefix removed:
        return name[len("MAV_SYS_STATUS_"):]

    def run(self):

        self.progress("Creating connection")
        self.conn = mavutil.mavlink_connection(master)

        fields = ['present', 'enabled', 'health']

        current = dict()
        for f in fields:
            current[f] = 0
        while True:
            m = self.conn.recv_match(type="SYS_STATUS")
            if m is None:
                break

            line = ""
            for f in fields:
                current_values = current[f]
                new_values = getattr(m, "onboard_control_sensors_" + f)
                for bit in range(0, 32):
                    mask = 1 << bit
                    old_bit_set = current_values & mask
                    new_bit_set = new_values & mask
                    if new_bit_set and not old_bit_set:
                        line += " %s+%s" % (f, self.bit_description(bit))
                    elif not new_bit_set and old_bit_set:
                        line += " %s-%s" % (f, self.bit_description(bit))
                current[f] = new_values

            if len(line) == 0:
                continue

            timestamp = getattr(m, '_timestamp', 0.0)
            formatted_timestamp = "%s.%02u" % (
                time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp)),
                int(timestamp * 100.0) % 100)

            print("%s: %s" % (formatted_timestamp, line))


if __name__ == '__main__':
    parser = optparse.OptionParser("sys_status_change.py [options]")

    (opts, args) = parser.parse_args()

    if len(args) < 1:
        parser.print_help()
        sys.exit(1)

    master = args[0]

    tester = SYS_STATUS_Change(master)
    tester.run()
