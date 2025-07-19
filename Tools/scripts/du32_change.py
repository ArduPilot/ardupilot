#!/usr/bin/env python3

"""
Parses a log file and shows how Copter's du32 changes over time

AP_FLAKE8_CLEAN

"""

import optparse
import sys
import time

from pymavlink import mavutil


class DU32Change(object):
    def __init__(self, master):
        self.master = master

    def progress(self, text):
        '''emit text with possible timestamps etc'''
        print("%u: %s" % (time.time(), text))

    def run(self):

        self.progress("Creating connection")
        self.conn = mavutil.mavlink_connection(master)

        # this eas was generated from Copter.h's structure for ap_t:
        bit_descriptions_list = [
            "unused1",
            "unused_was_simple_mode bit1",
            "unused_was_simple_mode bit2",
            "pre_arm_rc_check",
            "pre_arm_check",
            "auto_armed",
            "logging_started",
            "land_complete",
            "new_radio_frame",
            "usb_connected_unused",
            "rc_receiver_present_unused",
            "compass_mot",
            "motor_test",
            "initialised",
            "land_complete_maybe",
            "throttle_zero",
            "system_time_set_unused",
            "gps_glitching",
            "using_interlock",
            "land_repo_active",
            "motor_interlock_switch",
            "in_arming_delay",
            "initialised_params",
            "unused3",
            "unused2",
            "armed_with_airmode_switch",
            "prec_land_active",
        ]
        bit_descriptions = {}
        count = 0
        for bit in bit_descriptions_list:
            bit_descriptions[bit] = count
            count += 1

        old_m = None

        desired_type = "DU32"
        while True:
            m = self.conn.recv_match(type=desired_type)
            if m is None:
                break
            if m.Id != 7:
                # 7 is LOG_DATA_ID from AP_Logger.h
                continue
            if old_m is not None and m.Value == old_m.Value:
                continue
            line = ""
            if old_m is None:
                for bit in sorted(bit_descriptions.keys()):
                    bit_set = m.Value & (1 << bit_descriptions[bit])
                    if bit_set:
                        print("Original %s: 1" % bit)
                    else:
                        print("Original %s: 0" % bit)
            else:
                for bit in bit_descriptions.keys():
                    old_bit_set = old_m.Value & (1 << bit_descriptions[bit])
                    new_bit_set = m.Value & (1 << bit_descriptions[bit])
                    if new_bit_set and not old_bit_set:
                        line += " +%s" % bit
                    elif not new_bit_set and old_bit_set:
                        line += " -%s" % bit

                timestamp = getattr(m, '_timestamp', 0.0)
                formatted_timestamp = "%s.%02u" % (
                    time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp)),
                    int(timestamp * 100.0) % 100)

                print("%s: %s" % (formatted_timestamp, line))
            old_m = m


if __name__ == '__main__':
    parser = optparse.OptionParser("du32_change.py [options]")

    (opts, args) = parser.parse_args()

    if len(args) < 1:
        parser.print_help()
        sys.exit(1)

    master = args[0]

    tester = DU32Change(master)
    tester.run()
