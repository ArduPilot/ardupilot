#!/usr/bin/env python3

import sys, os, argparse

from pymavlink import mavutil

import param_manip

if __name__ == "__main__":

    parser = argparse.ArgumentParser("param_set.py")
    parser.add_argument('connection', type=str, default="/dev/ttyACM2:115200", help='connection')
    parser.add_argument('param_id', type=str)
    parser.add_argument('param_value', type=float)
    parser.add_argument('-r', '--reboot', action=argparse.BooleanOptionalAction)

    args = parser.parse_args()
    print(args)

    device, baud = args.connection.split(":")

    some_master = mavutil.mavlink_connection(device, baud=int(baud))
    print(some_master.wait_heartbeat())

    res = not param_manip.write_param(some_master, args.param_id, args.param_value)

    if args.reboot:
        some_master.reboot_autopilot()
        print("Rebooting ..")

    sys.exit(res)
