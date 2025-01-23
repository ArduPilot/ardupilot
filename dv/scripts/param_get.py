#!/usr/bin/env python3

import sys, os, argparse

from pymavlink import mavutil

import param_manip

if __name__ == "__main__":

    parser = argparse.ArgumentParser("param_set.py")
    parser.add_argument('connection', type=str, default="/dev/ttyACM0:115200", help='connection')
    parser.add_argument('param_id', type=str)

    args = parser.parse_args()
    print(args)

    device, baud = args.connection.split(":")

    some_master = mavutil.mavlink_connection(device, baud=int(baud))
    print(some_master.wait_heartbeat())

    res = param_manip.read_param(some_master, args.param_id)

    if res is not None:
        print(res["param_value"])

    sys.exit(res is not None)
