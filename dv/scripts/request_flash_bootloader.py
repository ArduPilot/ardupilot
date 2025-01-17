#!/usr/bin/env python3

import sys, os, argparse

from pymavlink import mavutil

import utils

if __name__ == "__main__":

    parser = argparse.ArgumentParser("request_flash_bootloader.py")
    parser.add_argument('connection', type=str, default="/dev/ttyACM0:115200", help='connection')

    args = parser.parse_args()
    print(args)

    device, baud = args.connection.split(":")

    some_master = mavutil.mavlink_connection(device, baud=int(baud))
    print(some_master.wait_heartbeat())

    res = utils.flash_bootloader(some_master)

    sys.exit(not res)
