#!/usr/bin/env python3
'''
playback a set of CAN frames from libraries/AP_Scripting/examples/CAN_logger.lua onto a CAN bus
'''

import dronecan
import time
import sys
import threading
from pymavlink import mavutil
from dronecan.driver.common import CANFrame


# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='CAN playback')
parser.add_argument("logfile", default=None, type=str, help="logfile")
parser.add_argument("canport", default=None, type=str, help="CAN port")

args = parser.parse_args()

print("Connecting to %s" % args.canport)
driver = dronecan.driver.make_driver(args.canport)

print("Opening %s" % args.logfile)
mlog = mavutil.mavlink_connection(args.logfile)

tstart = time.time()
first_tstamp = None

while True:
    m = mlog.recv_match(type='CANF')

    if m is None:
        print("Rewinding")
        mlog.rewind()
        tstart = time.time()
        first_tstamp = None
        continue

    if first_tstamp is None:
        first_tstamp = m.TimeUS
    dt = time.time() - tstart
    dt2 = (m.TimeUS - first_tstamp)*1.0e-6
    if dt2 > dt:
        time.sleep(dt2 - dt)
    data = [m.B0, m.B1, m.B2, m.B3, m.B4, m.B5, m.B6, m.B7]
    data = data[:m.DLC]
    fid = m.Id
    is_extended = (fid & (1<<31)) != 0
    driver.send(fid, data, extended=is_extended, canfd=False)
    print(m)
