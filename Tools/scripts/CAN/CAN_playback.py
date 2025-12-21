#!/usr/bin/env python3

# flake8: noqa

'''
 playback a set of CAN frames
 capture frames either using libraries/AP_Scripting/examples/CAN_logger.lua
 or the CAN_Pn_OPTIONS bit to enable CAN logging
'''

import dronecan
import time
import sys
import threading
from pymavlink import mavutil
from dronecan.driver.common import CANFrame
import struct


# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='CAN playback')
parser.add_argument("logfile", default=None, type=str, help="logfile")
parser.add_argument("canport", default=None, type=str, help="CAN port")
parser.add_argument("--bus", default=0, type=int, help="CAN bus")

args = parser.parse_args()

print("Connecting to %s" % args.canport)
driver = dronecan.driver.make_driver(args.canport)

print("Opening %s" % args.logfile)
mlog = mavutil.mavlink_connection(args.logfile)

tstart = time.time()
first_tstamp = None

def dlc_to_datalength(dlc):
    # Data Length Code      9  10  11  12  13  14  15
    # Number of data bytes 12  16  20  24  32  48  64
    if (dlc <= 8):
        return dlc
    elif (dlc == 9):
        return 12
    elif (dlc == 10):
        return 16
    elif (dlc == 11):
        return 20
    elif (dlc == 12):
        return 24
    elif (dlc == 13):
        return 32
    elif (dlc == 14):
        return 48
    return 64

while True:
    m = mlog.recv_match(type=['CANF','CAFD'])

    if m is None:
        print("Rewinding")
        mlog.rewind()
        tstart = time.time()
        first_tstamp = None
        continue

    if getattr(m,'bus',0) != args.bus:
        continue

    if first_tstamp is None:
        first_tstamp = m.TimeUS
    dt = time.time() - tstart
    dt2 = (m.TimeUS - first_tstamp)*1.0e-6
    if dt2 > dt:
        time.sleep(dt2 - dt)

    canfd = m.get_type() == 'CAFD'
    if canfd:
        data = struct.pack("<QQQQQQQQ", m.D0, m.D1, m.D2, m.D3, m.D4, m.D5, m.D6, m.D7)
        data = data[:dlc_to_datalength(m.DLC)]
    else:
        data = struct.pack("<BBBBBBBB", m.B0, m.B1, m.B2, m.B3, m.B4, m.B5, m.B6, m.B7)
        data = data[:m.DLC]

    fid = m.Id
    is_extended = (fid & (1<<31)) != 0
    driver.send(fid, data, extended=is_extended, canfd=canfd)
    print(m)
