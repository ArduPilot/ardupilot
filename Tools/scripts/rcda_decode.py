#!/usr/bin/env python3

# flake8: noqa

'''
decode RCDA messages from a log and optionally play back to a serial port. The RCDA message is
captures RC input bytes when RC_OPTIONS=16 is set
'''

import struct

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--baudrate", type=int, default=115200, help="baudrate")
parser.add_argument("--port", type=str, default=None, help="port")
parser.add_argument("--delay-mul", type=float, default=1.0, help="delay multiplier")
parser.add_argument("log", metavar="LOG")
import time
import serial

args = parser.parse_args()

from pymavlink import mavutil

print("Processing log %s" % args.log)
mlog = mavutil.mavlink_connection(args.log)

if args.port:
    port = serial.Serial(args.port, args.baudrate, timeout=1.0)

tlast = -1
counter = 0

while True:
    msg = mlog.recv_match(type=['RCDA'], condition=args.condition)
    if msg is None:
        mlog.rewind()
        tlast = -1
        continue
    tnow = msg.TimeUS
    if tlast == -1:
        tlast = tnow
    buf = struct.pack("<IIIIIIIIII",
                      msg.U0, msg.U1, msg.U2, msg.U3, msg.U4,
                        msg.U5, msg.U6, msg.U7, msg.U8, msg.U9)[0:msg.Len]
    ibuf = [ ord(b) for b in buf ]
    dt = tnow - tlast
    tlast = tnow
    print(len(ibuf), ibuf, dt)
    if args.port:
        time.sleep(dt*1.0e-6*args.delay_mul)
        port.write(buf)
