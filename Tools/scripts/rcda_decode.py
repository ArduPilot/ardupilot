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
parser.add_argument("--csv-output", metavar="FILE", default=None,
                    help="write a Saleae async-serial CSV suitable for parse_capture.py")
parser.add_argument("--baud-raw", type=int, default=1500000,
                    help="baud rate of the raw bus, used to space bytes in CSV output (default 1500000)")
parser.add_argument("log", metavar="LOG")
import time
import serial

args = parser.parse_args()

from pymavlink import mavutil

print("Processing log %s" % args.log)
mlog = mavutil.mavlink_connection(args.log)

if args.port:
    port = serial.Serial(args.port, args.baudrate, timeout=1.0)

csv_out = None
if args.csv_output:
    csv_out = open(args.csv_output, 'w')
    csv_out.write("Time [s],Value,Parity Error,Framing Error\n")

# Seconds per byte at the raw bus baud rate (10 bits per byte: 1 start + 8 data + 1 stop).
SECS_PER_BYTE = 10.0 / args.baud_raw

tlast = -1
csv_time_s = 0.0  # running absolute time for CSV output
counter = 0

while True:
    msg = mlog.recv_match(type=['RCDA'], condition=args.condition)
    if msg is None:
        break  # end of log; don't loop (unlike serial-playback mode)
    tnow = msg.TimeUS
    if tlast == -1:
        tlast = tnow
    buf = struct.pack("<IIIIIIIIII",
                      msg.U0, msg.U1, msg.U2, msg.U3, msg.U4,
                      msg.U5, msg.U6, msg.U7, msg.U8, msg.U9)[0:msg.Len]
    ibuf = [b for b in buf]
    dt = tnow - tlast
    tlast = tnow
    print(len(ibuf), ibuf, dt)

    if csv_out is not None:
        # Advance the CSV clock by the inter-packet gap.
        csv_time_s += dt * 1e-6
        for byte_val in ibuf:
            # Encode as \xNN to match Saleae async-serial CSV format.
            csv_out.write(f"{csv_time_s:.9f},\\x{byte_val:02X},,\n")
            csv_time_s += SECS_PER_BYTE

    if args.port:
        time.sleep(dt * 1.0e-6 * args.delay_mul)
        port.write(buf)

if csv_out is not None:
    csv_out.close()
    print(f"CSV written to {args.csv_output}")

# Serial-playback loop: rewind and replay continuously if --port was given.
if args.port:
    while True:
        mlog.rewind()
        tlast = -1
        while True:
            msg = mlog.recv_match(type=['RCDA'], condition=args.condition)
            if msg is None:
                break
            tnow = msg.TimeUS
            if tlast == -1:
                tlast = tnow
            buf = struct.pack("<IIIIIIIIII",
                              msg.U0, msg.U1, msg.U2, msg.U3, msg.U4,
                              msg.U5, msg.U6, msg.U7, msg.U8, msg.U9)[0:msg.Len]
            dt = tnow - tlast
            tlast = tnow
            time.sleep(dt * 1.0e-6 * args.delay_mul)
            port.write(buf)
