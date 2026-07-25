#!/usr/bin/env python3

'''
fix time drift in the TimeUS field of logs based on GPS time
this helps when post-processing a log for a time sensitive task for flight
controllers with very bad crystals
it assumes constant drift
'''

import os
import sys
from argparse import ArgumentParser
from progress.bar import Bar
import time
import struct

parser = ArgumentParser(description=__doc__)

parser.add_argument("log_in")
parser.add_argument("log_out")
parser.add_argument("--fix", action='store_true', default=False)

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink import DFReader

log1 = mavutil.mavlink_connection(args.log_in)

if not isinstance(log1,DFReader.DFReader_binary):
    print("Must be a bin log")
    sys.exit(1)

gps_id = log1.name_to_id.get('GPS', None)
if gps_id is None:
    print("No GPS messages")
    sys.exit(1)

print("Finding drift")
first_gps_msg = None
last_gps_msg = None

while True:
    m = log1.recv_match(type='GPS')
    if m is None:
        break
    if m.I != 0 or m.Status < 3:
        continue
    if first_gps_msg is None:
        first_gps_msg = m
    last_gps_msg = m

if first_gps_msg is None:
    print("No GPS[0] messages")
    sys.exit(1)

dt1 = (last_gps_msg.GWk*7*24*3600 + last_gps_msg.GMS*0.001) - (first_gps_msg.GWk*7*24*3600 + first_gps_msg.GMS*0.001)
dt2 = (last_gps_msg.TimeUS - first_gps_msg.TimeUS) * 1.0e-6

PPM = 1.0e6*(dt2-dt1)/dt1

print("Drift is %.2f seconds, %.1f PPM" % ((dt2-dt1), PPM))

if not args.fix:
    print("--fix not specified, just reporting")
    sys.exit(0)

if abs(PPM) < 10:
    print("Not enough drift to fix")
    sys.exit(1)

log1.rewind()

TimeUS_start = first_gps_msg.TimeUS

output = open(args.log_out, mode='wb')

def write_message(m):
    mtype = m.get_type()
    buf = bytearray(m.get_msgbuf())
    if hasattr(m,'TimeUS'):
        dt = (m.TimeUS - TimeUS_start)*1.0e-6
        new_TimeUS = m.TimeUS - (dt * PPM)
        if new_TimeUS < 0:
            return
        timeus_buf = struct.pack("Q",int(new_TimeUS))
        buf = buf[:3] + timeus_buf + buf[11:]
    output.write(buf)

bar = Bar('Processing log', max=100)

now = time.time()

print("Processing log")

pct = 0

while True:
    m = log1.recv_msg()
    if m is None:
        break
    write_message(m)
    new_pct = (log1.offset * 100) // log1.data_len
    if new_pct != pct:
        bar.next()
        pct = new_pct

print("\nDone")
