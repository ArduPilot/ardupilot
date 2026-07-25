#!/usr/bin/env python3
'''
example of rewriting a log with changed values

this can be useful when wanting to replay a log with different values
'''

import os
from argparse import ArgumentParser
from progress.bar import Bar
import time
import struct
import sys

parser = ArgumentParser(description=__doc__)

parser.add_argument("login")
parser.add_argument("logout")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink import DFReader
import struct

print("Opening %s" % args.login)
login = mavutil.mavlink_connection(args.login)
output = open(args.logout, mode='wb')

def write_message(m):
    mtype = m.get_type()
    buf = bytearray(m.get_msgbuf())
    if mtype == 'RRLT' and m.Idx == 2:
        # fix beacon location
        buf2 = buf[:3] + struct.pack("<ffiiiIB", m.Range, m.Uncertainty, int(-35.28230025*1e7), int(149.00661081*1e7), 58400, m.TS, m.Idx)
        if len(buf2) != len(buf):
            print("Buffer length error", m)
            sys.exit(1)
        buf = buf2
    output.write(buf)

bar = Bar('Processing log', max=100)

pct = 0

while True:
    m = login.recv_msg()
    write_message(m)

    new_pct = (login.offset * 100) // login.data_len
    if new_pct != pct:
        bar.next()
        pct = new_pct
