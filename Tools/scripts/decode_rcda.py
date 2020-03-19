#!/usr/bin/env python

'''
decode RC input data
'''
from __future__ import print_function

import numpy
import pylab

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil

def decode_unit32(u):
    b3 = u & 0xFF
    b2 = (u >> 8) & 0xFF
    b1 = (u >> 16) & 0xFF
    b0 = (u >> 24) & 0xFF
    return "0x%02x, 0x%02x, 0x%02x, 0x%02x" % (b0, b1, b2, b3)

def decode_rcda(logfile):
    '''decode RC input data in logfile'''

    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename)

    messages = 0;
    while True:
        m = mlog.recv_match(type='RCDA')
        if m is None:
            break
        messages = messages + 1
        print("%s %s %s %s %s %s %s %s %s %s" % (decode_unit32(m.U0), decode_unit32(m.U1), decode_unit32(m.U2), decode_unit32(m.U3), decode_unit32(m.U4), decode_unit32(m.U5), decode_unit32(m.U6), decode_unit32(m.U7), decode_unit32(m.U8), decode_unit32(m.U9)))

    print("Extracted %u RCDA messages" % messages)


for filename in args.logs:
    decode_rcda(filename)

