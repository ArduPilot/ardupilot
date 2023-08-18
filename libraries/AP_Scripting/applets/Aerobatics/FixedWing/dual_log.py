#!/usr/bin/env python

'''
combined two aerobatics logs to generate a new log for showing the combined
path of more than one aircraft
'''

import os
from argparse import ArgumentParser

parser = ArgumentParser(description=__doc__)

parser.add_argument("log1", metavar="LOG1")
parser.add_argument("log2", metavar="LOG2")
parser.add_argument("logout", metavar="LOGOUT")
parser.add_argument("--decimate", type=int, default=1)

args = parser.parse_args()

os.environ['MAVLINK20'] = '1'

from pymavlink import mavutil

mlog1 = mavutil.mavlink_connection(args.log1)
mlog2 = mavutil.mavlink_connection(args.log2)
output = open(args.logout, mode='wb')

types1 = ['ORGN','VEH','PARM','MSG','FMT','FMTU','MULT','MODE','EVT']
types2 = ['VEH']

m1 = None
m2 = None

veh_fmt = None

def write_VEH(m, veh_fmt):
    buf = bytearray(m.get_msgbuf())
    if veh_fmt is None:
        return
    buf[2] = veh_fmt
    output.write(buf)

m1_count = 0
m2_count = 0

while True:
    if m1 is None:
        m1 = mlog1.recv_match(type=types1)
    if m2 is None:
        m2 = mlog2.recv_match(type=types2)

    if m1 is not None and m1.get_type() != 'VEH':
        output.write(m1.get_msgbuf())
        m1 = None
        continue

    if m2 is not None and m2.get_type() != 'VEH':
        continue
    
    if veh_fmt is None and m1 is not None:
        veh_fmt = m1.get_msgbuf()[2]

    if m1 is None and m2 is None:
        break

    if m1 is None:
        write_VEH(m2, veh_fmt)
        m2 = None
        continue

    if m2 is None:
        write_VEH(m1, veh_fmt)
        m1 = None
        continue

    if hasattr(m1,'TSec'):
        # old format
        t1 = m1.TSec + m1.TUSec*1.0e-6
        t2 = m2.TSec + m2.TUSec*1.0e-6
    else:
        t1 = m1.GWk*7*24*60*60 + m1.GMS*0.001
        t2 = m2.GWk*7*24*60*60 + m2.GMS*0.001
    if t1 <= t2:
        m1_count += 1
        if m1_count % args.decimate == 0:
            write_VEH(m1, veh_fmt)
        m1 = None
    else:
        m2_count += 1
        if m2_count % args.decimate == 0:
            write_VEH(m2, veh_fmt)
        m2 = None
