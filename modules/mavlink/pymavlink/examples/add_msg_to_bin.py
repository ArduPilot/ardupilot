#!/usr/bin/env python3
'''
this shows how to add a new message to a bin log
'''

from pymavlink import DFReader

import argparse
parser = argparse.ArgumentParser("add msg example")
parser.add_argument("login")
parser.add_argument("logout")

args = parser.parse_args()

inf = DFReader.DFReader_binary(args.login)
outf = open(args.logout, 'wb')

# make a GPSX message which will mirror GPS message
GPSX = inf.add_format(DFReader.DFFormat(0, "GPSX", 0, "QLLf", "TimeUS,Lat,Lng,Alt"))
outf.write(inf.make_format_msgbuf(GPSX))

count = 0

while True:
    m = inf.recv_msg()
    if m is None:
        break
    outf.write(m.get_msgbuf())
    if m.get_type() == 'GPS':
        # mirror GPS message as GPSX
        values = [m.TimeUS, int(m.Lat*1.0e7), int(m.Lng*1.0e7), m.Alt]
        outf.write(inf.make_msgbuf(GPSX, values))
        count += 1

print("Added %u GPSX messages to %s from %s" % (count, args.logout, args.login))

