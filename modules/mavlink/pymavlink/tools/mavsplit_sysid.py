#!/usr/bin/env python3

'''
split log by system ID
'''
import os
import struct

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--types", default=None, help="list of types to include")
parser.add_argument("logs", metavar="LOG", nargs="+")
args = parser.parse_args()

from pymavlink import mavutil

def process(filename):
    '''process one logfile'''
    print("Processing %s" % filename)

    types = args.types
    if types is not None:
        types = types.split(',')

    mlog = mavutil.mavlink_connection(filename, notimestamps=args.notimestamps)

    base, ext = os.path.splitext(filename)
    output = None
    count = 1
    dirname = os.path.dirname(filename)
    extension = "tlog"

    file_header = bytearray()

    messages = []

    # dictionary of outputs by sysid
    output = {}

    while True:
        m = mlog.recv_match(type=types)
        if m is None:
            break

        if args.condition and not mavutil.evaluate_condition(args.condition, mlog.messages):
            continue

        if m.get_type() == 'BAD_DATA':
            continue

        sysid = m.get_srcSystem()
        if not sysid in output:
            fname = "%s-%u.%s" % (base, sysid, extension)
            print("Creating %s" % fname)
            output[sysid] = open(fname, mode='wb')

        if output[sysid]:
            timestamp = getattr(m, '_timestamp', None)
            output[sysid].write(struct.pack('>Q', int(timestamp*1.0e6)))
            output[sysid].write(m.get_msgbuf())

for filename in args.logs:
    process(filename)

