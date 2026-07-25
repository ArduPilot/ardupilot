#!/usr/bin/env python3

'''
extract one mode type from a log
'''
import os
import struct

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_argument("--robust", action='store_true', help="Enable robust parsing (skip over bad data)")
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--mode", default='auto', help="mode to extract")
parser.add_argument("--link", default=None, type=int, help="only extract specific comms link")
parser.add_argument("logs", metavar="LOG", nargs="+")
args = parser.parse_args()

from pymavlink import mavutil

def older_message(m, lastm):
    '''return true if m is older than lastm by timestamp'''
    atts = {'time_boot_ms' : 1.0e-3,
            'time_unix_usec' : 1.0e-6,
            'time_usec' : 1.0e-6}
    for a in list(atts.keys()):
        if hasattr(m, a):
            mul = atts[a]
            t1 = m.getattr(a) * mul
            t2 = lastm.getattr(a) * mul
            if t2 >= t1 and t2 - t1 < 60:
                return True
    return False

def process(filename):
    '''process one logfile'''
    print("Processing %s" % filename)
    mlog = mavutil.mavlink_connection(filename, notimestamps=args.notimestamps,
                                      robust_parsing=args.robust)


    ext = os.path.splitext(filename)[1]
    isbin = ext in ['.bin', '.BIN']
    islog = ext in ['.log', '.LOG']
    output = None
    count = 1
    dirname = os.path.dirname(filename)

    if isbin or islog:
        extension = "bin"
    else:
        extension = "tlog"

    file_header = bytearray()

    messages = []

    # we allow a list of modes that map to one mode number. This allows for --mode=AUTO,RTL and consider the RTL as part of AUTO
    modes = args.mode.upper().split(',')
    flightmode = None

    while True:
        m = mlog.recv_match()
        if m is None:
            break
        if args.link is not None and m._link != args.link:
            continue

        mtype = m.get_type()
        if mtype in messages:
            if older_message(m, messages[mtype]):
                continue

        # we don't use mlog.flightmode as that can be wrong if we are extracting a single link
        if mtype == 'HEARTBEAT' and m.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_GIMBAL and m.type != mavutil.mavlink.MAV_TYPE_GCS:
            flightmode = mavutil.mode_string_v10(m).upper()
        if mtype == 'MODE':
            flightmode = mlog.flightmode

        if (isbin or islog) and m.get_type() in ["FMT", "PARM", "CMD", "FMTU", "MULT"]:
            file_header += m.get_msgbuf()
        if (isbin or islog) and m.get_type() == 'MSG' and m.Message.startswith("Ardu"):
            file_header += m.get_msgbuf()
        if m.get_type() in ['PARAM_VALUE','MISSION_ITEM','MISSION_ITEM_INT']:
            timestamp = getattr(m, '_timestamp', None)
            file_header += struct.pack('>Q', int(timestamp*1.0e6)) + m.get_msgbuf()

        if not mavutil.evaluate_condition(args.condition, mlog.messages):
            continue

        if flightmode in modes:
            if output is None:
                path = os.path.join(dirname, "%s%u.%s" % (modes[0], count, extension))
                count += 1
                print("Creating %s" % path)
                output = open(path, mode='wb')
                output.write(file_header)
        else:
            if output is not None:
                output.close()
                output = None

        if output and m.get_type() != 'BAD_DATA':
            timestamp = getattr(m, '_timestamp', None)
            if not isbin:
                output.write(struct.pack('>Q', int(timestamp*1.0e6)))
            output.write(m.get_msgbuf())

for filename in args.logs:
    process(filename)

