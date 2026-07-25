#!/usr/bin/env python3

'''
extract mavlink parameter values
'''
import time

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("-c", "--changes", dest="changesOnly", default=False, action="store_true", help="Show only changes to parameters.")
parser.add_argument("--qgc", action='store_true', help="output in QGC-friendly format")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil

parms = {}

def mavparms(logfile):
    '''extract mavlink parameters'''
    mlog = mavutil.mavlink_connection(filename)

    while True:
        try:
            m = mlog.recv_match(type=['PARAM_VALUE', 'PARM'])
            if m is None:
                return
        except Exception:
            return
        if m.get_type() == 'PARAM_VALUE':
            pname = str(m.param_id).strip()
            value = m.param_value
        else:
            pname = m.Name
            value = m.Value
        if len(pname) > 0:
            if args.changesOnly is True and pname in parms and parms[pname] != value:
                print("%s %-15s %.6f -> %.6f" % (time.asctime(time.localtime(m._timestamp)), pname, parms[pname], value))

            parms[pname] = value

total = 0.0
for filename in args.logs:
    mavparms(filename)

if args.qgc:
    # see https://dev.qgroundcontrol.com/master/en/file_formats/parameters.html
    print("""
# # Vehicle-Id Component-Id Name Value Type
""")

if (args.changesOnly is False):
    keys = list(parms.keys())
    keys.sort()
    for p in keys:
        if args.qgc:
            MAV_PARAM_TYPE_REAL32 = 9
            sysid = 1
            compid = 1
            print("%u %u %-15s %.6f %u" %
                  (sysid, compid, p, parms[p], MAV_PARAM_TYPE_REAL32))
        else:
            print("%-15s %.6f" % (p, parms[p]))
