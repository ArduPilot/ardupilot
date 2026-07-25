#!/usr/bin/env python3

'''
convert a MAVLink tlog file to a MATLab mfile
'''

import os
import re
import sys
from pymavlink import mavutil

def process_tlog(filename):
    '''convert a tlog to a .m file'''

    print("Processing %s" % filename)

    mlog = mavutil.mavlink_connection(filename, dialect=args.dialect, zero_time_base=True)

    # first walk the entire file, grabbing all messages into a hash of lists,
    #and the first message of each type into a hash
    msg_types = {}
    msg_lists = {}

    types = args.types
    if types is not None:
        types = types.split(',')

    # note that Octave doesn't like any extra '.', '*', '-', characters in the filename
    (head, tail) = os.path.split(filename)
    basename = '.'.join(tail.split('.')[:-1])
    mfilename = re.sub('[\.\-\+\*]','_', basename) + '.m'
    # Octave also doesn't like files that don't start with a letter
    if re.match('^[a-zA-z]', mfilename) is None:
        mfilename = 'm_' + mfilename

    if head is not None:
        mfilename = os.path.join(head, mfilename)
    print("Creating %s" % mfilename)

    f = open(mfilename, "w")

    type_counters = {}

    while True:
        m = mlog.recv_match(condition=args.condition)
        if m is None:
            break

        if types is not None and m.get_type() not in types:
            continue
        if m.get_type() == 'BAD_DATA':
            continue

        fieldnames = m._fieldnames
        mtype = m.get_type()
        if mtype in ['FMT', 'PARM']:
            continue

        if mtype not in type_counters:
            type_counters[mtype] = 0
            f.write("%s.columns = {'timestamp'" % mtype)
            for field in fieldnames:
                val = getattr(m, field)

                if not isinstance(val, str):
                    if type(val) is not list:
                        f.write(",'%s'" % field)
                    else:
                        for i in range(0, len(val)):
                            f.write(",'%s%d'" % (field, i + 1))
            f.write("};\n")

        type_counters[mtype] += 1
        f.write("%s.data(%u,:) = [%f" % (mtype, type_counters[mtype], m._timestamp))
        for field in m._fieldnames:
            val = getattr(m, field)

            if not isinstance(val, str):
                if type(val) is not list:
                    f.write(",%.20g" % val)
                else:
                    for i in range(0, len(val)):
                        f.write(",%.20g" % val[i])
        f.write("];\n")
    f.close()

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("-o", "--output", default=None, help="output filename")
parser.add_argument("--types", default=None, help="types of messages (comma separated)")
parser.add_argument("--dialect", default="ardupilotmega", help="MAVLink dialect")
parser.add_argument("logs", metavar="LOG", nargs="+")
args = parser.parse_args()


for filename in args.logs:
    process_tlog(filename)
