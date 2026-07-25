#!/usr/bin/env python3

'''
merge 2 logs into a 3rd log, mapping some messages from LOG2 to new message names
'''

import os
from argparse import ArgumentParser
from progress.bar import Bar
import time
import struct

parser = ArgumentParser(description=__doc__)

parser.add_argument("log1", help="First log")
parser.add_argument("log2", help="Second log")
parser.add_argument("logout", metavar="LOGOUT")
parser.add_argument("--mapping", default="POS:POS2,ATT:ATT2,FOLL:FOL2", help="message type mapping")

args = parser.parse_args()

from pymavlink import mavutil
from pymavlink import DFReader

log1 = mavutil.mavlink_connection(args.log1)
log2 = mavutil.mavlink_connection(args.log2)
output = open(args.logout, mode='wb')

# create a mapping between msg names in log and new msg names
msg_map = {}
a = args.mapping.split(',')
for m in a:
    b = m.split(':')
    msg_map[b[0]] = b[1]

fmt_done = {}
used_ids = set()

def allocate_id():
    '''
    allocate an unused id from the ardupilot bin log
    '''
    for id in range(100, 254):
        if not id in log1.id_to_name and not id in used_ids:
            used_ids.add(id)
            return id
    return None

def write_message(m):
    '''possibly write message from log2'''
    mtype = m.get_type()
    if mtype == "FMT":
        if m.Name not in msg_map.keys():
            return
        if m.Name in fmt_done:
            return
        id = allocate_id()
        if id is None:
            return
        fmt_done[m.Name] = id
        m.Name = msg_map[m.Name]
        fmt = DFReader.DFFormat(id, m.Name, m.Length, m.Format, m.Columns)
        buf = bytearray(m.get_msgbuf())
        buf[3] = id
        for i in range(len(m.Name)):
            buf[5+i] = m.Name.encode('utf-8')[i]
        output.write(buf)
        print("Added %s with id %u" % (m.Name, id))
        return
    if mtype not in msg_map.keys() or mtype not in fmt_done:
        return
    buf = bytearray(m.get_msgbuf())
    buf[2] = fmt_done[mtype]
    output.write(buf)

bar = Bar('Merging logs', max=100)

now = time.time()

print("Processing logs")

m1 = None
m2 = None
pct = 0

while True:
    if m1 is None:
        m1 = log1.recv_msg()
    if m2 is None:
        m2 = log2.recv_msg()

    if m1 is not None and m1.get_type() == "FMT":
        output.write(m1.get_msgbuf())
        m1 = None
        continue

    new_pct = (log1.offset * 100) // log1.data_len
    if new_pct != pct:
        bar.next()
        pct = new_pct

    if m1 is None and m2 is None:
        # all done
        break

    if m2 is None and m1 is not None:
        # pass through m1
        output.write(m1.get_msgbuf())
        m1 = None
        continue

    if m1 is None and m2 is not None:
        # process m2
        write_message(m2)
        m2 = None
        continue

    if m1._timestamp < m2._timestamp:
        # m1 is older, pass through m1
        #print("write m1", m2._timestamp - m1._timestamp)
        output.write(m1.get_msgbuf())
        m1 = None
        continue

    # m2 is older
    #print("write m2")
    write_message(m2)
    m2 = None
