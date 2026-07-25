#!/usr/bin/env python3
'''
bridge between two CAN interfaces
this can be used to bridge a hardware device with SLCAN to a SITL vcan0 environment
'''

import dronecan
import time
import sys
import threading

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='CAN bridge - connect two CAN interfaces')
parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
parser.add_argument("ports", default=None, type=str, nargs='+', help="interfaces")
parser.add_argument("--filter-nodeid", nargs='+', type=int, help="filter by node ID"
                    " (only forward messages from this node ID)")
args = parser.parse_args()

if len(args.ports) < 2:
    print("Must specify at least 2 ports to connect")
    sys.exit(1)

drivers = []

for p in args.ports:
    try:
        drivers.append(dronecan.driver.make_driver(p, bitrate=args.bitrate))
    except Exception as ex:
        print("Unable to connect to %s - %s" % (p, ex))
        sys.exit(1)
    print("Connected to %s" % p)

class BridgeThread(object):
    def __init__(self, d, drivers, name):
        self.d = d
        self.drivers = drivers
        self.count = 0
        self.filter_nodeid = args.filter_nodeid
        if self.filter_nodeid:
            print("Filtering Node IDs: %s" % self.filter_nodeid)
        self.thd = threading.Thread(target=self.loop, name=name)
        self.thd.start()

    def loop(self):
        while True:
            try:
                frame = self.d.receive(timeout=0.1)
            except dronecan.driver.common.TransferError:
                continue
            if not frame:
                continue
            if self.filter_nodeid is not None:
                source_node_id = frame.id & 0x7F
                if source_node_id not in self.filter_nodeid:
                    continue
            self.count += 1
            for d in self.drivers:
                if d == self.d:
                    continue
                try:
                    d.send_frame(frame)
                except dronecan.driver.common.TxQueueFullError:
                    pass

threads = []
for i in range(len(drivers)):
    d = drivers[i]
    threads.append(BridgeThread(d, drivers, "d%u" % i))

last_print = time.time()

last_c = [0]*len(drivers)

while True:
    time.sleep(1)
    now = time.time()
    dt = now - last_print
    c = [ t.count for t in threads ]
    rates = []
    for i in range(len(drivers)):
        dcount = c[i] - last_c[i]
        rates.append("%.3f" % (dcount/dt))
    print("%s pkts/sec" % '/'.join(rates))
    last_print = now
    last_c = c[:]
