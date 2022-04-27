#!/usr/bin/env python3
'''
test script to check if all CAN GPS nodes are producing Fix2 frames at the expected rate
'''

import dronecan, time
from dronecan import uavcan

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='Fix2 gap example')
parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
parser.add_argument("--max-gap", default=0.25, type=float, help="max gap in seconds")
parser.add_argument("port", default=None, type=str, help="serial port or mavcan URI")
args = parser.parse_args()

# Initializing a DroneCAN node instance.
node = dronecan.make_node(args.port, node_id=args.node_id, bitrate=args.bitrate)

# Initializing a node monitor
node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

last_fix2 = {}

def handle_fix2(msg):
    nodeid = msg.transfer.source_node_id
    tstamp = msg.transfer.ts_real
    if not nodeid in last_fix2:
        last_fix2[nodeid] = tstamp
        return
    dt = tstamp - last_fix2[nodeid]
    last_fix2[nodeid] = tstamp
    if dt > args.max_gap:
        print("Node %u gap=%.3f" % (nodeid, dt))

# callback for printing ESC status message to stdout in human-readable YAML format.
node.add_handler(dronecan.uavcan.equipment.gnss.Fix2, handle_fix2)

while True:
    try:
        node.spin()
    except Exception as ex:
        print(ex)
