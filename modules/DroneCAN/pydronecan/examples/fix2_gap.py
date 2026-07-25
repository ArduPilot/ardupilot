#!/usr/bin/env python3

import dronecan, time
from dronecan import uavcan

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='Fix2 gap example')
parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
parser.add_argument("--source-node-id", default=None, type=int, help="Source CAN node ID")
parser.add_argument("--threshold", default=0.25, type=float, help="Threhold for showing gap")
parser.add_argument("port", default=None, type=str, help="serial port")
args = parser.parse_args()

# Initializing a DroneCAN node instance.
node = dronecan.make_node(args.port, node_id=args.node_id, bitrate=args.bitrate)

# Initializing a node monitor
node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

last_fix2 = {}
lowpass = {}

def handle_fix2(msg):
    nodeid = msg.transfer.source_node_id
    if args.source_node_id is not None and nodeid != args.source_node_id:
        return
    tstamp = msg.transfer.ts_real
    if not nodeid in last_fix2:
        last_fix2[nodeid] = tstamp
        lowpass[nodeid] = 0.2
        return
    dt = tstamp - last_fix2[nodeid]
    lowpass[nodeid] = 0.9 * lowpass[nodeid] + 0.1 * dt
    last_fix2[nodeid] = tstamp
    if dt > args.threshold:
        print("gap[%u] %.3f avg=%.2fHz" % (nodeid, dt, 1.0/lowpass[nodeid]))

# callback for printing ESC status message to stdout in human-readable YAML format.
node.add_handler(dronecan.uavcan.equipment.gnss.Fix2, handle_fix2)

while True:
    try:
        node.spin()
    except Exception as ex:
        print(ex)
