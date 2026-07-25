#!/usr/bin/env python3
'''
example sending CANFD messages
'''

import dronecan, time, math

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='ESC throttle control example')
parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
parser.add_argument("--dna-server", action='store_true', default=False, help="run DNA server")
parser.add_argument("port", default=None, type=str, help="serial port")
args = parser.parse_args()
    
# Initializing a DroneCAN node instance.
node = dronecan.make_node(args.port, node_id=args.node_id, bitrate=args.bitrate)

# Initializing a node monitor, so we can see what nodes are online
node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

if args.dna_server:
    # optionally start a DNA server
    dynamic_node_id_allocator = dronecan.app.dynamic_node_id.CentralizedServer(node, node_monitor)

def publish_message():
    m = dronecan.uavcan.protocol.debug.LogMessage()
    m.source = "X"
    m.text = "Y"
    node.broadcast(m, canfd=True)

    m.source = "CANFDTest"
    m.text = "ThisIsALongerTest"
    node.broadcast(m, canfd=True)

    m.source = "CANFDTestLong"
    m.text = "ThisIsAVeryLongTest.MaryHadALittleLampWhosFleeceWasWhiteAsSnowAndEverywhereThatMaryWentThe"
    node.broadcast(m, canfd=True)
    
# setup to a LogMessage at 1Hz
node.periodic(1, publish_message)

# Running the node until the application is terminated or until first error.
try:
    node.spin()
except KeyboardInterrupt:
    pass
