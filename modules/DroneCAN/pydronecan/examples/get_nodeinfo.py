#!/usr/bin/env python3

'''
loop calling GetNodeInfo on a node
'''
import dronecan, time
from dronecan import uavcan

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='loop calling GetNodeInfo')
parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
parser.add_argument("--target-node-id", default=101, type=int, help="target CAN node ID")
parser.add_argument("--rate", default=1, type=float, help="request rate")
parser.add_argument("--canfd", default=False, action="store_true", help="send as CANFD")
parser.add_argument("port", default=None, type=str, help="CAN port")
args = parser.parse_args()

# Initializing a DroneCAN node instance.
node = dronecan.make_node(args.port, node_id=args.node_id, bitrate=args.bitrate)

last_req = time.time()

def handle_node_info(msg):
    print('CANFD=%d' % msg.transfer.canfd)
    print(dronecan.to_yaml(msg))

while True:
    try:
        node.spin(timeout=1.0/args.rate)
        now = time.time()
        if now - last_req > 1.0/args.rate:
            req = uavcan.protocol.GetNodeInfo.Request(timeout_sec=0.5)
            node.request(req, args.target_node_id, handle_node_info, canfd=args.canfd)

    except Exception as ex:
        print(ex)
