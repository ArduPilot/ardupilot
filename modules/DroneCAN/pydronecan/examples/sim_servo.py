#!/usr/bin/env python3
'''
simulate CAN actuators
'''

import dronecan, time, math

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='simulate DroneCAN servo')
parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
parser.add_argument("--debug", action='store_true', help="enable debug")
parser.add_argument("--rate", type=float, default=70, help="broadcast rate Hz")
parser.add_argument("uri", default=None, type=str, help="CAN URI")
args = parser.parse_args()

def publish_ActuatorStatus():
    '''send ActuatorStatus message'''
    msg = dronecan.uavcan.equipment.actuator.Status()
    msg.position = math.sin(time.time()*math.pi*2)
    msg.force = 1
    msg.speed = 2
    msg.power_rating_pct = 17

    # two actuators
    msg.actuator_id = 3
    node.broadcast(msg)

    msg.actuator_id = 5
    node.broadcast(msg)
    
    if args.debug:
        # display the message on the console in human readable format
        print(dronecan.to_yaml(msg))

# Initializing a DroneCAN node instance.
node = dronecan.make_node(args.uri, node_id=args.node_id, bitrate=1000000)

# setup to publish servo status
node.periodic(1.0/args.rate, publish_ActuatorStatus)

# Running the node until the application is terminated
while True:
    try:
        node.spin()
    except KeyboardInterrupt:
        break
    except dronecan.transport.TransferError as ex:
        print(ex)
        pass
