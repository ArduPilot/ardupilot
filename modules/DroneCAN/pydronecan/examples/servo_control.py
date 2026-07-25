#!/usr/bin/env python3
'''
demonstrate servo monitoring and servo control
the selected servo will be given sinisoidal control while printing position
'''

import dronecan
import time
import math
from dronecan import uavcan

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='test servo actuators')
parser.add_argument("port", default=None, help="CAN URI")
parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
parser.add_argument("--node-id", default=125, type=int, help="CAN node ID")
parser.add_argument("--target-node-id", default=100, type=int, help="target CAN node ID")
parser.add_argument("--rate", default=20, type=float, help="servo update rate")
parser.add_argument("--freq", default=1, type=float, help="servo sinisoid frequency")
parser.add_argument("--actuator-min", default=-1, type=float, help="actuator min")
parser.add_argument("--actuator-max", default=1, type=float, help="actuator max")
parser.add_argument("--actuator-id", default=1, type=float, help="actuator ID")

args = parser.parse_args()

port = args.port

# Initializing a DroneCAN node instance.
node = dronecan.make_node(port, node_id=args.node_id, bitrate=args.bitrate)

def handle_actuator_status(msg):
    '''handle actuator status message'''
    print(dronecan.to_yaml(msg))

def control_actuator():
    '''control for actuator, called at rate Hz'''
    t = time.time()
    sint = math.sin(t * args.freq * math.pi * 2)
    amplitude = 0.5 * (args.actuator_max-args.actuator_min)
    mid = args.actuator_min + amplitude
    v = mid + sint * amplitude

    req = uavcan.equipment.actuator.ArrayCommand()
    cmd = uavcan.equipment.actuator.Command()
    cmd.actuator_id = args.actuator_id
    cmd.command_type = cmd.COMMAND_TYPE_UNITLESS
    cmd.command_value = v
    req.commands = [ cmd ]

    node.broadcast(req)
    

# Subscribe only to actuator status messages
node.add_handler(uavcan.equipment.actuator.Status, handle_actuator_status)

node.periodic(1.0/args.rate, control_actuator)

while True:
    node.spin()
