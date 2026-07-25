#!/usr/bin/env python3

import dronecan, time, math

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='simulate DroneCAN battery')
parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
parser.add_argument("--debug", action='store_true', help="enable debug")
parser.add_argument("--balance", action='store_true', help="simulate cell balance node")
parser.add_argument("--battery-id", type=int, default=1, help="battery ID")
parser.add_argument("--rate", type=float, default=10, help="broadcast rate Hz")
parser.add_argument("--voltage", type=float, default=24.6, help="voltage")
parser.add_argument("--num-cells", type=int, default=6, help="number of cells")
parser.add_argument("uri", default=None, type=str, help="CAN URI")
args = parser.parse_args()

C_TO_KELVIN = 273.15

def publish_BatteryInfo():
    '''send BatteryInfo message'''
    msg = dronecan.uavcan.equipment.power.BatteryInfo()
    msg.battery_id = args.battery_id
    msg.voltage = args.voltage
    msg.current = 0.3
    msg.temperature = 17+C_TO_KELVIN
    msg.state_of_charge_pct = 0
    msg.remaining_capacity_wh = 0
    msg.full_charge_capacity_wh = 0
    node.broadcast(msg)
    if args.debug:
        # display the message on the console in human readable format
        print(dronecan.to_yaml(msg))

def publish_BatteryInfoAux():
    '''send BatteryInfoAux message'''
    msg = dronecan.ardupilot.equipment.power.BatteryInfoAux()
    msg.battery_id = args.battery_id
    msg.voltage_cell = [0.0]*args.num_cells
    for i in range(args.num_cells):
        msg.voltage_cell[i] = args.voltage / args.num_cells
    node.broadcast(msg)
    if args.debug:
        # display the message on the console in human readable format
        print(dronecan.to_yaml(msg))
        
# Initializing a DroneCAN node instance.
node = dronecan.make_node(args.uri, node_id=args.node_id, bitrate=1000000)

# setup to publish battery info
if args.balance:
    node.periodic(1.0/args.rate, publish_BatteryInfoAux)
else:
    node.periodic(1.0/args.rate, publish_BatteryInfo)

# Running the node until the application is terminated or until first error.
try:
    node.spin()
except KeyboardInterrupt:
    pass
