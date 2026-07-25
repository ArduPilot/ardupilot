#!/usr/bin/env python3
'''
simulate CAN rangefinder
'''

import dronecan, time, math

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='simulate DroneCAN rangefinder')
parser.add_argument("--node-id", default=120, type=int, help="CAN node ID")
parser.add_argument("--debug", action='store_true', help="enable debug")
parser.add_argument("--rate", type=float, default=20, help="broadcast rate Hz")
parser.add_argument("uri", default=None, type=str, help="CAN URI")
args = parser.parse_args()

# Initialize the DroneCAN node
node = dronecan.make_node(args.uri, node_id=args.node_id, bitrate=1000000)

# Load the range sensor DSDL definition
RangeMeasurement = dronecan.uavcan.equipment.range_sensor.Measurement

def publish_range_measurement():
    msg = RangeMeasurement()

    # Sensor ID (choose arbitrary ID, e.g. 1)
    msg.sensor_id = 1

    # Field of view (radians, example 30 degrees FOV)
    msg.field_of_view = math.radians(30)

    # Sensor type (LIDAR = 2)
    msg.sensor_type = 2

    # Reading type (VALID_RANGE = 1)
    msg.reading_type = 1

    # Generate dummy range data (oscillating between 0.5 and 5 meters)
    msg.range = 2.5 + 2.0 * math.sin(time.time() * 2 * math.pi * 0.2)

    node.broadcast(msg)

    if args.debug:
        print(dronecan.to_yaml(msg))

# schedule periodic publication
node.periodic(1.0 / args.rate, publish_range_measurement)

# Run the node
while True:
    try:
        node.spin()
    except KeyboardInterrupt:
        break
    except dronecan.transport.TransferError as ex:
        print(ex)
        pass
