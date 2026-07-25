#!/usr/bin/env python3

import dronecan, time, math

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='ESC throttle control example')
parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
parser.add_argument("--dna-server", action='store_true', default=False, help="run DNA server")
parser.add_argument("--send-safety", action='store_true', default=False, help="send safety off messages")
parser.add_argument("port", default=None, type=str, help="serial port")
args = parser.parse_args()
    

# Publishing setpoint values from this function; it is invoked periodically from the node thread.
def publish_throttle_setpoint():
    if args.send_safety:
        # optionally send safety off messages. These are needed for some ESCs
        message = dronecan.ardupilot.indication.SafetyState()
        message.status = message.STATUS_SAFETY_OFF
        node.broadcast(message)
        print(dronecan.to_yaml(message))

    # Generating a sine wave
    setpoint = int(512 * (math.sin(time.time()) + 2))
    # Commanding ESC with indices 0, 1, 2, 3 only
    commands = [setpoint, setpoint, setpoint, setpoint]
    message = dronecan.uavcan.equipment.esc.RawCommand(cmd=commands)
    node.broadcast(message)
    # display the message on the console in human readable format
    print(dronecan.to_yaml(message))

# Initializing a DroneCAN node instance.
node = dronecan.make_node(args.port, node_id=args.node_id, bitrate=args.bitrate)

# Initializing a node monitor, so we can see what nodes are online
node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

if args.dna_server:
    # optionally start a DNA server
    dynamic_node_id_allocator = dronecan.app.dynamic_node_id.CentralizedServer(node, node_monitor)

# Waiting for at least one other node to appear online
while len(node_monitor.get_all_node_id()) <= 1:
    print('Waiting for other nodes to become online...')
    node.spin(timeout=1)

print("There are %u nodes online" % len(node_monitor.get_all_node_id()))

# setup to publish ESC RawCommand at 20Hz
node.periodic(0.05, publish_throttle_setpoint)

# callback for printing ESC status message to stdout in human-readable YAML format.
node.add_handler(dronecan.uavcan.equipment.esc.Status, lambda msg: print(dronecan.to_yaml(msg)))

# Running the node until the application is terminated or until first error.
try:
    node.spin()
except KeyboardInterrupt:
    pass
