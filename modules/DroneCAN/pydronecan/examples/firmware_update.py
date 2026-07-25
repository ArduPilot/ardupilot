#!/usr/bin/env python3

import dronecan
from argparse import ArgumentParser

import time
import sys
import os
import base64
import zlib
import struct
from dronecan.introspect import value_to_constant_name, to_yaml

# import logging
# logging.basicConfig(level=logging.DEBUG)

parser = ArgumentParser(description='dump all DroneCAN messages')
parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
parser.add_argument("--target-node-id", default=-1, type=int, help="Target CAN node ID (-1 for auto)")
parser.add_argument("--dna", action='store_true', default=False, help="run dynamic node allocation server")
parser.add_argument("--port", default='/dev/ttyACM0', type=str, help="serial port")
parser.add_argument("--fw", default='', required=True, type=str, help="app firmware path")

args = parser.parse_args()

firmware_path = os.path.abspath(args.fw)

# Ensure it is readable
try:
    with open(firmware_path, 'rb') as f:
        f.read(100)
except Exception as ex:
    print('Could not read file')
    sys.exit(1)

# map file path to a hash to keep it in a single packet
file_hash = base64.b64encode(struct.pack("<I",zlib.crc32(bytearray(firmware_path,'utf-8'))))[:7].decode('utf-8')

# Create the CAN Node
global node
print('Starting server as Node ID', args.node_id)
node = dronecan.make_node(args.port, node_id=args.node_id, bitrate=args.bitrate)

node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

if args.dna:
    dynamic_node_id_allocator = dronecan.app.dynamic_node_id.CentralizedServer(node, node_monitor)


# Waiting for at least one other node to appear online
print('Waiting for node to come online')
while len(node_monitor.get_all_node_id()) < 1:
    try:
        node.spin(timeout=0.2)
    except KeyboardInterrupt:
        sys.exit(0)
    except dronecan.transport.TransferError:
        pass

if args.target_node_id != -1:
    target_node_id = args.target_node_id
else:
    target_node_id = int(list(node_monitor.get_all_node_id())[0])
print("Target node: " + str(target_node_id))

# Set up the file server
file_server = dronecan.app.file_server.FileServer(node,
                                                  path_map={file_hash:firmware_path})

update_started = False
update_complete = False
start_time = None

def on_node_status(e):
    global update_started
    global update_complete
    global start_time

    if e.transfer.source_node_id == target_node_id:
        if e.message.mode == e.message.MODE_SOFTWARE_UPDATE:
            if not update_started:
                print('Performing update')
                update_started = True;
                start_time = time.time()
                #request_update()
        else:
            if update_started:
                elapsed = time.time() - start_time
                print('Update complete in %.2f seconds' % elapsed)
                update_complete = True;


def on_response(e):
    if e is not None:
        print('Firmware update response:', to_yaml(e.response))
        if e.response.error != e.response.ERROR_IN_PROGRESS:
            # NOTE: the timing here is fickle, we can't be too early
            # because we are trying to catch the bootloader
            node.defer(4, request_update)


def request_update():
    global update_started

    if not update_started:
        print('REQUESTING UPDATE')
        request = dronecan.uavcan.protocol.file.BeginFirmwareUpdate.Request(
                    source_node_id=node.node_id,
                    image_file_remote_path=dronecan.uavcan.protocol.file.Path(path=file_hash))
        node.request(request, target_node_id, on_response, priority=30)


node_status_handle = node.add_handler(dronecan.uavcan.protocol.NodeStatus, on_node_status)

request_update()

while not update_started or not update_complete:
    try:
        node.spin(0.1)
    except KeyboardInterrupt:
        sys.exit(0)
    except dronecan.transport.TransferError:
        pass

node.close()
