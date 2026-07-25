#!/usr/bin/env python3
import dronecan, time, threading, socket

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='mavlink DroneCAN tunnel')
parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
parser.add_argument("--debug", action='store_true', help="enable debug")
parser.add_argument("--tcp-host", type=str, default="localhost", help="tcp server host")
parser.add_argument("--tcp-port", type=int, default=5790, help="tcp server port")
parser.add_argument("--rate", type=float, default=10, help="broadcast rate Hz")
parser.add_argument("uri", default=None, type=str, help="CAN URI")
args = parser.parse_args()

# Create a TCP/IP socket
tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address
server_address = (args.tcp_host, args.tcp_port)
tcp_sock.bind(server_address)

# Listen for incoming connections
tcp_sock.listen(1)
connection = None

def handle_Targetted(msg):
    '''handle Targetted message'''
    global last_mode, is_armed, connection
    if args.debug:
        # display the message on the console in human readable format
        print(dronecan.to_yaml(msg))

    if msg.message.target_node == args.node_id:
        if msg.message.buffer:
            if connection:
                connection.sendall(bytes(msg.message.buffer))

def publish_NodeInfo():
    msg = dronecan.uavcan.protocol.NodeStatus()
    msg.uptime_sec = int(time.time())
    msg.health = msg.HEALTH_OK
    msg.mode = msg.MODE_OPERATIONAL 
    msg.submode = 0
    msg.vendor_specific_status_code = 0
    node.broadcast(msg)
    if args.debug:
        # display the message on the console in human readable format
        print(dronecan.to_yaml(msg))

def publish_MavlinkMsg(mav_pkt):
    '''send Tunnel message'''
    buffer = list(mav_pkt)
    chunks = [buffer[i:i + 120] for i in range(0, len(buffer), 120)]
    for chunk in chunks:
        msg = dronecan.uavcan.tunnel.Targetted()
        msg.serial_id = 1
        msg.target_node = 10 
        msg.protocol.protocol = 1 # MAVLink2
        msg.baudrate = 115200
        msg.options = msg.OPTION_LOCK_PORT
        msg.buffer = chunk 
        node.broadcast(msg)
        if args.debug:
            # display the message on the console in human readable format
            print(dronecan.to_yaml(msg))

def tcp_server_loop():
    global connection
    while True:
        print(f'Waiting for GCS Connection on tcp:{args.tcp_host}:{args.tcp_port}...')
        connection, client_address = tcp_sock.accept()
        print("GCS connected")

        try:
            # Receive the data in small chunks and retransmit it
            while True:
                data = connection.recv(1024)
                if data:
                    publish_MavlinkMsg(data)

        finally:
            # Clean up the connection
            connection.close()
        
# Initializing a DroneCAN node instance.
node = dronecan.make_node(args.uri, node_id=args.node_id, bitrate=1000000)

# Initializing a node monitor, so we can see what nodes are online
node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

node.periodic(1.0/args.rate, publish_NodeInfo)

node.add_handler(dronecan.uavcan.tunnel.Targetted, handle_Targetted)

tcp_server_loop_thread = threading.Thread(target=tcp_server_loop)
tcp_server_loop_thread.start()

# Running the node until the application is terminated or until first error.
try:
    node.spin()
except KeyboardInterrupt:
    tcp_server_loop.join()  
    if connection:
        connection.close()