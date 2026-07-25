#!/usr/bin/env python3
'''
map between a TCP port and a DroneCAN serial port tunnel
'''

import time
import sys
import dronecan
import socket

if __name__ == "__main__":
    import progressbar
    from argparse import ArgumentParser
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--baud", default=0, help="remote baudrate", type=int)
    parser.add_argument("--listen-port", default=2001, help="TCP listen port", type=int)
    parser.add_argument("--listen-address", default='0.0.0.0', help="TCP listen port")
    parser.add_argument("--lock", action='store_true', default=False, help="lock remote serial port")
    parser.add_argument("--can-target-node", type=int, default=None, help='target CAN node')
    parser.add_argument("--can-local-node", type=int, default=100, help='local CAN node')
    parser.add_argument("--can-serial-dev", type=int, default=-1, help='target serial device')
    parser.add_argument("--logfile", default=None, help='log data to file')
    parser.add_argument("uri", default=None, help="DroneCAN URI")

    args = parser.parse_args()

    port = dronecan.DroneCANSerial(args.uri, args.can_target_node, args.can_serial_dev,
                                   lock_port=args.lock, baudrate=args.baud, node_id=args.can_local_node)

    listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listen_sock.bind((args.listen_address, args.listen_port))
    listen_sock.listen(1)

    print("Waiting for connection on %s:%u" % (args.listen_address, args.listen_port))
    sock, addr = listen_sock.accept()
    sock.setblocking(0)

    last_print = time.time()
    sock_recv_count = 0
    dronecan_recv_count = 0
    logfile = None if args.logfile is None else open(args.logfile,'wb')

    while True:
        buf = None
        try:
            buf = sock.recv(1024)
        except socket.error:
            pass
        if buf:
            sock_recv_count += len(buf)
            port.write(buf)
            if logfile is not None:
                logfile.write(buf)
        buf = port.read(1024)
        if buf:
            dronecan_recv_count += len(buf)
            if logfile is not None:
                logfile.write(buf)
            sock.send(buf)

        now = time.time()
        dt = now - last_print
        if dt >= 1.0:
            print("%.2f/%.2f bytes/sec" % (sock_recv_count/dt, dronecan_recv_count/dt))
            sock_recv_count = 0
            dronecan_recv_count = 0
            last_print = now
