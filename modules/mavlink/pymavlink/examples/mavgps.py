#!/usr/bin/env python3
"""
Allows connection of the uBlox u-Center software to
a uBlox GPS device connected to a PX4 or Pixhawk device,
using Mavlink's SERIAL_CONTROL support to route serial
traffic to/from the GPS, and exposing the data to u-Center
via a local TCP connection.

@author: Matthew Lloyd (github@matthewlloyd.net)
"""

from pymavlink import mavutil
from argparse import ArgumentParser
import socket


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--mavport", required=True,
                      help="Mavlink port name")
    parser.add_argument("--mavbaud", type=int,
                      help="Mavlink port baud rate", default=115200)
    parser.add_argument("--devnum", default=2, type=int,
                      help="PX4 UART device number (defaults to GPS port)")
    parser.add_argument("--devbaud", default=115200, type=int,
                      help="PX4 UART baud rate (defaults to u-Blox GPS baud)")
    parser.add_argument("--tcpport", default=2001, type=int,
                      help="local TCP port (defaults to %(default)s)")
    parser.add_argument("--tcpaddr", default='127.0.0.1', type=str,
                      help="local TCP address (defaults to %(default)s)")
    parser.add_argument("--debug", default=0, type=int,
                      help="debug level")
    parser.add_argument("--buffsize", default=128, type=int,
                      help="buffer size")
    parser.add_argument("--login", default=None, help="input log file")
    parser.add_argument("--logout", default=None, help="output log file")
    args = parser.parse_args()

    print("Connecting to MAVLINK...")
    mav_serialport = mavutil.MavlinkSerialPort(
        args.mavport, args.mavbaud,
        devnum=args.devnum, devbaud=args.devbaud, debug=args.debug)

    listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listen_sock.bind((args.tcpaddr, args.tcpport))
    listen_sock.listen(1)

    print("Waiting for a TCP connection.")
    print("Use tcp://%s:%d in u-Center." % (args.tcpaddr, args.tcpport))
    conn_sock, addr = listen_sock.accept()
    conn_sock.setblocking(0)  # non-blocking mode
    print("TCP connection accepted. Use Ctrl+C to exit.")

    login = None
    logout = None
    if args.login:
        login = open(args.login, "w")
    if args.logout:
        logout = open(args.logout, "w")

    while True:
        try:
            data = conn_sock.recv(args.buffsize)
            if data:
                if login:
                    login.write(data)
                if args.debug >= 1:
                    print('>', len(data))
                mav_serialport.write(data)
        except socket.error:
            pass

        data = mav_serialport.read(args.buffsize)
        if data:
            if logout:
                logout.write(data)
            if args.debug >= 1:
                print('<', len(data))
            conn_sock.send(data)


if __name__ == '__main__':
    main()
