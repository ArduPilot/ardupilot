#!/usr/bin/env python3

"""
map a serial port to an outgoing TCP connection
Released under GNU GPLv3 or later
"""
from argparse import ArgumentParser
import errno
import serial
import socket
import time

parser = ArgumentParser(description=__doc__)
parser.add_argument("--baudrate", default=57600, type=int, help="baud rate")
parser.add_argument("serialport", type=str, help="serial port")
parser.add_argument("desthost", type=str, help="destination host")
parser.add_argument("destport", type=int, help="destination port")
args = parser.parse_args()

serport = serial.Serial(args.serialport, args.baudrate, timeout=0)

tcpsock = None


def open_socket():
    global tcpsock
    try:
        tcpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcpsock.connect((args.desthost, args.destport))
        tcpsock.setblocking(0)
        print("Connected to %s:%u" % (args.desthost, args.destport))
    except Exception:
        pass

open_socket()

while True:
    gotdata = False

    if serport is None:
        try:
            print("Reopening %s" % args.serialport)
            serport = serial.Serial(args.serialport, args.baudrate, timeout=0)
        except Exception:
            time.sleep(1)
            continue
        print("Opened %s" % args.serialport)


    if tcpsock is None:
        open_socket()
        time.sleep(0.1)
        continue

    try:
        n = serport.inWaiting()
    except Exception:
        serport = None
        time.sleep(1)
        continue
    if n > 0:
        b = serport.read(n)
        if b:
            try:
                tcpsock.send(b)
            except socket.error:
                tcpsock.close()
                tcpsock = None
                continue
            gotdata = True

    try:
        b = tcpsock.recv(1000)
    except socket.error as e:
        if e.args[0] in [errno.EWOULDBLOCK, errno.EAGAIN]:
            time.sleep(0.02)
            continue
        tcpsock.close()
        tcpsock = None
        continue
    if b:
        serport.write(b)
        gotdata = True
    if not gotdata:
        time.sleep(0.02)
