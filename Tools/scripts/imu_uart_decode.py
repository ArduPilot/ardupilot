#!/usr/bin/env python
'''
script to decode IMU data on a UART
'''

import socket, struct, crc16, serial, sys

import argparse

parser = argparse.ArgumentParser(description='decode IMU data from ArduPilot')

parser.add_argument('--set-file', type=str, default=None, help='replace parameter defaults from a file')
parser.add_argument('--tcp', default=None, help='TCP endpoint as IP:port')
parser.add_argument('--device', default=None, help='serial port to read from')
parser.add_argument('--baudrate', default=921600, help='baudrate for serial port')

args = parser.parse_args()

if args.tcp is None and args.device is None:
    print("Must specicy --tcp or --device")
    sys.exit(1)

if args.tcp is not None:
    server_address = args.tcp.split(':')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.connect((server_address[0], int(server_address[1])))
else:
    port = serial.Serial(args.device, baudrate=args.baudrate)


class Packet(object):
    def __init__(self, pkt):
        '''parse the packet'''
        self.pkt = pkt
        self.delta_velocity = [0.0]*3
        self.delta_angle = [0.0]*3
        (self.magic,
             self.length,
             self.timestamp_us,
             self.delta_velocity[0],
             self.delta_velocity[1],
             self.delta_velocity[2],
             self.delta_angle[0],
             self.delta_angle[1],
             self.delta_angle[2],
             self.delta_velocity_dt,
             self.delta_angle_dt,
             self.counter,
             self.crc) = struct.unpack("<HHIffffffffHH", pkt)

    def crc_ok(self):
        '''check CRC is OK'''
        return self.crc == crc16.crc16xmodem(self.pkt[:-2])

    def __str__(self):
        return "%u (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f) %.3f %.3f" % (self.counter,
                                                                       self.delta_velocity[0],
                                                                       self.delta_velocity[1],
                                                                       self.delta_velocity[2],
                                                                       self.delta_angle[0],
                                                                       self.delta_angle[1],
                                                                       self.delta_angle[2],
                                                                       self.delta_velocity_dt,
                                                                       self.delta_angle_dt)


class UARTDecoder(object):
    def __init__(self):
        self.buf = ''
        self.magic = [0xc4, 0x29]
        self.full_length = 44
        self.last_counter = 0

    def add_byte(self, b):
        '''add one byte to buffer'''
        if len(self.buf) < len(self.magic) and ord(b) != self.magic[len(self.buf)]:
            self.buf = ''
            return None
        self.buf += b
        if len(self.buf) == self.full_length:
            ret = Packet(self.buf)
            self.buf = ''
            if ret.crc_ok():
                if ret.counter != self.last_counter+1:
                    print("LOST! %u %u\n", ret.counter, self.last_counter+1)
                self.last_counter = ret.counter
                return ret
        return None

    def add_data(self, msg):
        '''add a block of data'''
        ret = None
        for b in msg:
            r = self.add_byte(b)
            if r is not None:
                ret = r
        return ret


decoder = UARTDecoder()

while True:
    if args.tcp:
        msg, addr = sock.recvfrom(64)
    else:
        msg = port.read(64)
    pkt = decoder.add_data(msg)
    if pkt is not None:
        print(str(pkt))
