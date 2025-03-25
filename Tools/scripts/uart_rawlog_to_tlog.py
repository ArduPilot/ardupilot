#!/usr/bin/env python3

'''
Convert a serial capture file into a mavlink tlog.

AP_FLAKE8_CLEAN
'''

import argparse
import os
import pathlib
import struct
import sys

from pymavlink import mavutil


class UARTRawLogToTLog():
    def __init__(self, filepath, output_filepath=None):
        self.filepath = filepath
        self.output_filepath = output_filepath

    def run(self):
        input_fh = sys.stdin
        if self.filepath is not None:
            input_fh = pathlib.Path(self.filepath).open('rb')

        output_fh = sys.stdout.buffer
        if self.output_filepath is not None:
            output_fh = pathlib.Path(self.output_filepath).open('wb')

        HEADER_FORMAT = "<I I H B"  # Little-endian: uint32_t, uint32_t, uint16_t, uint8_t
        HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
        MAGIC_NUMBER = 0xEAEF0D0F

        os.environ['MAVLINK20'] = '1'
        read_mav = mavutil.mavlink.MAVLink(self)
        read_mav.robust_parsing = True
        write_mav = mavutil.mavlink.MAVLink(self)
        write_mav.robust_parsing = True
        while True:
            header = input_fh.read(HEADER_SIZE)
            if len(header) < HEADER_SIZE:
                break
            magic, time_ms, length, flags = struct.unpack(HEADER_FORMAT, header)
            if magic != MAGIC_NUMBER:
                raise ValueError(f"{hex(magic)}")
            data = input_fh.read(length)

            if flags & 0x1:
                msgs = write_mav.parse_buffer(data)
            else:
                msgs = read_mav.parse_buffer(data)
            if msgs is None:
                continue
            for msg in msgs:
                output_fh.write(struct.pack(">Q", int(time_ms*1000)))
                output_fh.write(msg.get_msgbuf())


if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description='turn a captured serial stream into a tlog',
    )
    parser.add_argument('file', nargs='?', default=None, help='serial capture file')
    parser.add_argument('--output', '-o', default=None, help='output file')

    args = parser.parse_args()

    u = UARTRawLogToTLog(args.file, output_filepath=args.output)
    u.run()
