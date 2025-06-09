#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3

# flake8: noqa

import argparse
import fcntl
import os
import serial
import struct
import sys
import time

try:
    import argcomplete
except:
    pass

SYSTEM_RESET  = struct.pack('B' * 4, 0x5A, 0x04, 0x02, 0x60)
MODE_I2C      = struct.pack('B' * 5, 0x5A, 0x05, 0x0A, 0x01, 0x6A)
MODE_UART     = struct.pack('B' * 5, 0x5A, 0x05, 0x0A, 0x00, 0x69)
SAVE_SETTINGS = struct.pack('B' * 4, 0x5A, 0x04, 0x11, 0x6F)

def cmd_switch_to_i2c(args):
    with serial.Serial(args.uart, 115200, timeout=1) as ser:
        ser.write(MODE_I2C)
        time.sleep(0.1)
        ser.write(SAVE_SETTINGS)


def cmd_switch_to_uart(args):
    I2C_SLAVE = 0x703

    with open(args.i2c_dev, "wb") as f:
        fcntl.ioctl(f.fileno(), I2C_SLAVE, args.addr)
        f.write(MODE_UART)
        time.sleep(0.1)
        f.write(SAVE_SETTINGS)


def parse_args(proc_args):
    parser = argparse.ArgumentParser(description="Configure TFMini Plus Lidar")

    subparsers = parser.add_subparsers(title="Commands", dest="command")

    p = subparsers.add_parser('switch-to-i2c', help="Switch sensor to I2C mode")
    p.add_argument('uart', help="UART device the sensor is currently using (e.g. /dev/ttyUSB0, /dev/ttyS1, etc)")
    p.set_defaults(func=cmd_switch_to_i2c)

    p = subparsers.add_parser('switch-to-uart', help="Switch sensor to UART mode")
    p.add_argument('i2c_dev', help="I2C device the sensor is currently using (e.g. /dev/i2c-1, /dev/i2c-2, etc)")
    p.add_argument('addr', type=lambda x: int(x, 0), help="I2C device's address the sensor is currently using (e.g. 0x10, 0x11, etc)")
    p.set_defaults(func=cmd_switch_to_uart)

    try:
        argcomplete.autocomplete(parser)
    except NameError:
        pass

    args = parser.parse_args(proc_args)
    if not hasattr(args, "func"):
        parser.print_help()
        return None

    return args

def main(*proc_args):
    args = parse_args(proc_args)
    if not args:
        return 1

    try:
        return args.func(args)
    except KeyboardInterrupt:
        return 130

    return 1

if __name__ == "__main__":
    main(*sys.argv[1:])
