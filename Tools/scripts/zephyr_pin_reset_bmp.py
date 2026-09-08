#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Hardware-reset the target by pulsing nRST through a Black Magic Probe.

The Black Magic Probe variant of zephyr_pin_reset.py. That script drives the
reset pin through pyocd, which only sees CMSIS-DAP probes - a BMP presents a
GDB remote-serial server instead, so `pyocd list` reports no probes at all and
the pyocd path cannot be used on a BMP bench.

This speaks the GDB remote-serial protocol directly over the probe's first CDC
interface, so it needs no gdb binary and no toolchain: it sends the two
monitor commands the probe understands, `swdp_scan` to bring the SW-DP up and
`reset` to pulse nRST.

This is the autonomous-flash-recipe reset, same shape as the pyocd one:
    uploader.py <apj> &  sleep 3;  zephyr_pin_reset_bmp.py

Usage: Tools/scripts/zephyr_pin_reset_bmp.py [port]
       default /dev/serial/by-id/usb-Black_Magic_Debug*-if00
"""
import glob
import sys
import time

import serial

DEFAULT_GLOB = '/dev/serial/by-id/usb-Black_Magic_Debug*-if00'


def rsp_packet(payload):
    '''Frame a GDB remote-serial packet: $<payload>#<2-hex checksum>.'''
    csum = sum(payload.encode()) & 0xFF
    return ('$%s#%02x' % (payload, csum)).encode()


def monitor(ser, command, settle=0.5):
    '''Send one `monitor <command>` as qRcmd and return the raw reply.'''
    payload = 'qRcmd,' + command.encode().hex()
    ser.write(b'+')
    ser.write(rsp_packet(payload))
    ser.flush()
    time.sleep(settle)
    return ser.read(4096)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    if port is None:
        found = sorted(glob.glob(DEFAULT_GLOB))
        if not found:
            print('no Black Magic Probe matching %s' % DEFAULT_GLOB)
            return 1
        port = found[0]

    try:
        ser = serial.Serial(port, 115200, timeout=1)
    except OSError as e:
        print('cannot open %s: %s' % (port, e))
        return 1

    with ser:
        # bring the SW-DP up; without this the probe has no target to reset
        monitor(ser, 'swdp_scan', settle=1.5)
        # pulse nRST - "Pulse the nRST line - disconnects target" per `monitor help`
        monitor(ser, 'reset')

    print('nRST pulsed via %s' % port)
    return 0


if __name__ == '__main__':
    sys.exit(main())
