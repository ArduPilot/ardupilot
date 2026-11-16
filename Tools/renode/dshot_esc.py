#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Scripted bidirectional DShot ESCs for the Renode DShot bridge.

Speaks the AM32 SITL / ESCSim UDP signal protocol used by
AP_STM32_Timer_BDShot, so it can stand in for AM32 SITL ESCs in tests.
Each ESC listens on its own port, port + 10 * (n - 1) for ESC n, and
answers every bidirectional frame with an eRPM reply unless it is silent.

Packets are little endian: u16 magic 0x4453, u8 type (1-3 = DShot150-600),
u8 len (4), u16 flags (bit0 idle high), u16 DShot frame. A reply carries the
GCR-decoded telemetry frame: 12 bit eRPM period value and 4 bit CRC.
"""

import argparse
import socket
import struct
import threading
import time

MAGIC = 0x4453
FLAG_IDLE_HIGH = 0x0001
PORT_STRIDE = 10
PACKET = struct.Struct('<HBBHH')


def checksum(value12):
    return (value12 ^ (value12 >> 4) ^ (value12 >> 8)) & 0xF


def erpm_telemetry_frame(erpm):
    '''16 bit telemetry frame for an eRPM, as an ESC sends it before GCR.'''
    if erpm <= 0:
        value = 0x0FFF
    else:
        period_us = max(1, round(60000000 / erpm))
        exponent = 0
        while period_us >= 512 and exponent < 7:
            period_us = (period_us + 1) >> 1
            exponent += 1
        value = (exponent << 9) | min(period_us, 511)
    return (value << 4) | (~checksum(value) & 0xF)


class ScriptedESC:
    '''One ESC on a UDP signal port; port 0 picks a free one.'''

    def __init__(self, port, erpm=0, silent=False):
        self.erpm = erpm
        self.silent = silent
        self.frames = 0
        self.bidirectional_frames = 0
        self.replies = 0
        self.last_frame = None
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', port))
        self.port = self.socket.getsockname()[1]
        self.socket.settimeout(0.2)
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while self.running:
            try:
                data, sender = self.socket.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(data) < PACKET.size:
                continue
            magic, kind, length, flags, frame = PACKET.unpack_from(data)
            if magic != MAGIC or length != 4 or not 1 <= kind <= 3:
                continue
            self.frames += 1
            self.last_frame = frame
            value = frame >> 4
            bidirectional = (frame & 0xF) == (~checksum(value) & 0xF)
            if not bidirectional:
                continue
            self.bidirectional_frames += 1
            if self.silent:
                continue
            reply = erpm_telemetry_frame(self.erpm)
            self.replies += 1
            self.socket.sendto(PACKET.pack(MAGIC, kind, 4, FLAG_IDLE_HIGH, reply), sender)

    def close(self):
        self.running = False
        self.thread.join(timeout=1)
        self.socket.close()


class ESCBench:
    '''ESCs 1..count on the bridge port layout. ESC n reports
    erpm + (n - 1) * erpm_step, so crossed channels are visible.'''

    def __init__(self, base_port, count, erpm=0, silent=(), erpm_step=0):
        self.escs = []
        try:
            for index in range(count):
                self.escs.append(ScriptedESC(base_port + PORT_STRIDE * index,
                                             erpm + index * erpm_step, index + 1 in silent))
        except OSError:
            self.close()
            raise

    def close(self):
        for esc in self.escs:
            esc.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=57733, help='ESC 1 signal port')
    parser.add_argument('--count', type=int, default=4, help='number of ESCs')
    parser.add_argument('--erpm', type=int, default=20000, help='ESC 1 reported eRPM')
    parser.add_argument('--erpm-step', type=int, default=0, help='eRPM added per ESC')
    parser.add_argument('--silent', type=int, action='append', default=[],
                        help='ESC number that never replies (repeatable)')
    args = parser.parse_args()
    with ESCBench(args.port, args.count, args.erpm, set(args.silent), args.erpm_step) as bench:
        try:
            while True:
                time.sleep(1)
                print(' '.join('ESC%u %u/%u' % (index + 1, esc.frames, esc.replies)
                               for index, esc in enumerate(bench.escs)), flush=True)
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
