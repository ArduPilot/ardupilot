#!/usr/bin/env python3

import errno
import socket
import sys
import time

from pymavlink import fgFDM


class udp_socket(object):
    """A UDP socket."""
    def __init__(self, device, blocking=True, is_input=True):
        a = device.split(':')
        if len(a) != 2:
            print("UDP ports must be specified as host:port")
            sys.exit(1)
        self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if is_input:
            self.port.bind((a[0], int(a[1])))
            self.destination_addr = None
        else:
            self.destination_addr = (a[0], int(a[1]))
        if not blocking:
            self.port.setblocking(0)
        self.last_address = None

    def recv(self, n=1000):
        try:
            data, self.last_address = self.port.recvfrom(n)
        except socket.error as e:
            if e.errno in [errno.EAGAIN, errno.EWOULDBLOCK]:
                return ""
            raise
        return data

    def write(self, buf):
        try:
            if self.destination_addr:
                self.port.sendto(buf, self.destination_addr)
            else:
                self.port.sendto(buf, self.last_addr)
        except socket.error:
            pass


udp = udp_socket("127.0.0.1:5123")
fgout = udp_socket("127.0.0.1:5124", is_input=False)

tlast = time.time()
count = 0

fg = fgFDM.fgFDM()

while True:
    udp_buffer = udp.recv(1000)
    fg.parse(udp_buffer)
    fgout.write(fg.pack())
    count += 1
    if time.time() - tlast > 1.0:
        print("%u FPS len=%u" % (count, len(udp_buffer)))
        count = 0
        tlast = time.time()
        print(fg.get('latitude', units='degrees'),
              fg.get('longitude', units='degrees'),
              fg.get('altitude', units='meters'),
              fg.get('vcas', units='mps'))
