#
# Copyright (C) 2023 DroneCAN Development Team  <dronecan.org>
#
# This software is distributed under the terms of the MIT License.
#
'''
 driver for CAN over UDP multicast

 multicast address is 239.65.82.B, last octet is logical bus number
 UDP port number is 57732

 packet format is

   - MAGIC 0x2934 16 bit
   - 16 bit CRC CRC-16-CCITT (over all bytes after CRC)
   - 16 bit flags
   - 32 bit message ID
   - data[]

 all data is little endian

 standard URLs
   mcast:  -> first bus
   mcast:0  -> first bus
   mcast:1  -> 2nd bus
'''

MCAST_ADDRESS_BASE = "239.65.82"
MCAST_PORT = 57732
MCAST_MAGIC = 0x2934
MCAST_FLAG_CANFD = 0x0001
MCAST_MAX_PKT_LEN = 74 # 64 byte data + 10 byte header

import os
import sys
import time
import multiprocessing
import socket
import struct
import errno
import select
import dronecan.dsdl.common as common
from logging import getLogger
from .common import DriverError, CANFrame, AbstractDriver

try:
    import queue
except ImportError:
    # noinspection PyPep8Naming,PyUnresolvedReferences
    import Queue as queue

if 'darwin' in sys.platform:
    RX_QUEUE_SIZE = 32767   # http://stackoverflow.com/questions/5900985/multiprocessing-queue-maxsize-limit-is-32767
else:
    RX_QUEUE_SIZE = 1000000
TX_QUEUE_SIZE = 1000

logger = getLogger(__name__)

RUNNING_ON_WINDOWS = False
try:
    # noinspection PyUnresolvedReferences
    sys.getwindowsversion()
    RUNNING_ON_WINDOWS = True
except Exception:
    pass

def io_process(url, tx_queue, rx_queue):
    port = None
    port_out = None
    myport = None
    need_connect = True

    def connect():
        nonlocal port, port_out

        a = url.split(':')
        # acceptable URL forms
        # mcast:  -> DEFAULT_MCAST_ADDRESS
        mcast_port = MCAST_PORT
        mcast_bus = 0
        if len(a) == 1 and len(a[0]) > 0:
            mcast_bus = int(a[0])
        mcast_ip = MCAST_ADDRESS_BASE + ".%u" % mcast_bus

        port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        if port is None:
            raise DriverError('unable to connect to %s' % url)
        port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if RUNNING_ON_WINDOWS:
            port.bind(('0.0.0.0', mcast_port))
        else:
            port.bind((mcast_ip, mcast_port))
        mreq = struct.pack("4sl", socket.inet_aton(mcast_ip), socket.INADDR_ANY)
        port.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        port.setblocking(0)

        port_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if port_out is None:
            raise DriverError('unable to connect to %s' % url)
        port_out.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        port_out.setblocking(0)
        port_out.connect((mcast_ip, mcast_port))


    while True:
        if need_connect:
            time.sleep(0.1)
            need_connect = False
            connect()
            continue

        while not tx_queue.empty():
            frame = tx_queue.get()
            message_id = frame.id
            if frame.extended:
                message_id |= 1<<31
            flags = MCAST_FLAG_CANFD if frame.canfd else 0
            body = struct.pack("<HI", flags, message_id) + frame.data
            hdr = struct.pack("<HH", MCAST_MAGIC, common.crc16_from_bytes(body))
            pkt = hdr + body

            try:
                port_out.send(pkt)
            except Exception as ex:
                need_connect = True
                continue

        try:
            select.select([port.fileno()], [], [], 0.01)
        except Exception as ex:
            need_connect = True
            pass

        try:
            data, new_addr = port.recvfrom(MCAST_MAX_PKT_LEN)
            if myport is None:
                try:
                    (myaddr,myport) = port_out.getsockname()
                except Exception:
                    pass
        except socket.error as e:
            if e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK, errno.ECONNREFUSED ]:
                continue
            need_connect = True
            continue
        if myport == new_addr[1]:
            # data from ourselves, discard
            continue

        if len(data) < 10:
            continue
        magic, crc16, flags, message_id = struct.unpack("<HHHI", data[:10])
        if magic != MCAST_MAGIC:
            continue
        if crc16 != common.crc16_from_bytes(data[4:]):
            continue

        is_extended = (message_id & (1<<31)) != 0
        is_canfd = flags & MCAST_FLAG_CANFD != 0
        canid = message_id & 0x1FFFFFFF
        frame = CANFrame(canid, data[10:], is_extended, canfd=is_canfd)
        rx_queue.put_nowait(frame)


# UDP multicast CAN driver
#
class mcast(AbstractDriver):
    """
    Driver for multicast UDP CAN
    """

    def __init__(self, url, **kwargs):
        super(mcast, self).__init__()

        self.rx_queue = multiprocessing.Queue(maxsize=RX_QUEUE_SIZE)
        self.tx_queue = multiprocessing.Queue(maxsize=TX_QUEUE_SIZE)

        self.proc = multiprocessing.Process(target=io_process, name='mcast_io_process',
                                            args=(url, self.tx_queue, self.rx_queue))
        self.proc.daemon = True
        self.proc.start()

    def close(self):
        pass

    def __del__(self):
        self.close()

    def receive(self, timeout=None):
        tstart = time.time()
        while True:
            try:
                frame = self.rx_queue.get(block=0)
            except queue.Empty:
                frame = None
            if frame is not None:
                self._rx_hook(frame)
                return frame
            if timeout is not None:
                timeout = max(timeout, 0.001)
                if time.time() >= tstart + timeout:
                    return

    def send_frame(self, frame):
        self._tx_hook(frame)
        self.tx_queue.put_nowait(frame)
