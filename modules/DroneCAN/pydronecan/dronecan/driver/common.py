#
# Copyright (C) 2014-2016  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ben Dyer <ben_dyer@mac.com>
#         Pavel Kirienko <pavel.kirienko@zubax.com>
#

import time
import sys
from logging import getLogger
from .. import UAVCANException


logger = getLogger(__name__)


class DriverError(UAVCANException):
    pass


class TxQueueFullError(DriverError):
    pass


class CANFrame:

    def __init__(self, can_id, data, extended, ts_monotonic=None, ts_real=None, canfd=False):
        self.id = can_id
        self.data = data
        self.extended = extended
        self.ts_monotonic = ts_monotonic or time.monotonic()
        self.ts_real = ts_real or time.time()
        self.canfd = canfd
        if self.canfd:
            self.MAX_DATA_LENGTH = 64
        else:
            self.MAX_DATA_LENGTH = 8
        if isinstance(self.data, list):
            self.data = bytearray(self.data)

    def __str__(self):
        if sys.version_info[0] > 2:
            b2int = lambda x: x
        else:
            b2int = ord

        id_str = ('%s %0*x' % ('FD' if self.canfd else '',8 if self.extended else 3, self.id)).rjust(11)
        hex_data = ' '.join(['%02x' % b2int(x) for x in self.data]).ljust(3 * self.MAX_DATA_LENGTH)
        ascii_data = ''.join([(chr(x) if 32 <= x <= 126 else '.') for x in self.data])

        return "%12.6f %12.6f  %s  %s  '%s'" % \
               (self.ts_monotonic, self.ts_real, id_str, hex_data, ascii_data)

    def dlc_to_datalength(dlc):
        # Data Length Code      9  10  11  12  13  14  15
        # Number of data bytes 12  16  20  24  32  48  64
        if (dlc <= 8):
            return dlc
        elif (dlc == 9):
            return 12
        elif (dlc == 10):
            return 16
        elif (dlc == 11):
            return 20
        elif (dlc == 12):
            return 24
        elif (dlc == 13):
            return 32
        elif (dlc == 14):
            return 48
        return 64

    def datalength_to_dlc(data_length):
        if (data_length <= 8):
            return data_length
        elif (data_length <= 12):
            return 9
        elif (data_length <= 16):
            return 10
        elif (data_length <= 20):
            return 11
        elif (data_length <= 24):
            return 12
        elif (data_length <= 32):
            return 13
        elif (data_length <= 48):
            return 14
        return 15

    __repr__ = __str__


class AbstractDriver(object):
    FRAME_DIRECTION_INCOMING = 'rx'
    FRAME_DIRECTION_OUTGOING = 'tx'

    class HookRemover:
        def __init__(self, remover):
            self.remove = remover

    def __init__(self):
        self._io_hooks = []

    def add_io_hook(self, hook):
        """
        Args:
            hook:   This hook will be invoked for every incoming and outgoing CAN frame.
                    Hook arguments: (direction, frame)
                    See FRAME_DIRECTION_*, CANFrame.
        """
        def proxy(*args):
            hook(*args)

        self._io_hooks.append(proxy)

        return self.HookRemover(lambda: self._io_hooks.remove(proxy))

    def _call_io_hooks(self, direction, frame):
        for h in self._io_hooks:
            try:
                h(direction, frame)
            except Exception as ex:
                logger.error('Uncaught exception from CAN IO hook: %r', ex, exc_info=True)

    def _tx_hook(self, frame):
        self._call_io_hooks(self.FRAME_DIRECTION_OUTGOING, frame)

    def _rx_hook(self, frame):
        self._call_io_hooks(self.FRAME_DIRECTION_INCOMING, frame)

    def set_filter_list(self, ids):
        '''set list of message IDs to accept, sent to the remote capture node with mavcan'''
        pass

    def get_filter_list(self, ids):
        '''get list of message IDs to accept, None means accept all'''
        return None
    
    def set_bus(self, busnum):
        '''set the remote bus number to attach to'''
        pass

    def get_bus(self):
        '''get the remote bus number we are attached to'''
        return None

    def get_filter_list(self):
        '''get the current filter list'''
        return None
    
    def set_bus(self, busnum):
        '''set the remote bus number to attach to'''
        pass

    def set_signing_passphrase(self, passphrase):
        '''set MAVLink2 signing passphrase'''
        pass

    def stream_progress(self):
        '''stream progress of the current stream'''
        pass

    def end_of_stream(self):
        '''end of stream'''
        pass

    def send(self, message_id, message, extended=False, canfd=False):
        self.send_frame(CANFrame(message_id, message, extended, canfd=canfd))
