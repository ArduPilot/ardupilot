#
# Copyright (C) 2023 DroneCAN Development Team  <dronecan.org>
#
# This software is distributed under the terms of the MIT License.
#
'''
 driver for reading/writing CAN frames from/to a file

   - MAGIC 0x2934 16 bit
   - 64 bit monotonic timestamp (microseconds)
   - 16 bit CRC CRC-16-CCITT (over all bytes after CRC)
   - 16 bit length
   - 16 bit flags
   - 32 bit message ID
   - data[]

 all data is little endian
'''
import os
import dronecan.dsdl.common as common
from logging import getLogger
from .common import DriverError, CANFrame, AbstractDriver
import struct
import time
import dronecan.dsdl.common as common

FILE_MAGIC = 0x2934
FILE_FLAG_CANFD = 0x0001
class file(AbstractDriver):

    """
    Driver for CAN Log File
    """

    def __init__(self, filename, **kwargs):
        """
        :param filename: filename to read/write
        :param kwargs: see AbstractDriver
        """
        super(file, self).__init__()
        self.filename = filename
        self.start_file_monotonic_ts = 0
        self.speedup = kwargs.get('speedup', 1)
        # check if read only option selected
        if kwargs.get('readonly', False):
            self.readonly = True
            self.file = open(filename, 'rb')
            self.curr_frame = None
            self.start_monotonic_ts = time.monotonic()
            self.file_size = os.path.getsize(filename)
            self.set_first_and_last_timestamp()
            self.eof = False
        else:
            self.readonly = False
            self.file = open(filename, 'wb')
            self.file.seek(0, os.SEEK_END)
        self.first_receive = False

    def close(self):
        if self.file is not None:
            self.file.flush()
            self.file.close()

    def __del__(self):
        self.close()

    def _read_frame(self):
        if (self.file.tell() >= self.file_size):
            self.eof = True
            return None, None
        # read header
        header = self.file.read(20)
        if len(header) < 20:
            return None, None
        magic, timestamp, crc, length, flags, msgid = struct.unpack('<HQHHHL', header)
        timestamp = timestamp/1e6
        if magic != FILE_MAGIC:
            raise DriverError("invalid magic")
        # read data
        data = self.file.read(length)
        if len(data) < length:
            raise DriverError("short read")
        # calculate CRC
        crc2 = common.crc16_from_bytes(data)
        if crc != crc2:
            raise DriverError("CRC error")
        # create frame
        is_extended = (msgid & (1<<31)) != 0
        is_canfd = flags & FILE_FLAG_CANFD != 0
        canid = msgid & 0x1FFFFFFF
        ts_monotonic = self.start_monotonic_ts + timestamp - self.start_file_monotonic_ts
        frame = CANFrame(canid,data,is_extended,ts_monotonic,canfd=is_canfd)
        return frame, timestamp

    def set_first_and_last_timestamp(self):
        # seek to beginning of file
        self.file.seek(0, os.SEEK_SET)
        # read first frame
        frame, timestamp = self._read_frame()
        if frame is None:
            raise DriverError("empty file")
        self.start_file_monotonic_ts = timestamp

        # run through file to find last frame
        while True:
            frame, timestamp = self._read_frame()
            self.end_file_monotonic_ts = timestamp
            if frame is None:
                break
        # seek to start of file
        self.eof = False
        self.file.seek(0, os.SEEK_SET)

    def get_start_timestamp(self):
        return self.start_file_monotonic_ts

    def get_last_timestamp(self):
        return self.end_file_monotonic_ts

    def set_start_timestamp(self, timestamp):
        self.start_monotonic_ts = timestamp

    def end_of_stream(self):
        if not self.readonly:
            return False
        return self.eof

    def stream_progress(self):
        '''stream progress of the current stream'''
        if not self.readonly or self.eof:
            return 100
        # return file seek position vs file size
        return (self.file.tell()/self.file_size)*100

    def receive(self, timeout=None):
        if not self.readonly or self.eof:
            if timeout is not None:
                time.sleep(timeout)
            return None
        curr_time = time.monotonic()
        if self.start_monotonic_ts is None:
            self.start_monotonic_ts = curr_time
        if self.curr_frame is None:
            self.curr_frame, _ = self._read_frame()
            if self.curr_frame is None:
                return None
        if self.speedup == -1:
            time_to_next_frame = 0
        else:
            time_to_next_frame = (self.curr_frame.ts_monotonic - curr_time)/self.speedup
        if time_to_next_frame > 0:
            if timeout is not None and time_to_next_frame <= timeout:
                # sleep until next frame
                time.sleep(time_to_next_frame)
            else:
                return None
        self.curr_frame.ts_real = time.time()
        frame = self.curr_frame
        self.curr_frame = None
        return frame

    def send_frame(self, frame, timeout=None):
        if self.readonly:
            return
        if self.first_receive is False:
            self.first_receive = True
            self.start_file_monotonic_ts = time.monotonic()
            self.start_monotonic_ts = self.start_file_monotonic_ts
        # calculate flags
        flags = 0
        if frame.canfd:
            flags |= FILE_FLAG_CANFD
        # calculate message id
        msgid = frame.id
        if frame.extended:
            msgid |= 1<<31
        # calculate CRC
        crc = common.crc16_from_bytes(frame.data)
        # write header
        header = struct.pack('<HQHHHL', FILE_MAGIC, int(frame.ts_monotonic*1e6), crc, len(frame.data), flags, msgid)
        self.file.write(header)
        # write data
        self.file.write(frame.data)
        # flush
        self.file.flush()
