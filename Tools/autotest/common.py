'''
Common base class for each of the autotest suites

AP_FLAKE8_CLEAN

'''
from __future__ import print_function

import abc
import copy
import errno
import glob
import math
import os
import re
import shutil
import sys
import time
import traceback
import pexpect
import fnmatch
import operator
import numpy
import socket
import struct
import random
import tempfile
import threading
import enum

from MAVProxy.modules.lib import mp_util

from pymavlink import mavparm
from pymavlink import mavwp, mavutil, DFReader
from pymavlink import mavextra
from pymavlink.rotmat import Vector3
from pymavlink import quaternion

from pysim import util, vehicleinfo

try:
    import queue as Queue
except ImportError:
    import Queue


# Enumeration convenience class for mavlink POSITION_TARGET_TYPEMASK
class MAV_POS_TARGET_TYPE_MASK(enum.IntEnum):
    POS_IGNORE = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                  mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                  mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE)
    VEL_IGNORE = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                  mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                  mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE)
    ACC_IGNORE = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                  mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                  mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE)
    FORCE_SET  = mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET
    YAW_IGNORE = mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    YAW_RATE_IGNORE = mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    POS_ONLY   = VEL_IGNORE | ACC_IGNORE | YAW_IGNORE | YAW_RATE_IGNORE
    LAST_BYTE  = 0xF000


MAV_FRAMES_TO_TEST = [
    mavutil.mavlink.MAV_FRAME_GLOBAL,
    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT
]

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

# Check python version for abstract base class
if sys.version_info[0] >= 3 and sys.version_info[1] >= 4:
    ABC = abc.ABC
else:
    ABC = abc.ABCMeta('ABC', (), {})

if sys.version_info[0] >= 3:
    import io as StringIO  # srsly, we just did that.
else:
    import StringIO

try:
    from itertools import izip as zip
except ImportError:
    # probably python2
    pass


class ErrorException(Exception):
    """Base class for other exceptions"""
    pass


class AutoTestTimeoutException(ErrorException):
    pass


if sys.version_info[0] < 3:
    ConnectionResetError = AutoTestTimeoutException


class WaitModeTimeout(AutoTestTimeoutException):
    """Thrown when fails to achieve given mode change."""
    pass


class WaitAltitudeTimout(AutoTestTimeoutException):
    """Thrown when fails to achieve given altitude range."""
    pass


class WaitGroundSpeedTimeout(AutoTestTimeoutException):
    """Thrown when fails to achieve given ground speed range."""
    pass


class WaitRollTimeout(AutoTestTimeoutException):
    """Thrown when fails to achieve given roll in degrees."""
    pass


class WaitPitchTimeout(AutoTestTimeoutException):
    """Thrown when fails to achieve given pitch in degrees."""
    pass


class WaitHeadingTimeout(AutoTestTimeoutException):
    """Thrown when fails to achieve given heading."""
    pass


class WaitDistanceTimeout(AutoTestTimeoutException):
    """Thrown when fails to attain distance"""
    pass


class WaitLocationTimeout(AutoTestTimeoutException):
    """Thrown when fails to attain location"""
    pass


class WaitWaypointTimeout(AutoTestTimeoutException):
    """Thrown when fails to attain waypoint ranges"""
    pass


class SetRCTimeout(AutoTestTimeoutException):
    """Thrown when fails to send RC commands"""
    pass


class MsgRcvTimeoutException(AutoTestTimeoutException):
    """Thrown when fails to receive an expected message"""
    pass


class NotAchievedException(ErrorException):
    """Thrown when fails to achieve a goal"""
    pass


class YawSpeedNotAchievedException(NotAchievedException):
    """Thrown when fails to achieve given yaw speed."""
    pass


class SpeedVectorNotAchievedException(NotAchievedException):
    """Thrown when fails to achieve given speed vector."""
    pass


class PreconditionFailedException(ErrorException):
    """Thrown when a precondition for a test is not met"""
    pass


class ArmedAtEndOfTestException(ErrorException):
    """Created when test left vehicle armed"""
    pass


class Context(object):
    def __init__(self):
        self.parameters = []
        self.sitl_commandline_customised = False
        self.message_hooks = []
        self.collections = {}
        self.heartbeat_interval_ms = 1000
        self.original_heartbeat_interval_ms = None


# https://stackoverflow.com/questions/616645/how-do-i-duplicate-sys-stdout-to-a-log-file-in-python
class TeeBoth(object):
    def __init__(self, name, mode, mavproxy_logfile):
        self.file = open(name, mode)
        self.stdout = sys.stdout
        self.stderr = sys.stderr
        self.mavproxy_logfile = mavproxy_logfile
        self.mavproxy_logfile.set_fh(self)
        sys.stdout = self
        sys.stderr = self

    def close(self):
        sys.stdout = self.stdout
        sys.stderr = self.stderr
        self.mavproxy_logfile.set_fh(None)
        self.mavproxy_logfile = None
        self.file.close()
        self.file = None

    def write(self, data):
        self.file.write(data)
        self.stdout.write(data)

    def flush(self):
        self.file.flush()


class MAVProxyLogFile(object):
    def __init__(self):
        self.fh = None

    def close(self):
        pass

    def set_fh(self, fh):
        self.fh = fh

    def write(self, data):
        if self.fh is not None:
            self.fh.write(data)
        else:
            sys.stdout.write(data)

    def flush(self):
        if self.fh is not None:
            self.fh.flush()
        else:
            sys.stdout.flush()


class Telem(object):
    def __init__(self, destination_address, progress_function=None, verbose=False):
        self.destination_address = destination_address
        self.progress_function = progress_function
        self.verbose = verbose

        self.buffer = bytes()
        self.connected = False
        self.port = None
        self.progress_log = ""

    def progress(self, message):
        message = "%s: %s" % (self.progress_tag(), message)
        if self.progress_function is not None:
            self.progress_function(message)
            return
        if not self.verbose:
            self.progress_log += message
            return
        print(message)

    def connect(self):
        try:
            self.connected = False
            self.progress("Connecting to (%s:%u)" % self.destination_address)
            if self.port is not None:
                try:
                    self.port.close() # might be reopening
                except Exception:
                    pass
            self.port = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.port.connect(self.destination_address)
            self.port.setblocking(False)
            self.port.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
            self.connected = True
            self.progress("Connected")
        except IOError as e:
            self.progress("Failed to connect: %s" % str(e))
            time.sleep(0.5)
            return False
        return True

    def do_read(self):
        try:
            data = self.port.recv(1024)
        except socket.error as e:
            if e.errno not in [errno.EAGAIN, errno.EWOULDBLOCK]:
                self.progress("Exception: %s" % str(e))
                self.connected = False
            return bytes()
        if len(data) == 0:
            self.progress("EOF")
            self.connected = False
            return bytes()
#        self.progress("Read %u bytes" % len(data))
        return data

    def do_write(self, some_bytes):
        try:
            written = self.port.send(some_bytes)
        except socket.error as e:
            if e.errno in [errno.EAGAIN, errno.EWOULDBLOCK]:
                return 0
            self.progress("Exception: %s" % str(e))
            raise
        if written != len(some_bytes):
            raise ValueError("Short write")

    def update(self):
        if not self.connected:
            if not self.connect():
                return
        self.update_read()


class MSP_Generic(Telem):
    def __init__(self, destination_address):
        super(MSP_Generic, self).__init__(destination_address)

        self.callback = None

        self.STATE_IDLE = "IDLE"
        self.STATE_WANT_HEADER_DOLLARS = "WANT_DOLLARS"
        self.STATE_WANT_HEADER_M = "WANT_M"
        self.STATE_WANT_HEADER_GT = "WANT_GT"
        self.STATE_WANT_DATA_SIZE = "WANT_DATA_SIZE"
        self.STATE_WANT_COMMAND = "WANT_COMMAND"
        self.STATE_WANT_DATA = "WANT_DATA"
        self.STATE_WANT_CHECKSUM = "WANT_CHECKSUM"

        self.state = self.STATE_IDLE

    def progress(self, message):
        print("MSP: %s" % message)

    def set_state(self, state):
        # self.progress("Moving to state (%s)" % state)
        self.state = state

    def init_checksum(self, b):
        self.checksum = 0
        self.add_to_checksum(b)

    def add_to_checksum(self, b):
        self.checksum ^= (b & 0xFF)

    def process_command(self, cmd, data):
        if self.callback is not None:
            self.callback(cmd, data)
        else:
            print("cmd=%s" % str(cmd))

    def update_read(self):
        for byte in self.do_read():
            if sys.version_info[0] < 3:
                c = byte[0]
                byte = ord(c)
            else:
                c = chr(byte)
            # print("Got (0x%02x) (%s) (%s) state=%s" % (byte, chr(byte), str(type(byte)), self.state))
            if self.state == self.STATE_IDLE:
                # reset state
                self.set_state(self.STATE_WANT_HEADER_DOLLARS)
                # deliberate fallthrough right here
            if self.state == self.STATE_WANT_HEADER_DOLLARS:
                if c == '$':
                    self.set_state(self.STATE_WANT_HEADER_M)
                continue
            if self.state == self.STATE_WANT_HEADER_M:
                if c != 'M':
                    raise Exception("Malformed packet")
                self.set_state(self.STATE_WANT_HEADER_GT)
                continue
            if self.state == self.STATE_WANT_HEADER_GT:
                if c != '>':
                    raise Exception("Malformed packet")
                self.set_state(self.STATE_WANT_DATA_SIZE)
                continue
            if self.state == self.STATE_WANT_DATA_SIZE:
                self.data_size = byte
                self.set_state(self.STATE_WANT_COMMAND)
                self.data = bytearray()
                self.checksum = 0
                self.add_to_checksum(byte)
                continue
            if self.state == self.STATE_WANT_COMMAND:
                self.command = byte
                self.add_to_checksum(byte)
                if self.data_size != 0:
                    self.set_state(self.STATE_WANT_DATA)
                else:
                    self.set_state(self.STATE_WANT_CHECKSUM)
                continue
            if self.state == self.STATE_WANT_DATA:
                self.add_to_checksum(byte)
                self.data.append(byte)
                if len(self.data) == self.data_size:
                    self.set_state(self.STATE_WANT_CHECKSUM)
                continue
            if self.state == self.STATE_WANT_CHECKSUM:
                if self.checksum != byte:
                    raise Exception("Checksum fail (want=0x%02x calced=0x%02x" %
                                    (byte, self.checksum))
                self.process_command(self.command, self.data)
                self.set_state(self.STATE_IDLE)


class MSP_DJI(MSP_Generic):
    FRAME_GPS_RAW  = 106
    FRAME_ATTITUDE = 108

    def __init__(self, destination_address):
        super(MSP_DJI, self).__init__(destination_address)
        self.callback = self.command_callback
        self.frames = {}

    class Frame(object):
        def __init__(self, data):
            self.data = data

        def intn(self, offset, count):
            ret = 0
            for i in range(offset, offset+count):
                ret = ret | (ord(self.data[i]) << ((i-offset)*8))
            return ret

        def int32(self, offset):
            t = struct.unpack("<i", self.data[offset:offset+4])
            return t[0]

        def int16(self, offset):
            t = struct.unpack("<h", self.data[offset:offset+2])
            return t[0]

    class FrameATTITUDE(Frame):
        def roll(self):
            '''roll in degrees'''
            return self.int16(0) * 10

        def pitch(self):
            '''pitch in degrees'''
            return self.int16(2) * 10

        def yaw(self):
            '''yaw in degrees'''
            return self.int16(4)

    class FrameGPS_RAW(Frame):
        '''see gps_state_s'''
        def fix_type(self):
            return self.uint8(0)

        def num_sats(self):
            return self.uint8(1)

        def lat(self):
            return self.int32(2) / 1e7

        def lon(self):
            return self.int32(6) / 1e7

        def LocationInt(self):
            # other fields are available, I'm just lazy
            return LocationInt(self.int32(2), self.int32(6), 0, 0)

    def command_callback(self, frametype, data):
        # print("X: %s %s" % (str(frametype), str(data)))
        if frametype == MSP_DJI.FRAME_ATTITUDE:
            frame = MSP_DJI.FrameATTITUDE(data)
        elif frametype == MSP_DJI.FRAME_GPS_RAW:
            frame = MSP_DJI.FrameGPS_RAW(data)
        else:
            return
        self.frames[frametype] = frame

    def get_frame(self, frametype):
        return self.frames[frametype]


class LTM(Telem):
    def __init__(self, destination_address):
        super(LTM, self).__init__(destination_address)

        self.HEADER1 = 0x24
        self.HEADER2 = 0x54

        self.FRAME_G = 0x47
        self.FRAME_A = 0x41
        self.FRAME_S = 0x53

        self.frame_lengths = {
            self.FRAME_G: 18,
            self.FRAME_A: 10,
            self.FRAME_S: 11,
        }
        self.frame_lengths = {
            self.FRAME_G: 18,
            self.FRAME_A: 10,
            self.FRAME_S: 11,
        }

        self.data_by_id = {}
        self.frames = {}

    def g(self):
        return self.frames.get(self.FRAME_G, None)

    def a(self):
        return self.frames.get(self.FRAME_A, None)

    def s(self):
        return self.frames.get(self.FRAME_S, None)

    def progress_tag(self):
        return "LTM"

    def handle_data(self, dataid, value):
        self.progress("%u=%u" % (dataid, value))
        self.data_by_id[dataid] = value

    def consume_frame(self):
        b2 = self.buffer[2]
        if sys.version_info.major < 3:
            b2 = ord(b2)
        frame_type = b2
        frame_length = self.frame_lengths[frame_type]
        # check frame CRC
        crc = 0
        count = 0
        for c in self.buffer[3:frame_length-1]:
            if sys.version_info.major < 3:
                c = ord(c)
            crc ^= c
            count += 1
        buffer_crc = self.buffer[frame_length-1]
        if sys.version_info.major < 3:
            buffer_crc = ord(buffer_crc)
        if crc != buffer_crc:
            raise NotAchievedException("Invalid checksum on frame type %s" % str(chr(frame_type)))
#        self.progress("Received valid %s frame" % str(chr(frame_type)))

        class Frame(object):
            def __init__(self, buffer):
                self.buffer = buffer

            def intn(self, offset, count):
                ret = 0
                for i in range(offset, offset+count):
                    ret = ret | (ord(self.buffer[i]) << ((i-offset)*8))
                return ret

            def int32(self, offset):
                t = struct.unpack("<i", self.buffer[offset:offset+4])
                return t[0]
#                return self.intn(offset, 4)

            def int16(self, offset):
                t = struct.unpack("<h", self.buffer[offset:offset+2])
                return t[0]
#                return self.intn(offset, 2)

        class FrameG(Frame):
            def __init__(self, buffer):
                super(FrameG, self,).__init__(buffer)

            def lat(self):
                return self.int32(3)

            def lon(self):
                return self.int32(7)

            def gndspeed(self):
                ret = self.buffer[11]
                if sys.version_info.major < 3:
                    ret = ord(ret)
                return ret

            def alt(self):
                return self.int32(12)

            def sats(self):
                s = self.buffer[16]
                if sys.version_info.major < 3:
                    s = ord(s)
                return (s >> 2)

            def fix_type(self):
                s = self.buffer[16]
                if sys.version_info.major < 3:
                    s = ord(s)
                return s & 0b11

        class FrameA(Frame):
            def __init__(self, buffer):
                super(FrameA, self,).__init__(buffer)

            def pitch(self):
                return self.int16(3)

            def roll(self):
                return self.int16(5)

            def hdg(self):
                return self.int16(7)

        class FrameS(Frame):
            def __init__(self, buffer):
                super(FrameS, self,).__init__(buffer)

        if frame_type == self.FRAME_G:
            frame = FrameG(self.buffer[0:frame_length-1])
        elif frame_type == self.FRAME_A:
            frame = FrameA(self.buffer[0:frame_length-1])
        elif frame_type == self.FRAME_S:
            frame = FrameS(self.buffer[0:frame_length-1])
        else:
            raise NotAchievedException("Bad frame?!?!?!")
        self.buffer = self.buffer[frame_length:]
        self.frames[frame_type] = frame

    def update_read(self):
        self.buffer += self.do_read()
        while len(self.buffer):
            if len(self.buffer) == 0:
                break
            b0 = self.buffer[0]
            if sys.version_info.major < 3:
                b0 = ord(b0)
            if b0 != self.HEADER1:
                self.bad_chars += 1
                self.buffer = self.buffer[1:]
                continue
            b1 = self.buffer[1]
            if sys.version_info.major < 3:
                b1 = ord(b1)
            if b1 != self.HEADER2:
                self.bad_chars += 1
                self.buffer = self.buffer[1:]
                continue
            b2 = self.buffer[2]
            if sys.version_info.major < 3:
                b2 = ord(b2)
            if b2 not in [self.FRAME_G, self.FRAME_A, self.FRAME_S]:
                self.bad_chars += 1
                self.buffer = self.buffer[1:]
                continue
            frame_len = self.frame_lengths[b2]
            if len(self.buffer) < frame_len:
                continue
            self.consume_frame()

    def get_data(self, dataid):
        try:
            return self.data_by_id[dataid]
        except KeyError:
            pass
        return None


class CRSF(Telem):
    def __init__(self, destination_address):
        super(CRSF, self).__init__(destination_address)

        self.dataid_vtx_frame = 0
        self.dataid_vtx_telem = 1
        self.dataid_vtx_unknown = 2

        self.data_id_map = {
            self.dataid_vtx_frame: bytearray([0xC8, 0x8, 0xF, 0xCE, 0x30, 0x8, 0x16, 0xE9, 0x0, 0x5F]),
            self.dataid_vtx_telem: bytearray([0xC8, 0x7, 0x10, 0xCE, 0xE, 0x16, 0x65, 0x0, 0x1B]),
            self.dataid_vtx_unknown: bytearray([0xC8, 0x9, 0x8, 0x0, 0x9E, 0x0, 0x0, 0x0, 0x0, 0x0, 0x95]),
        }

    def write_data_id(self, dataid):
        self.do_write(self.data_id_map[dataid])

    def progress_tag(self):
        return "CRSF"


class DEVO(Telem):
    def __init__(self, destination_address):
        super(DEVO, self).__init__(destination_address)

        self.HEADER = 0xAA
        self.frame_length = 20

        # frame is 'None' until we receive a frame with VALID header and checksum
        self.frame = None
        self.bad_chars = 0

    def progress_tag(self):
        return "DEVO"

    def consume_frame(self):
        # check frame checksum
        checksum = 0
        for c in self.buffer[:self.frame_length-1]:
            if sys.version_info.major < 3:
                c = ord(c)
            checksum += c
        checksum &= 0xff    # since we receive 8 bit checksum
        buffer_checksum = self.buffer[self.frame_length-1]
        if sys.version_info.major < 3:
            buffer_checksum = ord(buffer_checksum)
        if checksum != buffer_checksum:
            raise NotAchievedException("Invalid checksum")

        class FRAME(object):
            def __init__(self, buffer):
                self.buffer = buffer

            def int32(self, offset):
                t = struct.unpack("<i", self.buffer[offset:offset+4])
                return t[0]

            def int16(self, offset):
                t = struct.unpack("<h", self.buffer[offset:offset+2])
                return t[0]

            def lon(self):
                return self.int32(1)

            def lat(self):
                return self.int32(5)

            def alt(self):
                return self.int32(9)

            def speed(self):
                return self.int16(13)

            def temp(self):
                return self.int16(15)

            def volt(self):
                return self.int16(17)

        self.frame = FRAME(self.buffer[0:self.frame_length-1])
        self.buffer = self.buffer[self.frame_length:]

    def update_read(self):
        self.buffer += self.do_read()
        while len(self.buffer):
            if len(self.buffer) == 0:
                break
            b0 = self.buffer[0]
            if sys.version_info.major < 3:
                b0 = ord(b0)
            if b0 != self.HEADER:
                self.bad_chars += 1
                self.buffer = self.buffer[1:]
                continue
            if len(self.buffer) < self.frame_length:
                continue
            self.consume_frame()


class FRSky(Telem):
    def __init__(self, destination_address, verbose=False):
        super(FRSky, self).__init__(destination_address, verbose=verbose)

        self.dataid_GPS_ALT_BP          = 0x01
        self.dataid_TEMP1               = 0x02
        self.dataid_FUEL                = 0x04
        self.dataid_TEMP2               = 0x05
        self.dataid_GPS_ALT_AP          = 0x09
        self.dataid_BARO_ALT_BP         = 0x10
        self.dataid_GPS_SPEED_BP        = 0x11
        self.dataid_GPS_LONG_BP         = 0x12
        self.dataid_GPS_LAT_BP          = 0x13
        self.dataid_GPS_COURS_BP        = 0x14
        self.dataid_GPS_SPEED_AP        = 0x19
        self.dataid_GPS_LONG_AP         = 0x1A
        self.dataid_GPS_LAT_AP          = 0x1B
        self.dataid_BARO_ALT_AP         = 0x21
        self.dataid_GPS_LONG_EW         = 0x22
        self.dataid_GPS_LAT_NS          = 0x23
        self.dataid_CURRENT             = 0x28
        self.dataid_VFAS                = 0x39


class FRSkyD(FRSky):
    def __init__(self, destination_address):
        super(FRSkyD, self).__init__(destination_address)

        self.state_WANT_START_STOP_D = 16,
        self.state_WANT_ID = 17
        self.state_WANT_BYTE1 = 18
        self.state_WANT_BYTE2 = 19

        self.START_STOP_D = 0x5E
        self.BYTESTUFF_D = 0x5D

        self.state = self.state_WANT_START_STOP_D

        self.data_by_id = {}
        self.bad_chars = 0

    def progress_tag(self):
        return "FRSkyD"

    def handle_data(self, dataid, value):
        self.progress("%u=%u" % (dataid, value))
        self.data_by_id[dataid] = value

    def update_read(self):
        self.buffer += self.do_read()
        consume = None
        while len(self.buffer):
            if consume is not None:
                self.buffer = self.buffer[consume:]
            if len(self.buffer) == 0:
                break
            consume = 1
            if sys.version_info.major >= 3:
                b = self.buffer[0]
            else:
                b = ord(self.buffer[0])
            if self.state == self.state_WANT_START_STOP_D:
                if b != self.START_STOP_D:
                    # we may come into a stream mid-way, so we can't judge
                    self.bad_chars += 1
                    continue
                self.state = self.state_WANT_ID
                continue
            elif self.state == self.state_WANT_ID:
                self.dataid = b
                self.state = self.state_WANT_BYTE1
                continue
            elif self.state in [self.state_WANT_BYTE1, self.state_WANT_BYTE2]:
                if b == 0x5D:
                    # byte-stuffed
                    if len(self.buffer) < 2:
                        # try again in a little while
                        consume = 0
                        return
                    if ord(self.buffer[1]) == 0x3E:
                        b = self.START_STOP_D
                    elif ord(self.buffer[1]) == 0x3D:
                        b = self.BYTESTUFF_D
                    else:
                        raise ValueError("Unknown stuffed byte")
                    consume = 2
                if self.state == self.state_WANT_BYTE1:
                    self.b1 = b
                    self.state = self.state_WANT_BYTE2
                    continue

                data = self.b1 | b << 8
                self.handle_data(self.dataid, data)
                self.state = self.state_WANT_START_STOP_D

    def get_data(self, dataid):
        try:
            return self.data_by_id[dataid]
        except KeyError:
            pass
        return None


class SPortPacket(object):
    def __init__(self):
        self.START_STOP_SPORT = 0x7E
        self.BYTESTUFF_SPORT  = 0x7D


class SPortUplinkPacket(SPortPacket):
    def __init__(self, appid0, appid1, data0, data1, data2, data3):
        super(SPortUplinkPacket, self).__init__()
        self.appid0 = appid0
        self.appid1 = appid1
        self.data0 = data0
        self.data1 = data1
        self.data2 = data2
        self.data3 = data3
        self.SENSOR_ID_UPLINK_ID         = 0x0D
        self.SPORT_UPLINK_FRAME          = 0x30
        self.uplink_id = self.SENSOR_ID_UPLINK_ID
        self.frame = self.SPORT_UPLINK_FRAME

    def packed(self):
        return struct.pack(
            '<BBBBBBBB',
            self.uplink_id,
            self.frame,
            self.appid0 & 0xff,
            self.appid1 & 0xff,
            self.data0 & 0xff,
            self.data1 & 0xff,
            self.data2 & 0xff,
            self.data3 & 0xff,
        )

    def update_checksum(self, byte):
        self.checksum += byte
        self.checksum += self.checksum >> 8
        self.checksum &= 0xFF

    def checksum(self):
        self.checksum = 0
        self.update_checksum(self.frame & 0xff)
        self.update_checksum(self.appid0 & 0xff)
        self.update_checksum(self.appid1 & 0xff)
        self.update_checksum(self.data0 & 0xff)
        self.update_checksum(self.data1 & 0xff)
        self.update_checksum(self.data2 & 0xff)
        self.update_checksum(self.data3 & 0xff)
        self.checksum = 0xff - ((self.checksum & 0xff) + (self.checksum >> 8))
        return self.checksum & 0xff

    def for_wire(self):
        out = bytearray()
        out.extend(self.packed())
        out.extend(struct.pack('<B', self.checksum()))
        stuffed = bytearray()
        stuffed.extend(struct.pack('<B', self.START_STOP_SPORT))
        for pbyte in out:
            if pbyte in [self.BYTESTUFF_SPORT,
                         self.START_STOP_SPORT]:
                # bytestuff
                stuffed.append(self.BYTESTUFF_SPORT)
                stuffed.append(pbyte ^ self.SPORT_FRAME_XOR)
            else:
                stuffed.append(pbyte)
        return stuffed


class SPortPollPacket(SPortPacket):
    def __init__(self, sensor):
        super(SPortPollPacket, self).__init__()
        self.sensor = sensor

    def for_wire(self):
        return struct.pack(
            '<BB',
            self.START_STOP_SPORT,
            self.sensor & 0xff,
        )


class MAVliteMessage(object):
    def __init__(self, msgid, body):
        self.msgid = msgid
        self.body = body
        self.SENSOR_ID_UPLINK_ID         = 0x0D
        self.SPORT_UPLINK_FRAME          = 0x30

    def checksum_bytes(self, some_bytes):
        checksum = 0
        for b in some_bytes:
            checksum += b
            checksum += checksum >> 8
            checksum &= 0xFF
        return checksum

    def to_sport_packets(self):
        ret = []
        all_bytes = bytearray([len(self.body), self.msgid])
        all_bytes.extend(self.body)

        # insert sequence numbers:
        seq = 0
        sequenced = bytearray()
        while len(all_bytes):
            chunk = all_bytes[0:5]
            all_bytes = all_bytes[5:]
            sequenced.append(seq)
            sequenced.extend(chunk)
            seq += 1

        # we may need another sport packet just for the checksum:
        if len(sequenced) % 6 == 0:
            sequenced.append(seq)
            seq += 1

        checksum = self.checksum_bytes(sequenced)
        sequenced.append(checksum)

        while len(sequenced):
            chunk = sequenced[0:6]
            sequenced = sequenced[6:]
            chunk.extend([0] * (6-len(chunk))) # pad to 6
            packet = SPortUplinkPacket(
                *chunk
            )
            ret.append(packet)
        return ret


class SPortToMAVlite(object):
    def __init__(self):
        self.state_WANT_LEN = "want len"
        self.state_WANT_MSGID = "want msgid"
        self.state_WANT_PAYLOAD = "want payload"
        self.state_WANT_CHECKSUM = "want checksum"
        self.state_MESSAGE_RECEIVED = "message received"

        self.reset()

    def progress(self, message):
        print("SPortToMAVLite: %s" % message)

    def reset(self):
        self.want_seq = 0
        self.all_bytes = bytearray()
        self.payload = bytearray()
        self.state = self.state_WANT_LEN

    def checksum_bytes(self, some_bytes):
        checksum = 0
        for b in some_bytes:
            checksum += b
            checksum += checksum >> 8
            checksum &= 0xFF
        return checksum

    def downlink_handler(self, some_bytes):
        '''adds some_bytes into a mavlite message'''
        if some_bytes[0] == 0x00:
            self.reset()
        if some_bytes[0] != self.want_seq:
            raise NotAchievedException("Unexpected seqno; want=%u got=%u" %
                                       (self.want_seq, some_bytes[0]))
        self.all_bytes.append(some_bytes[0])
        self.want_seq += 1
        for byte in some_bytes[1:]:
            if self.state == self.state_WANT_LEN:
                self.payload_len = byte
                self.all_bytes.append(byte)
                self.state = self.state_WANT_MSGID
                continue
            if self.state == self.state_WANT_MSGID:
                self.msgid = byte
                self.all_bytes.append(byte)
                if self.payload_len == 0:
                    self.state = self.state_WANT_CHECKSUM
                else:
                    self.state = self.state_WANT_PAYLOAD
                continue
            if self.state == self.state_WANT_PAYLOAD:
                self.payload.append(byte)
                self.all_bytes.append(byte)
                if len(self.payload) == self.payload_len:
                    self.state = self.state_WANT_CHECKSUM
                continue
            if self.state == self.state_WANT_CHECKSUM:
                calculated_checksum = self.checksum_bytes(self.all_bytes)
                if calculated_checksum != byte:
                    raise Exception("Checksum failure (calc=%u) (recv=%u)" % (calculated_checksum, byte))
                self.state = self.state_MESSAGE_RECEIVED
                break

    def get_message(self):
        if self.state != self.state_MESSAGE_RECEIVED:
            raise Exception("Wrong state")
        return MAVliteMessage(self.msgid, self.payload)


class FRSkySPort(FRSky):
    def __init__(self, destination_address, verbose=True, get_time=time.time):
        super(FRSkySPort, self).__init__(
            destination_address,
            verbose=verbose
        )

        self.get_time = get_time

        self.state_SEND_POLL = "sendpoll"
        self.state_WANT_FRAME_TYPE = "want_frame_type"
        self.state_WANT_ID1 = "want_id1"
        self.state_WANT_ID2 = "want id2"
        self.state_WANT_DATA = "want data"
        self.state_WANT_CRC = "want crc"

        self.START_STOP_SPORT     = 0x7E
        self.BYTESTUFF_SPORT      = 0x7D
        self.SPORT_DATA_FRAME     = 0x10
        self.SPORT_DOWNLINK_FRAME = 0x32
        self.SPORT_FRAME_XOR      = 0x20

        self.SENSOR_ID_VARIO             = 0x00 # Sensor ID  0
        self.SENSOR_ID_FAS               = 0x22 # Sensor ID  2
        self.SENSOR_ID_GPS               = 0x83 # Sensor ID  3
        self.SENSOR_ID_RPM               = 0xE4 # Sensor ID  4
        self.SENSOR_ID_SP2UR             = 0xC6 # Sensor ID  6
        self.SENSOR_ID_27                = 0x1B # Sensor ID 27

        # MAVlite support:
        self.SENSOR_ID_DOWNLINK1_ID      = 0x34
        self.SENSOR_ID_DOWNLINK2_ID      = 0x67
        self.SENSOR_ID_UPLINK_ID         = 0x0D

        self.state = self.state_WANT_FRAME_TYPE

        self.data_by_id = {}
        self.dataid_counts = {}
        self.bad_chars = 0

        self.poll_sent = 0
        self.sensor_id_poll_counts = {}

        self.id_descriptions = {
            0x5000: "status text (dynamic)",
            0x5006: "Attitude and range (dynamic)",
            0x800: "GPS lat or lon (600 with 1 sensor)",
            0x5005: "Vel and Yaw",
            0x5001: "AP status",
            0x5002: "GPS Status",
            0x5004: "Home",
            0x5008: "Battery 2 status",
            0x5003: "Battery 1 status",
            0x5007: "parameters",
            0x500A: "rpm",
            0x500B: "terrain",
            0x500C: "wind",

            # SPort non-passthrough:
            0x082F: "GALT", # gps altitude integer cm
            0x040F: "TMP1", # Tmp1
            0x060F: "Fuel", # fuel % 0-100
            0x041F: "TMP2", # Tmp2
            0x010F: "ALT",  # baro alt cm
            0x083F: "GSPD", # gps speed integer mm/s
            0x084F: "HDG",  # yaw in cd
            0x020F: "CURR", # current dA
            0x011F: "VSPD", # vertical speed cm/s
            0x021F: "VFAS", # battery 1 voltage cV
            # 0x800: "GPS", ## comments as duplicated dictrionary key
            0x050E: "RPM1",

            0x34: "DOWNLINK1_ID",
            0x67: "DOWNLINK2_ID",
            0x0D: "UPLINK_ID",
        }

        self.sensors_to_poll = [
            self.SENSOR_ID_VARIO,
            self.SENSOR_ID_FAS,
            self.SENSOR_ID_GPS,
            self.SENSOR_ID_RPM,
            self.SENSOR_ID_SP2UR,
        ]
        self.next_sensor_id_to_poll = 0 # offset into sensors_to_poll

        self.data_downlink_handler = None

        self.last_poll_sensor = None
        self.last_data_time = None

    def progress_tag(self):
        return "FRSkySPort"

    def handle_data_downlink(self, some_bytes):
        self.progress("DOWNLINK %s" % (str(some_bytes),))
        if self.data_downlink_handler is not None:
            self.data_downlink_handler(some_bytes)
        self.last_data_time = self.get_time()

    def handle_data(self, dataid, value):
        if dataid not in self.id_descriptions:
            raise KeyError("dataid 0x%02x" % dataid)
        self.progress("%s (0x%x)=%u" % (self.id_descriptions[dataid], dataid, value))
        self.data_by_id[dataid] = value
        if dataid not in self.dataid_counts:
            self.dataid_counts[dataid] = 0
        self.dataid_counts[dataid] += 1
        self.last_data_time = self.get_time()

    def dump_dataid_counts_as_progress_messages(self):
        for dataid in self.dataid_counts:
            self.progress("0x%x: %u  (%s)" % (dataid, self.dataid_counts[dataid], self.id_descriptions[dataid]))

    def dump_sensor_id_poll_counts_as_progress_messages(self):
        for sensor_id in self.sensor_id_poll_counts:
            self.progress("(0x%x): %u" % (sensor_id, self.sensor_id_poll_counts[sensor_id]))

    def read_bytestuffed_byte(self):
        if sys.version_info.major >= 3:
            b = self.buffer[0]
        else:
            b = ord(self.buffer[0])
        if b == 0x7D:
            # byte-stuffed
            if len(self.buffer) < 2:
                self.consume = 0
                return None
            self.consume = 2
            if sys.version_info.major >= 3:
                b2 = self.buffer[1]
            else:
                b2 = ord(self.buffer[1])
            if b2 == 0x5E:
                return self.START_STOP_SPORT
            if b2 == 0x5D:
                return self.BYTESTUFF_SPORT
            raise ValueError("Unknown stuffed byte (0x%02x)" % b2)
        return b

    def calc_crc(self, byte):
        self.crc += byte
        self.crc += self.crc >> 8
        self.crc &= 0xFF

    def next_sensor(self):
        ret = self.sensors_to_poll[self.next_sensor_id_to_poll]
        self.next_sensor_id_to_poll += 1
        if self.next_sensor_id_to_poll >= len(self.sensors_to_poll):
            self.next_sensor_id_to_poll = 0
        return ret

    def check_poll(self):
        now = self.get_time()
        # self.progress("check poll (%u)" % now)

        # sometimes ArduPilot will not respond to a poll - for
        # example, if you poll an unhealthy RPM sensor then we will
        # *never* get a response back.  So we must re-poll (which
        # moves onto the next sensor):
        if now - self.poll_sent > 5:
            if self.last_poll_sensor is None:
                self.progress("Re-polling (last poll sensor was None)")
            else:
                msg = ("Re-polling (last_poll_sensor=0x%02x state=%s)" %
                       (self.last_poll_sensor, self.state))
                self.progress(msg)
            if self.state != self.state_WANT_FRAME_TYPE:
                raise ValueError("Expected to be wanting a frame type when repolling (state=%s)" % str(self.state))
            self.state = self.state_SEND_POLL

        if self.state == self.state_SEND_POLL:
            sensor_id = self.next_sensor()
            self.progress("Sending poll for 0x%02x" % sensor_id)
            self.last_poll_sensor = sensor_id
            if sensor_id not in self.sensor_id_poll_counts:
                self.sensor_id_poll_counts[sensor_id] = 0
            self.sensor_id_poll_counts[sensor_id] += 1
            packet = SPortPollPacket(sensor_id)
            self.send_sport_packet(packet)
            self.state = self.state_WANT_FRAME_TYPE
            self.poll_sent = now

    def send_sport_packets(self, packets):
        for packet in packets:
            self.send_sport_packet(packet)

    def send_sport_packet(self, packet):
        stuffed = packet.for_wire()
        self.progress("Sending (%s) (%u)" %
                      (["0x%02x" % x for x in bytearray(stuffed)], len(stuffed)))
        self.port.sendall(stuffed)

    def send_mavlite_param_request_read(self, parameter_name):
        mavlite_msg = MAVliteMessage(
            mavutil.mavlink.MAVLINK_MSG_ID_PARAM_REQUEST_READ,
            bytearray(parameter_name.encode())
        )

        packets = mavlite_msg.to_sport_packets()

        self.send_sport_packets(packets)

    def send_mavlite_param_set(self, parameter_name, value):
        out = bytearray(struct.pack("<f", value))
        out.extend(parameter_name.encode())

        mavlite_msg = MAVliteMessage(
            mavutil.mavlink.MAVLINK_MSG_ID_PARAM_SET,
            out
        )

        packets = mavlite_msg.to_sport_packets()

        self.send_sport_packets(packets)

    def send_mavlite_command_long(
            self,
            command,
            p1=None,
            p2=None,
            p3=None,
            p4=None,
            p5=None,
            p6=None,
            p7=None,
    ):
        params = bytearray()
        seen_none = False
        for p in p1, p2, p3, p4, p5, p6, p7:
            if p is None:
                seen_none = True
                continue
            if seen_none:
                raise ValueError("Can't have values after Nones!")
            params.extend(bytearray(struct.pack("<f", p)))

        out = bytearray(struct.pack("<H", command))  # first two bytes are command-id
        options = len(params) // 4  # low-three-bits is parameter count
        out.extend(bytearray(struct.pack("<B", options)))  # second byte is options
        out.extend(params)  # then the float values

        mavlite_msg = MAVliteMessage(
            mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_LONG,
            out
        )

        packets = mavlite_msg.to_sport_packets()

        self.send_sport_packets(packets)

    def update(self):
        if not self.connected:
            if not self.connect():
                self.progress("Failed to connect")
                return
        self.check_poll()
        self.do_sport_read()

    def do_sport_read(self):
        self.buffer += self.do_read()
        self.consume = None
        while len(self.buffer):
            if self.consume is not None:
                self.buffer = self.buffer[self.consume:]
            if len(self.buffer) == 0:
                break
            self.consume = 1
            if sys.version_info.major >= 3:
                b = self.buffer[0]
            else:
                b = ord(self.buffer[0])
#            self.progress("Have (%s) bytes state=%s b=0x%02x" % (str(len(self.buffer)), str(self.state), b));
            if self.state == self.state_WANT_FRAME_TYPE:
                if b in [self.SPORT_DATA_FRAME, self.SPORT_DOWNLINK_FRAME]:
                    self.frame = b
                    self.crc = 0
                    self.calc_crc(b)
                    self.state = self.state_WANT_ID1
                    continue
                # we may come into a stream mid-way, so we can't judge
                self.progress("############# Bad char %x" % b)
                raise ValueError("Bad char (0x%02x)" % b)
                self.bad_chars += 1
                continue
            elif self.state == self.state_WANT_ID1:
                self.id1 = self.read_bytestuffed_byte()
                if self.id1 is None:
                    break
                self.calc_crc(self.id1)
                self.state = self.state_WANT_ID2
                continue
            elif self.state == self.state_WANT_ID2:
                self.id2 = self.read_bytestuffed_byte()
                if self.id2 is None:
                    break
                self.calc_crc(self.id2)
                self.state = self.state_WANT_DATA
                self.data_bytes = []
                self.data = 0
                continue
            elif self.state == self.state_WANT_DATA:
                data_byte = self.read_bytestuffed_byte()
                if data_byte is None:
                    break
                self.calc_crc(data_byte)
                self.data = self.data | (data_byte << (8*(len(self.data_bytes))))
                self.data_bytes.append(data_byte)
                if len(self.data_bytes) == 4:
                    self.state = self.state_WANT_CRC
                continue
            elif self.state == self.state_WANT_CRC:
                crc = self.read_bytestuffed_byte()
                if crc is None:
                    break
                self.crc = 0xFF - self.crc
                dataid = (self.id2 << 8) | self.id1
                if self.crc != crc:
                    self.progress("Incorrect frsky checksum (received=%02x calculated=%02x id=0x%x)" % (crc, self.crc, dataid))
#                    raise ValueError("Incorrect frsky checksum (want=%02x got=%02x id=0x%x)" % (crc, self.crc, dataid))
                else:
                    if self.frame == self.SPORT_DOWNLINK_FRAME:
                        self.handle_data_downlink([
                            self.id1,
                            self.id2,
                            self.data_bytes[0],
                            self.data_bytes[1],
                            self.data_bytes[2],
                            self.data_bytes[3]]
                        )
                    else:
                        self.handle_data(dataid, self.data)
                self.state = self.state_SEND_POLL
            elif self.state == self.state_SEND_POLL:
                # this is done in check_poll
                self.progress("in send_poll state")
                pass
            else:
                raise ValueError("Unknown state (%s)" % self.state)

    def get_data(self, dataid):
        try:
            return self.data_by_id[dataid]
        except KeyError:
            pass
        return None


class FRSkyPassThrough(FRSkySPort):
    def __init__(self, destination_address, get_time=time.time):
        super(FRSkyPassThrough, self).__init__(destination_address,
                                               get_time=get_time)

        self.sensors_to_poll = [self.SENSOR_ID_27]

    def progress_tag(self):
        return "FRSkyPassthrough"


class LocationInt(object):
    def __init__(self, lat, lon, alt, yaw):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.yaw = yaw


class Test(object):
    '''a test definition - information about a test'''
    def __init__(self, name, description, function, attempts=1):
        self.name = name
        self.description = description
        self.function = function
        self.attempts = attempts


class AutoTest(ABC):
    """Base abstract class.
    It implements the common function for all vehicle types.
    """
    def __init__(self,
                 binary,
                 valgrind=False,
                 callgrind=False,
                 gdb=False,
                 gdb_no_tui=False,
                 speedup=None,
                 frame=None,
                 params=None,
                 gdbserver=False,
                 lldb=False,
                 breakpoints=[],
                 disable_breakpoints=False,
                 viewerip=None,
                 use_map=False,
                 _show_test_timings=False,
                 logs_dir=None,
                 force_ahrs_type=None,
                 replay=False,
                 sup_binaries=[],
                 reset_after_every_test=False,
                 sitl_32bit=False):

        self.start_time = time.time()
        global __autotest__ # FIXME; make progress a non-staticmethod
        __autotest__ = self

        if binary is None:
            raise ValueError("Should always have a binary")

        self.binary = binary
        self.valgrind = valgrind
        self.callgrind = callgrind
        self.gdb = gdb
        self.gdb_no_tui = gdb_no_tui
        self.lldb = lldb
        self.frame = frame
        self.params = params
        self.gdbserver = gdbserver
        self.breakpoints = breakpoints
        self.disable_breakpoints = disable_breakpoints
        self.speedup = speedup
        if self.speedup is None:
            self.speedup = self.default_speedup()
        self.sup_binaries = sup_binaries
        self.reset_after_every_test = reset_after_every_test
        self.sitl_32bit = sitl_32bit

        self.mavproxy = None
        self._mavproxy = None  # for auto-cleanup on failed tests
        self.mav = None
        self.viewerip = viewerip
        self.use_map = use_map
        self.contexts = []
        self.context_push()
        self.buildlog = None
        self.copy_tlog = False
        self.logfile = None
        self.max_set_rc_timeout = 0
        self.last_wp_load = 0
        self.forced_post_test_sitl_reboots = 0
        self.skip_list = []
        self.run_tests_called = False
        self._show_test_timings = _show_test_timings
        self.test_timings = dict()
        self.total_waiting_to_arm_time = 0
        self.waiting_to_arm_count = 0
        self.force_ahrs_type = force_ahrs_type
        self.replay = replay
        if self.force_ahrs_type is not None:
            self.force_ahrs_type = int(self.force_ahrs_type)
        self.logs_dir = logs_dir
        self.timesync_number = 137
        self.last_progress_sent_as_statustext = None
        self.last_heartbeat_time_ms = None
        self.last_heartbeat_time_wc_s = 0
        self.in_drain_mav = False
        self.tlog = None

        self.rc_thread = None
        self.rc_thread_should_quit = False
        self.rc_queue = Queue.Queue()

        self.expect_list = []

        self.start_mavproxy_count = 0

        self.last_sim_time_cached = 0
        self.last_sim_time_cached_wallclock = 0

    def __del__(self):
        if self.rc_thread is not None:
            self.progress("Joining RC thread in __del__")
            self.rc_thread_should_quit = True
            self.rc_thread.join()
            self.rc_thread = None

    def default_speedup(self):
        return 8

    def progress(self, text, send_statustext=True):
        """Display autotest progress text."""
        global __autotest__
        delta_time = time.time() - __autotest__.start_time
        formatted_text = "AT-%06.1f: %s" % (delta_time, text)
        print(formatted_text)
        if (send_statustext and
                self.mav is not None and
                self.mav.port is not None and
                self.last_progress_sent_as_statustext != text):
            self.send_statustext(formatted_text)
            self.last_progress_sent_as_statustext = text

    # following two functions swiped from autotest.py:
    @staticmethod
    def buildlogs_dirpath():
        return os.getenv("BUILDLOGS", util.reltopdir("../buildlogs"))

    def sitl_home(self):
        HOME = self.sitl_start_location()
        return "%f,%f,%u,%u" % (HOME.lat,
                                HOME.lng,
                                HOME.alt,
                                HOME.heading)

    def mavproxy_version(self):
        '''return the current version of mavproxy as a tuple e.g. (1,8,8)'''
        return util.MAVProxy_version()

    def mavproxy_version_gt(self, major, minor, point):
        if os.getenv("AUTOTEST_FORCE_MAVPROXY_VERSION", None) is not None:
            return True
        (got_major, got_minor, got_point) = self.mavproxy_version()
        self.progress("Got: %s.%s.%s" % (got_major, got_minor, got_point))
        if got_major > major:
            return True
        elif got_major < major:
            return False
        if got_minor > minor:
            return True
        elif got_minor < minor:
            return False
        return got_point > point

    def open_mavproxy_logfile(self):
        return MAVProxyLogFile()

    def buildlogs_path(self, path):
        """Return a string representing path in the buildlogs directory."""
        bits = [self.buildlogs_dirpath()]
        if isinstance(path, list):
            bits.extend(path)
        else:
            bits.append(path)
        return os.path.join(*bits)

    def sitl_streamrate(self):
        """Allow subclasses to override SITL streamrate."""
        return 10

    def autotest_connection_string_to_ardupilot(self):
        return "tcp:127.0.0.1:5760"

    def mavproxy_options(self):
        """Returns options to be passed to MAVProxy."""
        ret = [
            '--sitl=127.0.0.1:5502',
            '--streamrate=%u' % self.sitl_streamrate(),
            '--target-system=%u' % self.sysid_thismav(),
            '--target-component=1',
        ]
        if self.viewerip:
            ret.append("--out=%s:14550" % self.viewerip)
        if self.use_map:
            ret.append('--map')

        return ret

    def vehicleinfo_key(self):
        return self.log_name()

    def repeatedly_apply_parameter_file(self, filepath):
        if False:
            return self.repeatedly_apply_parameter_file_mavproxy(filepath)
        parameters = mavparm.MAVParmDict()
#        correct_parameters = set()
        if not parameters.load(filepath):
            raise ValueError("Param load failed")
        param_dict = {}
        for p in parameters.keys():
            param_dict[p] = parameters[p]
        self.set_parameters(param_dict)

    def repeatedly_apply_parameter_file_mavproxy(self, filepath):
        '''keep applying a parameter file until no parameters changed'''
        for i in range(0, 3):
            self.mavproxy.send("param load %s\n" % filepath)
            while True:
                line = self.mavproxy.readline()
                match = re.match(".*Loaded [0-9]+ parameters.*changed ([0-9]+)",
                                 line)
                if match is not None:
                    if int(match.group(1)) == 0:
                        return
                    break
        raise NotAchievedException()

    def apply_defaultfile_parameters(self):
        """Apply parameter file."""
        self.progress("Applying default parameters file")
        # setup test parameters
        if self.params is None:
            self.params = self.model_defaults_filepath(self.frame)
        for x in self.params:
            self.repeatedly_apply_parameter_file(x)

    def count_lines_in_filepath(self, filepath):
        return len([i for i in open(filepath)])

    def count_expected_fence_lines_in_filepath(self, filepath):
        count = 0
        is_qgc = False
        for i in open(filepath):
            i = re.sub("#.*", "", i) # trim comments
            if i.isspace():
                # skip empty lines
                continue
            if re.match("QGC", i):
                # skip QGC header line
                is_qgc = True
                continue
            count += 1
        if is_qgc:
            count += 2 # file doesn't include return point + closing point
        return count

    def load_fence_using_mavproxy(self, mavproxy, filename):
        self.set_parameter("FENCE_TOTAL", 0)
        filepath = os.path.join(testdir, self.current_test_name_directory, filename)
        count = self.count_expected_fence_lines_in_filepath(filepath)
        mavproxy.send('fence load %s\n' % filepath)
#        self.mavproxy.expect("Loaded %u (geo-)?fence" % count)
        tstart = self.get_sim_time_cached()
        while True:
            t2 = self.get_sim_time_cached()
            if t2 - tstart > 10:
                raise AutoTestTimeoutException("Failed to do load")
            newcount = self.get_parameter("FENCE_TOTAL")
            self.progress("fence total: %u want=%u" % (newcount, count))
            if count == newcount:
                break
            self.delay_sim_time(1)

    def get_fence_point(self, idx, target_system=1, target_component=1):
        self.mav.mav.fence_fetch_point_send(target_system,
                                            target_component,
                                            idx)
        m = self.assert_receive_message("FENCE_POINT", timeout=2)
        self.progress("m: %s" % str(m))
        if m.idx != idx:
            raise NotAchievedException("Invalid idx returned (want=%u got=%u)" %
                                       (idx, m.seq))
        return m

    def fencepoint_protocol_epsilon(self):
        return 0.00002

    def roundtrip_fencepoint_protocol(self, offset, count, lat, lng, target_system=1, target_component=1):
        self.progress("Sending FENCE_POINT offs=%u count=%u" % (offset, count))
        self.mav.mav.fence_point_send(target_system,
                                      target_component,
                                      offset,
                                      count,
                                      lat,
                                      lng)

        self.progress("Requesting fence point")
        m = self.get_fence_point(offset, target_system=target_system, target_component=target_component)
        if abs(m.lat - lat) > self.fencepoint_protocol_epsilon():
            raise NotAchievedException("Did not get correct lat in fencepoint: got=%f want=%f" % (m.lat, lat))
        if abs(m.lng - lng) > self.fencepoint_protocol_epsilon():
            raise NotAchievedException("Did not get correct lng in fencepoint: got=%f want=%f" % (m.lng, lng))
        self.progress("Roundtrip OK")

    def roundtrip_fence_using_fencepoint_protocol(self, loc_list, target_system=1, target_component=1, ordering=None):
        count = len(loc_list)
        offset = 0
        self.set_parameter("FENCE_TOTAL", count)
        if ordering is None:
            ordering = range(count)
        elif len(ordering) != len(loc_list):
            raise ValueError("ordering list length mismatch")

        for offset in ordering:
            loc = loc_list[offset]
            self.roundtrip_fencepoint_protocol(offset,
                                               count,
                                               loc.lat,
                                               loc.lng,
                                               target_system,
                                               target_component)

        self.progress("Validating uploaded fence")
        returned_count = self.get_parameter("FENCE_TOTAL")
        if returned_count != count:
            raise NotAchievedException("Returned count mismatch (want=%u got=%u)" %
                                       (count, returned_count))
        for i in range(count):
            self.progress("Requesting fence point")
            m = self.get_fence_point(offset, target_system=target_system, target_component=target_component)
            if abs(m.lat-loc.lat) > self.fencepoint_protocol_epsilon():
                raise NotAchievedException("Returned lat mismatch (want=%f got=%f" %
                                           (loc.lat, m.lat))
            if abs(m.lng-loc.lng) > self.fencepoint_protocol_epsilon():
                raise NotAchievedException("Returned lng mismatch (want=%f got=%f" %
                                           (loc.lng, m.lng))
            if m.count != count:
                raise NotAchievedException("Count mismatch (want=%u got=%u)" %
                                           (count, m.count))

    def load_fence(self, filename):
        filepath = os.path.join(testdir, self.current_test_name_directory, filename)
        self.progress("Loading fence from (%s)" % str(filepath))
        locs = []
        for line in open(filepath, 'rb'):
            if len(line) == 0:
                continue
            m = re.match(r"([-\d.]+)\s+([-\d.]+)\s*", line.decode('ascii'))
            if m is None:
                raise ValueError("Did not match (%s)" % line)
            locs.append(mavutil.location(float(m.group(1)), float(m.group(2)), 0, 0))
        if self.is_plane():
            # create return point as the centroid:
            total_lat = 0
            total_lng = 0
            total_cnt = 0
            for loc in locs:
                total_lat += loc.lat
                total_lng += loc.lng
                total_cnt += 1
            locs2 = [mavutil.location(total_lat/total_cnt,
                                      total_lng/total_cnt,
                                      0,
                                      0)]  # return point
            locs2.extend(locs)
            locs2.append(copy.copy(locs2[1]))
            return self.roundtrip_fence_using_fencepoint_protocol(locs2)

        self.upload_fences_from_locations(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            [
                locs
            ])

    def send_reboot_command(self):
        self.mav.mav.command_long_send(self.sysid_thismav(),
                                       1,
                                       mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                       1,  # confirmation
                                       1, # reboot autopilot
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0)

    def reboot_check_valgrind_log(self):
        valgrind_log = util.valgrind_log_filepath(binary=self.binary,
                                                  model=self.frame)
        if os.path.getsize(valgrind_log) > 0:
            backup_valgrind_log = ("%s-%s" % (str(int(time.time())), valgrind_log))
            shutil.move(valgrind_log, backup_valgrind_log)

    def run_cmd_reboot(self):
        self.run_cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                     1,  # confirmation
                     1,  # reboot autopilot
                     0,
                     0,
                     0,
                     0,
                     0,
                     0)

    def run_cmd_enable_high_latency(self, new_state):
        p1 = 0
        if new_state:
            p1 = 1

        self.run_cmd(
            mavutil.mavlink.MAV_CMD_CONTROL_HIGH_LATENCY,
            p1, # p1 - enable/disable
            0,  # p2
            0,  # p3
            0,  # p4
            0,  # p5
            0,  # p6
            0,  # p7
        )

    def reboot_sitl_mav(self, required_bootcount=None):
        """Reboot SITL instance using mavlink and wait for it to reconnect."""
        # we must make sure that stats have been reset - otherwise
        # when we reboot we'll reset statistics again and lose our
        # STAT_BOOTCNT increment:
        tstart = time.time()
        while True:
            if time.time() - tstart > 30:
                raise NotAchievedException("STAT_RESET did not go non-zero")
            if self.get_parameter('STAT_RESET', timeout_in_wallclock=True) != 0:
                break

        old_bootcount = self.get_parameter('STAT_BOOTCNT')
        # ardupilot SITL may actually NAK the reboot; replace with
        # run_cmd when we don't do that.
        if self.valgrind or self.callgrind:
            self.reboot_check_valgrind_log()
            self.progress("Stopping and restarting SITL")
            if getattr(self, 'valgrind_restart_customisations', None) is not None:
                self.customise_SITL_commandline(
                    self.valgrind_restart_customisations,
                    model=self.valgrind_restart_model,
                    defaults_filepath=self.valgrind_restart_defaults_filepath,
                )
            else:
                self.stop_SITL()
                self.start_SITL(wipe=False)
        else:
            self.progress("Executing reboot command")
            self.run_cmd_reboot()
        self.detect_and_handle_reboot(old_bootcount, required_bootcount=required_bootcount)

    def send_cmd_enter_cpu_lockup(self):
        """Poke ArduPilot to stop the main loop from running"""
        self.mav.mav.command_long_send(self.sysid_thismav(),
                                       1,
                                       mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                       1,  # confirmation
                                       42, # lockup autopilot
                                       24, # no, really, we mean it
                                       71, # seriously, we're not kidding
                                       93, # we know exactly what we're
                                       0,
                                       0,
                                       0)

    def reboot_sitl(self, required_bootcount=None):
        """Reboot SITL instance and wait for it to reconnect."""
        self.progress("Rebooting SITL")
        self.reboot_sitl_mav(required_bootcount=required_bootcount)
        self.do_heartbeats(force=True)
        self.assert_simstate_location_is_at_startup_location()

    def reboot_sitl_mavproxy(self, required_bootcount=None):
        """Reboot SITL instance using MAVProxy and wait for it to reconnect."""
        old_bootcount = self.get_parameter('STAT_BOOTCNT')
        self.mavproxy.send("reboot\n")
        self.detect_and_handle_reboot(old_bootcount, required_bootcount=required_bootcount)

    def detect_and_handle_reboot(self, old_bootcount, required_bootcount=None, timeout=10):
        tstart = time.time()
        if required_bootcount is None:
            required_bootcount = old_bootcount + 1
        while True:
            if time.time() - tstart > timeout:
                raise AutoTestTimeoutException("Did not detect reboot")
            try:
                current_bootcount = self.get_parameter('STAT_BOOTCNT',
                                                       timeout=1,
                                                       attempts=1,
                                                       verbose=True,
                                                       timeout_in_wallclock=True)
                self.progress("current=%s required=%u" %
                              (str(current_bootcount), required_bootcount))
                if current_bootcount == required_bootcount:
                    break
            except NotAchievedException:
                pass
            except AutoTestTimeoutException:
                pass
            except ConnectionResetError:
                pass
            except socket.error:
                pass
            except Exception as e:
                self.progress("Got unexpected exception (%s)" % str(type(e)))
                pass

        # empty mav to avoid getting old timestamps:
        self.do_timesync_roundtrip(timeout_in_wallclock=True)

        self.progress("Calling initialise-after-reboot")
        self.initialise_after_reboot_sitl()

    def set_streamrate(self, streamrate, timeout=20, stream=mavutil.mavlink.MAV_DATA_STREAM_ALL):
        '''set MAV_DATA_STREAM_ALL; timeout is wallclock time'''
        tstart = time.time()
        while True:
            if time.time() - tstart > timeout:
                raise NotAchievedException("Failed to set streamrate")
            self.mav.mav.request_data_stream_send(
                1,
                1,
                stream,
                streamrate,
                1)
            m = self.mav.recv_match(type='SYSTEM_TIME',
                                    blocking=True,
                                    timeout=1)
            if m is not None:
                break

    def set_streamrate_mavproxy(self, streamrate, timeout=10):
        tstart = time.time()
        while True:
            if time.time() - tstart > timeout:
                raise AutoTestTimeoutException("stream rate change failed")

            self.mavproxy.send("set streamrate %u\n" % (streamrate))
            self.mavproxy.send("set streamrate\n")
            try:
                self.mavproxy.expect('.*streamrate ((?:-)?[0-9]+)', timeout=1)
            except pexpect.TIMEOUT:
                continue
            rate = self.mavproxy.match.group(1)
#            self.progress("rate: %s" % str(rate))
            if int(rate) == int(streamrate):
                break

        if streamrate <= 0:
            return

        self.progress("Waiting for SYSTEM_TIME for confirmation streams are working")
        self.drain_mav_unparsed()
        timeout = 60
        tstart = time.time()
        while True:
            self.drain_all_pexpects()
            if time.time() - tstart > timeout:
                raise NotAchievedException("Did not get SYSTEM_TIME within %f seconds" % timeout)
            m = self.mav.recv_match(timeout=0.1)
            if m is None:
                continue
#            self.progress("Received (%s)" % str(m))
            if m.get_type() == 'SYSTEM_TIME':
                break
        self.drain_mav()

    def htree_from_xml(self, xml_filepath):
        '''swiped from mavproxy_param.py'''
        xml = open(xml_filepath, 'rb').read()
        from lxml import objectify
        objectify.enable_recursive_str()
        tree = objectify.fromstring(xml)
        htree = {}
        for p in tree.vehicles.parameters.param:
            n = p.get('name').split(':')[1]
            htree[n] = p
        for lib in tree.libraries.parameters:
            for p in lib.param:
                n = p.get('name')
                htree[n] = p
        return htree

    def test_adsb_send_threatening_adsb_message(self, here):
        self.progress("Sending ABSD_VEHICLE message")
        self.mav.mav.adsb_vehicle_send(
            37, # ICAO address
            int(here.lat * 1e7),
            int(here.lng * 1e7),
            mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
            int(here.alt*1000 + 10000), # 10m up
            0, # heading in cdeg
            0, # horizontal velocity cm/s
            0, # vertical velocity cm/s
            "bob".encode("ascii"), # callsign
            mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,
            1, # time since last communication
            65535, # flags
            17 # squawk
        )

    def test_parameter_documentation_get_all_parameters(self):
        xml_filepath = os.path.join(self.buildlogs_dirpath(), "apm.pdef.xml")
        param_parse_filepath = os.path.join(self.rootdir(), 'Tools', 'autotest', 'param_metadata', 'param_parse.py')
        try:
            os.unlink(xml_filepath)
        except OSError:
            pass
        vehicle = self.log_name()
        if vehicle == "HeliCopter":
            vehicle = "ArduCopter"
        if vehicle == "QuadPlane":
            vehicle = "ArduPlane"
        cmd = [param_parse_filepath, '--vehicle', vehicle]
        # cmd.append("--verbose")
        if util.run_cmd(cmd, directory=self.buildlogs_dirpath()) != 0:
            self.progress("Failed param_parse.py (%s)" % vehicle)
            return False
        htree = self.htree_from_xml(xml_filepath)

        target_system = self.sysid_thismav()
        target_component = 1

        self.customise_SITL_commandline([
            "--unhide-groups"
        ])

        (parameters, seq_id) = self.download_parameters(target_system, target_component)

        self.reset_SITL_commandline()

        fail = False
        for param in parameters.keys():
            if param.startswith("SIM_"):
                # too many of these to worry about
                continue
            if param not in htree:
                self.progress("%s not in XML" % param)
                fail = True
        if fail:
            raise NotAchievedException("Downloaded parameters missing in XML")

        # FIXME: this should be doable if we filter out e.g BRD_* and CAN_*?
#        self.progress("Checking no extra parameters present in XML")
#        fail = False
#        for param in htree:
#            if param.startswith("SIM_"):
#                # too many of these to worry about
#                continue
#            if param not in parameters:
#                print("%s not in downloaded parameters but in XML" % param)
#                fail = True
#        if fail:
#            raise NotAchievedException("Extra parameters in XML")

    def find_format_defines(self, lines):
        ret = {}
        for line in lines:
            if type(line) == bytes:
                line = line.decode("utf-8")
            m = re.match(r'#define (\w+_(?:LABELS|FMT|UNITS|MULTS))\s+(".*")', line)
            if m is None:
                continue
            (a, b) = (m.group(1), m.group(2))
            if a in ret:
                raise NotAchievedException("Duplicate define for (%s)" % a)
            ret[a] = b
        return ret

    def vehicle_code_dirpath(self):
        '''returns path to vehicle-specific code directory e.g. ~/ardupilot/Rover'''
        dirname = self.log_name()
        if dirname == "QuadPlane":
            dirname = "ArduPlane"
        elif dirname == "HeliCopter":
            dirname = "ArduCopter"
        return os.path.join(self.rootdir(), dirname)

    def find_LogStructureFiles(self):
        '''return list of files named LogStructure.h'''
        ret = []
        for root, _, files in os.walk(self.rootdir()):
            for f in files:
                if f == 'LogStructure.h':
                    ret.append(os.path.join(root, f))
                if f == 'LogStructure_SBP.h':
                    ret.append(os.path.join(root, f))
        return ret

    def all_log_format_ids(self):
        '''parse C++ code to extract definitions of log messages'''
        structure_files = self.find_LogStructureFiles()
        structure_lines = []
        for f in structure_files:
            structure_lines.extend(open(f).readlines())
        ids = {}
        state_outside = 0
        state_inside = 1
        state = state_outside

        defines = self.find_format_defines(structure_lines)

        linestate_none = 45
        linestate_within = 46
        linestate = linestate_none
        message_infos = []
        for line in structure_lines:
            #            print("line: %s" % line)
            if type(line) == bytes:
                line = line.decode("utf-8")
            line = re.sub("//.*", "", line) # trim comments
            if re.match(r"\s*$", line):
                # blank line
                continue
            if state == state_outside:
                if ("#define LOG_COMMON_STRUCTURES" in line or
                        re.match("#define LOG_STRUCTURE_FROM_.*", line)):
                    #                    self.progress("Moving inside")
                    state = state_inside
                continue
            if state == state_inside:
                if linestate == linestate_none:
                    allowed_list = ['LOG_STRUCTURE_FROM_']

                    allowed = False
                    for a in allowed_list:
                        if a in line:
                            allowed = True
                    if allowed:
                        continue
                    m = re.match(r"\s*{(.*)},\s*", line)
                    if m is not None:
                        # complete line
                        # print("Complete line: %s" % str(line))
                        message_infos.append(m.group(1))
                        continue
                    m = re.match(r"\s*{(.*)\\", line)
                    if m is None:
                        state = state_outside
                        continue
                    partial_line = m.group(1)
                    linestate = linestate_within
                    continue
                if linestate == linestate_within:
                    m = re.match("(.*)}", line)
                    if m is None:
                        line = line.rstrip()
                        newline = re.sub(r"\\$", "", line)
                        if newline == line:
                            raise NotAchievedException("Expected backslash at end of line")
                        line = newline
                        line = line.rstrip()
                        # cpp-style string concatenation:
                        line = re.sub(r'"\s*"', '', line)
                        partial_line += line
                        continue
                    message_infos.append(partial_line + m.group(1))
                    linestate = linestate_none
                    continue
                raise NotAchievedException("Bad line (%s)")

        if linestate != linestate_none:
            raise NotAchievedException("Must be linestate-none at end of file")

        # now look in the vehicle-specific logfile:
        filepath = os.path.join(self.vehicle_code_dirpath(), "Log.cpp")
        state_outside = 67
        state_inside = 68
        state = state_outside
        linestate_none = 89
        linestate_within = 90
        linestate = linestate_none
        for line in open(filepath, 'rb').readlines():
            if type(line) == bytes:
                line = line.decode("utf-8")
            line = re.sub("//.*", "", line) # trim comments
            if re.match(r"\s*$", line):
                # blank line
                continue
            if state == state_outside:
                if ("const LogStructure" in line or
                        "const struct LogStructure" in line):
                    state = state_inside
                continue
            if state == state_inside:
                if re.match("};", line):
                    state = state_outside
                    break
                if linestate == linestate_none:
                    if "#if HAL_QUADPLANE_ENABLED" in line:
                        continue
                    if "#if FRAME_CONFIG == HELI_FRAME" in line:
                        continue
                    if "#if PRECISION_LANDING == ENABLED" in line:
                        continue
                    if "#end" in line:
                        continue
                    if "LOG_COMMON_STRUCTURES" in line:
                        continue
                    m = re.match(r"\s*{(.*)},\s*", line)
                    if m is not None:
                        # complete line
                        # print("Complete line: %s" % str(line))
                        message_infos.append(m.group(1))
                        continue
                    m = re.match(r"\s*{(.*)", line)
                    if m is None:
                        raise NotAchievedException("Bad line %s" % line)
                    partial_line = m.group(1)
                    linestate = linestate_within
                    continue
                if linestate == linestate_within:
                    m = re.match("(.*)}", line)
                    if m is None:
                        line = line.rstrip()
                        newline = re.sub(r"\\$", "", line)
                        if newline == line:
                            raise NotAchievedException("Expected backslash at end of line")
                        line = newline
                        line = line.rstrip()
                        # cpp-style string concatenation:
                        line = re.sub(r'"\s*"', '', line)
                        partial_line += line
                        continue
                    message_infos.append(partial_line + m.group(1))
                    linestate = linestate_none
                    continue
                raise NotAchievedException("Bad line (%s)")

        if state == state_inside:
            raise NotAchievedException("Should not be in state_inside at end")

        for message_info in message_infos:
            for define in defines:
                message_info = re.sub(define, defines[define], message_info)
            m = re.match(r'\s*LOG_\w+\s*,\s*sizeof\([^)]+\)\s*,\s*"(\w+)"\s*,\s*"(\w+)"\s*,\s*"([\w,]+)"\s*,\s*"([^"]+)"\s*,\s*"([^"]+)"\s*(,\s*true)?\s*$', message_info)  # noqa
            if m is None:
                continue
            (name, fmt, labels, units, multipliers) = (m.group(1), m.group(2), m.group(3), m.group(4), m.group(5))
            if name in ids:
                raise NotAchievedException("Already seen a (%s) message" % name)
            ids[name] = {
                "name": name,
                "format": fmt,
                "labels": labels,
                "units": units,
                "multipliers": multipliers,
            }

        # now look for Log_Write(...) messages:
        base_directories = [
            os.path.join(self.rootdir(), 'libraries'),
            self.vehicle_code_dirpath(),
        ]
        log_write_statements = []
        for base_directory in base_directories:
            for root, dirs, files in os.walk(base_directory):
                state_outside = 37
                state_inside = 38
                state = state_outside
                for f in files:
                    if not re.search("[.]cpp$", f):
                        continue
                    filepath = os.path.join(root, f)
                    if "AP_Logger/examples" in filepath:
                        # this is the sample file which contains examples...
                        continue
                    count = 0
                    for line in open(filepath, 'rb').readlines():
                        if type(line) == bytes:
                            line = line.decode("utf-8")
                        if state == state_outside:
                            if (re.match(r"\s*AP::logger\(\)[.]Write(?:Streaming)?\(", line) or
                                    re.match(r"\s*logger[.]Write(?:Streaming)?\(", line)):
                                state = state_inside
                                line = re.sub("//.*", "", line) # trim comments
                                log_write_statement = line
                            continue
                        if state == state_inside:
                            line = re.sub("//.*", "", line) # trim comments
                            log_write_statement += line
                            if re.match(r".*\);", line):
                                log_write_statements.append(log_write_statement)
                                state = state_outside
                        count += 1
                    if state != state_outside:
                        raise NotAchievedException("Expected to be outside at end of file")
#                    print("%s has %u lines" % (f, count))
        # change all whitespace to single space
        log_write_statements = [re.sub(r"\s+", " ", x) for x in log_write_statements]
#        print("Got log-write-statements: %s" % str(log_write_statements))
        results = []
        for log_write_statement in log_write_statements:
            for define in defines:
                log_write_statement = re.sub(define, defines[define], log_write_statement)
            # fair warning: order is important here because of the
            # NKT/XKT special case below....
            my_re = r' logger[.]Write(?:Streaming)?\(\s*"(\w+)"\s*,\s*"([\w,]+)".*\);'
            m = re.match(my_re, log_write_statement)
            if m is None:
                my_re = r' AP::logger\(\)[.]Write(?:Streaming)?\(\s*"(\w+)"\s*,\s*"([\w,]+)".*\);'
                m = re.match(my_re, log_write_statement)
            if m is None:
                raise NotAchievedException("Did not match (%s) with (%s)" % (log_write_statement, str(my_re)))
            else:
                results.append((m.group(1), m.group(2)))

        for result in results:
            (name, labels) = result
            if name in ids:
                raise Exception("Already have id for (%s)" % name)
#            self.progress("Adding Log_Write result (%s)" % name)
            ids[name] = {
                "name": name,
                "labels": labels,
            }

        if len(ids) == 0:
            raise NotAchievedException("Did not get any ids")

        return ids

    def test_onboard_logging_generation(self):
        '''just generates, as we can't do a lot of testing'''
        xml_filepath = os.path.join(self.buildlogs_dirpath(), "LogMessages.xml")
        parse_filepath = os.path.join(self.rootdir(), 'Tools', 'autotest', 'logger_metadata', 'parse.py')
        try:
            os.unlink(xml_filepath)
        except OSError:
            pass
        vehicle = self.log_name()
        vehicle_map = {
            "ArduCopter": "Copter",
            "HeliCopter": "Copter",
            "ArduPlane": "Plane",
            "QuadPlane": "Plane",
            "Rover": "Rover",
            "AntennaTracker": "Tracker",
            "ArduSub": "Sub",
        }
        vehicle = vehicle_map[vehicle]

        cmd = [parse_filepath, '--vehicle', vehicle]
#        cmd.append("--verbose")
        if util.run_cmd(cmd, directory=self.buildlogs_dirpath()) != 0:
            self.progress("Failed parse.py (%s)" % vehicle)
            return False
        length = os.path.getsize(xml_filepath)
        min_length = 1024
        if length < min_length:
            raise NotAchievedException("short xml file (%u < %u)" %
                                       (length, min_length))
        self.progress("xml file length is %u" % length)

        from lxml import objectify
        xml = open(xml_filepath, 'rb').read()
        objectify.enable_recursive_str()
        tree = objectify.fromstring(xml)

        # we allow for no docs for replay messages, as these are not for end-users. They are
        # effectively binary blobs for replay
        REPLAY_MSGS = ['RFRH', 'RFRF', 'REV2', 'RSO2', 'RWA2', 'REV3', 'RSO3', 'RWA3', 'RMGI',
                       'REY3', 'RFRN', 'RISH', 'RISI', 'RISJ', 'RBRH', 'RBRI', 'RRNH', 'RRNI',
                       'RGPH', 'RGPI', 'RGPJ', 'RASH', 'RASI', 'RBCH', 'RBCI', 'RVOH', 'RMGH',
                       'ROFH', 'REPH', 'REVH', 'RWOH', 'RBOH']

        docco_ids = {}
        for thing in tree.logformat:
            name = str(thing.get("name"))
            docco_ids[name] = {
                "name": name,
                "labels": [],
            }
            if getattr(thing.fields, 'field', None) is None:
                if name in REPLAY_MSGS:
                    continue
                raise NotAchievedException("no doc fields for %s" % name)
            for field in thing.fields.field:
                # print("field: (%s)" % str(field))
                fieldname = field.get("name")
#                print("Got (%s.%s)" % (name,str(fieldname)))
                docco_ids[name]["labels"].append(fieldname)

        code_ids = self.all_log_format_ids()
        # self.progress("Code ids: (%s)" % str(sorted(code_ids.keys())))
        # self.progress("Docco ids: (%s)" % str(sorted(docco_ids.keys())))

        for name in sorted(code_ids.keys()):
            if name not in docco_ids:
                self.progress("Undocumented message: %s" % str(name))
                continue
            seen_labels = {}
            for label in code_ids[name]["labels"].split(","):
                if label in seen_labels:
                    raise NotAchievedException("%s.%s is duplicate label" %
                                               (name, label))
                seen_labels[label] = True
                if label not in docco_ids[name]["labels"]:
                    raise NotAchievedException("%s.%s not in documented fields (have (%s))" %
                                               (name, label, ",".join(docco_ids[name]["labels"])))
        missing = []
        for name in sorted(docco_ids):
            if name not in code_ids and name not in REPLAY_MSGS:
                missing.append(name)
                continue
            for label in docco_ids[name]["labels"]:
                if label not in code_ids[name]["labels"].split(","):
                    # "name" was found in the XML, so was found in an
                    # @LoggerMessage markup line, but was *NOT* found
                    # in our bodgy parsing of the C++ code (in a
                    # Log_Write call or in the static structures
                    raise NotAchievedException("documented field %s.%s not found in code" %
                                               (name, label))
        if len(missing) > 0:
            raise NotAchievedException("Documented messages (%s) not in code" % missing)

    def initialise_after_reboot_sitl(self):

        # after reboot stream-rates may be zero.  Request streams.
        self.drain_mav()
        self.wait_heartbeat()
        self.set_streamrate(self.sitl_streamrate())
        self.progress("Reboot complete")

    def customise_SITL_commandline(self,
                                   customisations,
                                   model=None,
                                   defaults_filepath=None,
                                   wipe=False,
                                   set_streamrate_callback=None):
        '''customisations could be "--uartF=sim:nmea" '''
        self.contexts[-1].sitl_commandline_customised = True
        self.stop_SITL()
        self.start_SITL(model=model,
                        defaults_filepath=defaults_filepath,
                        customisations=customisations,
                        wipe=wipe)
        tstart = time.time()
        while True:
            if time.time() - tstart > 30:
                raise NotAchievedException("Failed to customise")
            try:
                m = self.wait_heartbeat(drain_mav=True)
                if m.type == 0:
                    self.progress("Bad heartbeat: %s" % str(m))
                    continue
            except IOError:
                pass
            break
        if set_streamrate_callback is not None:
            set_streamrate_callback()
        else:
            self.set_streamrate(self.sitl_streamrate())
        m = self.mav.recv_match(type='RC_CHANNELS', blocking=True, timeout=15)
        if m is None:
            raise NotAchievedException("No RC_CHANNELS message after restarting SITL")

        # stash our arguments in case we need to preserve them in
        # reboot_sitl with Valgrind active:
        if self.valgrind or self.callgrind:
            self.valgrind_restart_model = model
            self.valgrind_restart_defaults_filepath = defaults_filepath
            self.valgrind_restart_customisations = customisations

    def default_parameter_list(self):
        ret = {
            'LOG_DISARMED': 1,
        }
        if self.force_ahrs_type is not None:
            if self.force_ahrs_type == 2:
                ret["EK2_ENABLE"] = 1
            if self.force_ahrs_type == 3:
                ret["EK3_ENABLE"] = 1
            ret["AHRS_EKF_TYPE"] = self.force_ahrs_type
        if self.replay:
            ret["LOG_REPLAY"] = 1
        return ret

    def apply_default_parameter_list(self):
        self.set_parameters(self.default_parameter_list())

    def apply_default_parameters(self):
        self.apply_defaultfile_parameters()
        self.apply_default_parameter_list()
        self.reboot_sitl()

    def reset_SITL_commandline(self):
        self.progress("Resetting SITL commandline to default")
        self.stop_SITL()
        try:
            del self.valgrind_restart_customisations
        except Exception:
            pass
        self.start_SITL(wipe=True)
        self.set_streamrate(self.sitl_streamrate())
        self.apply_default_parameters()
        self.progress("Reset SITL commandline to default")

    def stop_SITL(self):
        self.progress("Stopping SITL")
        self.expect_list_remove(self.sitl)
        util.pexpect_close(self.sitl)
        self.sitl = None

    def close(self):
        """Tidy up after running all tests."""

        if self.mav is not None:
            self.mav.close()
            self.mav = None
        self.stop_SITL()

        valgrind_log = util.valgrind_log_filepath(binary=self.binary,
                                                  model=self.frame)
        files = glob.glob("*" + valgrind_log)
        for valgrind_log in files:
            os.chmod(valgrind_log, 0o644)
            if os.path.getsize(valgrind_log) > 0:
                target = self.buildlogs_path("%s-%s" % (
                    self.log_name(),
                    os.path.basename(valgrind_log)))
                self.progress("Valgrind log: moving %s to %s" % (valgrind_log, target))
                shutil.move(valgrind_log, target)

    def start_test(self, description):
        self.progress("##################################################################################")
        self.progress("########## %s  ##########" % description)
        self.progress("##################################################################################")

    def try_symlink_tlog(self):
        self.buildlog = self.buildlogs_path(self.log_name() + "-test.tlog")
        self.progress("buildlog=%s" % self.buildlog)
        if os.path.exists(self.buildlog):
            os.unlink(self.buildlog)
        try:
            os.link(self.logfile, self.buildlog)
        except OSError as error:
            self.progress("OSError [%d]: %s" % (error.errno, error.strerror))
            self.progress("WARN: Failed to create symlink: %s => %s, "
                          "will copy tlog manually to target location" %
                          (self.logfile, self.buildlog))
            self.copy_tlog = True

    #################################################
    # GENERAL UTILITIES
    #################################################
    def expect_list_clear(self):
        """clear the expect list."""
        for p in self.expect_list[:]:
            self.expect_list.remove(p)

    def expect_list_extend(self, list_to_add):
        """Extend the expect list."""
        self.expect_list.extend(list_to_add)

    def expect_list_add(self, item):
        """Extend the expect list."""
        self.expect_list.extend([item])

    def expect_list_remove(self, item):
        """Remove item from the expect list."""
        self.expect_list.remove(item)

    def heartbeat_interval_ms(self):
        c = self.context_get()
        if c is None:
            return 1000
        return c.heartbeat_interval_ms

    def set_heartbeat_interval_ms(self, interval_ms):
        c = self.context_get()
        if c is None:
            raise ValueError("No context")
        if c.original_heartbeat_interval_ms is None:
            c.original_heartbeat_interval_ms = c.heartbeat_interval_ms
        c.heartbeat_interval_ms = interval_ms

    def set_heartbeat_rate(self, rate_hz):
        if rate_hz == 0:
            self.set_heartbeat_interval_ms(None)
            return
        self.set_heartbeat_interval_ms(1000.0/rate_hz)

    def do_heartbeats(self, force=False):
        # self.progress("do_heartbeats")
        if self.heartbeat_interval_ms() is None and not force:
            return
        x = self.mav.messages.get("SYSTEM_TIME", None)
        now_wc = time.time()
        if (force or
            x is None or
            self.last_heartbeat_time_ms is None or
            self.last_heartbeat_time_ms < x.time_boot_ms or
            x.time_boot_ms - self.last_heartbeat_time_ms > self.heartbeat_interval_ms() or
                now_wc - self.last_heartbeat_time_wc_s > 1):
            if x is not None:
                self.last_heartbeat_time_ms = x.time_boot_ms
                self.last_heartbeat_time_wc_s = now_wc
                self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                            0,
                                            0,
                                            0)

    def drain_all_pexpects(self):
        for p in self.expect_list:
            util.pexpect_drain(p)

    def idle_hook(self, mav):
        """Called when waiting for a mavlink message."""
        if self.in_drain_mav:
            return
        self.drain_all_pexpects()

    def message_hook(self, mav, msg):
        """Called as each mavlink msg is received."""
#        print("msg: %s" % str(msg))
        if msg.get_type() == 'STATUSTEXT':
            self.progress("AP: %s" % msg.text)

        self.write_msg_to_tlog(msg)

        self.idle_hook(mav)
        self.do_heartbeats()

    def send_message_hook(self, msg, x):
        self.write_msg_to_tlog(msg)

    def write_msg_to_tlog(self, msg):
        usec = int(time.time() * 1.0e6)
        if self.tlog is None:
            tlog_filename = "autotest-%u.tlog" % usec
            self.tlog = open(tlog_filename, 'wb')

        content = bytearray(struct.pack('>Q', usec) + msg.get_msgbuf())
        self.tlog.write(content)

    def expect_callback(self, e):
        """Called when waiting for a expect pattern."""
        for p in self.expect_list:
            if p == e:
                continue
            util.pexpect_drain(p)
        self.drain_mav(quiet=True)
        self.do_heartbeats()

    def drain_mav_unparsed(self, mav=None, quiet=True, freshen_sim_time=False):
        if mav is None:
            mav = self.mav
        count = 0
        tstart = time.time()
        while True:
            this = mav.recv(1000000)
            if len(this) == 0:
                break
            count += len(this)
        if quiet:
            return
        tdelta = time.time() - tstart
        if tdelta == 0:
            rate = "instantly"
        else:
            rate = "%f/s" % (count/float(tdelta),)
        self.progress("Drained %u bytes from mav (%s).  These were unparsed." % (count, rate), send_statustext=False)
        if freshen_sim_time:
            self.get_sim_time()

    def drain_mav(self, mav=None, unparsed=False, quiet=True):
        if unparsed:
            return self.drain_mav_unparsed(quiet=quiet, mav=mav)
        if mav is None:
            mav = self.mav
        self.in_drain_mav = True
        count = 0
        tstart = time.time()
        while mav.recv_match(blocking=False) is not None:
            count += 1
        if quiet:
            self.in_drain_mav = False
            return
        tdelta = time.time() - tstart
        if tdelta == 0:
            rate = "instantly"
        else:
            rate = "%f/s" % (count/float(tdelta),)

        if not quiet:
            self.progress("Drained %u messages from mav (%s)" % (count, rate), send_statustext=False)
        self.in_drain_mav = False

    def do_timesync_roundtrip(self, quiet=False, timeout_in_wallclock=False):
        if not quiet:
            self.progress("Doing timesync roundtrip")
        if timeout_in_wallclock:
            tstart = time.time()
        else:
            tstart = self.get_sim_time()
        self.mav.mav.timesync_send(0, self.timesync_number * 1000 + self.mav.source_system)
        while True:
            if timeout_in_wallclock:
                now = time.time()
            else:
                now = self.get_sim_time_cached()
            if now - tstart > 5:
                raise AutoTestTimeoutException("Did not get timesync response")
            m = self.mav.recv_match(type='TIMESYNC', blocking=True, timeout=1)
            if not quiet:
                self.progress("Received: %s" % str(m))
            if m is None:
                continue
            if m.tc1 == 0:
                self.progress("this is a timesync request, which we don't answer")
                continue
            if m.ts1 % 1000 != self.mav.source_system:
                self.progress("this isn't a response to our timesync (%s)" % (m.ts1 % 1000))
                continue
            if int(m.ts1 / 1000) != self.timesync_number:
                self.progress("this isn't the one we just sent")
                continue
            if m.get_srcSystem() != self.mav.target_system:
                self.progress("response from system other than our target")
                continue
            # no component check ATM because we send broadcast...
#            if m.get_srcComponent() != self.mav.target_component:
#                self.progress("response from component other than our target (got=%u want=%u)" % (m.get_srcComponent(), self.mav.target_component))  # noqa
#                continue
            if not quiet:
                self.progress("Received TIMESYNC response after %fs" % (now - tstart))
            self.timesync_number += 1
            break

    def log_filepath(self, lognum):
        '''return filepath to lognum (where lognum comes from LOG_ENTRY'''
        log_list = self.log_list()
        return log_list[lognum-1]

    def assert_bytes_equal(self, bytes1, bytes2, maxlen=None):
        tocheck = len(bytes1)
        if maxlen is not None:
            if tocheck > maxlen:
                tocheck = maxlen
        for i in range(0, tocheck):
            if bytes1[i] != bytes2[i]:
                raise NotAchievedException("differ at offset %u" % i)

    def HIGH_LATENCY2(self):
        '''test sending of HIGH_LATENCY2'''

        # set airspeed sensor type to DLVR for air temperature message testing
        self.set_parameter("ARSPD_BUS", 2)
        self.set_parameter("ARSPD_TYPE", 7)
        self.reboot_sitl()

        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS, True, True, True, verbose=True, timeout=30)

        # should not be getting HIGH_LATENCY2 by default
        m = self.mav.recv_match(type='HIGH_LATENCY2', blocking=True, timeout=2)
        if m is not None:
            raise NotAchievedException("Shouldn't be getting HIGH_LATENCY2 by default")
        m = self.poll_message("HIGH_LATENCY2")
        if (m.failure_flags & mavutil.mavlink.HL_FAILURE_FLAG_GPS) != 0:
            raise NotAchievedException("Expected GPS to be OK")
        self.assert_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS, True, True, True)
        self.set_parameter("SIM_GPS_TYPE", 0)
        self.delay_sim_time(10)
        self.assert_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS, False, False, False)
        m = self.poll_message("HIGH_LATENCY2")
        self.progress(self.dump_message_verbose(m))
        if (m.failure_flags & mavutil.mavlink.HL_FAILURE_FLAG_GPS) == 0:
            raise NotAchievedException("Expected GPS to be failed")

        self.start_subtest("HIGH_LATENCY2 location")
        self.set_parameter("SIM_GPS_TYPE", 1)
        self.delay_sim_time(10)
        m = self.poll_message("HIGH_LATENCY2")
        self.progress(self.dump_message_verbose(m))
        loc = mavutil.location(m.latitude, m.longitude, m.altitude, 0)
        dist = self.get_distance_int(loc, self.sim_location_int())

        if dist > 1:
            raise NotAchievedException("Bad location from HIGH_LATENCY2")

        self.start_subtest("HIGH_LATENCY2 Air Temperature")
        m = self.poll_message("HIGH_LATENCY2")
        mavutil.dump_message_verbose(sys.stdout, m)

        if m.temperature_air == -128: # High_Latency2 defaults to INT8_MIN for no temperature available
            raise NotAchievedException("Air Temperature not received from HIGH_LATENCY2")
        self.HIGH_LATENCY2_links()

    def HIGH_LATENCY2_links(self):

        self.start_subtest("SerialProtocol_MAVLinkHL links")

        ex = None
        self.context_push()
        mav2 = None
        try:

            self.set_parameter("SERIAL2_PROTOCOL", 43)  # HL)

            self.reboot_sitl()

            mav2 = mavutil.mavlink_connection(
                "tcp:localhost:5763",
                robust_parsing=True,
                source_system=7,
                source_component=7,
            )

            self.start_subsubtest("Don't get HIGH_LATENCY2 by default")
            for mav in self.mav, mav2:
                self.assert_not_receive_message('HIGH_LATENCY2', mav=mav, timeout=10)

            self.start_subsubtest("Get HIGH_LATENCY2 upon link enabled only on HL link")
            self.run_cmd_enable_high_latency(True)
            self.assert_receive_message("HIGH_LATENCY2", mav=mav2, timeout=10)
            self.assert_not_receive_message("HIGH_LATENCY2", mav=self.mav, timeout=10)

            self.start_subsubtest("Not get HIGH_LATENCY2 upon HL disable")
            self.run_cmd_enable_high_latency(False)
            self.delay_sim_time(10)
            self.assert_not_receive_message('HIGH_LATENCY2', mav=self.mav, timeout=10)
            self.drain_mav(mav2)
            self.assert_not_receive_message('HIGH_LATENCY2', mav=mav2, timeout=10)

            self.start_subsubtest("Stream rate adjustments")
            self.run_cmd_enable_high_latency(True)
            self.assert_message_rate_hz("HIGH_LATENCY2", 0.2, ndigits=1, mav=mav2)
            for test_rate in (1, 0.1, 2):
                self.test_rate(
                    "HIGH_LATENCY2 on enabled link",
                    test_rate,
                    test_rate,
                    mav=mav2,
                    ndigits=1,
                    victim_message="HIGH_LATENCY2",
                    message_rate_sample_period=60,
                )
            self.assert_not_receive_message("HIGH_LATENCY2", mav=self.mav, timeout=10)
            self.run_cmd_enable_high_latency(False)

            self.start_subsubtest("Not get HIGH_LATENCY2 after disabling after playing with rates")
            self.assert_not_receive_message('HIGH_LATENCY2', mav=self.mav, timeout=10)
            self.delay_sim_time(1)
            self.drain_mav(mav2)
            self.assert_not_receive_message('HIGH_LATENCY2', mav=mav2, timeout=10)

            self.start_subsubtest("Enable and disable should not affect non-HL links getting HIGH_LATENCY2")
            self.set_message_rate_hz("HIGH_LATENCY2", 5, mav=self.mav)
            self.assert_message_rate_hz("HIGH_LATENCY2", 5, mav=self.mav)

            self.run_cmd_enable_high_latency(True)
            self.assert_message_rate_hz("HIGH_LATENCY2", 5, mav=self.mav),

            self.run_cmd_enable_high_latency(False)
            self.assert_message_rate_hz("HIGH_LATENCY2", 5, mav=self.mav)
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.context_pop()

        self.reboot_sitl()

        self.set_message_rate_hz("HIGH_LATENCY2", 0)

        if ex is not None:
            raise ex

    def test_log_download(self):
        if self.is_tracker():
            # tracker starts armed, which is annoying
            return
        self.progress("Ensuring we have contents we care about")
        self.set_parameter("LOG_FILE_DSRMROT", 1)
        self.set_parameter("LOG_DISARMED", 0)
        self.reboot_sitl()
        original_log_list = self.log_list()
        for i in range(0, 10):
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.delay_sim_time(1)
            self.disarm_vehicle()
        new_log_list = self.log_list()
        new_log_count = len(new_log_list) - len(original_log_list)
        if new_log_count != 10:
            raise NotAchievedException("Expected exactly 10 new logs got %u (%s) to (%s)" %
                                       (new_log_count, original_log_list, new_log_list))
        self.progress("Directory contents: %s" % str(new_log_list))

        tstart = self.get_sim_time()
        self.mav.mav.log_request_list_send(self.sysid_thismav(),
                                           1, # target component
                                           0,
                                           0xff)
        logs = []
        last_id = None
        num_logs = None
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 5:
                raise NotAchievedException("Did not download list")
            m = self.mav.recv_match(type='LOG_ENTRY',
                                    blocking=True,
                                    timeout=1)
            self.progress("Received (%s)" % str(m))
            if m is None:
                continue
            logs.append(m)
            if last_id is None:
                if m.num_logs == 0:
                    # caller to guarantee this works:
                    raise NotAchievedException("num_logs is zero")
                num_logs = m.num_logs
            else:
                if m.id != last_id + 1:
                    raise NotAchievedException("Sequence not increasing")
                if m.num_logs != num_logs:
                    raise NotAchievedException("Number of logs changed")
                if m.time_utc < 1000:
                    raise NotAchievedException("Bad timestamp")
                if m.id != m.last_log_num:
                    if m.size == 0:
                        raise NotAchievedException("Zero-sized log")
            last_id = m.id
            if m.id == m.last_log_num:
                self.progress("Got all logs")
                break

        # ensure we don't get any extras:
        m = self.mav.recv_match(type='LOG_ENTRY',
                                blocking=True,
                                timeout=2)
        if m is not None:
            raise NotAchievedException("Received extra LOG_ENTRY?!")

        log_id = 5
        ofs = 6
        count = 2
        self.start_subtest("downloading %u bytes from offset %u from log_id %u" %
                           (count, ofs, log_id))
        self.mav.mav.log_request_data_send(self.sysid_thismav(),
                                           1, # target component
                                           log_id,
                                           ofs,
                                           count)
        m = self.assert_receive_message('LOG_DATA', timeout=2)
        if m.ofs != ofs:
            raise NotAchievedException("Incorrect offset")
        if m.count != count:
            raise NotAchievedException("Did not get correct number of bytes")
        log_filepath = self.log_filepath(log_id)
        self.progress("Checking against log_filepath (%s)" % str(log_filepath))
        with open(log_filepath, "rb") as bob:
            bob.seek(ofs)
            actual_bytes = bob.read(2)
            actual_bytes = bytearray(actual_bytes)
        if m.data[0] != actual_bytes[0]:
            raise NotAchievedException("Bad first byte got=(0x%02x) want=(0x%02x)" %
                                       (m.data[0], actual_bytes[0]))
        if m.data[1] != actual_bytes[1]:
            raise NotAchievedException("Bad second byte")

        log_id = 7
        log_filepath = self.log_filepath(log_id)
        self.start_subtest("Downloading log id %u (%s)" % (log_id, log_filepath))
        with open(log_filepath, "rb") as bob:
            actual_bytes = bytearray(bob.read())

        # get the size first
        self.mav.mav.log_request_list_send(self.sysid_thismav(),
                                           1, # target component
                                           log_id,
                                           log_id)
        log_entry = self.mav.recv_match(type='LOG_ENTRY',
                                        blocking=True,
                                        timeout=2)
        if log_entry.size != len(actual_bytes):
            raise NotAchievedException("Incorrect bytecount")
        self.progress("Using log entry (%s)" % str(log_entry))
        if log_entry.id != log_id:
            raise NotAchievedException("Incorrect log id received")

        # download the log file in the normal way:
        bytes_to_fetch = 100000
        self.progress("Sending request for %u bytes at offset 0" % (bytes_to_fetch,))
        tstart = self.get_sim_time()
        self.mav.mav.log_request_data_send(
            self.sysid_thismav(),
            1, # target component
            log_id,
            0,
            bytes_to_fetch
        )
        bytes_to_read = bytes_to_fetch
        if log_entry.size < bytes_to_read:
            bytes_to_read = log_entry.size
        data_downloaded = []
        bytes_read = 0
        last_print = 0
        while True:
            if bytes_read >= bytes_to_read:
                break
            if self.get_sim_time_cached() - tstart > 120:
                raise NotAchievedException("Did not download log in good time")
            m = self.assert_receive_message('LOG_DATA', timeout=2)
            if m.ofs != bytes_read:
                raise NotAchievedException("Unexpected offset")
            if m.id != log_id:
                raise NotAchievedException("Unexpected id")
            if m.count == 0:
                raise NotAchievedException("Zero bytes read")
            data_downloaded.extend(m.data[0:m.count])
            bytes_read += m.count
            # self.progress("Read %u bytes at offset %u" % (m.count, m.ofs))
            if time.time() - last_print > 10:
                last_print = time.time()
                self.progress("Read %u/%u" % (bytes_read, bytes_to_read))

        self.progress("actual_bytes_len=%u data_downloaded_len=%u" %
                      (len(actual_bytes), len(data_downloaded)))
        self.assert_bytes_equal(actual_bytes, data_downloaded, maxlen=bytes_to_read)

        if False:
            bytes_to_read = log_entry.size
            bytes_read = 0
            data_downloaded = []
            while bytes_read < bytes_to_read:
                bytes_to_fetch = int(random.random() * 100)
                if bytes_to_fetch > 90:
                    bytes_to_fetch = 90
                self.progress("Sending request for %u bytes at offset %u" % (bytes_to_fetch, bytes_read))
                self.mav.mav.log_request_data_send(
                    self.sysid_thismav(),
                    1, # target component
                    log_id,
                    bytes_read,
                    bytes_to_fetch
                )
                m = self.assert_receive_message('LOG_DATA', timeout=2)
                self.progress("Read %u bytes at offset %u" % (m.count, m.ofs))
                if m.ofs != bytes_read:
                    raise NotAchievedException("Incorrect offset in reply want=%u got=%u (%s)" % (bytes_read, m.ofs, str(m)))
                stuff = m.data[0:m.count]
                data_downloaded.extend(stuff)
                bytes_read += m.count
                if len(data_downloaded) != bytes_read:
                    raise NotAchievedException("extend fail")

            if len(actual_bytes) != len(data_downloaded):
                raise NotAchievedException("Incorrect length: disk:%u downloaded: %u" %
                                           (len(actual_bytes), len(data_downloaded)))
            self.assert_bytes_equal(actual_bytes, data_downloaded)

        self.start_subtest("Download log backwards")
        bytes_to_read = bytes_to_fetch
        if log_entry.size < bytes_to_read:
            bytes_to_read = log_entry.size
        bytes_read = 0
        backwards_data_downloaded = []
        last_print = 0
        while bytes_read < bytes_to_read:
            bytes_to_fetch = int(random.random() * 99) + 1
            if bytes_to_fetch > 90:
                bytes_to_fetch = 90
            if bytes_to_fetch > bytes_to_read - bytes_read:
                bytes_to_fetch = bytes_to_read - bytes_read
            ofs = bytes_to_read - bytes_read - bytes_to_fetch
            # self.progress("bytes_to_read=%u bytes_read=%u bytes_to_fetch=%u ofs=%d" %
            # (bytes_to_read, bytes_read, bytes_to_fetch, ofs))
            self.mav.mav.log_request_data_send(
                self.sysid_thismav(),
                1, # target component
                log_id,
                ofs,
                bytes_to_fetch
            )
            m = self.assert_receive_message('LOG_DATA', timeout=2)
            if m.count == 0:
                raise NotAchievedException("xZero bytes read (ofs=%u)" % (ofs,))
            if m.count > bytes_to_fetch:
                raise NotAchievedException("Read too many bytes?!")
            stuff = m.data[0:m.count]
            stuff.extend(backwards_data_downloaded)
            backwards_data_downloaded = stuff
            bytes_read += m.count
            # self.progress("Read %u bytes at offset %u" % (m.count, m.ofs))
            if time.time() - last_print > 10:
                last_print = time.time()
                self.progress("xRead %u/%u" % (bytes_read, bytes_to_read))

        self.assert_bytes_equal(actual_bytes, backwards_data_downloaded, maxlen=bytes_to_read)
        # if len(actual_bytes) != len(backwards_data_downloaded):
        #     raise NotAchievedException("Size delta: actual=%u vs downloaded=%u" %
        #                                (len(actual_bytes), len(backwards_data_downloaded)))

    #################################################
    # SIM UTILITIES
    #################################################
    def get_sim_time(self, timeout=60):
        """Get SITL time in seconds."""
        self.drain_mav()
        tstart = time.time()
        while True:
            self.drain_all_pexpects()
            if time.time() - tstart > timeout:
                raise AutoTestTimeoutException("Did not get SYSTEM_TIME message after %f seconds" % timeout)

            m = self.mav.recv_match(type='SYSTEM_TIME', blocking=True, timeout=0.1)
            if m is None:
                continue

            return m.time_boot_ms * 1.0e-3

    def get_sim_time_cached(self):
        """Get SITL time in seconds."""
        x = self.mav.messages.get("SYSTEM_TIME", None)
        if x is None:
            raise NotAchievedException("No cached time available (%s)" % (self.mav.sysid,))
        ret = x.time_boot_ms * 1.0e-3
        if ret != self.last_sim_time_cached:
            self.last_sim_time_cached = ret
            self.last_sim_time_cached_wallclock = time.time()
        else:
            timeout = 30
            if self.valgrind:
                timeout *= 10
            if time.time() - self.last_sim_time_cached_wallclock > timeout and not self.gdb:
                raise AutoTestTimeoutException("sim_time_cached is not updating!")
        return ret

    def sim_location(self):
        """Return current simulator location."""
        m = self.mav.recv_match(type='SIMSTATE', blocking=True)
        return mavutil.location(m.lat*1.0e-7,
                                m.lng*1.0e-7,
                                0,
                                math.degrees(m.yaw))

    def sim_location_int(self):
        """Return current simulator location."""
        m = self.mav.recv_match(type='SIMSTATE', blocking=True)
        return mavutil.location(m.lat,
                                m.lng,
                                0,
                                math.degrees(m.yaw))

    def save_wp(self, ch=7):
        """Trigger RC Aux to save waypoint."""
        self.set_rc(ch, 1000)
        self.delay_sim_time(1)
        self.set_rc(ch, 2000)
        self.delay_sim_time(1)
        self.set_rc(ch, 1000)
        self.delay_sim_time(1)

    def create_simple_relhome_mission(self, items_in, target_system=1, target_component=1):
        '''takes a list of (type, n, e, alt) items.  Creates a mission in
        absolute frame using alt as relative-to-home and n and e as
        offsets in metres from home'''

        # add a dummy waypoint for home
        items = [(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0)]
        items.extend(items_in)
        seq = 0
        ret = []
        for (t, n, e, alt) in items:
            lat = 0
            lng = 0
            if n != 0 or e != 0:
                loc = self.home_relative_loc_ne(n, e)
                lat = loc.lat
                lng = loc.lng
            p1 = 0
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            if not self.ardupilot_stores_frame_for_cmd(t):
                frame = mavutil.mavlink.MAV_FRAME_GLOBAL
            ret.append(self.mav.mav.mission_item_int_encode(
                target_system,
                target_component,
                seq, # seq
                frame,
                t,
                0, # current
                0, # autocontinue
                p1, # p1
                0, # p2
                0, # p3
                0, # p4
                int(lat*1e7), # latitude
                int(lng*1e7), # longitude
                alt, # altitude
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION),
            )
            seq += 1

        return ret

    def upload_simple_relhome_mission(self, items, target_system=1, target_component=1):
        mission = self.create_simple_relhome_mission(
            items,
            target_system=target_system,
            target_component=target_component)
        self.check_mission_upload_download(mission)

    def get_mission_count(self):
        return self.get_parameter("MIS_TOTAL")

    def assert_mission_count(self, expected):
        count = self.get_mission_count()
        if count != expected:
            raise NotAchievedException("Unexpected count got=%u want=%u" %
                                       (count, expected))

    def clear_wp(self, ch=8):
        """Trigger RC Aux to clear waypoint."""
        self.progress("Clearing waypoints")
        self.set_rc(ch, 1000)
        self.delay_sim_time(0.5)
        self.set_rc(ch, 2000)
        self.delay_sim_time(0.5)
        self.set_rc(ch, 1000)
        self.assert_mission_count(0)

    def log_list(self):
        '''return a list of log files present in POSIX-style loging dir'''
        ret = sorted(glob.glob("logs/00*.BIN"))
        self.progress("log list: %s" % str(ret))
        return ret

    def assert_parameter_value(self, parameter, required):
        got = self.get_parameter(parameter)
        if got != required:
            raise NotAchievedException("%s has unexpected value; want=%f got=%f" %
                                       (parameter, required, got))
        self.progress("%s has value %f" % (parameter, required))

    def assert_reach_imu_temperature(self, target, timeout):
        '''wait to reach a target temperature'''
        tstart = self.get_sim_time()
        temp_ok = False
        last_print_temp = -100
        while self.get_sim_time_cached() - tstart < timeout:
            m = self.assert_receive_message('RAW_IMU', timeout=2)
            temperature = m.temperature*0.01
            if temperature >= target:
                self.progress("Reached temperature %.1f" % temperature)
                temp_ok = True
                break
            if temperature - last_print_temp > 1:
                self.progress("temperature %.1f" % temperature)
                last_print_temp = temperature

        if not temp_ok:
            raise NotAchievedException("target temperature")

    def assert_message_field_values(self, m, fieldvalues, verbose=True):
        for (fieldname, value) in fieldvalues.items():
            got = getattr(m, fieldname)
            if got != value:
                raise NotAchievedException("Expected %s.%s to be %s, got %s" %
                                           (m.get_type(), fieldname, value, got))
            if verbose:
                self.progress("%s.%s has expected value %s" %
                              (m.get_type(), fieldname, value))

    def assert_received_message_field_values(self, message, fieldvalues, verbose=True, very_verbose=False):
        m = self.assert_receive_message(message, verbose=verbose, very_verbose=very_verbose)
        self.assert_message_field_values(m, fieldvalues, verbose=verbose)
        return m

    def onboard_logging_not_log_disarmed(self):
        self.start_subtest("Test LOG_DISARMED-is-false behaviour")
        self.set_parameter("LOG_DISARMED", 0)
        self.set_parameter("LOG_FILE_DSRMROT", 0)
        self.reboot_sitl()
        self.wait_ready_to_arm() # let things setttle
        self.start_subtest("Ensure setting LOG_DISARMED yields a new file")
        original_list = self.log_list()
        self.progress("original list: %s" % str(original_list))
        self.set_parameter("LOG_DISARMED", 1)
        self.delay_sim_time(1) # LOG_DISARMED is polled by the logger code
        new_list = self.log_list()
        self.progress("new list: %s" % str(new_list))
        if len(new_list) - len(original_list) != 1:
            raise NotAchievedException("Got more than one new log")
        self.set_parameter("LOG_DISARMED", 0)
        self.delay_sim_time(1) # LOG_DISARMED is polled by the logger code
        new_list = self.log_list()
        if len(new_list) - len(original_list) != 1:
            raise NotAchievedException("Got more or less than one new log after toggling LOG_DISARMED off")

        self.start_subtest("Ensuring toggling LOG_DISARMED on and off doesn't increase the number of files")
        self.set_parameter("LOG_DISARMED", 1)
        self.delay_sim_time(1) # LOG_DISARMED is polled by the logger code
        new_new_list = self.log_list()
        if len(new_new_list) != len(new_list):
            raise NotAchievedException("Got extra files when toggling LOG_DISARMED")
        self.set_parameter("LOG_DISARMED", 0)
        self.delay_sim_time(1) # LOG_DISARMED is polled by the logger code
        new_new_list = self.log_list()
        if len(new_new_list) != len(new_list):
            raise NotAchievedException("Got extra files when toggling LOG_DISARMED to 0 again")
        self.end_subtest("Ensuring toggling LOG_DISARMED on and off doesn't increase the number of files")

        self.start_subtest("Check disarm rot when log disarmed is zero")
        self.assert_parameter_value("LOG_DISARMED", 0)
        self.set_parameter("LOG_FILE_DSRMROT", 1)
        old_speedup = self.get_parameter("SIM_SPEEDUP")
        # reduce speedup to reduce chance of race condition here
        self.set_parameter("SIM_SPEEDUP", 1)
        pre_armed_list = self.log_list()
        if self.is_copter() or self.is_heli():
            self.set_parameter("DISARM_DELAY", 0)
        self.arm_vehicle()
        post_armed_list = self.log_list()
        if len(post_armed_list) != len(pre_armed_list):
            raise NotAchievedException("Got unexpected new log")
        self.disarm_vehicle()
        old_speedup = self.set_parameter("SIM_SPEEDUP", old_speedup)
        post_disarmed_list = self.log_list()
        if len(post_disarmed_list) != len(post_armed_list):
            raise NotAchievedException("Log rotated immediately")
        self.progress("Allowing time for post-disarm-logging to occur if it will")
        self.delay_sim_time(5)
        post_disarmed_post_delay_list = self.log_list()
        if len(post_disarmed_post_delay_list) != len(post_disarmed_list):
            raise NotAchievedException("Got log rotation when we shouldn't have")
        self.progress("Checking that arming does produce a new log")
        self.arm_vehicle()
        post_armed_list = self.log_list()
        if len(post_armed_list) - len(post_disarmed_post_delay_list) != 1:
            raise NotAchievedException("Did not get new log for rotation")
        self.progress("Now checking natural rotation after HAL_LOGGER_ARM_PERSIST")
        self.disarm_vehicle()
        post_disarmed_list = self.log_list()
        if len(post_disarmed_list) != len(post_armed_list):
            raise NotAchievedException("Log rotated immediately")
        self.delay_sim_time(30)
        delayed_post_disarmed_list = self.log_list()
        # should *still* not get another log as LOG_DISARMED is false
        if len(post_disarmed_list) != len(delayed_post_disarmed_list):
            self.progress("Unexpected new log found")

    def onboard_logging_log_disarmed(self):
        self.start_subtest("Test LOG_DISARMED-is-true behaviour")
        start_list = self.log_list()
        self.set_parameter("LOG_FILE_DSRMROT", 0)
        self.set_parameter("LOG_DISARMED", 0)
        self.reboot_sitl()
        restart_list = self.log_list()
        if len(start_list) != len(restart_list):
            raise NotAchievedException(
                "Unexpected log detected (pre-delay) initial=(%s) restart=(%s)" %
                (str(sorted(start_list)), str(sorted(restart_list))))
        self.delay_sim_time(20)
        restart_list = self.log_list()
        if len(start_list) != len(restart_list):
            raise NotAchievedException("Unexpected log detected (post-delay)")
        self.set_parameter("LOG_DISARMED", 1)
        self.delay_sim_time(5) # LOG_DISARMED is polled
        post_log_disarmed_set_list = self.log_list()
        if len(post_log_disarmed_set_list) == len(restart_list):
            raise NotAchievedException("Did not get new log when LOG_DISARMED set")
        self.progress("Ensuring we get a new log after a reboot")
        self.reboot_sitl()
        self.delay_sim_time(5)
        post_reboot_log_list = self.log_list()
        if len(post_reboot_log_list) == len(post_log_disarmed_set_list):
            raise NotAchievedException("Did not get fresh log-disarmed log after a reboot")
        self.progress("Ensuring no log rotation when we toggle LOG_DISARMED off then on again")
        self.set_parameter("LOG_DISARMED", 0)
        current_log_filepath = self.current_onboard_log_filepath()
        self.delay_sim_time(10) # LOG_DISARMED is polled
        post_toggleoff_list = self.log_list()
        if len(post_toggleoff_list) != len(post_reboot_log_list):
            raise NotAchievedException("Shouldn't get new file yet")
        self.progress("Ensuring log does not grow when LOG_DISARMED unset...")
        current_log_filepath_size = os.path.getsize(current_log_filepath)
        self.delay_sim_time(5)
        current_log_filepath_new_size = os.path.getsize(current_log_filepath)
        if current_log_filepath_new_size != current_log_filepath_size:
            raise NotAchievedException(
                "File growing after LOG_DISARMED unset (new=%u old=%u" %
                (current_log_filepath_new_size, current_log_filepath_size))
        self.progress("Turning LOG_DISARMED back on again")
        self.set_parameter("LOG_DISARMED", 1)
        self.delay_sim_time(5) # LOG_DISARMED is polled
        post_toggleon_list = self.log_list()
        if len(post_toggleon_list) != len(post_toggleoff_list):
            raise NotAchievedException("Log rotated when it shouldn't")
        self.progress("Checking log is now growing again")
        if os.path.getsize(current_log_filepath) == current_log_filepath_size:
            raise NotAchievedException("Log is not growing")

        # self.progress("Checking LOG_FILE_DSRMROT behaviour when log_DISARMED set")
        # self.set_parameter("LOG_FILE_DSRMROT", 1)
        # self.wait_ready_to_arm()
        # pre = self.log_list()
        # self.arm_vehicle()
        # post = self.log_list()
        # if len(pre) != len(post):
        #     raise NotAchievedException("Rotation happened on arming?!")
        # size_a = os.path.getsize(current_log_filepath)
        # self.delay_sim_time(5)
        # size_b = os.path.getsize(current_log_filepath)
        # if size_b <= size_a:
        #     raise NotAchievedException("Log not growing")
        # self.disarm_vehicle()
        # instant_post_disarm_list = self.log_list()
        # self.progress("Should not rotate straight away")
        # if len(instant_post_disarm_list) != len(post):
        #     raise NotAchievedException("Should not rotate straight away")
        # self.delay_sim_time(20)
        # post_disarm_list = self.log_list()
        # if len(post_disarm_list) - len(instant_post_disarm_list) != 1:
        #     raise NotAchievedException("Did not get exactly one more log")

        # self.progress("If we re-arm during the HAL_LOGGER_ARM_PERSIST period it should rotate")

    def onboard_logging_forced_arm(self):
        '''ensure a bug where we didn't start logging when arming was forced
        does not reappear'''
        self.start_subtest("Ensure we get a log when force-arming")
        self.set_parameter("LOG_DISARMED", 0)
        self.reboot_sitl()  # so we'll definitely start a log on arming
        pre_arming_list = self.log_list()
        self.wait_ready_to_arm()
        self.arm_vehicle(force=True)
        # we might be relying on a thread to actually create the log
        # file when doing forced-arming; give the file time to appear:
        self.delay_sim_time(10)
        post_arming_list = self.log_list()
        self.disarm_vehicle()
        if len(post_arming_list) <= len(pre_arming_list):
            raise NotAchievedException("Did not get a log on forced arm")

    def test_onboard_logging(self):
        if self.is_tracker():
            return
        self.onboard_logging_forced_arm()
        self.onboard_logging_log_disarmed()
        self.onboard_logging_not_log_disarmed()

    def test_log_download_mavproxy(self, upload_logs=False):
        """Download latest log."""
        filename = "MAVProxy-downloaded-log.BIN"
        mavproxy = self.start_mavproxy()
        self.mavproxy_load_module(mavproxy, 'log')
        mavproxy.send("log list\n")
        mavproxy.expect("numLogs")
        self.wait_heartbeat()
        self.wait_heartbeat()
        mavproxy.send("set shownoise 0\n")
        mavproxy.send("log download latest %s\n" % filename)
        mavproxy.expect("Finished downloading", timeout=120)
        self.mavproxy_unload_module(mavproxy, 'log')
        self.stop_mavproxy(mavproxy)

    def log_upload(self):
        self.progress("Log upload disabled as CI artifacts are good")
        return
        if len(self.fail_list) > 0 and os.getenv("AUTOTEST_UPLOAD"):
            # optionally upload logs to server so we can see travis failure logs
            import datetime
            import glob
            import subprocess
            logdir = self.buildlogs_dirpath()
            datedir = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M")
            flist = glob.glob("logs/*.BIN")
            for e in ['BIN', 'bin', 'tlog']:
                flist += glob.glob(os.path.join(logdir, '*.%s' % e))
            self.progress("Uploading %u logs to https://firmware.ardupilot.org/CI-Logs/%s" % (len(flist), datedir))
            cmd = ['rsync', '-avz'] + flist + ['cilogs@autotest.ardupilot.org::CI-Logs/%s/' % datedir]
            subprocess.call(cmd)

    def show_gps_and_sim_positions(self, on_off):
        """Allow to display gps and actual position on map."""
        if on_off is True:
            # turn on simulator display of gps and actual position
            self.mavproxy.send('map set showgpspos 1\n')
            self.mavproxy.send('map set showsimpos 1\n')
        else:
            # turn off simulator display of gps and actual position
            self.mavproxy.send('map set showgpspos 0\n')
            self.mavproxy.send('map set showsimpos 0\n')

    @staticmethod
    def mission_count(filename):
        """Load a mission from a file and return number of waypoints."""
        wploader = mavwp.MAVWPLoader()
        wploader.load(filename)
        return wploader.count()

    def install_message_hook(self, hook):
        self.mav.message_hooks.append(hook)

    def install_message_hook_context(self, hook):
        '''installs a message hook which will be removed when the context goes
        away'''
        if self.mav is None:
            return
        self.mav.message_hooks.append(hook)
        self.context_get().message_hooks.append(hook)

    def remove_message_hook(self, hook):
        if self.mav is None:
            return
        oldlen = len(self.mav.message_hooks)
        self.mav.message_hooks = list(filter(lambda x: x != hook,
                                             self.mav.message_hooks))
        if len(self.mav.message_hooks) == oldlen:
            raise NotAchievedException("Failed to remove hook")

    def rootdir(self):
        this_dir = os.path.dirname(__file__)
        return os.path.realpath(os.path.join(this_dir, "../.."))

    def ardupilot_stores_frame_for_cmd(self, t):
        # ardupilot doesn't remember frame on these commands
        return t not in [
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
            mavutil.mavlink.MAV_CMD_DO_JUMP,
            mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
        ]

    def assert_mission_files_same(self, file1, file2, match_comments=False):
        self.progress("Comparing (%s) and (%s)" % (file1, file2, ))

        f1 = open(file1)
        f2 = open(file2)
        lines1 = f1.readlines()
        lines2 = f2.readlines()

        if not match_comments:
            # strip comments from all lines
            lines1 = [re.sub(r"\s*#.*", "", x, re.DOTALL) for x in lines1]
            lines2 = [re.sub(r"\s*#.*", "", x, re.DOTALL) for x in lines2]
            # FIXME: because DOTALL doesn't seem to work as expected:
            lines1 = [x.rstrip() for x in lines1]
            lines2 = [x.rstrip() for x in lines2]
            # remove now-empty lines:
            lines1 = filter(lambda x: len(x), lines1)
            lines2 = filter(lambda x: len(x), lines2)

        for l1, l2 in zip(lines1, lines2):
            l1 = l1.rstrip("\r\n")
            l2 = l2.rstrip("\r\n")
            if l1 == l2:
                # e.g. the first "QGC WPL 110" line
                continue
            if re.match(r"0\s", l1):
                # home changes...
                continue
            l1 = l1.rstrip()
            l2 = l2.rstrip()
            fields1 = re.split(r"\s+", l1)
            fields2 = re.split(r"\s+", l2)
            # line = int(fields1[0])
            t = int(fields1[3]) # mission item type
            for (count, (i1, i2)) in enumerate(zip(fields1, fields2)):
                if count == 2: # frame
                    if not self.ardupilot_stores_frame_for_cmd(t):
                        if int(i1) in [3, 10]: # 3 is relative, 10 is terrain
                            i1 = 0
                        if int(i2) in [3, 10]:
                            i2 = 0
                if count == 6: # param 3
                    if t in [mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME]:
                        # ardupilot canonicalises this to -1 for ccw or 1 for cw.
                        if float(i1) == 0:
                            i1 = 1.0
                        if float(i2) == 0:
                            i2 = 1.0
                if count == 7: # param 4
                    if t == mavutil.mavlink.MAV_CMD_NAV_LAND:
                        # ardupilot canonicalises "0" to "1" param 4 (yaw)
                        if int(float(i1)) == 0:
                            i1 = 1
                        if int(float(i2)) == 0:
                            i2 = 1
                if 0 <= count <= 3 or 11 <= count <= 11:
                    if int(i1) != int(i2):
                        raise ValueError("Files have different content: (%s vs %s) (%s vs %s) (%d vs %d) (count=%u)" %
                                         (file1, file2, l1, l2, int(i1), int(i2), count))  # NOCI
                    continue
                if 4 <= count <= 10:
                    f_i1 = float(i1)
                    f_i2 = float(i2)
                    delta = abs(f_i1 - f_i2)
                    max_allowed_delta = 0.000009
                    if delta > max_allowed_delta:
                        raise ValueError(
                            ("Files have different (float) content: " +
                             "(%s) and (%s) " +
                             "(%s vs %s) " +
                             "(%f vs %f) " +
                             "(%.10f) " +
                             "(count=%u)") %
                            (file1, file2,
                             l1, l2,
                             f_i1, f_i2,
                             delta,
                             count)) # NOCI
                    continue
                raise ValueError("count %u not handled" % count)
        self.progress("Files same")

    def assert_not_receive_message(self, message, timeout=1, mav=None):
        '''this is like assert_not_receiving_message but uses sim time not
        wallclock time'''
        self.progress("making sure we're not getting %s messages" % message)
        if mav is None:
            mav = self.mav

        tstart = self.get_sim_time_cached()
        while True:
            m = mav.recv_match(type=message, blocking=True, timeout=0.1)
            if m is not None:
                self.progress("Received: %s" % self.dump_message_verbose(m))
                raise PreconditionFailedException("Receiving %s messages" % message)
            if mav != self.mav:
                # update timestamp....
                self.drain_mav(self.mav)
            if self.get_sim_time_cached() - tstart > timeout:
                return

    def assert_receive_message(self, type, timeout=1, verbose=False, very_verbose=False, mav=None):
        if mav is None:
            mav = self.mav
        m = mav.recv_match(type=type, blocking=True, timeout=timeout)
        if verbose:
            self.progress("Received (%s)" % str(m))
        if very_verbose:
            self.progress(self.dump_message_verbose(m))
        if m is None:
            raise NotAchievedException("Did not get %s" % type)
        return m

    def assert_rally_files_same(self, file1, file2):
        self.progress("Comparing (%s) and (%s)" % (file1, file2, ))
        f1 = open(file1)
        f2 = open(file2)
        lines_f1 = f1.readlines()
        lines_f2 = f2.readlines()
        self.assert_rally_content_same(lines_f1, lines_f2)

    def assert_rally_filepath_content(self, file1, content):
        f1 = open(file1)
        lines_f1 = f1.readlines()
        lines_content = content.split("\n")
        print("lines content: %s" % str(lines_content))
        self.assert_rally_content_same(lines_f1, lines_content)

    def assert_rally_content_same(self, f1, f2):
        '''check each line in f1 matches one-to-one with f2'''
        for l1, l2 in zip(f1, f2):
            print("l1: %s" % l1)
            print("l2: %s" % l2)
            l1 = l1.rstrip("\n")
            l2 = l2.rstrip("\n")
            l1 = l1.rstrip("\r")
            l2 = l2.rstrip("\r")
            if l1 == l2:
                # e.g. the first "QGC WPL 110" line
                continue
            if re.match(r"0\s", l1):
                # home changes...
                continue
            l1 = l1.rstrip()
            l2 = l2.rstrip()
            print("al1: %s" % str(l1))
            print("al2: %s" % str(l2))
            fields1 = re.split(r"\s+", l1)
            fields2 = re.split(r"\s+", l2)
            # line = int(fields1[0])
            # t = int(fields1[3]) # mission item type
            for (count, (i1, i2)) in enumerate(zip(fields1, fields2)):
                # if count == 2: # frame
                #     if t in [mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                #              mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                #              mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                #              mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                #              mavutil.mavlink.MAV_CMD_DO_JUMP,
                #              mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
                #              ]:
                #         # ardupilot doesn't remember frame on these commands
                #         if int(i1) == 3:
                #             i1 = 0
                #         if int(i2) == 3:
                #             i2 = 0
                # if count == 6: # param 3
                #     if t in [mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME]:
                #         # ardupilot canonicalises this to -1 for ccw or 1 for cw.
                #         if float(i1) == 0:
                #             i1 = 1.0
                #         if float(i2) == 0:
                #             i2 = 1.0
                # if count == 7: # param 4
                #     if t == mavutil.mavlink.MAV_CMD_NAV_LAND:
                #         # ardupilot canonicalises "0" to "1" param 4 (yaw)
                #         if int(float(i1)) == 0:
                #             i1 = 1
                #         if int(float(i2)) == 0:
                #             i2 = 1
                if 0 <= count <= 3 or 11 <= count <= 11:
                    if int(i1) != int(i2):
                        raise ValueError("Rally points different: (%s vs %s) (%d vs %d) (count=%u)" %
                                         (l1, l2, int(i1), int(i2), count))  # NOCI
                    continue
                if 4 <= count <= 10:
                    f_i1 = float(i1)
                    f_i2 = float(i2)
                    delta = abs(f_i1 - f_i2)
                    max_allowed_delta = 0.000009
                    if delta > max_allowed_delta:
                        raise ValueError(
                            ("Rally has different (float) content: " +
                             "(%s vs %s) " +
                             "(%f vs %f) " +
                             "(%.10f) " +
                             "(count=%u)") %
                            (l1, l2,
                             f_i1, f_i2,
                             delta,
                             count)) # NOCI
                    continue
                raise ValueError("count %u not handled" % count)
        self.progress("Rally content same")

    def load_rally(self, filename):
        """Load rally points from a file to flight controller."""
        self.progress("Loading rally points (%s)" % filename)
        path = os.path.join(testdir, self.current_test_name_directory, filename)
        mavproxy = self.start_mavproxy()
        mavproxy.send('rally load %s\n' % path)
        mavproxy.expect("Loaded")
        self.stop_mavproxy(mavproxy)

    def load_sample_mission(self):
        self.load_mission(self.sample_mission_filename())

    def load_generic_mission(self, filename, strict=True):
        return self.load_mission_from_filepath(
            os.path.join(testdir, "Generic_Missions"),
            filename,
            strict=strict)

    def load_mission(self, filename, strict=True):
        return self.load_mission_from_filepath(
            self.current_test_name_directory,
            filename,
            strict=strict)

    def wp_to_mission_item_int(self, wp):
        '''convert a MISSION_ITEM to a MISSION_ITEM_INT. We always send as
           MISSION_ITEM_INT to give cm level accuracy
           Swiped from mavproxy_wp.py
        '''
        if wp.get_type() == 'MISSION_ITEM_INT':
            return wp
        wp_int = mavutil.mavlink.MAVLink_mission_item_int_message(
            wp.target_system,
            wp.target_component,
            wp.seq,
            wp.frame,
            wp.command,
            wp.current,
            wp.autocontinue,
            wp.param1,
            wp.param2,
            wp.param3,
            wp.param4,
            int(wp.x*1.0e7),
            int(wp.y*1.0e7),
            wp.z)
        return wp_int

    def load_mission_from_filepath(self, filepath, filename, target_system=1, target_component=1, strict=True):
        self.progress("Loading mission (%s)" % filename)
        path = os.path.join(testdir, filepath, filename)
        wploader = mavwp.MAVWPLoader(
            target_system=target_system,
            target_component=target_component
        )
        wploader.load(path)
        wpoints_int = [self.wp_to_mission_item_int(x) for x in wploader.wpoints]
        self.check_mission_upload_download(wpoints_int, strict=strict)
        return len(wpoints_int)

    def load_mission_using_mavproxy(self, mavproxy, filename):
        return self.load_mission_from_filepath_using_mavproxy(
            mavproxy,
            self.current_test_name_directory,
            filename)

    def load_mission_from_filepath_using_mavproxy(self,
                                                  mavproxy,
                                                  filepath,
                                                  filename):
        """Load a mission from a file to flight controller."""
        self.progress("Loading mission (%s)" % filename)
        path = os.path.join(testdir, filepath, filename)
        tstart = self.get_sim_time()
        while True:
            t2 = self.get_sim_time()
            if t2 - tstart > 10:
                raise AutoTestTimeoutException("Failed to do waypoint thing")
            # the following hack is to get around MAVProxy statustext deduping:
            while time.time() - self.last_wp_load < 3:
                self.progress("Waiting for MAVProxy de-dupe timer to expire")
                self.drain_mav_unparsed()
                time.sleep(0.1)
            mavproxy.send('wp load %s\n' % path)
            mavproxy.expect('Loaded ([0-9]+) waypoints from')
            load_count = mavproxy.match.group(1)
            self.last_wp_load = time.time()
            mavproxy.expect("Flight plan received")
            mavproxy.send('wp list\n')
            mavproxy.expect('Requesting ([0-9]+) waypoints')
            request_count = mavproxy.match.group(1)
            if load_count != request_count:
                self.progress("request_count=%s != load_count=%s" %
                              (request_count, load_count))
                continue
            mavproxy.expect('Saved ([0-9]+) waypoints to (.+?way.txt)')
            save_count = mavproxy.match.group(1)
            if save_count != request_count:
                raise NotAchievedException("request count != load count")
            # warning: this assumes MAVProxy was started in the CWD!
            # on the autotest server we invoke autotest.py one-up from
            # the git root, like this:
            # timelimit 32000 APM/Tools/autotest/autotest.py --timeout=30000 > buildlogs/autotest-output.txt 2>&1
            # that means the MAVProxy log files are not reltopdir!
            saved_filepath = mavproxy.match.group(2)
            saved_filepath = saved_filepath.rstrip()
            self.assert_mission_files_same(path, saved_filepath)
            break
        mavproxy.send('wp status\n')
        mavproxy.expect(r'Have (\d+) of (\d+)')
        status_have = mavproxy.match.group(1)
        status_want = mavproxy.match.group(2)
        if status_have != status_want:
            raise ValueError("status count mismatch")
        if status_have != save_count:
            raise ValueError("status have not equal to save count")

        wploader = mavwp.MAVWPLoader()
        wploader.load(path)
        num_wp = wploader.count()
        if num_wp != int(status_have):
            raise ValueError("num_wp=%u != status_have=%u" %
                             (num_wp, int(status_have)))
        if num_wp == 0:
            raise ValueError("No waypoints loaded?!")

        return num_wp

    def save_mission_to_file_using_mavproxy(self, mavproxy, filename):
        """Save a mission to a file"""
        mavproxy.send('wp list\n')
        mavproxy.expect('Requesting [0-9]+ waypoints')
        mavproxy.send('wp save %s\n' % filename)
        mavproxy.expect('Saved ([0-9]+) waypoints')
        num_wp = int(mavproxy.match.group(1))
        self.progress("num_wp: %d" % num_wp)
        return num_wp

    def string_for_frame(self, frame):
        return mavutil.mavlink.enums["MAV_FRAME"][frame].name

    def frames_equivalent(self, f1, f2):
        pairs = [
            (mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
             mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT),
            (mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT),
            (mavutil.mavlink.MAV_FRAME_GLOBAL,
             mavutil.mavlink.MAV_FRAME_GLOBAL_INT),
        ]
        for pair in pairs:
            if (f1 == pair[0] and f2 == pair[1]):
                return True
            if (f1 == pair[1] and f2 == pair[0]):
                return True
        return f1 == f2

    def check_mission_items_same(self,
                                 check_atts,
                                 want,
                                 got,
                                 epsilon=None,
                                 skip_first_item=False,
                                 strict=True):
        self.progress("Checking mission items same")
        if epsilon is None:
            epsilon = 1
        if len(want) != len(got):
            raise NotAchievedException("Incorrect item count (want=%u got=%u)" % (len(want), len(got)))
        self.progress("Checking %u items" % len(want))
        for i in range(0, len(want)):
            if skip_first_item and i == 0:
                continue
            item = want[i]
            downloaded_item = got[i]

            check_atts = ['mission_type', 'command', 'x', 'y', 'seq', 'param1']
            # z is not preserved

            self.progress("Comparing (%s) and (%s)" % (str(item), str(downloaded_item)))

            for att in check_atts:
                item_val = getattr(item, att)
                downloaded_item_val = getattr(downloaded_item, att)
                if abs(item_val - downloaded_item_val) > epsilon:
                    raise NotAchievedException(
                        "Item %u (%s) has different %s after download want=%s got=%s (got-item=%s)" %
                        (i, str(item), att, str(item_val), str(downloaded_item_val), str(downloaded_item)))
                # for waypoint items ensure z and frame are preserved:
            self.progress("Type is %u" % got[0].mission_type)
            if got[0].mission_type == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
                item_val = getattr(item, 'frame')
                downloaded_item_val = getattr(downloaded_item, 'frame')
                # if you are thinking of adding another, "don't annoy
                # me, I know missions aren't troundtripped" non-strict
                # thing here, DON'T do it without first checking "def
                # assert_mission_files_same"; it makes the same checks
                # as will be needed here eventually.
                if ((strict or self.ardupilot_stores_frame_for_cmd(getattr(item, 'command'))) and
                        not self.frames_equivalent(item_val, downloaded_item_val)):
                    raise NotAchievedException("Frame not same (got=%s want=%s)" %
                                               (self.string_for_frame(downloaded_item_val),
                                                self.string_for_frame(item_val)))
                if abs(item.z - downloaded_item.z) > 1.0E-3: # error should be less than 1 mm
                    raise NotAchievedException("Z not preserved (got=%f want=%f)" %
                                               (downloaded_item.z, item.z))

    def check_fence_items_same(self, want, got, strict=True):
        check_atts = ['mission_type', 'command', 'x', 'y', 'seq', 'param1']
        return self.check_mission_items_same(check_atts, want, got, strict=strict)

    def check_mission_waypoint_items_same(self, want, got, strict=True):
        check_atts = ['mission_type', 'command', 'x', 'y', 'z', 'seq', 'param1']
        return self.check_mission_items_same(check_atts, want, got, skip_first_item=True, strict=strict)

    def check_mission_item_upload_download(self, items, itype, mission_type, strict=True):
        self.progress("check %s _upload/download: upload %u items" %
                      (itype, len(items),))
        self.upload_using_mission_protocol(mission_type, items)
        self.progress("check %s upload/download: download items" % itype)
        downloaded_items = self.download_using_mission_protocol(mission_type)
        self.progress("Downloaded items: (%s)" % str(downloaded_items))
        if len(items) != len(downloaded_items):
            raise NotAchievedException("Did not download same number of items as uploaded want=%u got=%u" %
                                       (len(items), len(downloaded_items)))
        if mission_type == mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
            self.check_fence_items_same(items, downloaded_items, strict=strict)
        elif mission_type == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
            self.check_mission_waypoint_items_same(items, downloaded_items, strict=strict)
        else:
            raise NotAchievedException("Unhandled")

    def check_fence_upload_download(self, items):
        self.check_mission_item_upload_download(
            items,
            "fence",
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE)

    def check_mission_upload_download(self, items, strict=True):
        self.check_mission_item_upload_download(
            items,
            "waypoints",
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            strict=strict)

    def check_dflog_message_rates(self, log_filepath, message_rates):
        reader = self.dfreader_for_path(log_filepath)

        counts = {}
        first = None
        while True:
            m = reader.recv_match()
            if m is None:
                break
            if (m.fmt.instance_field is not None and
                    getattr(m, m.fmt.instance_field) != 0):
                continue

            t = m.get_type()
#            print("t=%s" % str(t))
            if t not in counts:
                counts[t] = 0
            counts[t] += 1

            if hasattr(m, 'TimeUS'):
                if first is None:
                    first = m
                last = m

        if first is None:
            raise NotAchievedException("Did not get any messages")
        delta_time_us = last.TimeUS - first.TimeUS

        for (t, want_rate) in message_rates.items():
            if t not in counts:
                raise NotAchievedException("Wanted %s but got none" % t)
            self.progress("Got (%u)" % counts[t])
            got_rate = counts[t] / delta_time_us * 1000000

            if abs(want_rate - got_rate) > 5:
                raise NotAchievedException("Not getting %s data at wanted rate want=%f got=%f" %
                                           (t, want_rate, got_rate))

    def generate_rate_sample_log(self):
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.delay_sim_time(20)
        path = self.current_onboard_log_filepath()
        self.progress("Rate sample log (%s)" % path)
        self.reboot_sitl()
        return path

    def rc_defaults(self):
        return {
            1: 1500,
            2: 1500,
            3: 1500,
            4: 1500,
            5: 1500,
            6: 1500,
            7: 1500,
            8: 1500,
            9: 1500,
            10: 1500,
            11: 1500,
            12: 1500,
            13: 1500,
            14: 1500,
            15: 1500,
            16: 1500,
        }

    def set_rc_from_map(self, _map, timeout=20):
        map_copy = _map.copy()
        self.rc_queue.put(map_copy)

        if self.rc_thread is None:
            self.rc_thread = threading.Thread(target=self.rc_thread_main, name='RC')
            if self.rc_thread is None:
                raise NotAchievedException("Could not create thread")
            self.rc_thread.start()

        tstart = self.get_sim_time()
        while True:
            if tstart - self.get_sim_time_cached() > timeout:
                raise NotAchievedException("Failed to set RC values")
            m = self.mav.recv_match(type='RC_CHANNELS', blocking=True, timeout=1)
            if m is None:
                continue
            bad_channels = ""
            for chan in map_copy:
                chan_pwm = getattr(m, "chan" + str(chan) + "_raw")
                if chan_pwm != map_copy[chan]:
                    bad_channels += " (ch=%u want=%u got=%u)" % (chan, map_copy[chan], chan_pwm)
                    break
            if len(bad_channels) == 0:
                self.progress("RC values good")
                break
            self.progress("RC values bad:%s" % bad_channels)
            if not self.rc_thread.is_alive():
                self.rc_thread = None
                raise ValueError("RC thread is dead")  # FIXME: type

    def rc_thread_main(self):
        chan16 = [1000] * 16

        sitl_output = mavutil.mavudp("127.0.0.1:5501", input=False)
        buf = None

        while True:
            if self.rc_thread_should_quit:
                break

            # the 0.05 here means we're updating the RC values into
            # the autopilot at 20Hz - that's our 50Hz wallclock, , not
            # the autopilot's simulated 20Hz, so if speedup is 10 the
            # autopilot will see ~2Hz.
            try:
                map_copy = self.rc_queue.get(timeout=0.02)

                # 16 packed entries:
                for i in range(1, 17):
                    if i in map_copy:
                        chan16[i-1] = map_copy[i]

            except Queue.Empty:
                pass

            buf = struct.pack('<HHHHHHHHHHHHHHHH', *chan16)

            if buf is None:
                continue

            sitl_output.write(buf)

    def set_rc_default(self):
        """Setup all simulated RC control to 1500."""
        _defaults = self.rc_defaults()
        self.set_rc_from_map(_defaults)

    def check_rc_defaults(self):
        """Ensure all rc outputs are at defaults"""
        self.do_timesync_roundtrip()
        _defaults = self.rc_defaults()
        m = self.assert_receive_message('RC_CHANNELS', timeout=5)
        need_set = {}
        for chan in _defaults:
            default_value = _defaults[chan]
            current_value = getattr(m, "chan" + str(chan) + "_raw")
            if default_value != current_value:
                self.progress("chan=%u needs resetting is=%u want=%u" %
                              (chan, current_value, default_value))
                need_set[chan] = default_value
        self.set_rc_from_map(need_set)

    def set_rc(self, chan, pwm, timeout=20):
        """Setup a simulated RC control to a PWM value"""
        self.set_rc_from_map({chan: pwm}, timeout=timeout)

    def location_offset_ne(self, location, north, east):
        '''move location in metres'''
        print("old: %f %f" % (location.lat, location.lng))
        (lat, lng) = mp_util.gps_offset(location.lat, location.lng, east, north)
        location.lat = lat
        location.lng = lng
        print("new: %f %f" % (location.lat, location.lng))

    def home_relative_loc_ne(self, n, e):
        ret = self.home_position_as_mav_location()
        self.location_offset_ne(ret, n, e)
        return ret

    def zero_throttle(self):
        """Set throttle to zero."""
        if self.is_rover():
            self.set_rc(3, 1500)
        else:
            self.set_rc(3, 1000)

    def set_output_to_max(self, chan):
        """Set output to max with RC Radio taking into account REVERSED parameter."""
        is_reversed = self.get_parameter("RC%u_REVERSED" % chan)
        out_max = int(self.get_parameter("RC%u_MAX" % chan))
        out_min = int(self.get_parameter("RC%u_MIN" % chan))
        if is_reversed == 0:
            self.set_rc(chan, out_max)
        else:
            self.set_rc(chan, out_min)

    def set_output_to_min(self, chan):
        """Set output to min with RC Radio taking into account REVERSED parameter."""
        is_reversed = self.get_parameter("RC%u_REVERSED" % chan)
        out_max = int(self.get_parameter("RC%u_MAX" % chan))
        out_min = int(self.get_parameter("RC%u_MIN" % chan))
        if is_reversed == 0:
            self.set_rc(chan, out_min)
        else:
            self.set_rc(chan, out_max)

    def set_output_to_trim(self, chan):
        """Set output to trim with RC Radio."""
        out_trim = int(self.get_parameter("RC%u_TRIM" % chan))
        self.set_rc(chan, out_trim)

    def get_stick_arming_channel(self):
        """Return the Rudder channel number as set in parameter."""
        raise ErrorException("Rudder parameter is not supported by vehicle %s frame %s", (self.vehicleinfo_key(), self.frame))

    def get_disarm_delay(self):
        """Return disarm delay value."""
        raise ErrorException("Disarm delay is not supported by vehicle %s frame %s", (self.vehicleinfo_key(), self.frame))

    def arming_test_mission(self):
        """Load arming test mission.
        This mission is used to allow to change mode to AUTO. For each vehicle
        it get an unlimited wait waypoint and the starting takeoff if needed."""
        if self.is_rover() or self.is_plane() or self.is_sub():
            return os.path.join(testdir, self.current_test_name_directory + "test_arming.txt")
        else:
            return None

    def set_safetyswitch_on(self):
        self.set_safetyswitch(1)

    def set_safetyswitch_off(self):
        self.set_safetyswitch(0)

    def set_safetyswitch(self, value, target_system=1, target_component=1):
        self.mav.mav.set_mode_send(
            target_system,
            mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_SAFETY,
            value)

    def armed(self):
        """Return true if vehicle is armed and safetyoff"""
        return self.mav.motors_armed()

    def send_mavlink_arm_command(self):
        target_sysid = 1
        target_compid = 1
        self.mav.mav.command_long_send(
            target_sysid,
            target_compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            1, # confirmation
            1,  # ARM
            0,
            0,
            0,
            0,
            0,
            0)

    def set_analog_rangefinder_parameters(self):
        self.set_parameters({
            "RNGFND1_TYPE": 1,
            "RNGFND1_MIN_CM": 0,
            "RNGFND1_MAX_CM": 4000,
            "RNGFND1_SCALING": 12.12,
            "RNGFND1_PIN": 0,
        })

    def send_debug_trap(self, timeout=6000):
        self.progress("Sending trap to autopilot")
        self.run_cmd(mavutil.mavlink.MAV_CMD_DEBUG_TRAP,
                     32451, # magic number to trap
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=timeout)

    def try_arm(self, result=True, expect_msg=None, timeout=60):
        """Send Arming command, wait for the expected result and statustext."""
        self.progress("Try arming and wait for expected result")
        self.drain_mav()
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     1,  # ARM
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED if result else mavutil.mavlink.MAV_RESULT_FAILED,
                     timeout=timeout)
        if expect_msg is not None:
            self.wait_statustext(
                expect_msg,
                timeout=timeout,
                the_function=lambda: self.send_cmd(
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    1,  # ARM
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    target_sysid=None,
                    target_compid=None,
                ))

    def arm_vehicle(self, timeout=20, force=False):
        """Arm vehicle with mavlink arm message."""
        self.progress("Arm motors with MAVLink cmd")
        self.drain_mav()
        p2 = 0
        if force:
            p2 = 2989
        try:
            self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                         1,  # ARM
                         p2,
                         0,
                         0,
                         0,
                         0,
                         0,
                         timeout=timeout)
        except ValueError as e:
            # statustexts are queued; give it a second to arrive:
            self.delay_sim_time(5)
            raise e
        try:
            self.wait_armed()
        except AutoTestTimeoutException:
            raise AutoTestTimeoutException("Failed to ARM with mavlink")
        return True

    def wait_armed(self, timeout=20):
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < timeout:
            self.wait_heartbeat()
            if self.mav.motors_armed():
                self.progress("Motors ARMED")
                return
        raise AutoTestTimeoutException("Did not become armed")

    def disarm_vehicle(self, timeout=60, force=False):
        """Disarm vehicle with mavlink disarm message."""
        self.progress("Disarm motors with MAVLink cmd")
        self.drain_mav_unparsed()
        p2 = 0
        if force:
            p2 = 21196 # magic force disarm value
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     0,  # DISARM
                     p2,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=timeout)
        return self.wait_disarmed()

    def disarm_vehicle_expect_fail(self):
        '''disarm, checking first that non-forced disarm fails, then doing a forced disarm'''
        self.progress("Disarm - expect to fail")
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     0,  # DISARM
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=10,
                     want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.progress("Disarm - forced")
        self.disarm_vehicle(force=True)

    def wait_disarmed_default_wait_time(self):
        return 30

    def wait_disarmed(self, timeout=None, tstart=None):
        if timeout is None:
            timeout = self.wait_disarmed_default_wait_time()
        self.progress("Waiting for DISARM")
        if tstart is None:
            tstart = self.get_sim_time()
        last_print_time = 0
        while True:
            now = self.get_sim_time_cached()
            delta = now - tstart
            if delta > timeout:
                raise AutoTestTimeoutException("Failed to DISARM within %fs" %
                                               (timeout,))
            if now - last_print_time > 1:
                self.progress("Waiting for disarm (%.2fs so far of allowed %.2f)" % (delta, timeout))
                last_print_time = now
            self.wait_heartbeat(quiet=True)
#            self.progress("Got heartbeat")
            if not self.mav.motors_armed():
                self.progress("DISARMED after %.2f seconds (allowed=%.2f)" %
                              (delta, timeout))
                return True

    def wait_attitude(self, desroll=None, despitch=None, timeout=2, tolerance=10, message_type='ATTITUDE'):
        '''wait for an attitude (degrees)'''
        if desroll is None and despitch is None:
            raise ValueError("despitch or desroll must be supplied")
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise AutoTestTimeoutException("Failed to achieve attitude")
            m = self.assert_receive_message(message_type, timeout=60)
            roll_deg = math.degrees(m.roll)
            pitch_deg = math.degrees(m.pitch)
            self.progress("wait_att: roll=%f desroll=%s pitch=%f despitch=%s" %
                          (roll_deg, desroll, pitch_deg, despitch))
            if desroll is not None and abs(roll_deg - desroll) > tolerance:
                continue
            if despitch is not None and abs(pitch_deg - despitch) > tolerance:
                continue
            return

    def wait_attitude_quaternion(self,
                                 desroll=None,
                                 despitch=None,
                                 timeout=2,
                                 tolerance=10,
                                 message_type='ATTITUDE_QUATERNION'):
        '''wait for an attitude (degrees)'''
        if desroll is None and despitch is None:
            raise ValueError("despitch or desroll must be supplied")
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise AutoTestTimeoutException("Failed to achieve attitude")
            m = self.poll_message(message_type)
            q = quaternion.Quaternion([m.q1, m.q2, m.q3, m.q4])
            euler = q.euler
            roll = euler[0]
            pitch = euler[1]
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            self.progress("wait_att_quat: roll=%f desroll=%s pitch=%f despitch=%s" %
                          (roll_deg, desroll, pitch_deg, despitch))
            if desroll is not None and abs(roll_deg - desroll) > tolerance:
                continue
            if despitch is not None and abs(pitch_deg - despitch) > tolerance:
                continue
            self.progress("wait_att_quat: achieved")
            return

    def CPUFailsafe(self):
        '''Most vehicles just disarm on failsafe'''
        # customising the SITL commandline ensures the process will
        # get stopped/started at the end of the test
        if self.frame is None:
            raise ValueError("Frame is none?")
        self.customise_SITL_commandline([])
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("Sending enter-cpu-lockup")
        # when we're in CPU lockup we don't get SYSTEM_TIME messages,
        # so get_sim_time breaks:
        tstart = self.get_sim_time()
        self.send_cmd_enter_cpu_lockup()
        self.wait_disarmed(timeout=5, tstart=tstart)
        # we're not getting SYSTEM_TIME messages at this point.... and
        # we're in a weird state where the vehicle is armed but the
        # motors are not, and we can't disarm further because Copter
        # looks at whether its *motors* are armed as part of its
        # disarm process.
        self.reset_SITL_commandline()

    def cpufailsafe_wait_servo_channel_value(self, channel, value, timeout=30):
        '''we get restricted messages while doing cpufailsafe, this working then'''
        start = time.time()
        while True:
            if time.time() - start > timeout:
                raise NotAchievedException("Did not achieve value")
            m = self.assert_receive_message('SERVO_OUTPUT_RAW', timeout=1)
            channel_field = "servo%u_raw" % channel
            m_value = getattr(m, channel_field, None)
            self.progress("Servo%u=%u want=%u" % (channel, m_value, value))
            if m_value == value:
                break

    def plane_CPUFailsafe(self):
        '''In lockup Plane should copy RC inputs to RC outputs'''
        # customising the SITL commandline ensures the process will
        # get stopped/started at the end of the test
        self.customise_SITL_commandline([])
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("Sending enter-cpu-lockup")
        # when we're in CPU lockup we don't get SYSTEM_TIME messages,
        # so get_sim_time breaks:
        self.send_cmd_enter_cpu_lockup()
        start_time = time.time() # not sim time!
        self.context_push()
        self.context_collect("STATUSTEXT")
        while True:
            want = "Initialising ArduPilot"
            if time.time() - start_time > 30:
                raise NotAchievedException("Did not get %s" % want)
            # we still need to parse the incoming messages:
            try:
                self.wait_statustext(want, timeout=0.1, check_context=True, wallclock_timeout=1)
                break
            except AutoTestTimeoutException:
                pass
        self.context_pop()
        # Different scaling for RC input and servo output means the
        # servo output value isn't the rc input value:
        self.progress("Setting RC to 1200")
        self.rc_queue.put({2: 1200})
        self.progress("Waiting for servo of 1260")
        self.cpufailsafe_wait_servo_channel_value(2, 1260)
        self.rc_queue.put({2: 1700})
        self.cpufailsafe_wait_servo_channel_value(2, 1660)
        self.reset_SITL_commandline()

    def mavproxy_arm_vehicle(self, mavproxy):
        """Arm vehicle with mavlink arm message send from MAVProxy."""
        self.progress("Arm motors with MavProxy")
        mavproxy.send('arm throttle\n')
        self.mav.motors_armed_wait()
        self.progress("ARMED")
        return True

    def mavproxy_disarm_vehicle(self, mavproxy):
        """Disarm vehicle with mavlink disarm message send from MAVProxy."""
        self.progress("Disarm motors with MavProxy")
        mavproxy.send('disarm\n')
        self.wait_disarmed()
        return True

    def arm_motors_with_rc_input(self, timeout=20):
        """Arm motors with radio."""
        self.progress("Arm motors with radio")
        self.set_output_to_max(self.get_stick_arming_channel())
        tstart = self.get_sim_time()
        while True:
            self.wait_heartbeat()
            tdelta = self.get_sim_time_cached() - tstart
            if self.mav.motors_armed():
                self.progress("MOTORS ARMED OK WITH RADIO")
                self.set_output_to_trim(self.get_stick_arming_channel())
                self.progress("Arm in %ss" % tdelta)  # TODO check arming time
                return
            print("Not armed after %f seconds" % (tdelta))
            if tdelta > timeout:
                break
        self.set_output_to_trim(self.get_stick_arming_channel())
        raise NotAchievedException("Failed to ARM with radio")

    def disarm_motors_with_rc_input(self, timeout=20, watch_for_disabled=False):
        """Disarm motors with radio."""
        self.progress("Disarm motors with radio")
        self.do_timesync_roundtrip()
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_output_to_min(self.get_stick_arming_channel())
        tstart = self.get_sim_time()
        ret = False
        while self.get_sim_time_cached() < tstart + timeout:
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                disarm_delay = self.get_sim_time_cached() - tstart
                self.progress("MOTORS DISARMED OK WITH RADIO (in %ss)" % disarm_delay)
                ret = True
                break
            if self.statustext_in_collections("Rudder disarm: disabled"):
                self.progress("Found 'Rudder disarm: disabled' in statustext")
                break
            self.context_clear_collection('STATUSTEXT')
        self.set_output_to_trim(self.get_stick_arming_channel())
        self.context_pop()
        if not ret:
            raise NotAchievedException("Failed to DISARM with RC input")

    def arm_motors_with_switch(self, switch_chan, timeout=20):
        """Arm motors with switch."""
        self.progress("Arm motors with switch %d" % switch_chan)
        self.set_rc(switch_chan, 2000)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < timeout:
            self.wait_heartbeat()
            if self.mav.motors_armed():
                self.progress("MOTORS ARMED OK WITH SWITCH")
                return
        raise NotAchievedException("Failed to ARM with switch")

    def disarm_motors_with_switch(self, switch_chan, timeout=20):
        """Disarm motors with switch."""
        self.progress("Disarm motors with switch %d" % switch_chan)
        self.set_rc(switch_chan, 1000)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                self.progress("MOTORS DISARMED OK WITH SWITCH")
                return
        raise NotAchievedException("Failed to DISARM with switch")

    def disarm_wait(self, timeout=10):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not disarm")
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                return

    def wait_autodisarm_motors(self):
        """Wait for Autodisarm motors within disarm delay
        this feature is only available in copter (DISARM_DELAY) and plane (LAND_DISARMDELAY)."""
        self.progress("Wait autodisarming motors")
        disarm_delay = self.get_disarm_delay()
        tstart = self.get_sim_time()
        timeout = disarm_delay * 2
        while self.get_sim_time_cached() < tstart + timeout:
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                disarm_time = self.get_sim_time_cached() - tstart
                self.progress("MOTORS AUTODISARMED")
                self.progress("Autodisarm in %ss, expect less than %ss" % (disarm_time, disarm_delay))
                return disarm_time <= disarm_delay
        raise AutoTestTimeoutException("Failed to AUTODISARM")

    def set_autodisarm_delay(self, delay):
        """Set autodisarm delay"""
        raise ErrorException("Auto disarm is not supported by vehicle %s frame %s", (self.vehicleinfo_key(), self.frame))

    @staticmethod
    def should_fetch_all_for_parameter_change(param_name):
        return False  # FIXME: if we allow MAVProxy then allow this
        if fnmatch.fnmatch(param_name, "*_ENABLE") or fnmatch.fnmatch(param_name, "*_ENABLED"):
            return True
        if param_name in ["ARSPD_TYPE",
                          "ARSPD2_TYPE",
                          "BATT2_MONITOR",
                          "CAN_DRIVER",
                          "COMPASS_PMOT_EN",
                          "OSD_TYPE",
                          "RSSI_TYPE",
                          "WENC_TYPE"]:
            return True
        return False

    def send_set_parameter_direct(self, name, value):
        self.mav.mav.param_set_send(self.sysid_thismav(),
                                    1,
                                    name.encode('ascii'),
                                    value,
                                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def send_set_parameter_mavproxy(self, name, value):
        self.mavproxy.send("param set %s %s\n" % (name, str(value)))

    def send_set_parameter(self, name, value, verbose=False):
        if verbose:
            self.progress("Send set param for (%s) (%f)" % (name, value))
        return self.send_set_parameter_direct(name, value)

    def set_parameter(self, name, value, **kwargs):
        self.set_parameters({name: value}, **kwargs)

    def set_parameters(self, parameters, add_to_context=True, epsilon_pct=0.00001, verbose=True, attempts=None):
        """Set parameters from vehicle."""

        want = copy.copy(parameters)
        self.progress("set_parameters: (%s)" % str(want))
        self.drain_mav()
        if len(want) == 0:
            return

        if attempts is None:
            # we can easily fill ArduPilot's param-set/param-get queue
            # which is quite short.  So we retry *a lot*.
            attempts = len(want) * 10

        param_value_messages = []

        def add_param_value(mav, m):
            t = m.get_type()
            if t != "PARAM_VALUE":
                return
            param_value_messages.append(m)

        self.install_message_hook(add_param_value)

        original_values = {}
        autopilot_values = {}
        for i in range(attempts):
            self.drain_mav(quiet=True)
            self.drain_all_pexpects()
            received = set()
            for (name, value) in want.items():
                if verbose:
                    self.progress("%s want=%f autopilot=%s (attempt=%u/%u)" %
                                  (name, value, autopilot_values.get(name, 'None'), i+1, attempts))
                if name not in autopilot_values:
                    if verbose:
                        self.progress("Requesting (%s)" % (name,))
                    self.send_get_parameter_direct(name)
                    continue
                delta = abs(autopilot_values[name] - value)
                if delta <= epsilon_pct*0.01*abs(value):
                    # correct value
                    self.progress("%s is now %f" % (name, autopilot_values[name]))
                    if add_to_context:
                        context_param_name_list = [p[0] for p in self.context_get().parameters]
                        if name.upper() not in context_param_name_list:
                            self.context_get().parameters.append((name, original_values[name]))
                    received.add(name)
                    continue
                self.progress("Sending set (%s) to (%f) (old=%f)" % (name, value, original_values[name]))
                self.send_set_parameter_direct(name, value)
            for name in received:
                del want[name]
            if len(want):
                # problem here is that a reboot can happen after we
                # send the request but before we receive the reply:
                try:
                    self.do_timesync_roundtrip(quiet=True)
                except AutoTestTimeoutException:
                    pass
            for m in param_value_messages:
                if m.param_id in want:
                    self.progress("Received wanted PARAM_VALUE %s=%f" %
                                  (str(m.param_id), m.param_value))
                    autopilot_values[m.param_id] = m.param_value
                    if m.param_id not in original_values:
                        original_values[m.param_id] = m.param_value
            param_value_messages = []

        self.remove_message_hook(add_param_value)

        if len(want) == 0:
            return
        raise ValueError("Failed to set parameters (%s)" % want)

    def get_parameter(self, *args, **kwargs):
        return self.get_parameter_direct(*args, **kwargs)

    def send_get_parameter_direct(self, name):
        encname = name
        if sys.version_info.major >= 3 and type(encname) != bytes:
            encname = bytes(encname, 'ascii')
        self.mav.mav.param_request_read_send(self.sysid_thismav(),
                                             1,
                                             encname,
                                             -1)

    def get_parameter_direct(self, name, attempts=1, timeout=60, verbose=True, timeout_in_wallclock=False):
        while attempts > 0:
            attempts -= 1
            if verbose:
                self.progress("Sending param_request_read for (%s)" % name)
            # we MUST parse here or collections fail where we need
            # them to work!
            self.drain_mav(quiet=True)
            if timeout_in_wallclock:
                tstart = time.time()
            else:
                tstart = self.get_sim_time()
            self.send_get_parameter_direct(name)
            while True:
                if timeout_in_wallclock:
                    now = time.time()
                else:
                    now = self.get_sim_time_cached()
                    if tstart > now:
                        self.progress("Time wrap detected")
                        # we're going to have to send another request...
                        break
                delta_time = now - tstart
                if delta_time > timeout:
                    break
                m = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.1)
                if verbose:
                    self.progress("get_parameter(%s): %s" % (name, str(m), ))
                if m is None:
                    continue
                if m.param_id == name:
                    if delta_time > 5:
                        self.progress("Long time to get parameter: %fs" % (delta_time,))
                    return m.param_value
                if verbose:
                    self.progress("(%s) != (%s)" % (m.param_id, name,))
        raise NotAchievedException("Failed to retrieve parameter (%s)" % name)

    def get_parameter_mavproxy(self, mavproxy, name, attempts=1, timeout=60):
        """Get parameters from vehicle."""
        for i in range(0, attempts):
            mavproxy.send("param fetch %s\n" % name)
            try:
                mavproxy.expect("%s = ([-0-9.]*)\r\n" % (name,), timeout=timeout/attempts)
                try:
                    # sometimes race conditions garble the MAVProxy output
                    ret = float(mavproxy.match.group(1))
                except ValueError:
                    continue
                return ret
            except pexpect.TIMEOUT:
                pass
        raise NotAchievedException("Failed to retrieve parameter (%s)" % name)

    def context_get(self):
        """Get Saved parameters."""
        return self.contexts[-1]

    def context_push(self):
        """Save a copy of the parameters."""
        context = Context()
        self.contexts.append(context)
        # add a message hook so we can collect messages conveniently:

        def mh(mav, m):
            t = m.get_type()
            if t in context.collections:
                context.collections[t].append(m)
        self.install_message_hook_context(mh)

    def context_collect(self, msg_type):
        '''start collecting messages of type msg_type into context collection'''
        context = self.context_get()
        if msg_type in context.collections:
            return
        context.collections[msg_type] = []

    def context_collection(self, msg_type):
        '''return messages in collection'''
        context = self.context_get()
        if msg_type not in context.collections:
            raise NotAchievedException("Not collecting (%s)" % str(msg_type))
        return context.collections[msg_type]

    def context_clear_collection(self, msg_type):
        '''clear collection of message type msg_type'''
        context = self.context_get()
        if msg_type not in context.collections:
            raise NotAchievedException("Not collecting (%s)" % str(msg_type))
        context.collections[msg_type] = []

    def context_stop_collecting(self, msg_type):
        '''stop collecting messages of type msg_type in context collection.  Returns the collected messages'''
        context = self.context_get()
        if msg_type not in context.collections:
            raise Exception("Not collecting %s" % str(msg_type))
        ret = context.collections[msg_type]
        del context.collections[msg_type]
        return ret

    def context_pop(self):
        """Set parameters to origin values in reverse order."""
        dead = self.contexts.pop()
        if dead.sitl_commandline_customised and len(self.contexts):
            self.contexts[-1].sitl_commandline_customised = True

        dead_parameters_dict = {}
        for p in dead.parameters:
            dead_parameters_dict[p[0]] = p[1]
        self.set_parameters(dead_parameters_dict, add_to_context=False)
        for hook in dead.message_hooks:
            self.remove_message_hook(hook)

    class Context(object):
        def __init__(self, testsuite):
            self.testsuite = testsuite

        def __enter__(self):
            self.testsuite.context_push()

        def __exit__(self, type, value, traceback):
            self.testsuite.context_pop()
            return False # re-raise any exception

    def sysid_thismav(self):
        return 1

    def run_cmd_int(self,
                    command,
                    p1,
                    p2,
                    p3,
                    p4,
                    x,
                    y,
                    z,
                    want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    timeout=10,
                    target_sysid=None,
                    target_compid=None,
                    frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT):

        if target_sysid is None:
            target_sysid = self.sysid_thismav()
        if target_compid is None:
            target_compid = 1

        self.get_sim_time() # required for timeout in run_cmd_get_ack to work

        """Send a MAVLink command int."""
        self.mav.mav.command_int_send(target_sysid,
                                      target_compid,
                                      frame,
                                      command,
                                      0, # current
                                      0, # autocontinue
                                      p1,
                                      p2,
                                      p3,
                                      p4,
                                      x,
                                      y,
                                      z)
        self.run_cmd_get_ack(command, want_result, timeout)

    def send_cmd(self,
                 command,
                 p1,
                 p2,
                 p3,
                 p4,
                 p5,
                 p6,
                 p7,
                 target_sysid=None,
                 target_compid=None,
                 mav=None,
                 ):
        """Send a MAVLink command long."""
        if mav is None:
            mav = self.mav
        if target_sysid is None:
            target_sysid = self.sysid_thismav()
        if target_compid is None:
            target_compid = 1
        try:
            command_name = mavutil.mavlink.enums["MAV_CMD"][command].name
        except KeyError:
            command_name = "UNKNOWN=%u" % command
        self.progress("Sending COMMAND_LONG to (%u,%u) (%s) (p1=%f p2=%f p3=%f p4=%f p5=%f p6=%f  p7=%f)" %
                      (
                          target_sysid,
                          target_compid,
                          command_name,
                          p1,
                          p2,
                          p3,
                          p4,
                          p5,
                          p6,
                          p7))
        mav.mav.command_long_send(target_sysid,
                                  target_compid,
                                  command,
                                  1,  # confirmation
                                  p1,
                                  p2,
                                  p3,
                                  p4,
                                  p5,
                                  p6,
                                  p7)

    def run_cmd(self,
                command,
                p1,
                p2,
                p3,
                p4,
                p5,
                p6,
                p7,
                want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                target_sysid=None,
                target_compid=None,
                timeout=10,
                quiet=False,
                mav=None):
        self.drain_mav(mav=mav)
        self.get_sim_time() # required for timeout in run_cmd_get_ack to work
        self.send_cmd(
            command,
            p1,
            p2,
            p3,
            p4,
            p5,
            p6,
            p7,
            target_sysid=target_sysid,
            target_compid=target_compid,
            mav=mav,
        )
        self.run_cmd_get_ack(command, want_result, timeout, quiet=quiet, mav=mav)

    def run_cmd_get_ack(self, command, want_result, timeout, quiet=False, mav=None):
        # note that the caller should ensure that this cached
        # timestamp is reasonably up-to-date!
        if mav is None:
            mav = self.mav
        tstart = self.get_sim_time_cached()
        while True:
            if mav != self.mav:
                self.drain_mav()
            delta_time = self.get_sim_time_cached() - tstart
            if delta_time > timeout:
                raise AutoTestTimeoutException("Did not get good COMMAND_ACK within %fs" % timeout)
            m = mav.recv_match(type='COMMAND_ACK',
                               blocking=True,
                               timeout=0.1)
            if m is None:
                continue
            if not quiet:
                self.progress("ACK received: %s (%fs)" % (str(m), delta_time))
            if m.command == command:
                if m.result != want_result:
                    raise ValueError("Expected %s got %s" % (
                        mavutil.mavlink.enums["MAV_RESULT"][want_result].name,
                        mavutil.mavlink.enums["MAV_RESULT"][m.result].name))
                break

    def set_current_waypoint_using_mav_cmd_do_set_mission_current(
            self,
            seq,
            target_sysid=1,
            target_compid=1):
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                     seq,
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=1,
                     target_sysid=target_sysid,
                     target_compid=target_compid)

    def set_current_waypoint_using_mission_set_current(
            self,
            seq,
            target_sysid=1,
            target_compid=1,
            check_afterwards=True):
        self.mav.mav.mission_set_current_send(target_sysid,
                                              target_compid,
                                              seq)
        if check_afterwards:
            self.wait_current_waypoint(seq, timeout=10)

    def set_current_waypoint(self, seq, target_sysid=1, target_compid=1, check_afterwards=True):
        return self.set_current_waypoint_using_mission_set_current(
            seq,
            target_sysid,
            target_compid,
            check_afterwards=check_afterwards
        )

    def verify_parameter_values(self, parameter_stuff, max_delta=0.0):
        bad = ""
        for param in parameter_stuff:
            fetched_value = self.get_parameter(param)
            wanted_value = parameter_stuff[param]
            if type(wanted_value) == tuple:
                max_delta = wanted_value[1]
                wanted_value = wanted_value[0]
            if abs(fetched_value - wanted_value) > max_delta:
                bad += "%s=%f (want=%f +/-%f)    " % (param, fetched_value, wanted_value, max_delta)
        if len(bad):
            raise NotAchievedException("Bad parameter values: %s" %
                                       (bad,))

    #################################################
    # UTILITIES
    #################################################
    @staticmethod
    def longitude_scale(lat):
        ret = math.cos(lat * (math.radians(1)))
        print("scale=%f" % ret)
        return ret

    @staticmethod
    def get_distance(loc1, loc2):
        """Get ground distance between two locations."""
        return AutoTest.get_distance_accurate(loc1, loc2)
        # dlat = loc2.lat - loc1.lat
        # try:
        #     dlong = loc2.lng - loc1.lng
        # except AttributeError:
        #     dlong = loc2.lon - loc1.lon

        # return math.sqrt((dlat*dlat) + (dlong*dlong)*AutoTest.longitude_scale(loc2.lat)) * 1.113195e5

    @staticmethod
    def get_distance_accurate(loc1, loc2):
        """Get ground distance between two locations."""
        try:
            lon1 = loc1.lng
            lon2 = loc2.lng
        except AttributeError:
            lon1 = loc1.lon
            lon2 = loc2.lon

        return mp_util.gps_distance(loc1.lat, lon1, loc2.lat, lon2)

    @staticmethod
    def get_latlon_attr(loc, attrs):
        '''return any found latitude attribute from loc'''
        ret = None
        for attr in attrs:
            if hasattr(loc, attr):
                ret = getattr(loc, attr)
                break
        if ret is None:
            raise ValueError("None of %s in loc(%s)" % (str(attrs), str(loc)))
        return ret

    @staticmethod
    def get_lat_attr(loc):
        '''return any found latitude attribute from loc'''
        return AutoTest.get_latlon_attr(loc, ["lat", "latitude"])

    @staticmethod
    def get_lon_attr(loc):
        '''return any found latitude attribute from loc'''
        return AutoTest.get_latlon_attr(loc, ["lng", "lon", "longitude"])

    @staticmethod
    def get_distance_int(loc1, loc2):
        """Get ground distance between two locations in the normal "int" form
        - lat/lon multiplied by 1e7"""
        loc1_lat = AutoTest.get_lat_attr(loc1)
        loc2_lat = AutoTest.get_lat_attr(loc2)
        loc1_lon = AutoTest.get_lon_attr(loc1)
        loc2_lon = AutoTest.get_lon_attr(loc2)

        return AutoTest.get_distance_accurate(
            mavutil.location(loc1_lat*1e-7, loc1_lon*1e-7),
            mavutil.location(loc2_lat*1e-7, loc2_lon*1e-7))

        # dlat = loc2_lat - loc1_lat
        # dlong = loc2_lon - loc1_lon
        #
        # dlat /= 10000000.0
        # dlong /= 10000000.0
        #
        # return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    def bearing_to(self, loc):
        '''return bearing from here to location'''
        here = self.mav.location()
        return self.get_bearing(here, loc)

    @staticmethod
    def get_bearing(loc1, loc2):
        """Get bearing from loc1 to loc2."""
        off_x = loc2.lng - loc1.lng
        off_y = loc2.lat - loc1.lat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing

    def send_cmd_do_set_mode(self, mode):
        self.send_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            self.get_mode_from_mode_mapping(mode),
            0,
            0,
            0,
            0,
            0
        )

    def change_mode(self, mode, timeout=60):
        '''change vehicle flightmode'''
        self.wait_heartbeat()
        self.progress("Changing mode to %s" % mode)
        self.send_cmd_do_set_mode(mode)
        tstart = self.get_sim_time()
        while not self.mode_is(mode):
            custom_num = self.mav.messages['HEARTBEAT'].custom_mode
            self.progress("mav.flightmode=%s Want=%s custom=%u" % (
                self.mav.flightmode, mode, custom_num))
            if (timeout is not None and
                    self.get_sim_time_cached() > tstart + timeout):
                raise WaitModeTimeout("Did not change mode")
            self.send_cmd_do_set_mode(mode)
        self.progress("Got mode %s" % mode)

    def capable(self, capability):
        return self.get_autopilot_capabilities() & capability

    def assert_capability(self, capability):
        if not self.capable(capability):
            name = mavutil.mavlink.enums["MAV_PROTOCOL_CAPABILITY"][capability].name
            raise NotAchievedException("AutoPilot does not have capbility %s" % (name,))

    def assert_no_capability(self, capability):
        if self.capable(capability):
            name = mavutil.mavlink.enums["MAV_PROTOCOL_CAPABILITY"][capability].name
            raise NotAchievedException("AutoPilot has feature %s (when it shouln't)" % (name,))

    def get_autopilot_capabilities(self):
        # Cannot use run_cmd otherwise the respond is lost during the wait for ACK
        self.mav.mav.command_long_send(self.sysid_thismav(),
                                       1,
                                       mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
                                       0,  # confirmation
                                       1,  # 1: Request autopilot version
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0)
        m = self.assert_receive_message('AUTOPILOT_VERSION', timeout=10)
        return m.capabilities

    def test_get_autopilot_capabilities(self):
        self.assert_capability(mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT)
        self.assert_capability(mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION)

    def get_mode_from_mode_mapping(self, mode):
        """Validate and return the mode number from a string or int."""
        mode_map = self.mav.mode_mapping()
        if mode_map is None:
            mav_type = self.mav.messages['HEARTBEAT'].type
            mav_autopilot = self.mav.messages['HEARTBEAT'].autopilot
            raise ErrorException("No mode map for (mav_type=%s mav_autopilot=%s)" % (mav_type, mav_autopilot))
        if isinstance(mode, str):
            if mode in mode_map:
                return mode_map.get(mode)
        if mode in mode_map.values():
            return mode
        self.progress("No mode (%s); available modes '%s'" % (mode, mode_map))
        raise ErrorException("Unknown mode '%s'" % mode)

    def run_cmd_do_set_mode(self,
                            mode,
                            timeout=30,
                            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED):
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = self.get_mode_from_mode_mapping(mode)
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            base_mode,
            custom_mode,
            0,
            0,
            0,
            0,
            0,
            want_result=want_result,
            timeout=timeout
        )

    def do_set_mode_via_command_long(self, mode, timeout=30):
        """Set mode with a command long message."""
        tstart = self.get_sim_time()
        want_custom_mode = self.get_mode_from_mode_mapping(mode)
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise AutoTestTimeoutException("Failed to change mode")
            self.run_cmd_do_set_mode(mode, timeout=10)
            m = self.wait_heartbeat()
            self.progress("Got mode=%u want=%u" % (m.custom_mode, want_custom_mode))
            if m.custom_mode == want_custom_mode:
                return

    def mavproxy_do_set_mode_via_command_long(self, mavproxy, mode, timeout=30):
        """Set mode with a command long message with Mavproxy."""
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = self.get_mode_from_mode_mapping(mode)
        tstart = self.get_sim_time()
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise AutoTestTimeoutException("Failed to change mode")
            mavproxy.send("long DO_SET_MODE %u %u\n" %
                          (base_mode, custom_mode))
            m = self.wait_heartbeat()
            if m.custom_mode == custom_mode:
                return True

    def reach_heading_manual(self, heading, turn_right=True):
        """Manually direct the vehicle to the target heading."""
        if self.is_copter() or self.is_sub():
            self.set_rc(4, 1580)
            self.wait_heading(heading)
            self.set_rc(4, 1500)
        if self.is_plane():
            self.progress("NOT IMPLEMENTED")
        if self.is_rover():
            steering_pwm = 1700
            if not turn_right:
                steering_pwm = 1300
            self.set_rc(1, steering_pwm)
            self.set_rc(3, 1550)
            self.wait_heading(heading)
            self.set_rc(3, 1500)
            self.set_rc(1, 1500)

    def assert_vehicle_location_is_at_startup_location(self, dist_max=1):
        here = self.mav.location()
        start_loc = self.sitl_start_location()
        dist = self.get_distance(here, start_loc)
        data = "dist=%f max=%f (here: %s start-loc: %s)" % (dist, dist_max, here, start_loc)

        if dist > dist_max:
            raise NotAchievedException("Far from startup location: %s" % data)
        self.progress("Close to startup location: %s" % data)

    def assert_simstate_location_is_at_startup_location(self, dist_max=1):
        simstate_loc = self.sim_location()
        start_loc = self.sitl_start_location()
        dist = self.get_distance(simstate_loc, start_loc)
        data = "dist=%f max=%f (simstate: %s start-loc: %s)" % (dist, dist_max, simstate_loc, start_loc)

        if dist > dist_max:
            raise NotAchievedException("simstate far from startup location: %s" % data)
        self.progress("Simstate Close to startup location: %s" % data)

    def reach_distance_manual(self, distance):
        """Manually direct the vehicle to the target distance from home."""
        if self.is_copter():
            self.set_rc(2, 1350)
            self.wait_distance(distance, accuracy=5, timeout=60)
            self.set_rc(2, 1500)
        if self.is_plane():
            self.progress("NOT IMPLEMENTED")
        if self.is_rover():
            self.set_rc(3, 1700)
            self.wait_distance(distance, accuracy=2)
            self.set_rc(3, 1500)

    def guided_achieve_heading(self, heading, accuracy=None):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 200:
                raise NotAchievedException("Did not achieve heading")
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                heading,  # target angle
                10,  # degrees/second
                1,  # -1 is counter-clockwise, 1 clockwise
                0,  # 1 for relative, 0 for absolute
                0,  # p5
                0,  # p6
                0,  # p7
            )
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            self.progress("heading=%d want=%d" % (m.heading, int(heading)))
            if accuracy is not None:
                delta = abs(m.heading - int(heading))
                if delta <= accuracy:
                    return
            if m.heading == int(heading):
                return

    def do_set_relay(self, relay_num, on_off, timeout=10):
        """Set relay with a command long message."""
        self.progress("Set relay %d to %d" % (relay_num, on_off))
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                     relay_num,
                     on_off,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=timeout)

    def do_set_relay_mavproxy(self, relay_num, on_off):
        """Set relay with mavproxy."""
        self.progress("Set relay %d to %d" % (relay_num, on_off))
        self.mavproxy.send('module load relay\n')
        self.mavproxy.expect("Loaded module relay")
        self.mavproxy.send("relay set %d %d\n" % (relay_num, on_off))

    def do_fence_en_or_dis_able(self, value, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED):
        if value:
            p1 = 1
        else:
            p1 = 0
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
                     p1, # param1
                     0, # param2
                     0, # param3
                     0, # param4
                     0, # param5
                     0, # param6
                     0, # param7
                     want_result=want_result)

    def do_fence_enable(self, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED):
        self.do_fence_en_or_dis_able(True, want_result=want_result)

    def do_fence_disable(self, want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED):
        self.do_fence_en_or_dis_able(False, want_result=want_result)

    #################################################
    # WAIT UTILITIES
    #################################################
    def delay_sim_time(self, seconds_to_wait, reason=None):
        """Wait some second in SITL time."""
        self.drain_mav()
        tstart = self.get_sim_time()
        tnow = tstart
        r = "Delaying %f seconds"
        if reason is not None:
            r += " for %s" % reason
        self.progress(r % (seconds_to_wait,))
        while tstart + seconds_to_wait > tnow:
            tnow = self.get_sim_time()

    def get_altitude(self, relative=False, timeout=30):
        '''returns vehicles altitude in metres, possibly relative-to-home'''
        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                  blocking=True,
                                  timeout=timeout)
        if msg is None:
            raise MsgRcvTimeoutException("Failed to get Global Position")
        if relative:
            return msg.relative_alt / 1000.0  # mm -> m
        return msg.alt / 1000.0  # mm -> m

    def assert_rangefinder_distance_between(self, dist_min, dist_max):
        m = self.assert_receive_message('RANGEFINDER')

        if m.distance < dist_min:
            raise NotAchievedException("below min height (%f < %f)" %
                                       (m.distance, dist_min))

        if m.distance > dist_max:
            raise NotAchievedException("above max height (%f > %f)" %
                                       (m.distance, dist_max))

    def assert_distance_sensor_quality(self, quality):
        m = self.assert_receive_message('DISTANCE_SENSOR')

        if m.signal_quality != quality:
            raise NotAchievedException("Unexpected quality; want=%f got=%f" %
                                       (quality, m.signal_quality))

    def get_rangefinder_distance(self):
        m = self.assert_receive_message('RANGEFINDER', timeout=5)
        return m.distance

    def wait_rangefinder_distance(self, dist_min, dist_max, timeout=30, **kwargs):
        '''wait for RANGEFINDER distance'''
        def validator(value2, target2=None):
            if dist_min <= value2 <= dist_max:
                return True
            else:
                return False

        self.wait_and_maintain(
            value_name="RageFinderDistance",
            target=dist_min,
            current_value_getter=lambda: self.get_rangefinder_distance(),
            accuracy=(dist_max - dist_min),
            validator=lambda value2, target2: validator(value2, target2),
            timeout=timeout,
            **kwargs
        )

    def get_esc_rpm(self, esc):
        if esc > 4:
            raise ValueError("Only does 1-4")
        m = self.assert_receive_message('ESC_TELEMETRY_1_TO_4', timeout=1, verbose=True)
        return m.rpm[esc-1]

    def find_first_set_bit(self, mask):
        '''returns offset of first-set-bit (counting from right) in mask.  Returns None if no bits set'''
        pos = 0
        while mask != 0:
            if mask & 0x1:
                return pos
            mask = mask >> 1
            pos += 1
        return None

    def wait_esc_telem_rpm(self, esc, rpm_min, rpm_max, **kwargs):
        '''wait for ESC to be between rpm_min and rpm_max'''
        def validator(value2, target2=None):
            return rpm_min <= value2 <= rpm_max
        self.wait_and_maintain(
            value_name="ESC %u RPM" % esc,
            target=(rpm_min+rpm_max)/2.0,
            current_value_getter=lambda: self.get_esc_rpm(esc),
            accuracy=rpm_max-rpm_min,
            validator=lambda value2, target2: validator(value2, target2),
            **kwargs
        )

    def wait_altitude(self, altitude_min, altitude_max, relative=False, timeout=30, **kwargs):
        """Wait for a given altitude range."""
        assert altitude_min <= altitude_max, "Minimum altitude should be less than maximum altitude."

        def validator(value2, target2=None):
            if altitude_min <= value2 <= altitude_max:
                return True
            else:
                return False

        self.wait_and_maintain(
            value_name="Altitude",
            target=altitude_min,
            current_value_getter=lambda: self.get_altitude(
                relative=relative,
                timeout=timeout,
            ),
            accuracy=(altitude_max - altitude_min),
            validator=lambda value2, target2: validator(value2, target2),
            timeout=timeout,
            **kwargs
        )

    def wait_climbrate(self, speed_min, speed_max, timeout=30, **kwargs):
        """Wait for a given vertical rate."""
        assert speed_min <= speed_max, "Minimum speed should be less than maximum speed."

        def get_climbrate(timeout2):
            msg = self.mav.recv_match(type='VFR_HUD', blocking=True, timeout=timeout2)
            if msg:
                return msg.climb
            raise MsgRcvTimeoutException("Failed to get climb rate")

        def validator(value2, target2=None):
            if speed_min <= value2 <= speed_max:
                return True
            else:
                return False

        self.wait_and_maintain(
            value_name="Climbrate",
            target=speed_min,
            current_value_getter=lambda: get_climbrate(timeout),
            accuracy=(speed_max - speed_min),
            validator=lambda value2, target2: validator(value2, target2),
            timeout=timeout,
            **kwargs
        )

    def wait_groundspeed(self, speed_min, speed_max, timeout=30, **kwargs):
        """Wait for a given ground speed range."""
        assert speed_min <= speed_max, "Minimum speed should be less than maximum speed."

        def get_groundspeed(timeout2):
            msg = self.mav.recv_match(type='VFR_HUD', blocking=True, timeout=timeout2)
            if msg:
                return msg.groundspeed
            raise MsgRcvTimeoutException("Failed to get Groundspeed")

        def validator(value2, target2=None):
            if speed_min <= value2 <= speed_max:
                return True
            else:
                return False

        self.wait_and_maintain(
            value_name="Groundspeed",
            target=(speed_min+speed_max)/2,
            current_value_getter=lambda: get_groundspeed(timeout),
            accuracy=(speed_max - speed_min)/2,
            validator=lambda value2, target2: validator(value2, target2),
            timeout=timeout,
            **kwargs
        )

    def wait_roll(self, roll, accuracy, timeout=30, **kwargs):
        """Wait for a given roll in degrees."""
        def get_roll(timeout2):
            msg = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=timeout2)
            if msg:
                p = math.degrees(msg.pitch)
                r = math.degrees(msg.roll)
                self.progress("Roll %d Pitch %d" % (r, p))
                return r
            raise MsgRcvTimeoutException("Failed to get Roll")

        def validator(value2, target2):
            return math.fabs((value2 - target2 + 180) % 360 - 180) <= accuracy

        self.wait_and_maintain(
            value_name="Roll",
            target=roll,
            current_value_getter=lambda: get_roll(timeout),
            validator=lambda value2, target2: validator(value2, target2),
            accuracy=accuracy,
            timeout=timeout,
            **kwargs
        )

    def wait_pitch(self, pitch, accuracy, timeout=30, **kwargs):
        """Wait for a given pitch in degrees."""
        def get_pitch(timeout2):
            msg = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=timeout2)
            if msg:
                p = math.degrees(msg.pitch)
                r = math.degrees(msg.roll)
                self.progress("Pitch %d Roll %d" % (p, r))
                return p
            raise MsgRcvTimeoutException("Failed to get Pitch")

        def validator(value2, target2):
            return math.fabs((value2 - target2 + 180) % 360 - 180) <= accuracy

        self.wait_and_maintain(
            value_name="Pitch",
            target=pitch,
            current_value_getter=lambda: get_pitch(timeout),
            validator=lambda value2, target2: validator(value2, target2),
            accuracy=accuracy,
            timeout=timeout,
            **kwargs
        )

    def wait_and_maintain(self, value_name, target, current_value_getter, validator=None, accuracy=2.0, timeout=30, **kwargs):
        tstart = self.get_sim_time()
        achieving_duration_start = None
        if type(target) is Vector3:
            sum_of_achieved_values = Vector3()
            last_value = Vector3()
        else:
            sum_of_achieved_values = 0.0
            last_value = 0.0
        count_of_achieved_values = 0
        called_function = kwargs.get("called_function", None)
        minimum_duration = kwargs.get("minimum_duration", 0)
        if type(target) is Vector3:
            self.progress("Waiting for %s=(%s) with accuracy %.02f" % (value_name, str(target), accuracy))
        else:
            self.progress("Waiting for %s=%.02f with accuracy %.02f" % (value_name, target, accuracy))
        last_print_time = 0
        while self.get_sim_time_cached() < tstart + timeout:  # if we failed to received message with the getter the sim time isn't updated  # noqa
            last_value = current_value_getter()
            if called_function is not None:
                called_function(last_value, target)
            if self.get_sim_time_cached() - last_print_time > 1:
                if type(target) is Vector3:
                    self.progress("%s=(%s) (want (%s) +- %f)" %
                                  (value_name, str(last_value), str(target), accuracy))
                else:
                    self.progress("%s=%0.2f (want %f +- %f)" %
                                  (value_name, last_value, target, accuracy))
                last_print_time = self.get_sim_time_cached()
            if validator is not None:
                is_value_valid = validator(last_value, target)
            else:
                is_value_valid = math.fabs(last_value - target) <= accuracy
            if is_value_valid:
                sum_of_achieved_values += last_value
                count_of_achieved_values += 1.0
                if achieving_duration_start is None:
                    achieving_duration_start = self.get_sim_time_cached()
                if self.get_sim_time_cached() - achieving_duration_start >= minimum_duration:
                    if type(target) is Vector3:
                        self.progress("Attained %s=%s" % (
                            value_name,
                            str(sum_of_achieved_values * (1.0 / count_of_achieved_values))))
                    else:
                        self.progress("Attained %s=%f" % (value_name, sum_of_achieved_values / count_of_achieved_values))
                    return True
            else:
                achieving_duration_start = None
                if type(target) is Vector3:
                    sum_of_achieved_values.zero()
                else:
                    sum_of_achieved_values = 0.0
                count_of_achieved_values = 0
        raise AutoTestTimeoutException(
            "Failed to attain %s want %s, reached %s" %
            (value_name,
             str(target),
             str(sum_of_achieved_values / count_of_achieved_values) if count_of_achieved_values != 0 else str(last_value)))

    def wait_heading(self, heading, accuracy=5, timeout=30, **kwargs):
        """Wait for a given heading."""
        def get_heading_wrapped(timeout2):
            msg = self.mav.recv_match(type='VFR_HUD', blocking=True, timeout=timeout2)
            if msg:
                return msg.heading
            raise MsgRcvTimeoutException("Failed to get heading")

        def validator(value2, target2):
            return math.fabs((value2 - target2 + 180) % 360 - 180) <= accuracy

        self.wait_and_maintain(
            value_name="Heading",
            target=heading,
            current_value_getter=lambda: get_heading_wrapped(timeout),
            validator=lambda value2, target2: validator(value2, target2),
            accuracy=accuracy,
            timeout=timeout,
            **kwargs
        )

    def wait_yaw_speed(self, yaw_speed, accuracy=0.1, timeout=30, **kwargs):
        """Wait for a given yaw speed in radians per second."""
        def get_yawspeed(timeout2):
            msg = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=timeout2)
            if msg:
                return msg.yawspeed
            raise MsgRcvTimeoutException("Failed to get yaw speed")

        def validator(value2, target2):
            return math.fabs(value2 - target2) <= accuracy

        self.wait_and_maintain(
            value_name="YawSpeed",
            target=yaw_speed,
            current_value_getter=lambda: get_yawspeed(timeout),
            validator=lambda value2, target2: validator(value2, target2),
            accuracy=accuracy,
            timeout=timeout,
            **kwargs
        )

    def wait_speed_vector(self, speed_vector, accuracy=0.2, timeout=30, **kwargs):
        """Wait for a given speed vector."""
        def get_speed_vector(timeout2):
            msg = self.mav.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=timeout2)
            if msg:
                return Vector3(msg.vx, msg.vy, msg.vz)
            raise MsgRcvTimeoutException("Failed to get local speed vector")

        def validator(value2, target2):
            return (math.fabs(value2.x - target2.x) <= accuracy and
                    math.fabs(value2.y - target2.y) <= accuracy and
                    math.fabs(value2.z - target2.z) <= accuracy)

        self.wait_and_maintain(
            value_name="SpeedVector",
            target=speed_vector,
            current_value_getter=lambda: get_speed_vector(timeout),
            validator=lambda value2, target2: validator(value2, target2),
            accuracy=accuracy,
            timeout=timeout,
            **kwargs
        )

    def get_body_frame_velocity(self):
        gri = self.mav.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
        if gri is None:
            raise NotAchievedException("No GPS_RAW_INT")
        att = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if att is None:
            raise NotAchievedException("No ATTITUDE")
        return mavextra.gps_velocity_body(gri, att)

    def wait_speed_vector_bf(self, speed_vector, accuracy=0.2, timeout=30, **kwargs):
        """Wait for a given speed vector."""
        def get_speed_vector(timeout2):
            return self.get_body_frame_velocity()

        def validator(value2, target2):
            return (math.fabs(value2.x - target2.x) <= accuracy and
                    math.fabs(value2.y - target2.y) <= accuracy and
                    math.fabs(value2.z - target2.z) <= accuracy)

        self.wait_and_maintain(
            value_name="SpeedVectorBF",
            target=speed_vector,
            current_value_getter=lambda: get_speed_vector(timeout),
            validator=lambda value2, target2: validator(value2, target2),
            accuracy=accuracy,
            timeout=timeout,
            **kwargs
        )

    def wait_distance(self, distance, accuracy=2, timeout=30, **kwargs):
        """Wait for flight of a given distance."""
        start = self.mav.location()

        def get_distance():
            return self.get_distance(start, self.mav.location())

        def validator(value2, target2):
            return math.fabs(value2 - target2) <= accuracy

        self.wait_and_maintain(
            value_name="Distance",
            target=distance,
            current_value_getter=lambda: get_distance(),
            validator=lambda value2, target2: validator(value2, target2),
            accuracy=accuracy,
            timeout=timeout,
            **kwargs
        )

    def wait_distance_to_waypoint(self, wp_num, distance_min, distance_max, **kwargs):
        # TODO: use mission_request_partial_list_send
        wps = self.download_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        m = wps[wp_num]
        self.progress("m: %s" % str(m))
        loc = mavutil.location(m.x / 1.0e7, m.y / 1.0e7, 0, 0)
        self.progress("loc: %s" % str(loc))
        self.wait_distance_to_location(loc, distance_min, distance_max, **kwargs)

    def wait_distance_to_location(self, location, distance_min, distance_max, timeout=30, **kwargs):
        """Wait for flight of a given distance."""
        assert distance_min <= distance_max, "Distance min should be less than distance max."

        def get_distance():
            return self.get_distance(location, self.mav.location())

        def validator(value2, target2=None):
            return distance_min <= value2 <= distance_max

        self.wait_and_maintain(
            value_name="Distance",
            target=distance_min,
            current_value_getter=lambda: get_distance(),
            validator=lambda value2, target2: validator(value2, target2),
            accuracy=(distance_max - distance_min), timeout=timeout,
            **kwargs
        )

    def wait_distance_to_home(self, distance_min, distance_max, timeout=10, use_cached_home=True, **kwargs):
        """Wait for distance to home to be within specified bounds."""
        assert distance_min <= distance_max, "Distance min should be less than distance max."

        def get_distance():
            return self.distance_to_home(use_cached_home)

        def validator(value2, target2=None):
            return distance_min <= value2 <= distance_max

        self.wait_and_maintain(
            value_name="Distance to home",
            target=distance_min,
            current_value_getter=lambda: get_distance(),
            validator=lambda value2, target2: validator(value2, target2),
            accuracy=(distance_max - distance_min), timeout=timeout,
            **kwargs
        )

    def wait_distance_to_nav_target(self,
                                    distance_min,
                                    distance_max,
                                    timeout=10,
                                    use_cached_nav_controller_output=False,
                                    **kwargs):
        """Wait for distance to home to be within specified bounds."""
        assert distance_min <= distance_max, "Distance min should be less than distance max."

        def get_distance():
            return self.distance_to_nav_target(use_cached_nav_controller_output)

        def validator(value2, target2=None):
            return distance_min <= value2 <= distance_max

        self.wait_and_maintain(
            value_name="Distance to nav target",
            target=distance_min,
            current_value_getter=lambda: get_distance(),
            validator=lambda value2,
            target2: validator(value2, target2),
            accuracy=(distance_max - distance_min),
            timeout=timeout,
            **kwargs
        )

    def distance_to_local_position(self, local_pos, timeout=30):
        (x, y, z_down) = local_pos  # alt is *up*

        pos = self.mav.recv_match(
            type='LOCAL_POSITION_NED',
            blocking=True
        )

        delta_x = pos.x - x
        delta_y = pos.y - y
        delta_z = pos.z - z_down
        return math.sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z)

    def wait_distance_to_local_position(self,
                                        local_position,  # (x, y, z_down)
                                        distance_min,
                                        distance_max,
                                        timeout=10,
                                        **kwargs):
        """Wait for distance to home to be within specified bounds."""
        assert distance_min <= distance_max, "Distance min should be less than distance max."

        def get_distance():
            return self.distance_to_local_position(local_position)

        def validator(value2, target2=None):
            return distance_min <= value2 <= distance_max

        (x, y, z_down) = local_position
        self.wait_and_maintain(
            value_name="Distance to (%f,%f,%f)" % (x, y, z_down),
            target=distance_min,
            current_value_getter=lambda: get_distance(),
            validator=lambda value2,
            target2: validator(value2, target2),
            accuracy=(distance_max - distance_min),
            timeout=timeout,
            **kwargs
        )

    def wait_parameter_value(self, parameter, value, timeout=10):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("%s never got value %f" %
                                           (parameter, value))
            v = self.get_parameter(parameter, verbose=False)
            self.progress("Got parameter value (%s=%f)" %
                          (parameter, v))
            if v == value:
                return
            self.delay_sim_time(0.1)

    def get_servo_channel_value(self, channel, timeout=2):
        channel_field = "servo%u_raw" % channel
        tstart = self.get_sim_time()
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise NotAchievedException("Channel value condition not met")
            m = self.mav.recv_match(type='SERVO_OUTPUT_RAW',
                                    blocking=True,
                                    timeout=remaining)
            if m is None:
                continue
            m_value = getattr(m, channel_field, None)
            if m_value is None:
                raise ValueError("message (%s) has no field %s" %
                                 (str(m), channel_field))
            return m_value

    def wait_servo_channel_value(self, channel, value, timeout=2, comparator=operator.eq):
        """wait for channel value comparison (default condition is equality)"""
        channel_field = "servo%u_raw" % channel
        opstring = ("%s" % comparator)[-3:-1]
        tstart = self.get_sim_time()
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise NotAchievedException("Channel value condition not met")
            m = self.mav.recv_match(type='SERVO_OUTPUT_RAW',
                                    blocking=True,
                                    timeout=remaining)
            if m is None:
                continue
            m_value = getattr(m, channel_field, None)
            if m_value is None:
                raise ValueError("message (%s) has no field %s" %
                                 (str(m), channel_field))
            self.progress("want SERVO_OUTPUT_RAW.%s=%u %s %u" %
                          (channel_field, m_value, opstring, value))
            if comparator(m_value, value):
                return m_value

    def assert_servo_channel_value(self, channel, value, comparator=operator.eq):
        """assert channel value (default condition is equality)"""
        channel_field = "servo%u_raw" % channel
        opstring = ("%s" % comparator)[-3:-1]
        m = self.assert_receive_message('SERVO_OUTPUT_RAW', timeout=1)
        m_value = getattr(m, channel_field, None)
        if m_value is None:
            raise ValueError("message (%s) has no field %s" %
                             (str(m), channel_field))
        self.progress("assert SERVO_OUTPUT_RAW.%s=%u %s %u" %
                      (channel_field, m_value, opstring, value))
        if comparator(m_value, value):
            return m_value
        raise NotAchievedException("Wrong value")

    def get_rc_channel_value(self, channel, timeout=2):
        """wait for channel to hit value"""
        channel_field = "chan%u_raw" % channel
        tstart = self.get_sim_time()
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise NotAchievedException("Channel never achieved value")
            m = self.mav.recv_match(type='RC_CHANNELS',
                                    blocking=True,
                                    timeout=remaining)
            if m is None:
                continue
            m_value = getattr(m, channel_field)
            if m_value is None:
                raise ValueError("message (%s) has no field %s" %
                                 (str(m), channel_field))
            return m_value

    def wait_rc_channel_value(self, channel, value, timeout=2):
        channel_field = "chan%u_raw" % channel
        tstart = self.get_sim_time()
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise NotAchievedException("Channel never achieved value")
            m_value = self.get_rc_channel_value(channel, timeout=timeout)
            self.progress("RC_CHANNELS.%s=%u want=%u" %
                          (channel_field, m_value, value))
            if value == m_value:
                return

    def assert_rc_channel_value(self, channel, value):
        channel_field = "chan%u_raw" % channel

        m_value = self.get_rc_channel_value(channel, timeout=1)
        self.progress("RC_CHANNELS.%s=%u want=%u" %
                      (channel_field, m_value, value))
        if value != m_value:
            raise NotAchievedException("Expected %s to be %u got %u" %
                                       (channel, value, m_value))

    def wait_location(self,
                      loc,
                      accuracy=5.0,
                      timeout=30,
                      target_altitude=None,
                      height_accuracy=-1,
                      **kwargs):
        """Wait for arrival at a location."""
        def get_distance_to_loc():
            return self.get_distance(self.mav.location(), loc)

        def validator(value2, empty=None):
            if value2 <= accuracy:
                if target_altitude is not None:
                    height_delta = math.fabs(self.mav.location().alt - target_altitude)
                    if height_accuracy != -1 and height_delta > height_accuracy:
                        return False
                return True
            else:
                return False
        debug_text = "Distance to Location (%.4f, %.4f) " % (loc.lat, loc.lng)
        if target_altitude is not None:
            debug_text += ",at altitude %.1f height_accuracy=%.1f, d" % (target_altitude, height_accuracy)
        self.wait_and_maintain(
            value_name=debug_text,
            target=0,
            current_value_getter=lambda: get_distance_to_loc(),
            accuracy=accuracy,
            validator=lambda value2, target2: validator(value2, None),
            timeout=timeout,
            **kwargs
        )

    def wait_current_waypoint(self, wpnum, timeout=60):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > timeout:
                raise AutoTestTimeoutException("Did not get wanted current waypoint")
            seq = self.mav.waypoint_current()
            wp_dist = None
            try:
                wp_dist = self.mav.messages['NAV_CONTROLLER_OUTPUT'].wp_dist
            except (KeyError, AttributeError):
                pass
            self.progress("Waiting for wp=%u current=%u dist=%sm" % (wpnum, seq, wp_dist))
            if seq == wpnum:
                break

    def wait_waypoint(self,
                      wpnum_start,
                      wpnum_end,
                      allow_skip=True,
                      max_dist=2,
                      timeout=400):
        """Wait for waypoint ranges."""
        tstart = self.get_sim_time()
        # this message arrives after we set the current WP
        start_wp = self.mav.waypoint_current()
        current_wp = start_wp
        mode = self.mav.flightmode

        self.progress("wait for waypoint ranges start=%u end=%u"
                      % (wpnum_start, wpnum_end))
        # if start_wp != wpnum_start:
        #    raise WaitWaypointTimeout("test: Expected start waypoint %u "
        #                              "but got %u" %
        #                  (wpnum_start, start_wp))

        last_wp_msg = 0
        while self.get_sim_time_cached() < tstart + timeout:
            seq = self.mav.waypoint_current()
            m = self.assert_receive_message('NAV_CONTROLLER_OUTPUT')
            wp_dist = m.wp_dist
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)

            # if we changed mode, fail
            if self.mav.flightmode != mode:
                raise WaitWaypointTimeout('Exited %s mode' % mode)

            if self.get_sim_time_cached() - last_wp_msg > 1:
                self.progress("WP %u (wp_dist=%u Alt=%.02f), current_wp: %u,"
                              "wpnum_end: %u" %
                              (seq, wp_dist, m.alt, current_wp, wpnum_end))
                last_wp_msg = self.get_sim_time_cached()
            if seq == current_wp+1 or (seq > current_wp+1 and allow_skip):
                self.progress("test: Starting new waypoint %u" % seq)
                tstart = self.get_sim_time()
                current_wp = seq
                # the wp_dist check is a hack until we can sort out
                # the right seqnum for end of mission
            # if current_wp == wpnum_end or (current_wp == wpnum_end-1 and
            #                                wp_dist < 2):
            if current_wp == wpnum_end and wp_dist < max_dist:
                self.progress("Reached final waypoint %u" % seq)
                return True
            if seq >= 255:
                self.progress("Reached final waypoint %u" % seq)
                return True
            if seq > current_wp+1:
                raise WaitWaypointTimeout(("Skipped waypoint! Got wp %u expected %u"
                                           % (seq, current_wp+1)))
        raise WaitWaypointTimeout("Timed out waiting for waypoint %u of %u" %
                                  (wpnum_end, wpnum_end))

    def mode_is(self, mode, cached=False, drain_mav=True):
        if not cached:
            self.wait_heartbeat(drain_mav=drain_mav)
        try:
            return self.get_mode_from_mode_mapping(self.mav.flightmode) == self.get_mode_from_mode_mapping(mode)
        except Exception:
            pass
        # assume this is a number....
        return self.mav.messages['HEARTBEAT'].custom_mode == mode

    def wait_mode(self, mode, timeout=60):
        """Wait for mode to change."""
        self.progress("Waiting for mode %s" % mode)
        tstart = self.get_sim_time()
        while not self.mode_is(mode, drain_mav=False):
            custom_num = self.mav.messages['HEARTBEAT'].custom_mode
            self.progress("mav.flightmode=%s Want=%s custom=%u" % (
                self.mav.flightmode, mode, custom_num))
            if (timeout is not None and
                    self.get_sim_time_cached() > tstart + timeout):
                raise WaitModeTimeout("Did not change mode")
        self.progress("Got mode %s" % mode)

    def wait_gps_sys_status_not_present_or_enabled_and_healthy(self, timeout=30):
        self.progress("Waiting for GPS health")
        tstart = self.get_sim_time_cached()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise AutoTestTimeoutException("GPS status bits did not become good")
            m = self.mav.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
            if m is None:
                continue
            if (not (m.onboard_control_sensors_present & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)):
                self.progress("GPS not present")
                if now > 20:
                    # it's had long enough to be detected....
                    return
                continue
            if (not (m.onboard_control_sensors_enabled & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)):
                self.progress("GPS not enabled")
                continue
            if (not (m.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)):
                self.progress("GPS not healthy")
                continue
            self.progress("GPS healthy")
            return

    def assert_sensor_state(self, sensor, present=True, enabled=True, healthy=True, verbose=False):
        return self.sensor_has_state(sensor, present, enabled, healthy, do_assert=True, verbose=verbose)

    def sensor_has_state(self, sensor, present=True, enabled=True, healthy=True, do_assert=False, verbose=False):
        m = self.assert_receive_message('SYS_STATUS', timeout=5, very_verbose=verbose)
        reported_present = m.onboard_control_sensors_present & sensor
        reported_enabled = m.onboard_control_sensors_enabled & sensor
        reported_healthy = m.onboard_control_sensors_health & sensor
        if present:
            if not reported_present:
                if do_assert:
                    raise NotAchievedException("Sensor not present")
                return False
        else:
            if reported_present:
                if do_assert:
                    raise NotAchievedException("Sensor present when it shouldn't be")
                return False

        if enabled:
            if not reported_enabled:
                if do_assert:
                    raise NotAchievedException("Sensor not enabled")
                return False
        else:
            if reported_enabled:
                if do_assert:
                    raise NotAchievedException("Sensor enabled when it shouldn't be")
                return False

        if healthy:
            if not reported_healthy:
                if do_assert:
                    raise NotAchievedException("Sensor not healthy")
                return False
        else:
            if reported_healthy:
                if do_assert:
                    raise NotAchievedException("Sensor healthy when it shouldn't be")
                return False
        return True

    def wait_sensor_state(self, sensor, present=True, enabled=True, healthy=True, timeout=5, verbose=False):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Sensor did not achieve state")
            if self.sensor_has_state(sensor, present=present, enabled=enabled, healthy=healthy, verbose=verbose):
                break

    def wait_prearm_sys_status_healthy(self, timeout=60):
        self.do_timesync_roundtrip()
        tstart = self.get_sim_time()
        while True:
            t2 = self.get_sim_time_cached()
            if t2 - tstart > timeout:
                self.progress("Prearm bit never went true.  Attempting arm to elicit reason from autopilot")
                self.arm_vehicle()
                raise AutoTestTimeoutException("Prearm bit never went true")
            if self.sensor_has_state(mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK, True, True, True):
                break

    def assert_fence_enabled(self, timeout=2):
        # Check fence is enabled
        m = self.mav.recv_match(type='FENCE_STATUS', blocking=True, timeout=timeout)
        self.progress("Got (%s)" % str(m))
        if m is None:
            raise NotAchievedException("Fence status was not received")

    def assert_fence_disabled(self, timeout=2):
        # Check fence is not enabled
        m = self.mav.recv_match(type='FENCE_STATUS', blocking=True, timeout=timeout)
        self.progress("Got (%s)" % str(m))
        if m is not None:
            raise NotAchievedException("Fence status received unexpectedly")

    def assert_prearm_failure(self, expected_statustext, timeout=5, ignore_prearm_failures=[]):
        seen_statustext = False
        seen_command_ack = False

        self.drain_mav()
        tstart = self.get_sim_time_cached()
        arm_last_send = 0
        while True:
            if seen_command_ack and seen_statustext:
                break
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise NotAchievedException(
                    "Did not see failure-to-arm messages (statustext=%s command_ack=%s" %
                    (seen_statustext, seen_command_ack))
            if now - arm_last_send > 1:
                arm_last_send = now
                self.send_mavlink_arm_command()
            m = self.mav.recv_match(blocking=True, timeout=1)
            if m is None:
                continue
            if m.get_type() == "STATUSTEXT":
                if expected_statustext in m.text:
                    self.progress("Got: %s" % str(m))
                    seen_statustext = True
                elif "PreArm" in m.text and m.text[8:] not in ignore_prearm_failures:
                    self.progress("Got: %s" % str(m))
                    raise NotAchievedException("Unexpected prearm failure (%s)" % m.text)

            if m.get_type() == "COMMAND_ACK":
                print("Got: %s" % str(m))
                if m.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if m.result != 4:
                        raise NotAchievedException("command-ack says we didn't fail to arm")
                    self.progress("Got: %s" % str(m))
                    seen_command_ack = True
            if self.mav.motors_armed():
                raise NotAchievedException("Armed when we shouldn't have")

    def wait_ready_to_arm(self, timeout=120, require_absolute=True, check_prearm_bit=True):
        # wait for EKF checks to pass
        self.progress("Waiting for ready to arm")
        start = self.get_sim_time()
        self.wait_ekf_happy(timeout=timeout, require_absolute=require_absolute)
        if require_absolute:
            self.wait_gps_sys_status_not_present_or_enabled_and_healthy()
        armable_time = self.get_sim_time() - start
        if require_absolute:
            self.poll_home_position()
        if check_prearm_bit:
            self.wait_prearm_sys_status_healthy(timeout=timeout)
        self.progress("Took %u seconds to become armable" % armable_time)
        self.total_waiting_to_arm_time += armable_time
        self.waiting_to_arm_count += 1

    def wait_heartbeat(self, drain_mav=True, quiet=False, *args, **x):
        '''as opposed to mav.wait_heartbeat, raises an exception on timeout.
Also, ignores heartbeats not from our target system'''
        if drain_mav:
            self.drain_mav(quiet=quiet)
        orig_timeout = x.get("timeout", 20)
        x["timeout"] = 1
        tstart = time.time()
        while True:
            if time.time() - tstart > orig_timeout and not self.gdb:
                if not self.sitl_is_running():
                    self.progress("SITL is not running")
                raise AutoTestTimeoutException("Did not receive heartbeat")
            m = self.mav.wait_heartbeat(*args, **x)
            if m is None:
                continue
            if m.get_srcSystem() == self.sysid_thismav():
                return m

    def wait_ekf_happy(self, timeout=45, require_absolute=True):
        """Wait for EKF to be happy"""

        """ if using SITL estimates directly """
        if (int(self.get_parameter('AHRS_EKF_TYPE')) == 10):
            return True

        # all of these must be set for arming to happen:
        required_value = (mavutil.mavlink.EKF_ATTITUDE |
                          mavutil.mavlink.ESTIMATOR_VELOCITY_HORIZ |
                          mavutil.mavlink.ESTIMATOR_VELOCITY_VERT |
                          mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL |
                          mavutil.mavlink.ESTIMATOR_PRED_POS_HORIZ_REL)
        # none of these bits must be set for arming to happen:
        error_bits = (mavutil.mavlink.ESTIMATOR_CONST_POS_MODE |
                      mavutil.mavlink.ESTIMATOR_ACCEL_ERROR)
        if require_absolute:
            required_value |= (mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS |
                               mavutil.mavlink.ESTIMATOR_POS_VERT_ABS |
                               mavutil.mavlink.ESTIMATOR_PRED_POS_HORIZ_ABS)
            error_bits |= mavutil.mavlink.ESTIMATOR_GPS_GLITCH
        self.wait_ekf_flags(required_value, error_bits, timeout=timeout)

    def wait_ekf_flags(self, required_value, error_bits, timeout=30):
        self.progress("Waiting for EKF value %u" % required_value)
        self.drain_mav()
        last_print_time = 0
        tstart = self.get_sim_time()
        while timeout is None or self.get_sim_time_cached() < tstart + timeout:
            m = self.mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=timeout)
            if m is None:
                continue
            current = m.flags
            errors = current & error_bits
            everything_ok = (errors == 0 and
                             current & required_value == required_value)
            if everything_ok or self.get_sim_time_cached() - last_print_time > 1:
                self.progress("Wait EKF.flags: required:%u current:%u errors=%u" %
                              (required_value, current, errors))
                last_print_time = self.get_sim_time_cached()
            if everything_ok:
                self.progress("EKF Flags OK")
                return True
        raise AutoTestTimeoutException("Failed to get EKF.flags=%u" %
                                       required_value)

    def wait_gps_disable(self, position_horizontal=True, position_vertical=False, timeout=30):
        """Disable GPS and wait for EKF to report the end of assistance from GPS."""
        self.set_parameter("SIM_GPS_DISABLE", 1)
        tstart = self.get_sim_time()

        """ if using SITL estimates directly """
        if (int(self.get_parameter('AHRS_EKF_TYPE')) == 10):
            self.progress("GPS disable skipped")
            return

        # all of these must NOT be set for arming NOT to happen:
        not_required_value = 0
        if position_horizontal:
            not_required_value |= mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL
        if position_vertical:
            not_required_value |= mavutil.mavlink.ESTIMATOR_POS_VERT_AGL
        self.progress("Waiting for EKF not having bits %u" % not_required_value)
        last_print_time = 0
        while timeout is None or self.get_sim_time_cached() < tstart + timeout:
            m = self.mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=timeout)
            if m is None:
                continue
            current = m.flags
            if self.get_sim_time_cached() - last_print_time > 1:
                self.progress("Wait EKF.flags: not required:%u current:%u" %
                              (not_required_value, current))
                last_print_time = self.get_sim_time_cached()
            if current & not_required_value != not_required_value:
                self.progress("GPS disable OK")
                return
        raise AutoTestTimeoutException("Failed to get EKF.flags=%u disabled" % not_required_value)

    def wait_text(self, *args, **kwargs):
        '''wait for text to appear from vehicle, return that text'''
        statustext = self.wait_statustext(*args, **kwargs)
        if statustext is None:
            return None
        return statustext.text

    def statustext_in_collections(self, text, regex=False):
        '''searches for text in STATUSTEXT collection, returns message if found'''
        c = self.context_get()
        if "STATUSTEXT" not in c.collections:
            raise NotAchievedException("Asked to check context but it isn't collecting!")
        for x in c.collections["STATUSTEXT"]:
            self.progress("  statustext=%s vs text=%s" % (x.text, text))
            if regex:
                if re.match(text, x.text):
                    return x
            elif text.lower() in x.text.lower():
                return x
        return None

    def wait_statustext(self, text, timeout=20, the_function=None, check_context=False, regex=False, wallclock_timeout=False):
        """Wait for a specific STATUSTEXT, return that statustext message"""

        # Statustexts are often triggered by something we've just
        # done, so we have to be careful not to read any traffic that
        # isn't checked for being our statustext.  That doesn't work
        # well with getting the curent simulation time (which requires
        # a new SYSTEM_TIME message), so we install a message hook
        # which checks all incoming messages.
        self.progress("Waiting for text : %s" % text.lower())
        if check_context:
            statustext = self.statustext_in_collections(text, regex=regex)
            if statustext:
                self.progress("Found expected text in collection: %s" % text.lower())
                return statustext

        global statustext_found
        global statustext_full
        statustext_full = None
        statustext_found = False

        def mh(mav, m):
            global statustext_found
            global statustext_full
            if m.get_type() != "STATUSTEXT":
                return
            if regex:
                self.re_match = re.match(text, m.text)
                if self.re_match:
                    statustext_found = True
                    statustext_full = m
            if text.lower() in m.text.lower():
                self.progress("Received expected text: %s" % m.text.lower())
                statustext_found = True
                statustext_full = m

        self.install_message_hook(mh)
        if wallclock_timeout:
            tstart = time.time()
        else:
            tstart = self.get_sim_time()
        try:
            while not statustext_found:
                if wallclock_timeout:
                    now = time.time()
                else:
                    now = self.get_sim_time_cached()
                if now - tstart > timeout:
                    raise AutoTestTimeoutException("Failed to receive text: %s" %
                                                   text.lower())
                if the_function is not None:
                    the_function()
                self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=0.1)
        finally:
            self.remove_message_hook(mh)
        return statustext_full

    # routines helpful for testing LUA scripting:
    def script_example_source_path(self, scriptname):
        return os.path.join(self.rootdir(), "libraries", "AP_Scripting", "examples", scriptname)

    def script_test_source_path(self, scriptname):
        return os.path.join(self.rootdir(), "libraries", "AP_Scripting", "tests", scriptname)

    def installed_script_path(self, scriptname):
        return os.path.join("scripts", scriptname)

    def install_script(self, source, scriptname):
        dest = self.installed_script_path(scriptname)
        destdir = os.path.dirname(dest)
        if not os.path.exists(destdir):
            os.mkdir(destdir)
        self.progress("Copying (%s) to (%s)" % (source, dest))
        shutil.copy(source, dest)

    def install_example_script(self, scriptname):
        source = self.script_example_source_path(scriptname)
        self.install_script(source, scriptname)

    def install_test_script(self, scriptname):
        source = self.script_test_source_path(scriptname)
        self.install_script(source, scriptname)

    def remove_example_script(self, scriptname):
        dest = self.installed_script_path(scriptname)
        try:
            os.unlink(dest)
        except IOError:
            pass
        except OSError:
            pass

    def get_mavlink_connection_going(self):
        # get a mavlink connection going
        try:
            retries = 20
            if self.gdb:
                retries = 20000
            self.mav = mavutil.mavlink_connection(
                self.autotest_connection_string_to_ardupilot(),
                retries=retries,
                robust_parsing=True,
                source_system=250,
                source_component=250,
                autoreconnect=True,
                dialect="all",  # if we don't pass this in we end up with the wrong mavlink version...
            )
        except Exception as msg:
            self.progress("Failed to start mavlink connection on %s: %s" %
                          (self.autotest_connection_string_to_ardupilot(), msg,))
            raise
        self.mav.message_hooks.append(self.message_hook)
        self.mav.mav.set_send_callback(self.send_message_hook, self)
        self.mav.idle_hooks.append(self.idle_hook)

        self.set_streamrate(self.sitl_streamrate())

    def show_test_timings_key_sorter(self, t):
        (k, v) = t
        return ((v, k))

    def show_test_timings(self):
        if len(self.test_timings.keys()) == 0:
            return
        longest = 0
        for desc in self.test_timings.keys():
            if len(desc) > longest:
                longest = len(desc)
        tests_total_time = 0
        for desc, test_time in sorted(self.test_timings.items(),
                                      key=self.show_test_timings_key_sorter):
            fmt = "%" + str(longest) + "s: %.2fs"
            tests_total_time += test_time
            self.progress(fmt % (desc, test_time))
        self.progress(fmt % ("**--tests_total_time--**", tests_total_time))
        self.progress("mavproxy_start was called %u times" %
                      (self.start_mavproxy_count,))

    def send_statustext(self, text):
        if sys.version_info.major >= 3 and not isinstance(text, bytes):
            text = bytes(text, "ascii")
        elif 'unicode' in str(type(text)):
            text = text.encode('ascii')
        self.mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_WARNING, text)

    def get_stacktrace(self):
        return ''.join(traceback.format_stack())

    def get_exception_stacktrace(self, e):
        if sys.version_info[0] >= 3:
            ret = "%s\n" % e
            ret += ''.join(traceback.format_exception(type(e),
                                                      e,
                                                      tb=e.__traceback__))
            return ret

        # Python2:
        return traceback.format_exc(e)

    def bin_logs(self):
        return glob.glob("logs/*.BIN")

    def remove_bin_logs(self):
        util.run_cmd('/bin/rm -f logs/*.BIN logs/LASTLOG.TXT')

    def check_logs(self, name):
        '''called to move relevant log files from our working directory to the
        buildlogs directory'''
        to_dir = self.logs_dir
        # move telemetry log files
        for log in glob.glob("autotest-*.tlog"):
            bname = os.path.basename(log)
            newname = os.path.join(to_dir, "%s-%s-%s" % (self.log_name(), name, bname))
            print("Renaming %s to %s" % (log, newname))
            shutil.move(log, newname)
        # move binary log files
        for log in sorted(self.bin_logs()):
            bname = os.path.basename(log)
            newname = os.path.join(to_dir, "%s-%s-%s" % (self.log_name(), name, bname))
            print("Renaming %s to %s" % (log, newname))
            shutil.move(log, newname)
        # move core files
        save_binaries = False
        corefiles = []
        corefiles.extend(glob.glob("core*"))
        corefiles.extend(glob.glob("ap-*.core"))
        for log in sorted(corefiles):
            bname = os.path.basename(log)
            newname = os.path.join(to_dir, "%s-%s-%s" % (bname, self.log_name(), name))
            print("Renaming %s to %s" % (log, newname))
            shutil.move(log, newname)
            save_binaries = True
        if save_binaries:
            util.run_cmd('/bin/cp build/sitl/bin/* %s' % to_dir,
                         directory=util.reltopdir('.'))

    def run_one_test(self, test, interact=False):
        '''new-style run-one-test used by run_tests'''
        for i in range(0, test.attempts-1):
            if self.run_one_test_attempt(test, interact=interact, attempt=i+2, do_fail_list=False):
                return
            self.progress("Run attempt failed.  Retrying")
        self.run_one_test_attempt(test, interact=interact, attempt=1)

    def print_exception_caught(self, e, send_statustext=True):
        self.progress("Exception caught: %s" %
                      self.get_exception_stacktrace(e))
        path = None
        try:
            path = self.current_onboard_log_filepath()
        except IndexError:
            pass
        self.progress("Most recent logfile: %s" % (path, ), send_statustext=send_statustext)

    def progress_file_content(self, filepath):
        with open(filepath) as f:
            for line in f:
                self.progress(line.rstrip())

    def run_one_test_attempt(self, test, interact=False, attempt=1, do_fail_list=True):
        '''called by run_one_test to actually run the test in a retry loop'''
        name = test.name
        desc = test.description
        test_function = test.function
        if attempt != 1:
            self.progress("RETRYING %s" % name)
            test_output_filename = self.buildlogs_path("%s-%s-retry-%u.txt" %
                                                       (self.log_name(), name, attempt-1))
        else:
            test_output_filename = self.buildlogs_path("%s-%s.txt" %
                                                       (self.log_name(), name))

        tee = TeeBoth(test_output_filename, 'w', self.mavproxy_logfile)

        start_num_message_hooks = len(self.mav.message_hooks)

        prettyname = "%s (%s)" % (name, desc)
        self.start_test(prettyname)
        self.set_current_test_name(name)
        old_contexts_length = len(self.contexts)
        self.context_push()

        start_time = time.time()

        ex = None
        try:
            self.check_rc_defaults()
            self.change_mode(self.default_mode())
            self.drain_mav()
            self.drain_all_pexpects()

            test_function()
        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.test_timings[desc] = time.time() - start_time
        reset_needed = self.contexts[-1].sitl_commandline_customised

        passed = True
        if ex is not None:
            passed = False

        try:
            self.context_pop()
        except Exception as e:
            self.print_exception_caught(e, send_statustext=False)
            passed = False

        ardupilot_alive = False
        try:
            self.wait_heartbeat()
            ardupilot_alive = True
        except Exception:
            # process is dead
            self.progress("No heartbeat after test", send_statustext=False)
            if self.sitl.isalive():
                self.progress("pexpect says it is alive")
                for tool in "dumpstack.sh", "dumpcore.sh":
                    tool_filepath = os.path.join(self.rootdir(), 'Tools', 'scripts', tool)
                    if util.run_cmd([tool_filepath, str(self.sitl.pid)]) != 0:
                        self.progress("Failed %s" % (tool,))
                        return False
            else:
                self.progress("pexpect says it is dead")

            # try dumping the process status file for more information:
            status_filepath = "/proc/%u/status" % self.sitl.pid
            self.progress("Checking for status filepath (%s)" % status_filepath)
            if os.path.exists(status_filepath):
                self.progress_file_content(status_filepath)
            else:
                self.progress("... does not exist")

            passed = False
            reset_needed = True

        if ardupilot_alive and self.armed() and not self.is_tracker():
            if ex is None:
                ex = ArmedAtEndOfTestException("Still armed at end of test")
            self.progress("Armed at end of test; force-rebooting SITL")
            self.disarm_vehicle(force=True)
            self.forced_post_test_sitl_reboots += 1
            if reset_needed:
                self.progress("Force-resetting SITL")
                self.reset_SITL_commandline()
            else:
                self.progress("Force-rebooting SITL")
                self.reboot_sitl() # that'll learn it
            passed = False

        if self._mavproxy is not None:
            self.progress("Stopping auto-started mavproxy")
            if self.use_map:
                self.mavproxy.send("module unload map\n")
                self.mavproxy.expect("Unloaded module map")

            self.expect_list_remove(self._mavproxy)
            util.pexpect_close(self._mavproxy)
            self._mavproxy = None

        corefiles = []
        corefiles.extend(glob.glob("core*"))
        corefiles.extend(glob.glob("ap-*.core"))
        if corefiles:
            self.progress('Corefiles detected: %s' % str(corefiles))
            passed = False

        if len(self.contexts) != old_contexts_length:
            self.progress("context count mismatch (want=%u got=%u)" %
                          (old_contexts_length, len(self.contexts)))
            passed = False

        # make sure we don't leave around stray listeners:
        if len(self.mav.message_hooks) != start_num_message_hooks:
            self.progress("Stray message listeners: %s" %
                          str(self.mav.message_hooks))
            passed = False

        if passed:
#            self.remove_bin_logs() # can't do this as one of the binlogs is probably open for writing by the SITL process.  If we force a rotate before running tests then we can do this.  # noqa
            pass
        else:
            if self.logs_dir is not None:
                # stash the binary logs and corefiles away for later analysis
                self.check_logs(name)

        if passed:
            self.progress('PASSED: "%s"' % prettyname)
        else:
            self.progress('FAILED: "%s": %s (see %s)' %
                          (prettyname, repr(ex), test_output_filename))
            if do_fail_list:
                self.fail_list.append((prettyname, ex, test_output_filename))
            if interact:
                self.progress("Starting MAVProxy interaction as directed")
                self.mavproxy.interact()

        if self.reset_after_every_test:
            reset_needed = True

        if reset_needed:
            self.reset_SITL_commandline()

        if not self.is_tracker(): # FIXME - more to the point, fix Tracker's mission handling
            self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_ALL)

        tee.close()

        return passed

    def check_test_syntax(self, test_file):
        """Check mistake on autotest function syntax."""
        self.start_test("Check for syntax mistake in autotest lambda")
        if not os.path.isfile(test_file):
            self.progress("File %s does not exist" % test_file)
        test_file = test_file.rstrip('c')
        try:
            with open(test_file) as f:
                # check for lambda: test_function without paranthesis
                faulty_strings = re.findall(r"lambda\s*:\s*\w+.\w+\s*\)", f.read())
                if faulty_strings:
                    desc = "Syntax error in autotest lambda at : \n"
                    for x in range(len(faulty_strings)):
                        desc += faulty_strings[x] + "\n"
                    raise ErrorException(desc)
        except ErrorException as msg:
            self.progress("FAILED: Check for syntax mistake in autotest lambda. \n" + str(msg))
            exit(1)
        self.progress("PASSED: Check for syntax mistake in autotest lambda")

    def defaults_filepath(self):
        return None

    def start_mavproxy(self):
        self.start_mavproxy_count += 1
        if self.mavproxy is not None:
            return self.mavproxy
        self.progress("Starting MAVProxy")

        # determine a good pexpect timeout for reading MAVProxy's
        # output; some regmes may require longer timeouts.
        pexpect_timeout = 60
        if self.valgrind or self.callgrind:
            pexpect_timeout *= 10

        mavproxy = util.start_MAVProxy_SITL(
            self.vehicleinfo_key(),
            logfile=self.mavproxy_logfile,
            options=self.mavproxy_options(),
            pexpect_timeout=pexpect_timeout)
        mavproxy.expect(r'Telemetry log: (\S+)\r\n')
        self.logfile = mavproxy.match.group(1)
        self.progress("LOGFILE %s" % self.logfile)
        self.try_symlink_tlog()

        self.expect_list_add(mavproxy)
        util.expect_setup_callback(mavproxy, self.expect_callback)
        self._mavproxy = mavproxy  # so we can clean up after tests....
        return mavproxy

    def stop_mavproxy(self, mavproxy):
        if self.mavproxy is not None:
            return
        self.progress("Stopping MAVProxy")
        self.expect_list_remove(mavproxy)
        util.pexpect_close(mavproxy)
        self._mavproxy = None

    def start_SITL(self, **sitl_args):
        start_sitl_args = {
            "breakpoints": self.breakpoints,
            "disable_breakpoints": self.disable_breakpoints,
            "gdb": self.gdb,
            "gdb_no_tui": self.gdb_no_tui,
            "gdbserver": self.gdbserver,
            "lldb": self.lldb,
            "home": self.sitl_home(),
            "speedup": self.speedup,
            "valgrind": self.valgrind,
            "callgrind": self.callgrind,
            "wipe": True,
        }
        start_sitl_args.update(**sitl_args)
        if ("defaults_filepath" not in start_sitl_args or
                start_sitl_args["defaults_filepath"] is None):
            start_sitl_args["defaults_filepath"] = self.defaults_filepath()

        if "model" not in start_sitl_args or start_sitl_args["model"] is None:
            start_sitl_args["model"] = self.frame
        self.progress("Starting SITL", send_statustext=False)
        self.sitl = util.start_SITL(self.binary, **start_sitl_args)
        self.expect_list_add(self.sitl)
        self.sup_prog = []
        for sup_binary in self.sup_binaries:
            self.progress("Starting Supplementary Program ", sup_binary)
            start_sitl_args["customisations"] = [sup_binary[1]]
            start_sitl_args["supplementary"] = True
            sup_prog_link = util.start_SITL(sup_binary[0], **start_sitl_args)
            self.sup_prog.append(sup_prog_link)
            self.expect_list_add(sup_prog_link)

    def get_suplementary_programs(self):
        return self.sup_prog

    def sitl_is_running(self):
        if self.sitl is None:
            return False
        return self.sitl.isalive()

    def autostart_mavproxy(self):
        return self.use_map

    def init(self):
        """Initilialize autotest feature."""
        self.check_test_syntax(test_file=self.test_filepath())

        self.mavproxy_logfile = self.open_mavproxy_logfile()

        if self.frame is None:
            self.frame = self.default_frame()

        if self.frame is None:
            raise ValueError("frame must not be None")

        self.progress("Starting simulator")
        self.start_SITL()

        os.environ['MAVLINK20'] = '1'

        self.progress("Starting MAVLink connection")
        self.get_mavlink_connection_going()

        if self.autostart_mavproxy():
            self.mavproxy = self.start_mavproxy()

        self.expect_list_clear()
        self.expect_list_extend([self.sitl, self.mavproxy])
        self.expect_list_extend(self.sup_prog)

        # need to wait for a heartbeat to arrive as then mavutil will
        # select the correct set of messages for us to receive in
        # self.mav.messages.  You can actually receive messages with
        # recv_match and those will not be in self.mav.messages until
        # you do this!
        self.wait_heartbeat()
        self.progress("Sim time: %f" % (self.get_sim_time(),))
        self.apply_default_parameters()

        if not self.sitl_is_running():
            # we run this just to make sure exceptions are likely to
            # work OK.
            raise NotAchievedException("SITL is not running")
        self.progress("SITL is running")

        self.progress("Ready to start testing!")

    def upload_using_mission_protocol(self, mission_type, items):
        '''mavlink2 required'''
        target_system = 1
        target_component = 1
        self.do_timesync_roundtrip()
        tstart = self.get_sim_time()
        self.mav.mav.mission_count_send(target_system,
                                        target_component,
                                        len(items),
                                        mission_type)
        remaining_to_send = set(range(0, len(items)))
        sent = set()
        while True:
            if self.get_sim_time_cached() - tstart > (10 + len(items)/10):
                raise NotAchievedException("timeout uploading %s" % str(mission_type))
            if len(remaining_to_send) == 0:
                self.progress("All sent")
                break
            m = self.mav.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK'],
                                    blocking=True,
                                    timeout=1)
            if m is None:
                continue

            if m.get_type() == 'MISSION_ACK':
                if (m.target_system == 255 and
                        m.target_component == 0 and
                        m.type == 1 and
                        m.mission_type == 0):
                    # this is just MAVProxy trying to screw us up
                    continue
                else:
                    raise NotAchievedException("Received unexpected mission ack %s" % str(m))

            self.progress("Handling request for item %u/%u" % (m.seq, len(items)-1))
            self.progress("Item (%s)" % str(items[m.seq]))
            if m.seq in sent:
                self.progress("received duplicate request for item %u" % m.seq)
                continue

            if m.seq not in remaining_to_send:
                raise NotAchievedException("received request for unknown item %u" % m.seq)

            if m.mission_type != mission_type:
                raise NotAchievedException("received request for item from wrong mission type")

            if items[m.seq].mission_type != mission_type:
                raise NotAchievedException("supplied item not of correct mission type")
            if items[m.seq].target_system != target_system:
                raise NotAchievedException("supplied item not of correct target system")
            if items[m.seq].target_component != target_component:
                raise NotAchievedException("supplied item not of correct target component")
            if items[m.seq].seq != m.seq:
                raise NotAchievedException("supplied item has incorrect sequence number (%u vs %u)" %
                                           (items[m.seq].seq, m.seq))

            items[m.seq].pack(self.mav.mav)
            self.mav.mav.send(items[m.seq])
            remaining_to_send.discard(m.seq)
            sent.add(m.seq)
        m = self.assert_receive_message('MISSION_ACK', timeout=1)
        if m.mission_type != mission_type:
            raise NotAchievedException("Mission ack not of expected mission type")
        if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise NotAchievedException("Mission upload failed (%s)" %
                                       (mavutil.mavlink.enums["MAV_MISSION_RESULT"][m.type].name),)
        self.progress("Upload of all %u items succeeded" % len(items))

    def download_using_mission_protocol(self, mission_type, verbose=False, timeout=10):
        '''mavlink2 required'''
        target_system = 1
        target_component = 1
        self.drain_mav()
        self.progress("Sending mission_request_list")
        tstart = self.get_sim_time()
        self.mav.mav.mission_request_list_send(target_system,
                                               target_component,
                                               mission_type)

        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not get MISSION_COUNT packet")
            m = self.mav.recv_match(blocking=True, timeout=0.1)
            if m is None:
                raise NotAchievedException("Did not get MISSION_COUNT response")
            if verbose:
                self.progress(str(m))
            if m.get_type() == 'MISSION_ACK':
                if m.target_system == 255 and m.target_component == 0:
                    # this was for MAVProxy
                    continue
                self.progress("Mission ACK: %s" % str(m))
                raise NotAchievedException("Received MISSION_ACK while waiting for MISSION_COUNT")
            if m.get_type() != 'MISSION_COUNT':
                continue
            if m.target_component != 250: # FIXME: constant?!
                continue
            if m.mission_type != mission_type:
                raise NotAchievedException("Mission count response of incorrect type")
            break

        items = []
        tstart = self.get_sim_time_cached()
        remaining_to_receive = set(range(0, m.count))
        next_to_request = 0
        timeout = (10 + m.count/10)
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("timeout downloading type=%s" %
                                           (mavutil.mavlink.enums["MAV_MISSION_TYPE"][mission_type].name))
            if len(remaining_to_receive) == 0:
                self.progress("All received")
                return items
            self.progress("Requesting item %u" % next_to_request)
            self.mav.mav.mission_request_int_send(target_system,
                                                  target_component,
                                                  next_to_request,
                                                  mission_type)
            m = self.mav.recv_match(type='MISSION_ITEM_INT',
                                    blocking=True,
                                    timeout=5,
                                    condition='MISSION_ITEM_INT.mission_type==%u' % mission_type)
            if m is None:
                raise NotAchievedException("Did not receive MISSION_ITEM_INT")
            self.progress("Got (%s)" % str(m))
            if m.mission_type != mission_type:
                raise NotAchievedException("Received waypoint of wrong type")
            if m.seq != next_to_request:
                raise NotAchievedException("Received waypoint is out of sequence")
            self.progress("Item %u OK" % m.seq)
            items.append(m)
            next_to_request += 1
            remaining_to_receive.discard(m.seq)

    def dump_message_verbose(self, m):
        '''return verbose dump of m.  Wraps the pymavlink routine which
        inconveniently takes a filehandle'''
        f = StringIO.StringIO()
        mavutil.dump_message_verbose(f, m)
        return f.getvalue()

    def poll_home_position(self, quiet=True, timeout=30):
        old = self.mav.messages.get("HOME_POSITION", None)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Failed to poll home position")
            if not quiet:
                self.progress("Sending MAV_CMD_GET_HOME_POSITION")
            try:
                self.run_cmd(
                    mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    quiet=quiet)
            except ValueError:
                continue
            m = self.mav.messages.get("HOME_POSITION", None)
            if m is None:
                continue
            if old is None:
                break
            if m._timestamp != old._timestamp:
                break
        self.progress("Polled home position (%s)" % str(m))
        return m

    def distance_to_nav_target(self, use_cached_nav_controller_output=False):
        '''returns distance to waypoint navigation target in metres'''
        m = self.mav.messages.get("NAV_CONTROLLER_OUTPUT", None)
        if m is None or not use_cached_nav_controller_output:
            m = self.assert_receive_message('NAV_CONTROLLER_OUTPUT', timeout=10)
        return m.wp_dist

    def distance_to_home(self, use_cached_home=False):
        m = self.mav.messages.get("HOME_POSITION", None)
        if use_cached_home is False or m is None:
            m = self.poll_home_position(quiet=True)
        here = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                   blocking=True)
        return self.get_distance_int(m, here)

    def home_position_as_mav_location(self):
        m = self.poll_home_position()
        return mavutil.location(m.latitude*1.0e-7, m.longitude*1.0e-7, m.altitude*1.0e-3, 0)

    def offset_location_ne(self, location, metres_north, metres_east):
        '''return a new location offset from passed-in location'''
        (target_lat, target_lng) = mavextra.gps_offset(location.lat,
                                                       location.lng,
                                                       metres_east,
                                                       metres_north)
        return mavutil.location(target_lat,
                                target_lng,
                                location.alt,
                                location.heading)

    def monitor_groundspeed(self, want, tolerance=0.5, timeout=5):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                break
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            if m.groundspeed > want+tolerance:
                raise NotAchievedException("Too fast (%f > %f)" %
                                           (m.groundspeed, want))
            if m.groundspeed < want-tolerance:
                raise NotAchievedException("Too slow (%f < %f)" %
                                           (m.groundspeed, want))
            self.progress("GroundSpeed OK (got=%f) (want=%f)" %
                          (m.groundspeed, want))

    def fly_test_set_home(self):
        if self.is_tracker():
            # tracker starts armed...
            self.disarm_vehicle(force=True)
        self.reboot_sitl()

        # HOME_POSITION is used as a surrogate for origin until we
        # start emitting GPS_GLOBAL_ORIGIN
        self.wait_ekf_happy()
        orig_home = self.poll_home_position()
        if orig_home is None:
            raise AutoTestTimeoutException()
        self.progress("Original home: %s" % str(orig_home))
        # original home should be close to SITL home...
        start_loc = self.sitl_start_location()
        self.progress("SITL start loc: %s" % str(start_loc))
        delta = abs(orig_home.latitude * 1.0e-7 - start_loc.lat)
        if delta > 0.000001:
            raise ValueError("homes differ in lat got=%f vs want=%f delta=%f" %
                             (orig_home.latitude * 1.0e-7, start_loc.lat, delta))
        delta = abs(orig_home.longitude * 1.0e-7 - start_loc.lng)
        if delta > 0.000001:
            raise ValueError("homes differ in lon  got=%f vs want=%f delta=%f" %
                             (orig_home.longitude * 1.0e-7, start_loc.lng, delta))
        if self.is_rover():
            self.progress("### Rover skipping altitude check unti position fixes in")
        else:
            home_alt_m = orig_home.altitude * 1.0e-3
            if abs(home_alt_m - start_loc.alt) > 2: # metres
                raise ValueError("homes differ in alt got=%fm want=%fm" %
                                 (home_alt_m, start_loc.alt))
        new_x = orig_home.latitude + 1000
        new_y = orig_home.longitude + 2000
        new_z = orig_home.altitude + 300000 # 300 metres
        print("new home: %s %s %s" % (str(new_x), str(new_y), str(new_z)))
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                         0, # p1,
                         0, # p2,
                         0, # p3,
                         0, # p4,
                         new_x,
                         new_y,
                         new_z/1000.0, # mm => m
                         )

        home = self.poll_home_position()
        self.progress("home: %s" % str(home))
        got_home_latitude = home.latitude
        got_home_longitude = home.longitude
        got_home_altitude = home.altitude
        if (got_home_latitude != new_x or
                got_home_longitude != new_y or
                abs(got_home_altitude - new_z) > 100): # float-conversion issues
            self.reboot_sitl()
            raise NotAchievedException(
                "Home mismatch got=(%f, %f, %f) set=(%f, %f, %f)" %
                (got_home_latitude, got_home_longitude, got_home_altitude,
                 new_x, new_y, new_z))

        self.progress("monitoring home to ensure it doesn't drift at all")
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 10:
            home = self.poll_home_position(quiet=True)
            self.progress("home: %s" % str(home))
            if (home.latitude != got_home_latitude or
                    home.longitude != got_home_longitude or
                    home.altitude != got_home_altitude): # float-conversion issues
                self.reboot_sitl()
                raise NotAchievedException("home is drifting")

        self.progress("Waiting for EKF to start")
        self.wait_ready_to_arm()
        self.progress("now use lat=0, lon=0 to reset home to current location")
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                         0, # p1,
                         0, # p2,
                         0, # p3,
                         0, # p4,
                         0, # lat
                         0, # lon
                         new_z/1000.0, # mm => m
                         )
        home = self.poll_home_position()
        self.progress("home: %s" % str(home))
        if self.distance_to_home(use_cached_home=True) > 1:
            raise NotAchievedException("Setting home to current location did not work")

        self.progress("Setting home elsewhere again")
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                         0, # p1,
                         0, # p2,
                         0, # p3,
                         0, # p4,
                         new_x,
                         new_y,
                         new_z/1000.0, # mm => m
                         )
        if self.distance_to_home() < 10:
            raise NotAchievedException("Setting home to location did not work")

        self.progress("use param1=1 to reset home to current location")
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                         1, # p1,
                         0, # p2,
                         0, # p3,
                         0, # p4,
                         37, # lat
                         21, # lon
                         new_z/1000.0, # mm => m
                         )
        home = self.poll_home_position()
        self.progress("home: %s" % str(home))
        if self.distance_to_home() > 1:
            raise NotAchievedException("Setting home to current location did not work")

        if self.is_tracker():
            # tracker starts armed...
            self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def zero_mag_offset_parameters(self, compass_count=3):
        self.progress("Zeroing Mag OFS parameters")
        self.drain_mav()
        self.get_sim_time()
        self.run_cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS,
                     2, # param1 (compass0)
                     0, # param2
                     0, # param3
                     0, # param4
                     0, # param5
                     0, # param6
                     0 # param7
                     )
        self.run_cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS,
                     5, # param1 (compass1)
                     0, # param2
                     0, # param3
                     0, # param4
                     0, # param5
                     0, # param6
                     0 # param7
                     )
        self.run_cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS,
                     6, # param1 (compass2)
                     0, # param2
                     0, # param3
                     0, # param4
                     0, # param5
                     0, # param6
                     0 # param7
                     )
        self.progress("zeroed mag parameters")
        params = [
            [("SIM_MAG_OFS_X", "COMPASS_OFS_X", 0),
             ("SIM_MAG_OFS_Y", "COMPASS_OFS_Y", 0),
             ("SIM_MAG_OFS_Z", "COMPASS_OFS_Z", 0), ],
        ]
        for count in range(2, compass_count + 1):
            params += [
                [("SIM_MAG%d_OFS_X" % count, "COMPASS_OFS%d_X" % count, 0),
                 ("SIM_MAG%d_OFS_Y" % count, "COMPASS_OFS%d_Y" % count, 0),
                 ("SIM_MAG%d_OFS_Z" % count, "COMPASS_OFS%d_Z" % count, 0), ],
            ]
        self.check_zero_mag_parameters(params)

    def forty_two_mag_dia_odi_parameters(self, compass_count=3):
        self.progress("Forty twoing Mag DIA and ODI parameters")
        self.drain_mav()
        self.get_sim_time()
        params = [
            [("SIM_MAG_DIA_X", "COMPASS_DIA_X", 42.0),
             ("SIM_MAG_DIA_Y", "COMPASS_DIA_Y", 42.0),
             ("SIM_MAG_DIA_Z", "COMPASS_DIA_Z", 42.0),
             ("SIM_MAG_ODI_X", "COMPASS_ODI_X", 42.0),
             ("SIM_MAG_ODI_Y", "COMPASS_ODI_Y", 42.0),
             ("SIM_MAG_ODI_Z", "COMPASS_ODI_Z", 42.0), ],
        ]
        for count in range(2, compass_count + 1):
            params += [
                [("SIM_MAG%d_DIA_X" % count, "COMPASS_DIA%d_X" % count, 42.0),
                 ("SIM_MAG%d_DIA_Y" % count, "COMPASS_DIA%d_Y" % count, 42.0),
                 ("SIM_MAG%d_DIA_Z" % count, "COMPASS_DIA%d_Z" % count, 42.0),
                 ("SIM_MAG%d_ODI_X" % count, "COMPASS_ODI%d_X" % count, 42.0),
                 ("SIM_MAG%d_ODI_Y" % count, "COMPASS_ODI%d_Y" % count, 42.0),
                 ("SIM_MAG%d_ODI_Z" % count, "COMPASS_ODI%d_Z" % count, 42.0), ],
            ]
        self.wait_heartbeat()
        to_set = {}
        for param_set in params:
            for param in param_set:
                (_, _out, value) = param
                to_set[_out] = value
        self.set_parameters(to_set)
        self.check_zero_mag_parameters(params)

    def check_mag_parameters(self, parameter_stuff, compass_number):
        self.progress("Checking that Mag parameter")
        for idx in range(0, compass_number, 1):
            for param in parameter_stuff[idx]:
                (_in, _out, value) = param
                got_value = self.get_parameter(_out)
                if abs(got_value - value) > abs(value) * 0.15:
                    raise NotAchievedException("%s/%s not within 15%%; got %f want=%f" % (_in, _out, got_value, value))

    def check_zero_mag_parameters(self, parameter_stuff):
        self.progress("Checking that Mag OFS are zero")
        for param_set in parameter_stuff:
            for param in param_set:
                (_in, _out, _) = param
                got_value = self.get_parameter(_out)
                max = 0.15
                if "DIA" in _out or "ODI" in _out:
                    max += 42.0
                if abs(got_value) > max:
                    raise NotAchievedException(
                        "%s/%s not within 15%%; got %f want=%f" %
                        (_in, _out, got_value, 0.0 if max > 1 else 42.0))

    def check_zeros_mag_orient(self, compass_count=3):
        self.progress("zeroed mag parameters")
        self.verify_parameter_values({"COMPASS_ORIENT": 0})
        for count in range(2, compass_count + 1):
            self.verify_parameter_values({"COMPASS_ORIENT%d" % count: 0})

    def test_mag_calibration(self, compass_count=3, timeout=1000):
        def reset_pos_and_start_magcal(mavproxy, tmask):
            mavproxy.send("sitl_stop\n")
            mavproxy.send("sitl_attitude 0 0 0\n")
            self.drain_mav()
            self.get_sim_time()
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,
                         tmask, # p1: mag_mask
                         0, # p2: retry
                         0, # p3: autosave
                         0, # p4: delay
                         0, # param5
                         0, # param6
                         0, # param7
                         want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                         timeout=20,
                         )
            mavproxy.send("sitl_magcal\n")

        def do_prep_mag_cal_test(mavproxy, params):
            self.progress("Preparing the vehicle for magcal")
            MAG_OFS = 100
            MAG_DIA = 1.0
            MAG_ODI = 0.004
            params += [
                [("SIM_MAG_OFS_X", "COMPASS_OFS_X", MAG_OFS),
                 ("SIM_MAG_OFS_Y", "COMPASS_OFS_Y", MAG_OFS + 100),
                 ("SIM_MAG_OFS_Z", "COMPASS_OFS_Z", MAG_OFS + 200),
                 ("SIM_MAG_DIA_X", "COMPASS_DIA_X", MAG_DIA),
                 ("SIM_MAG_DIA_Y", "COMPASS_DIA_Y", MAG_DIA + 0.1),
                 ("SIM_MAG_DIA_Z", "COMPASS_DIA_Z", MAG_DIA + 0.2),
                 ("SIM_MAG_ODI_X", "COMPASS_ODI_X", MAG_ODI),
                 ("SIM_MAG_ODI_Y", "COMPASS_ODI_Y", MAG_ODI + 0.001),
                 ("SIM_MAG_ODI_Z", "COMPASS_ODI_Z", MAG_ODI + 0.001), ],
            ]
            for count in range(2, compass_count + 1):
                params += [
                    [("SIM_MAG%d_OFS_X" % count, "COMPASS_OFS%d_X" % count, MAG_OFS + 100 * ((count+2) % compass_count)),
                     ("SIM_MAG%d_OFS_Y" % count, "COMPASS_OFS%d_Y" % count, MAG_OFS + 100 * ((count+3) % compass_count)),
                     ("SIM_MAG%d_OFS_Z" % count, "COMPASS_OFS%d_Z" % count, MAG_OFS + 100 * ((count+1) % compass_count)),
                     ("SIM_MAG%d_DIA_X" % count, "COMPASS_DIA%d_X" % count, MAG_DIA + 0.1 * ((count+2) % compass_count)),
                     ("SIM_MAG%d_DIA_Y" % count, "COMPASS_DIA%d_Y" % count, MAG_DIA + 0.1 * ((count+3) % compass_count)),
                     ("SIM_MAG%d_DIA_Z" % count, "COMPASS_DIA%d_Z" % count, MAG_DIA + 0.1 * ((count+1) % compass_count)),
                     ("SIM_MAG%d_ODI_X" % count, "COMPASS_ODI%d_X" % count, MAG_ODI + 0.001 * ((count+2) % compass_count)),
                     ("SIM_MAG%d_ODI_Y" % count, "COMPASS_ODI%d_Y" % count, MAG_ODI + 0.001 * ((count+3) % compass_count)),
                     ("SIM_MAG%d_ODI_Z" % count, "COMPASS_ODI%d_Z" % count, MAG_ODI + 0.001 * ((count+1) % compass_count)), ],
                ]
            self.progress("Setting calibration mode")
            self.wait_heartbeat()
            self.customise_SITL_commandline(["-M", "calibration"])
            self.mavproxy_load_module(mavproxy, "sitl_calibration")
            self.mavproxy_load_module(mavproxy, "calibration")
            self.mavproxy_load_module(mavproxy, "relay")
            self.wait_statustext("is using GPS", timeout=60)
            mavproxy.send("accelcalsimple\n")
            mavproxy.expect("Calibrated")
            # disable it to not interfert with calibration acceptation
            self.mavproxy_unload_module(mavproxy, "calibration")
            if self.is_copter():
                # set frame class to pass arming check on copter
                self.set_parameter("FRAME_CLASS", 1)
            self.drain_mav()
            self.progress("Setting SITL Magnetometer model value")
            self.set_parameter("COMPASS_AUTO_ROT", 0)
            # MAG_ORIENT = 4
            # self.set_parameter("SIM_MAG_ORIENT", MAG_ORIENT)
            # for count in range(2, compass_count + 1):
            #     self.set_parameter("SIM_MAG%d_ORIENT" % count, MAG_ORIENT * (count % 41))
            #     # set compass external to check that orientation is found and auto set
            #     self.set_parameter("COMPASS_EXTERN%d" % count, 1)
            to_set = {}
            for param_set in params:
                for param in param_set:
                    (_in, _out, value) = param
                    to_set[_in] = value
                    to_set[_out] = value
            self.set_parameters(to_set)
            self.start_subtest("Zeroing Mag OFS parameters with Mavlink")
            self.zero_mag_offset_parameters()
            self.progress("=========================================")
            # Change the default value to unexpected 42
            self.forty_two_mag_dia_odi_parameters()
            self.progress("Zeroing Mags orientations")
            self.set_parameter("COMPASS_ORIENT", 0)
            for count in range(2, compass_count + 1):
                self.set_parameter("COMPASS_ORIENT%d" % count, 0)

            # Only care about compass prearm
            self.set_parameter("ARMING_CHECK", 4)

        #################################################
        def do_test_mag_cal(mavproxy, params, compass_tnumber):
            self.start_subtest("Try magcal and make it stop around 30%")
            self.progress("Compass mask is %s" % "{0:b}".format(target_mask))
            reset_pos_and_start_magcal(mavproxy, target_mask)
            tstart = self.get_sim_time()
            reached_pct = [0] * compass_tnumber
            tstop = None
            while True:
                if self.get_sim_time_cached() - tstart > timeout:
                    raise NotAchievedException("Cannot receive enough MAG_CAL_PROGRESS")
                m = self.mav.recv_match(type='MAG_CAL_PROGRESS', blocking=True, timeout=5)
                if m is None:
                    if tstop is not None:
                        # wait 3 second to unsure that the calibration is well stopped
                        if self.get_sim_time_cached() - tstop > 10:
                            if reached_pct[0] > 33:
                                raise NotAchievedException("Mag calibration didn't stop")
                            else:
                                break
                        else:
                            continue
                    else:
                        continue
                if m is not None:
                    cid = m.compass_id
                    new_pct = int(m.completion_pct)
                    if new_pct != reached_pct[cid]:
                        if new_pct < reached_pct[cid]:
                            raise NotAchievedException("Mag calibration restart when it shouldn't")
                        reached_pct[cid] = new_pct
                        self.progress("Calibration progress compass ID %d: %s%%" % (cid, str(reached_pct[cid])))
                        if cid == 0 and 13 <= reached_pct[0] <= 15:
                            self.progress("Request again to start calibration, it shouldn't restart from 0")
                            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,
                                         target_mask,
                                         0,
                                         0,
                                         0,
                                         0,
                                         0,
                                         0,
                                         want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                                         timeout=20,
                                         )

                if reached_pct[0] > 30:
                    self.run_cmd(mavutil.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL,
                                 target_mask,  # p1: mag_mask
                                 0,  # param2
                                 0,  # param3
                                 0,  # param4
                                 0,  # param5
                                 0,  # param6
                                 0,  # param7
                                 want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                                 )
                    if tstop is None:
                        tstop = self.get_sim_time_cached()
                if tstop is not None:
                    # wait 3 second to unsure that the calibration is well stopped
                    if self.get_sim_time_cached() - tstop > 3:
                        raise NotAchievedException("Mag calibration didn't stop")
            self.check_zero_mag_parameters(params)
            self.check_zeros_mag_orient()

            #################################################
            self.start_subtest("Try magcal and make it failed")
            self.progress("Compass mask is %s" % "{0:b}".format(target_mask))
            old_cal_fit = self.get_parameter("COMPASS_CAL_FIT")
            self.set_parameter("COMPASS_CAL_FIT", 0.001, add_to_context=False)
            reset_pos_and_start_magcal(mavproxy, target_mask)
            tstart = self.get_sim_time()
            reached_pct = [0] * compass_tnumber
            report_get = [0] * compass_tnumber
            while True:
                if self.get_sim_time_cached() - tstart > timeout:
                    raise NotAchievedException("Cannot receive enough MAG_CAL_PROGRESS")
                m = self.mav.recv_match(type=["MAG_CAL_PROGRESS", "MAG_CAL_REPORT"], blocking=True, timeout=10)
                if m.get_type() == "MAG_CAL_REPORT":
                    if report_get[m.compass_id] == 0:
                        self.progress("Report: %s" % str(m))
                        if m.cal_status == mavutil.mavlink.MAG_CAL_FAILED:
                            report_get[m.compass_id] = 1
                        else:
                            raise NotAchievedException("Mag calibration didn't failed")
                    if all(ele >= 1 for ele in report_get):
                        self.progress("All Mag report failure")
                        break
                if m is not None and m.get_type() == "MAG_CAL_PROGRESS":
                    cid = m.compass_id
                    new_pct = int(m.completion_pct)
                    if new_pct != reached_pct[cid]:
                        reached_pct[cid] = new_pct
                        self.progress("Calibration progress compass ID %d: %s%%" % (cid, str(reached_pct[cid])))
                        if cid == 0 and 49 <= reached_pct[0] <= 50:
                            self.progress("Try arming during calibration, should failed")
                            self.try_arm(False, "Compass calibration running")

            self.check_zero_mag_parameters(params)
            self.check_zeros_mag_orient()
            self.set_parameter("COMPASS_CAL_FIT", old_cal_fit, add_to_context=False)

            #################################################
            self.start_subtest("Try magcal and wait success")
            self.progress("Compass mask is %s" % "{0:b}".format(target_mask))
            reset_pos_and_start_magcal(mavproxy, target_mask)
            progress_count = [0] * compass_tnumber
            reached_pct = [0] * compass_tnumber
            report_get = [0] * compass_tnumber
            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time_cached() - tstart > timeout:
                    raise NotAchievedException("Cannot receive enough MAG_CAL_PROGRESS")
                m = self.mav.recv_match(type=["MAG_CAL_PROGRESS", "MAG_CAL_REPORT"], blocking=True, timeout=5)
                if m.get_type() == "MAG_CAL_REPORT":
                    if report_get[m.compass_id] == 0:
                        self.progress("Report: %s" % self.dump_message_verbose(m))
                        param_names = ["SIM_MAG_ORIENT"]
                        for i in range(2, compass_tnumber+1):
                            param_names.append("SIM_MAG%u_ORIENT" % i)
                        for param_name in param_names:
                            self.progress("%s=%f" % (param_name, self.get_parameter(param_name)))
                        if m.cal_status == mavutil.mavlink.MAG_CAL_SUCCESS:
                            threshold = 95
                            if reached_pct[m.compass_id] < threshold:
                                raise NotAchievedException(
                                    "Mag calibration report SUCCESS without >=%f%% completion (got %f%%)" %
                                    (threshold, reached_pct[m.compass_id]))
                            report_get[m.compass_id] = 1
                        else:
                            raise NotAchievedException(
                                "Mag calibration didn't SUCCEED (cal_status=%u) (progress_count=%s)" %
                                (m.cal_status, progress_count[m.compass_id],))
                    if all(ele >= 1 for ele in report_get):
                        self.progress("All Mag report SUCCESS")
                        break
                if m is not None and m.get_type() == "MAG_CAL_PROGRESS":
                    cid = m.compass_id
                    new_pct = int(m.completion_pct)
                    progress_count[cid] += 1
                    if new_pct != reached_pct[cid]:
                        reached_pct[cid] = new_pct
                        self.progress("Calibration progress compass ID %d: %s%%" % (cid, str(reached_pct[cid])))
            mavproxy.send("sitl_stop\n")
            mavproxy.send("sitl_attitude 0 0 0\n")
            self.progress("Checking that value aren't changed without acceptation")
            self.check_zero_mag_parameters(params)
            self.check_zeros_mag_orient()
            self.progress("Send acceptation and check value")
            self.wait_heartbeat()
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_ACCEPT_MAG_CAL,
                         target_mask, # p1: mag_mask
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                         timeout=20,
                         )
            self.check_mag_parameters(params, compass_tnumber)
            self.verify_parameter_values({"COMPASS_ORIENT": self.get_parameter("SIM_MAG_ORIENT")})
            for count in range(2, compass_tnumber + 1):
                self.verify_parameter_values({"COMPASS_ORIENT%d" % count: self.get_parameter("SIM_MAG%d_ORIENT" % count)})
            self.try_arm(False, "Compass calibrated requires reboot")

            # test buzzer/notify ?
            self.progress("Rebooting and making sure we could arm with these values")
            self.drain_mav()
            self.reboot_sitl()
            if False:   # FIXME!  This fails with compasses inconsistent!
                self.wait_ready_to_arm(timeout=60)
            self.progress("Setting manually the parameter for other sensor to avoid compass consistency error")
            for idx in range(compass_tnumber, compass_count, 1):
                for param in params[idx]:
                    (_in, _out, value) = param
                    self.set_parameter(_out, value)
            for count in range(compass_tnumber + 1, compass_count + 1):
                self.set_parameter("COMPASS_ORIENT%d" % count, self.get_parameter("SIM_MAG%d_ORIENT" % count))
            self.arm_vehicle()
            self.progress("Test calibration rejection when armed")
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,
                         target_mask, # p1: mag_mask
                         0, # p2: retry
                         0, # p3: autosave
                         0, # p4: delay
                         0, # param5
                         0, # param6
                         0, # param7
                         want_result=mavutil.mavlink.MAV_RESULT_FAILED,
                         timeout=20,
                         )
            self.disarm_vehicle()
            self.mavproxy_unload_module(mavproxy, "relay")
            self.mavproxy_unload_module(mavproxy, "sitl_calibration")

        ex = None

        mavproxy = self.start_mavproxy()
        try:
            self.set_parameter("AHRS_EKF_TYPE", 10)
            self.set_parameter("SIM_GND_BEHAV", 0)

            curr_params = []
            target_mask = 0
            # we test all bitmask plus 0 for all
            for run in range(-1, compass_count, 1):
                ntest_compass = compass_count
                if run < 0:
                    # use bitmask 0 for all compass
                    target_mask = 0
                else:
                    target_mask |= (1 << run)
                    ntest_compass = run + 1
                do_prep_mag_cal_test(mavproxy, curr_params)
                do_test_mag_cal(mavproxy, curr_params, ntest_compass)

        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e
            self.mavproxy_unload_module(mavproxy, "relay")
            self.mavproxy_unload_module(mavproxy, "sitl_calibration")
        if ex is not None:
            raise ex

        self.stop_mavproxy(mavproxy)

        # need to reboot SITL after moving away from EKF type 10; we
        # can end up with home set but origin not and that will lead
        # to bad things.
        self.reboot_sitl()

    def test_mag_reordering_assert_mag_transform(self, values, transforms):
        '''transforms ought to be read as, "take all the parameter values from
        the first compass parameters and shove them into the second indicating
        compass parameters'''

        # create a set of mappings from one parameter name to another
        # e.g. COMPASS_OFS_X => COMPASS_OFS2_X if the transform is
        # [(1,2)].  [(1,2),(2,1)] should swap the compass values

        parameter_mappings = {}
        for key in values.keys():
            parameter_mappings[key] = key
        for (old_compass_num, new_compass_num) in transforms:
            old_key_compass_bit = str(old_compass_num)
            if old_key_compass_bit == "1":
                old_key_compass_bit = ""
            new_key_compass_bit = str(new_compass_num)
            if new_key_compass_bit == "1":
                new_key_compass_bit = ""
            # vectors first:
            for key_vector_bit in ["OFS", "DIA", "ODI", "MOT"]:
                for axis in "X", "Y", "Z":
                    old_key = "COMPASS_%s%s_%s" % (key_vector_bit,
                                                   old_key_compass_bit,
                                                   axis)
                    new_key = "COMPASS_%s%s_%s" % (key_vector_bit,
                                                   new_key_compass_bit,
                                                   axis)
                    parameter_mappings[old_key] = new_key
            # then non-vectorey bits:
            for key_bit in "SCALE", "ORIENT":
                old_key = "COMPASS_%s%s" % (key_bit, old_key_compass_bit)
                new_key = "COMPASS_%s%s" % (key_bit, new_key_compass_bit)
                parameter_mappings[old_key] = new_key
            # then a sore thumb:
            if old_key_compass_bit == "":
                old_key = "COMPASS_EXTERNAL"
            else:
                old_key = "COMPASS_EXTERN%s" % old_key_compass_bit
            if new_key_compass_bit == "":
                new_key = "COMPASS_EXTERNAL"
            else:
                new_key = "COMPASS_EXTERN%s" % new_key_compass_bit
            parameter_mappings[old_key] = new_key

        for key in values.keys():
            newkey = parameter_mappings[key]
            current_value = self.get_parameter(newkey)
            expected_value = values[key]
            if abs(current_value - expected_value) > 0.001:
                raise NotAchievedException("%s has wrong value; want=%f got=%f transforms=%s (old parameter name=%s)" %
                                           (newkey, expected_value, current_value, str(transforms), key))

    def test_mag_reordering(self):
        self.context_push()
        ex = None
        try:
            originals = {
                "COMPASS_OFS_X": 1.1,
                "COMPASS_OFS_Y": 1.2,
                "COMPASS_OFS_Z": 1.3,
                "COMPASS_DIA_X": 1.4,
                "COMPASS_DIA_Y": 1.5,
                "COMPASS_DIA_Z": 1.6,
                "COMPASS_ODI_X": 1.7,
                "COMPASS_ODI_Y": 1.8,
                "COMPASS_ODI_Z": 1.9,
                "COMPASS_MOT_X": 1.91,
                "COMPASS_MOT_Y": 1.92,
                "COMPASS_MOT_Z": 1.93,
                "COMPASS_SCALE": 1.94,
                "COMPASS_ORIENT": 1,
                "COMPASS_EXTERNAL": 2,

                "COMPASS_OFS2_X": 2.1,
                "COMPASS_OFS2_Y": 2.2,
                "COMPASS_OFS2_Z": 2.3,
                "COMPASS_DIA2_X": 2.4,
                "COMPASS_DIA2_Y": 2.5,
                "COMPASS_DIA2_Z": 2.6,
                "COMPASS_ODI2_X": 2.7,
                "COMPASS_ODI2_Y": 2.8,
                "COMPASS_ODI2_Z": 2.9,
                "COMPASS_MOT2_X": 2.91,
                "COMPASS_MOT2_Y": 2.92,
                "COMPASS_MOT2_Z": 2.93,
                "COMPASS_SCALE2": 2.94,
                "COMPASS_ORIENT2": 3,
                "COMPASS_EXTERN2": 4,

                "COMPASS_OFS3_X": 3.1,
                "COMPASS_OFS3_Y": 3.2,
                "COMPASS_OFS3_Z": 3.3,
                "COMPASS_DIA3_X": 3.4,
                "COMPASS_DIA3_Y": 3.5,
                "COMPASS_DIA3_Z": 3.6,
                "COMPASS_ODI3_X": 3.7,
                "COMPASS_ODI3_Y": 3.8,
                "COMPASS_ODI3_Z": 3.9,
                "COMPASS_MOT3_X": 3.91,
                "COMPASS_MOT3_Y": 3.92,
                "COMPASS_MOT3_Z": 3.93,
                "COMPASS_SCALE3": 3.94,
                "COMPASS_ORIENT3": 5,
                "COMPASS_EXTERN3": 6,
            }

            # quick sanity check to ensure all values are unique:
            if len(originals.values()) != len(set(originals.values())):
                raise NotAchievedException("Values are not all unique!")

            self.progress("Setting parameters")
            self.set_parameters(originals)

            self.reboot_sitl()

            # no transforms means our originals should be our finals:
            self.test_mag_reordering_assert_mag_transform(originals, [])

            self.start_subtest("Pushing 1st mag to 3rd")
            ey = None
            self.context_push()
            try:
                # now try reprioritising compass 1 to be higher than compass 0:
                prio1_id = self.get_parameter("COMPASS_PRIO1_ID")
                prio2_id = self.get_parameter("COMPASS_PRIO2_ID")
                prio3_id = self.get_parameter("COMPASS_PRIO3_ID")
                self.set_parameter("COMPASS_PRIO1_ID", prio2_id)
                self.set_parameter("COMPASS_PRIO2_ID", prio3_id)
                self.set_parameter("COMPASS_PRIO3_ID", prio1_id)

                self.reboot_sitl()

                self.test_mag_reordering_assert_mag_transform(originals, [(2, 1),
                                                                          (3, 2),
                                                                          (1, 3)])

            except Exception as e:
                self.progress("Caught exception: %s" %
                              self.get_exception_stacktrace(e))
                ey = e
            self.context_pop()
            self.reboot_sitl()
            if ey is not None:
                raise ey

        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_fixed_yaw_calibration(self):
        self.context_push()
        ex = None
        try:
            MAG_OFS_X = 100
            MAG_OFS_Y = 200
            MAG_OFS_Z = 300
            wanted = {
                "COMPASS_OFS_X": (MAG_OFS_X, 3.0),
                "COMPASS_OFS_Y": (MAG_OFS_Y, 3.0),
                "COMPASS_OFS_Z": (MAG_OFS_Z, 3.0),
                "COMPASS_DIA_X": 1,
                "COMPASS_DIA_Y": 1,
                "COMPASS_DIA_Z": 1,
                "COMPASS_ODI_X": 0,
                "COMPASS_ODI_Y": 0,
                "COMPASS_ODI_Z": 0,

                "COMPASS_OFS2_X": (MAG_OFS_X, 3.0),
                "COMPASS_OFS2_Y": (MAG_OFS_Y, 3.0),
                "COMPASS_OFS2_Z": (MAG_OFS_Z, 3.0),
                "COMPASS_DIA2_X": 1,
                "COMPASS_DIA2_Y": 1,
                "COMPASS_DIA2_Z": 1,
                "COMPASS_ODI2_X": 0,
                "COMPASS_ODI2_Y": 0,
                "COMPASS_ODI2_Z": 0,

                "COMPASS_OFS3_X": (MAG_OFS_X, 3.0),
                "COMPASS_OFS3_Y": (MAG_OFS_Y, 3.0),
                "COMPASS_OFS3_Z": (MAG_OFS_Z, 3.0),
                "COMPASS_DIA3_X": 1,
                "COMPASS_DIA3_Y": 1,
                "COMPASS_DIA3_Z": 1,
                "COMPASS_ODI3_X": 0,
                "COMPASS_ODI3_Y": 0,
                "COMPASS_ODI3_Z": 0,
            }
            self.set_parameters({
                "SIM_MAG_OFS_X": MAG_OFS_X,
                "SIM_MAG_OFS_Y": MAG_OFS_Y,
                "SIM_MAG_OFS_Z": MAG_OFS_Z,

                "SIM_MAG2_OFS_X": MAG_OFS_X,
                "SIM_MAG2_OFS_Y": MAG_OFS_Y,
                "SIM_MAG2_OFS_Z": MAG_OFS_Z,

                "SIM_MAG3_OFS_X": MAG_OFS_X,
                "SIM_MAG3_OFS_Y": MAG_OFS_Y,
                "SIM_MAG3_OFS_Z": MAG_OFS_Z,
            })

            # set to some sensible-ish initial values.  If your initial
            # offsets are way, way off you can get some very odd effects.
            for param in wanted:
                value = 0.0
                if "DIA" in param:
                    value = 1.001
                elif "ODI" in param:
                    value = 0.001
                self.set_parameter(param, value)

            self.zero_mag_offset_parameters()

            # wait until we definitely know where we are:
            self.poll_home_position(timeout=120)

            ss = self.mav.recv_match(type='SIMSTATE', blocking=True, timeout=1)
            if ss is None:
                raise NotAchievedException("Did not get SIMSTATE")
            self.progress("Got SIMSTATE (%s)" % str(ss))

            self.run_cmd(mavutil.mavlink.MAV_CMD_FIXED_MAG_CAL_YAW,
                         math.degrees(ss.yaw), # param1
                         0, # param2
                         0, # param3
                         0, # param4

                         0, # param5
                         0, # param6
                         0 # param7
                         )
            self.verify_parameter_values(wanted)

            self.progress("Rebooting and making sure we could arm with these values")
            self.reboot_sitl()
            self.wait_ready_to_arm(timeout=60)

        except Exception as e:
            ex = e

        self.context_pop()

        if ex is not None:
            raise ex

    def test_dataflash_over_mavlink(self):
        self.context_push()
        ex = None
        mavproxy = self.start_mavproxy()
        try:
            self.set_parameter("LOG_BACKEND_TYPE", 2)
            self.reboot_sitl()
            self.wait_ready_to_arm(check_prearm_bit=False)
            mavproxy.send('arm throttle\n')
            mavproxy.expect('PreArm: Logging failed')
            self.mavproxy_load_module(mavproxy, 'dataflash_logger')
            mavproxy.send("dataflash_logger set verbose 1\n")
            mavproxy.expect('logging started')
            mavproxy.send("dataflash_logger set verbose 0\n")
            self.delay_sim_time(1)
            self.do_timesync_roundtrip()  # drain COMMAND_ACK from that failed arm
            self.arm_vehicle()
            tstart = self.get_sim_time()
            last_status = 0
            while True:
                now = self.get_sim_time()
                if now - tstart > 60:
                    break
                if now - last_status > 5:
                    last_status = now
                    mavproxy.send('dataflash_logger status\n')
                    # seen on autotest: Active Rate(3s):97.790kB/s Block:164 Missing:0 Fixed:0 Abandoned:0
                    mavproxy.expect(r"Active Rate\([0-9]+s\):([0-9]+[.][0-9]+)")
                    rate = float(mavproxy.match.group(1))
                    self.progress("Rate: %f" % rate)
                    desired_rate = 50
                    if self.valgrind or self.callgrind:
                        desired_rate /= 10
                    if rate < desired_rate:
                        raise NotAchievedException("Exceptionally low transfer rate (%u < %u)" % (rate, desired_rate))
            self.disarm_vehicle()
        except Exception as e:
            self.print_exception_caught(e)
            self.disarm_vehicle()
            ex = e
        self.context_pop()
        self.mavproxy_unload_module(mavproxy, 'dataflash_logger')

        # the following things won't work - but they shouldn't die either:
        self.mavproxy_load_module(mavproxy, 'log')

        self.progress("Try log list")
        mavproxy.send("log list\n")
        mavproxy.expect("No logs")

        self.progress("Try log erase")
        mavproxy.send("log erase\n")
        # no response to this...

        self.progress("Try log download")
        mavproxy.send("log download 1\n")
        # no response to this...

        self.mavproxy_unload_module(mavproxy, 'log')

        self.stop_mavproxy(mavproxy)
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_dataflash_sitl(self):
        """Test the basic functionality of block logging"""
        self.context_push()
        ex = None
        mavproxy = self.start_mavproxy()
        try:
            self.set_parameter("LOG_BACKEND_TYPE", 4)
            self.set_parameter("LOG_FILE_DSRMROT", 1)
            self.set_parameter("LOG_BLK_RATEMAX", 1)
            self.reboot_sitl()
            # First log created here, but we are in chip erase so ignored
            mavproxy.send("module load log\n")
            mavproxy.send("log erase\n")
            mavproxy.expect("Chip erase complete")

            self.wait_ready_to_arm()
            if self.is_copter() or self.is_plane():
                self.set_autodisarm_delay(0)
            self.arm_vehicle()
            self.delay_sim_time(5)
            self.disarm_vehicle()
            # First log created here
            self.delay_sim_time(2)
            self.arm_vehicle()
            self.delay_sim_time(5)
            self.disarm_vehicle()
            # Second log created here
            self.delay_sim_time(2)
            mavproxy.send("log list\n")
            mavproxy.expect("Log ([0-9]+)  numLogs ([0-9]+) lastLog ([0-9]+) size ([0-9]+)", timeout=120)
            log_num = int(mavproxy.match.group(1))
            numlogs = int(mavproxy.match.group(2))
            lastlog = int(mavproxy.match.group(3))
            size = int(mavproxy.match.group(4))
            if numlogs != 2 or log_num != 1 or size <= 0:
                raise NotAchievedException("Unexpected log information %d %d %d" % (log_num, numlogs, lastlog))
            self.progress("Log size: %d" % size)
            self.reboot_sitl()
            # This starts a new log with a time of 0, wait for arm so that we can insert the correct time
            self.wait_ready_to_arm()
            # Third log created here
            mavproxy.send("log list\n")
            mavproxy.expect("Log 1  numLogs 3 lastLog 3 size")

            # Download second and third logs
            mavproxy.send("log download 2 logs/dataflash-log-002.BIN\n")
            mavproxy.expect("Finished downloading", timeout=120)
            mavproxy.send("log download 3 logs/dataflash-log-003.BIN\n")
            mavproxy.expect("Finished downloading", timeout=120)

            # Erase the logs
            mavproxy.send("log erase\n")
            mavproxy.expect("Chip erase complete")

        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        mavproxy.send("module unload log\n")
        self.stop_mavproxy(mavproxy)
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def validate_log_file(self, logname, header_errors=0):
        """Validate the contents of a log file"""
        # read the downloaded log - it must parse without error
        class Capturing(list):
            def __enter__(self):
                self._stderr = sys.stderr
                sys.stderr = self._stringio = StringIO.StringIO()
                return self

            def __exit__(self, *args):
                self.extend(self._stringio.getvalue().splitlines())
                del self._stringio    # free up some memory
                sys.stderr = self._stderr

        with Capturing() as df_output:
            try:
                mlog = mavutil.mavlink_connection(logname)
                while True:
                    m = mlog.recv_match()
                    if m is None:
                        break
            except Exception as e:
                raise NotAchievedException("Error reading log file %s: %s" % (logname, str(e)))

        herrors = 0

        for msg in df_output:
            if msg.startswith("bad header") or msg.startswith("unknown msg type"):
                herrors = herrors + 1

        if herrors > header_errors:
            raise NotAchievedException("Error parsing log file %s, %d header errors" % (logname, herrors))

    def test_dataflash_erase(self):
        """Test that erasing the dataflash chip and creating a new log is error free"""
        mavproxy = self.start_mavproxy()

        ex = None
        self.context_push()
        try:
            self.set_parameter("LOG_BACKEND_TYPE", 4)
            self.reboot_sitl()
            mavproxy.send("module load log\n")
            mavproxy.send("log erase\n")
            mavproxy.expect("Chip erase complete")
            self.set_parameter("LOG_DISARMED", 1)
            self.delay_sim_time(3)
            self.set_parameter("LOG_DISARMED", 0)
            mavproxy.send("log download 1 logs/dataflash-log-erase.BIN\n")
            mavproxy.expect("Finished downloading", timeout=120)
            # read the downloaded log - it must parse without error
            self.validate_log_file("logs/dataflash-log-erase.BIN")

            self.start_subtest("Test file wrapping results in a valid file")
            # roughly 4mb
            self.set_parameter("LOG_FILE_DSRMROT", 1)
            self.set_parameter("LOG_BITMASK", 131071)
            self.wait_ready_to_arm()
            if self.is_copter() or self.is_plane():
                self.set_autodisarm_delay(0)
            self.arm_vehicle()
            self.delay_sim_time(30)
            self.disarm_vehicle()
            # roughly 4mb
            self.arm_vehicle()
            self.delay_sim_time(30)
            self.disarm_vehicle()
            # roughly 9mb, should wrap around
            self.arm_vehicle()
            self.delay_sim_time(50)
            self.disarm_vehicle()
            # make sure we have finished logging
            self.delay_sim_time(15)
            mavproxy.send("log list\n")
            try:
                mavproxy.expect("Log ([0-9]+)  numLogs ([0-9]+) lastLog ([0-9]+) size ([0-9]+)", timeout=120)
            except pexpect.TIMEOUT as e:
                if self.sitl_is_running():
                    self.progress("SITL is running")
                else:
                    self.progress("SITL is NOT running")
                raise NotAchievedException("Received %s" % str(e))
            if int(mavproxy.match.group(2)) != 3:
                raise NotAchievedException("Expected 3 logs got %s" % (mavproxy.match.group(2)))

            mavproxy.send("log download 1 logs/dataflash-log-erase2.BIN\n")
            mavproxy.expect("Finished downloading", timeout=120)
            self.validate_log_file("logs/dataflash-log-erase2.BIN", 1)

            mavproxy.send("log download latest logs/dataflash-log-erase3.BIN\n")
            mavproxy.expect("Finished downloading", timeout=120)
            self.validate_log_file("logs/dataflash-log-erase3.BIN", 1)

            # clean up
            mavproxy.send("log erase\n")
            mavproxy.expect("Chip erase complete")

            # clean up
            mavproxy.send("log erase\n")
            mavproxy.expect("Chip erase complete")

        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        mavproxy.send("module unload log\n")

        self.context_pop()
        self.reboot_sitl()

        self.stop_mavproxy(mavproxy)

        if ex is not None:
            raise ex

    def test_arm_feature(self):
        """Common feature to test."""
        # TEST ARMING/DISARM
        if self.get_parameter("ARMING_CHECK") != 1.0 and not self.is_sub():
            raise ValueError("Arming check should be 1")
        if not self.is_sub() and not self.is_tracker():
            self.set_parameter("ARMING_RUDDER", 2)  # allow arm and disarm with rudder on first tests
        if self.is_copter():
            interlock_channel = 8  # Plane got flighmode_ch on channel 8
            if not self.is_heli():  # heli don't need interlock option
                interlock_channel = 9
                self.set_parameter("RC%u_OPTION" % interlock_channel, 32)
            self.set_rc(interlock_channel, 1000)
        self.zero_throttle()
        # Disable auto disarm for next tests
        # Rover and Sub don't have auto disarm
        if self.is_copter() or self.is_plane():
            self.set_autodisarm_delay(0)
        self.start_subtest("Test normal arm and disarm features")
        self.wait_ready_to_arm()
        self.progress("default arm_vehicle() call")
        if not self.arm_vehicle():
            raise NotAchievedException("Failed to ARM")
        self.progress("default disarm_vehicle() call")
        if not self.disarm_vehicle():
            raise NotAchievedException("Failed to DISARM")
        self.progress("arm with mavproxy")
        mavproxy = self.start_mavproxy()
        if not self.mavproxy_arm_vehicle(mavproxy):
            raise NotAchievedException("Failed to ARM")
        self.progress("disarm with mavproxy")
        if not self.mavproxy_disarm_vehicle(mavproxy):
            raise NotAchievedException("Failed to DISARM")
        self.stop_mavproxy(mavproxy)

        if not self.is_sub():
            self.start_subtest("Test arm with rc input")
            self.arm_motors_with_rc_input()
            self.progress("disarm with rc input")
            if self.is_balancebot():
                self.progress("balancebot can't disarm with RC input")
                self.disarm_vehicle()
            else:
                self.disarm_motors_with_rc_input()

            self.start_subtest("Test arm and disarm with switch")
            arming_switch = 7
            self.set_parameter("RC%d_OPTION" % arming_switch, 153)
            self.set_rc(arming_switch, 1000)
            # delay so a transition is seen by the RC switch code:
            self.delay_sim_time(0.5)
            self.arm_motors_with_switch(arming_switch)
            self.disarm_motors_with_switch(arming_switch)
            self.set_rc(arming_switch, 1000)

            if self.is_copter():
                self.start_subtest("Test arming failure with throttle too high")
                self.set_rc(3, 1800)
                try:
                    if self.arm_vehicle():
                        raise NotAchievedException("Armed when throttle too high")
                except ValueError:
                    pass
                try:
                    self.arm_motors_with_rc_input()
                except NotAchievedException:
                    pass
                if self.armed():
                    raise NotAchievedException(
                        "Armed via RC when throttle too high")
                try:
                    self.arm_motors_with_switch(arming_switch)
                except NotAchievedException:
                    pass
                if self.armed():
                    raise NotAchievedException("Armed via RC when switch too high")
                self.zero_throttle()
                self.set_rc(arming_switch, 1000)

            # Sub doesn't have 'stick commands'
            self.start_subtest("Test arming failure with ARMING_RUDDER=0")
            self.set_parameter("ARMING_RUDDER", 0)
            try:
                self.arm_motors_with_rc_input()
            except NotAchievedException:
                pass
            if self.armed():
                raise NotAchievedException(
                    "Armed with rudder when ARMING_RUDDER=0")
            self.start_subtest("Test disarming failure with ARMING_RUDDER=0")
            self.arm_vehicle()
            try:
                self.disarm_motors_with_rc_input(watch_for_disabled=True)
            except NotAchievedException:
                pass
            if not self.armed():
                raise NotAchievedException(
                    "Disarmed with rudder when ARMING_RUDDER=0")
            self.disarm_vehicle()
            self.wait_heartbeat()
            self.start_subtest("Test disarming failure with ARMING_RUDDER=1")
            self.set_parameter("ARMING_RUDDER", 1)
            self.arm_vehicle()
            try:
                self.disarm_motors_with_rc_input()
            except NotAchievedException:
                pass
            if not self.armed():
                raise NotAchievedException(
                    "Disarmed with rudder with ARMING_RUDDER=1")
            self.disarm_vehicle()
            self.wait_heartbeat()
            self.set_parameter("ARMING_RUDDER", 2)

            if self.is_copter():
                self.start_subtest("Test arming failure with interlock enabled")
                self.set_rc(interlock_channel, 2000)
                try:
                    self.arm_motors_with_rc_input()
                except NotAchievedException:
                    pass
                if self.armed():
                    raise NotAchievedException(
                        "Armed with RC input when interlock enabled")
                try:
                    self.arm_motors_with_switch(arming_switch)
                except NotAchievedException:
                    pass
                if self.armed():
                    raise NotAchievedException("Armed with switch when interlock enabled")
                self.disarm_vehicle()
                self.wait_heartbeat()
                self.set_rc(arming_switch, 1000)
                self.set_rc(interlock_channel, 1000)
                if self.is_heli():
                    self.start_subtest("Test motor interlock enable can't be set while disarmed")
                    self.set_rc(interlock_channel, 2000)
                    channel_field = "servo%u_raw" % interlock_channel
                    interlock_value = self.get_parameter("SERVO%u_MIN" % interlock_channel)
                    tstart = self.get_sim_time()
                    while True:
                        if self.get_sim_time_cached() - tstart > 20:
                            self.set_rc(interlock_channel, 1000)
                            break # success!
                        m = self.mav.recv_match(type='SERVO_OUTPUT_RAW',
                                                blocking=True,
                                                timeout=2)
                        if m is None:
                            continue
                        m_value = getattr(m, channel_field, None)
                        if m_value is None:
                            self.set_rc(interlock_channel, 1000)
                            raise ValueError("Message has no %s field" %
                                             channel_field)
                        self.progress("SERVO_OUTPUT_RAW.%s=%u want=%u" %
                                      (channel_field, m_value, interlock_value))
                        if m_value != interlock_value:
                            self.set_rc(interlock_channel, 1000)
                            raise NotAchievedException("Motor interlock was changed while disarmed")
                self.set_rc(interlock_channel, 1000)

        self.start_subtest("Test all mode arming")
        if self.arming_test_mission() is not None:
            self.load_mission(self.arming_test_mission())

        for mode in self.mav.mode_mapping():
            self.drain_mav()
            self.start_subtest("Mode : %s" % mode)
            if mode == "FOLLOW":
                self.set_parameter("FOLL_ENABLE", 1)
            if mode in self.get_normal_armable_modes_list():
                self.progress("Armable mode : %s" % mode)
                self.change_mode(mode)
                self.arm_vehicle()
                if not self.disarm_vehicle():
                    raise NotAchievedException("Failed to DISARM")
                self.progress("PASS arm mode : %s" % mode)
            if mode in self.get_not_armable_mode_list():
                if mode in self.get_not_disarmed_settable_modes_list():
                    self.progress("Not settable mode : %s" % mode)
                    try:
                        self.change_mode(mode, timeout=15)
                    except AutoTestTimeoutException:
                        self.progress("PASS not able to set mode : %s disarmed" % mode)
                    except ValueError:
                        self.progress("PASS not able to set mode : %s disarmed" % mode)
                else:
                    self.progress("Not armable mode : %s" % mode)
                    self.change_mode(mode)
                    self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 1,  # ARM
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 want_result=mavutil.mavlink.MAV_RESULT_FAILED
                                 )
                self.progress("PASS not able to arm in mode : %s" % mode)
            if mode in self.get_position_armable_modes_list():
                self.progress("Armable mode needing Position : %s" % mode)
                self.wait_ekf_happy()
                self.change_mode(mode)
                self.arm_vehicle()
                self.wait_heartbeat()
                if not self.disarm_vehicle():
                    raise NotAchievedException("Failed to DISARM")
                self.progress("PASS arm mode : %s" % mode)
                self.progress("Not armable mode without Position : %s" % mode)
                self.wait_gps_disable()
                self.change_mode(mode)
                self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                             1,  # ARM
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             want_result=mavutil.mavlink.MAV_RESULT_FAILED
                             )
                self.set_parameter("SIM_GPS_DISABLE", 0)
                self.wait_ekf_happy() # EKF may stay unhappy for a while
                self.progress("PASS not able to arm without Position in mode : %s" % mode)
            if mode in self.get_no_position_not_settable_modes_list():
                self.progress("Setting mode need Position : %s" % mode)
                self.wait_ekf_happy()
                self.wait_gps_disable()
                try:
                    self.change_mode(mode, timeout=15)
                except AutoTestTimeoutException:
                    self.set_parameter("SIM_GPS_DISABLE", 0)
                    self.progress("PASS not able to set mode without Position : %s" % mode)
                except ValueError:
                    self.set_parameter("SIM_GPS_DISABLE", 0)
                    self.progress("PASS not able to set mode without Position : %s" % mode)
            if mode == "FOLLOW":
                self.set_parameter("FOLL_ENABLE", 0)
        self.change_mode(self.default_mode())
        if self.armed():
            if not self.disarm_vehicle():
                raise NotAchievedException("Failed to DISARM")

        # we should find at least one Armed event and one disarmed
        # event, and at least one ARM message for arm and disarm
        self.progress("Checking for an arm event")
        dfreader = self.dfreader_for_current_onboard_log()
        m = dfreader.recv_match(type="EV", condition="EV.Id==10") # armed
        if m is None:
            raise NotAchievedException("Did not find an Armed EV message")

        self.progress("Checking for a disarm event")
        dfreader = self.dfreader_for_current_onboard_log()
        m = dfreader.recv_match(type="EV", condition="EV.Id==11") # disarmed
        if m is None:
            raise NotAchievedException("Did not find a disarmed EV message")

        self.progress("Checking for ARM.ArmState==1")
        dfreader = self.dfreader_for_current_onboard_log()
        m = dfreader.recv_match(type="ARM", condition="ARM.ArmState==1")
        if m is None:
            raise NotAchievedException("Did not find a armed ARM message")

        self.progress("Checking for ARM.ArmState==0")
        dfreader = self.dfreader_for_current_onboard_log()
        m = dfreader.recv_match(type="ARM", condition="ARM.ArmState==0")
        if m is None:
            raise NotAchievedException("Did not find a disarmed ARM message")

        self.progress("ALL PASS")
    # TODO : Test arming magic;

    def get_message_rate(self, victim_message, timeout=10, mav=None):
        if mav is None:
            mav = self.mav
        tstart = self.get_sim_time()
        count = 0
        while self.get_sim_time_cached() < tstart + timeout:
            m = mav.recv_match(
                type=victim_message,
                blocking=True,
                timeout=0.1
            )
            if m is not None:
                count += 1
            if mav != self.mav:
                self.drain_mav(self.mav)

        time_delta = self.get_sim_time_cached() - tstart
        self.progress("%s count after %f seconds: %u" %
                      (victim_message, time_delta, count))
        return count/time_delta

    def rate_to_interval_us(self, rate):
        return 1/float(rate)*1000000.0

    def set_message_rate_hz(self, id, rate_hz, mav=None):
        '''set a message rate in Hz; 0 for original, -1 to disable'''
        if type(id) == str:
            id = eval("mavutil.mavlink.MAVLINK_MSG_ID_%s" % id)
        if rate_hz == 0 or rate_hz == -1:
            set_interval = rate_hz
        else:
            set_interval = self.rate_to_interval_us(rate_hz)
        self.run_cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                     id,
                     set_interval,
                     0,
                     0,
                     0,
                     0,
                     0,
                     mav=mav)

    def send_get_message_interval(self, victim_message, mav=None):
        if mav is None:
            mav = self.mav
        if type(victim_message) == str:
            victim_message = eval("mavutil.mavlink.MAVLINK_MSG_ID_%s" % victim_message)
        mav.mav.command_long_send(
            1,
            1,
            mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
            1,  # confirmation
            float(victim_message),
            0,
            0,
            0,
            0,
            0,
            0)

    def test_rate(self,
                  desc,
                  in_rate,
                  expected_rate
                  , mav=None,
                  victim_message="VFR_HUD",
                  ndigits=0,
                  message_rate_sample_period=10):
        if mav is None:
            mav = self.mav

        self.progress("###### %s" % desc)
        self.progress("Setting rate to %f" % round(in_rate, ndigits=ndigits))

        self.set_message_rate_hz(victim_message, in_rate, mav=mav)

        new_measured_rate = self.get_message_rate(victim_message, timeout=message_rate_sample_period, mav=mav)
        self.progress(
            "Measured rate: %f (want %f)" %
            (round(new_measured_rate, ndigits=ndigits),
             round(expected_rate, ndigits=ndigits))
        )
        if round(new_measured_rate, ndigits=ndigits) != round(expected_rate, ndigits=ndigits):
            raise NotAchievedException("Rate not achieved (got %f want %f)" %
                                       (round(new_measured_rate, ndigits), round(expected_rate, ndigits)))

        # make sure get_message_interval works:
        self.send_get_message_interval(victim_message, mav=mav)

        m = mav.recv_match(type='MESSAGE_INTERVAL', blocking=True)

        if in_rate == 0:
            want = self.rate_to_interval_us(expected_rate)
        elif in_rate == -1:
            want = in_rate
        else:
            want = self.rate_to_interval_us(in_rate)

        if m.interval_us != want:
            raise NotAchievedException("Did not read same interval back from autopilot: want=%d got=%d)" %
                                       (want, m.interval_us))
        m = mav.recv_match(type='COMMAND_ACK', blocking=True)
        if m.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise NotAchievedException("Expected ACCEPTED for reading message interval")

    def test_set_message_interval(self):
        self.start_subtest('Basic tests')
        self.test_set_message_interval_basic()
        self.start_subtest('Many-message tests')
        self.test_set_message_interval_many()

    def test_set_message_interval_many(self):
        messages = [
            'CAMERA_FEEDBACK',
            'RAW_IMU',
            'ATTITUDE',
        ]
        ex = None
        try:
            rate = 5
            for message in messages:
                self.set_message_rate_hz(message, rate)
            for message in messages:
                self.assert_message_rate_hz(message, rate)
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        # reset message rates to default:
        for message in messages:
            self.set_message_rate_hz(message, -1)

        if ex is not None:
            raise ex

    def assert_message_rate_hz(self, message, want_rate, sample_period=20, ndigits=0, mav=None):
        if mav is None:
            mav = self.mav
        self.drain_mav(mav)
        rate = round(self.get_message_rate(message, sample_period, mav=mav), ndigits=ndigits)
        self.progress("%s: Want=%f got=%f" % (message, round(want_rate, ndigits=ndigits), round(rate, ndigits=ndigits)))
        if rate != want_rate:
            raise NotAchievedException("Did not get expected rate (want=%f got=%f)" % (want_rate, rate))

    def test_set_message_interval_basic(self):
        ex = None
        try:
            rate = round(self.get_message_rate("VFR_HUD", 20))
            self.progress("Initial rate: %u" % rate)

            self.test_rate("Test set to %u" % (rate/2,), rate/2, rate/2, victim_message="VFR_HUD")
            # this assumes the streamrates have not been played with:
            self.test_rate("Resetting original rate using 0-value", 0, rate)
            self.test_rate("Disabling using -1-value", -1, 0)
            self.test_rate("Resetting original rate", rate, rate)

            self.progress("try getting a message which is not ordinarily streamed out")
            rate = round(self.get_message_rate("CAMERA_FEEDBACK", 20))
            if rate != 0:
                raise PreconditionFailedException("Already getting CAMERA_FEEDBACK")
            self.progress("try various message rates")
            for want_rate in range(5, 14):
                self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_FEEDBACK,
                                         want_rate)
                self.assert_message_rate_hz('CAMERA_FEEDBACK', want_rate)

            self.progress("try at the main loop rate")
            # have to reset the speedup as MAVProxy can't keep up otherwise
            old_speedup = self.get_parameter("SIM_SPEEDUP")
            self.set_parameter("SIM_SPEEDUP", 1.0)
            # ArduPilot currently limits message rate to 80% of main loop rate:
            want_rate = self.get_parameter("SCHED_LOOP_RATE") * 0.8
            self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_FEEDBACK,
                                     want_rate)
            rate = round(self.get_message_rate("CAMERA_FEEDBACK", 20))
            self.set_parameter("SIM_SPEEDUP", old_speedup)
            self.progress("Want=%f got=%f" % (want_rate, rate))
            if abs(rate - want_rate) > 2:
                raise NotAchievedException("Did not get expected rate")

            self.drain_mav()

            non_existant_id = 145
            self.send_get_message_interval(non_existant_id)
            m = self.mav.recv_match(type='MESSAGE_INTERVAL', blocking=True)
            if m.interval_us != 0:
                raise NotAchievedException("Supposed to get 0 back for unsupported stream")
            m = self.mav.recv_match(type='COMMAND_ACK', blocking=True)
            if m.result != mavutil.mavlink.MAV_RESULT_FAILED:
                raise NotAchievedException("Getting rate of unsupported message is a failure")

        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.progress("Resetting CAMERA_FEEDBACK rate to zero")
        self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_FEEDBACK, -1)

        if ex is not None:
            raise ex

    def send_poll_message(self, message_id, target_sysid=None, target_compid=None):
        if type(message_id) == str:
            message_id = eval("mavutil.mavlink.MAVLINK_MSG_ID_%s" % message_id)
        self.send_cmd(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                      message_id,
                      0,
                      0,
                      0,
                      0,
                      0,
                      0,
                      target_sysid=target_sysid,
                      target_compid=target_compid)

    def poll_message(self, message_id, timeout=10):
        if type(message_id) == str:
            message_id = eval("mavutil.mavlink.MAVLINK_MSG_ID_%s" % message_id)
        self.drain_mav()
        tstart = self.get_sim_time() # required for timeout in run_cmd_get_ack to work
        self.send_poll_message(message_id)
        self.run_cmd_get_ack(
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
            timeout,
            quiet=False
        )
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not receive polled message")
            m = self.mav.recv_match(blocking=True,
                                    timeout=0.1)
            if m is None:
                continue
            if m.id != message_id:
                continue
            return m

    def get_messages_frame(self, msg_names):
        '''try to get a "frame" of named messages - a set of messages as close
        in time as possible'''
        msgs = {}

        def get_msgs(mav, m):
            t = m.get_type()
            if t in msg_names:
                msgs[t] = m
        self.do_timesync_roundtrip()
        self.install_message_hook(get_msgs)
        for msg_name in msg_names:
            self.send_poll_message(msg_name)
        while True:
            self.mav.recv_match(blocking=True)
            if len(msgs.keys()) == len(msg_names):
                break

        self.remove_message_hook(get_msgs)

        return msgs

    def test_request_message(self, timeout=60):
        rate = round(self.get_message_rate("CAMERA_FEEDBACK", 10))
        if rate != 0:
            raise PreconditionFailedException("Receiving camera feedback")
        self.poll_message("CAMERA_FEEDBACK")

    def clear_mission(self, mission_type, target_system=1, target_component=1):
        '''clear mision_type from autopilot.  Note that this does NOT actually
        send a MISSION_CLEAR_ALL message
        '''
        if mission_type == mavutil.mavlink.MAV_MISSION_TYPE_ALL:
            # recurse
            if not self.is_tracker() and not self.is_plane():
                self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
            self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            if not self.is_sub() and not self.is_tracker():
                self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.last_wp_load = time.time()
            return

        self.mav.mav.mission_count_send(target_system,
                                        target_component,
                                        0,
                                        mission_type)
        m = self.assert_receive_message('MISSION_ACK', timeout=5)
        if m.target_system != self.mav.mav.srcSystem:
            raise NotAchievedException("ACK not targetted at correct system want=%u got=%u" %
                                       (self.mav.mav.srcSystem, m.target_system))
        if m.target_component != self.mav.mav.srcComponent:
            raise NotAchievedException("ACK not targetted at correct component want=%u got=%u" %
                                       (self.mav.mav.srcComponent, m.target_component))
        if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise NotAchievedException("Expected MAV_MISSION_ACCEPTED got %s" %
                                       (mavutil.mavlink.enums["MAV_MISSION_RESULT"][m.type].name,))

        if mission_type == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
            self.last_wp_load = time.time()

    def clear_fence_using_mavproxy(self, mavproxy, timeout=10):
        mavproxy.send("fence clear\n")
        tstart = self.get_sim_time_cached()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise AutoTestTimeoutException("FENCE_TOTAL did not go to zero")
            if self.get_parameter("FENCE_TOTAL") == 0:
                break

    def clear_fence(self):
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)

    def test_config_error_loop(self):
        '''test the sensor config error loop works and that parameter sets are persistent'''
        parameter_name = "SERVO8_MIN"
        old_parameter_value = self.get_parameter(parameter_name)
        old_sim_baro_count = self.get_parameter("SIM_BARO_COUNT")
        new_parameter_value = old_parameter_value + 5
        ex = None
        try:
            self.set_parameter("STAT_BOOTCNT", 0)
            self.set_parameter("SIM_BARO_COUNT", -1)

            if self.is_tracker():
                # starts armed...
                self.progress("Disarming tracker")
                self.disarm_vehicle(force=True)

            self.reboot_sitl(required_bootcount=1)
            self.progress("Waiting for 'Config error'")
            # SYSTEM_TIME not sent in config error loop:
            self.wait_statustext("Config error", wallclock_timeout=True)
            self.progress("Setting %s to %f" % (parameter_name, new_parameter_value))
            self.set_parameter(parameter_name, new_parameter_value)
        except Exception as e:
            ex = e

        self.progress("Resetting SIM_BARO_COUNT")
        self.set_parameter("SIM_BARO_COUNT", old_sim_baro_count)

        if self.is_tracker():
            # starts armed...
            self.progress("Disarming tracker")
            self.disarm_vehicle(force=True)

        self.progress("Calling reboot-sitl ")
        self.reboot_sitl(required_bootcount=2)

        if ex is not None:
            raise ex

        if self.get_parameter(parameter_name) != new_parameter_value:
            raise NotAchievedException("Parameter value did not stick")

    def test_initial_mode(self):
        if self.is_copter():
            init_mode = (9, "LAND")
        if self.is_rover():
            init_mode = (4, "HOLD")
        if self.is_plane():
            init_mode = (13, "TAKEOFF")
        if self.is_tracker():
            init_mode = (1, "STOP")
        if self.is_sub():
            return # NOT Supported yet
        self.context_push()
        self.set_parameter("SIM_RC_FAIL", 1)
        self.progress("Setting INITIAL_MODE to %s" % init_mode[1])
        self.set_parameter("INITIAL_MODE", init_mode[0])
        self.reboot_sitl()
        self.wait_mode(init_mode[1])
        self.progress("Testing back mode switch")
        self.set_parameter("SIM_RC_FAIL", 0)
        self.wait_for_mode_switch_poll()
        self.context_pop()
        self.reboot_sitl()

    def test_gripper(self):
        self.context_push()
        self.set_parameters({
            "GRIP_ENABLE": 1,
            "GRIP_GRAB": 2000,
            "GRIP_RELEASE": 1000,
            "GRIP_TYPE": 1,
            "SIM_GRPS_ENABLE": 1,
            "SIM_GRPS_PIN": 8,
            "SERVO8_FUNCTION": 28,
            "SERVO8_MIN": 1000,
            "SERVO8_MAX": 2000,
            "SERVO9_MIN": 1000,
            "SERVO9_MAX": 2000,
            "RC9_OPTION": 19,
        })
        self.set_rc(9, 1500)
        self.reboot_sitl()
        self.progress("Waiting for ready to arm")
        self.wait_ready_to_arm()
        self.progress("Test gripper with RC9_OPTION")
        self.progress("Releasing load")
        # non strict string matching because of catching text issue....
        self.context_collect('STATUSTEXT')
        self.set_rc(9, 1000)
        self.wait_text("Gripper load releas", check_context=True)
        self.progress("Grabbing load")
        self.set_rc(9, 2000)
        self.wait_text("Gripper load grabb", check_context=True)
        self.context_clear_collection('STATUSTEXT')
        self.progress("Releasing load")
        self.set_rc(9, 1000)
        self.wait_text("Gripper load releas", check_context=True)
        self.progress("Grabbing load")
        self.set_rc(9, 2000)
        self.wait_text("Gripper load grabb", check_context=True)
        self.progress("Test gripper with Mavlink cmd")
        self.progress("Releasing load")
        self.wait_text("Gripper load releas",
                       the_function=lambda: self.mav.mav.command_long_send(1,
                                                                           1,
                                                                           mavutil.mavlink.MAV_CMD_DO_GRIPPER,
                                                                           0,
                                                                           1,
                                                                           mavutil.mavlink.GRIPPER_ACTION_RELEASE,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           ))
        self.progress("Grabbing load")
        self.wait_text("Gripper load grabb",
                       the_function=lambda: self.mav.mav.command_long_send(1,
                                                                           1,
                                                                           mavutil.mavlink.MAV_CMD_DO_GRIPPER,
                                                                           0,
                                                                           1,
                                                                           mavutil.mavlink.GRIPPER_ACTION_GRAB,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           ))
        self.progress("Releasing load")
        self.wait_text("Gripper load releas",
                       the_function=lambda: self.mav.mav.command_long_send(1,
                                                                           1,
                                                                           mavutil.mavlink.MAV_CMD_DO_GRIPPER,
                                                                           0,
                                                                           1,
                                                                           mavutil.mavlink.GRIPPER_ACTION_RELEASE,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           ))
        self.progress("Grabbing load")
        self.wait_text("Gripper load grabb",
                       the_function=lambda: self.mav.mav.command_long_send(1,
                                                                           1,
                                                                           mavutil.mavlink.MAV_CMD_DO_GRIPPER,
                                                                           0,
                                                                           1,
                                                                           mavutil.mavlink.GRIPPER_ACTION_GRAB,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           ))
        self.context_pop()
        self.reboot_sitl()

    def test_set_position_global_int(self, timeout=100):
        """Test set position message in guided mode."""
        # Disable heading and yaw test on rover type

        if self.is_rover():
            test_alt = False
            test_heading = False
            test_yaw_rate = False
        else:
            test_alt = True
            test_heading = True
            test_yaw_rate = True

        # we must start mavproxy here as otherwise we can't get the
        # terrain database tiles - this leads to random failures in
        # CI!
        mavproxy = self.start_mavproxy()

        self.set_parameter("FS_GCS_ENABLE", 0)
        self.change_mode("GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        if self.is_copter() or self.is_heli():
            self.user_takeoff(alt_min=50)

        targetpos = self.mav.location()
        wp_accuracy = None
        if self.is_copter() or self.is_heli():
            wp_accuracy = self.get_parameter("WPNAV_RADIUS", attempts=2)
            wp_accuracy = wp_accuracy * 0.01  # cm to m
        if self.is_plane() or self.is_rover():
            wp_accuracy = self.get_parameter("WP_RADIUS", attempts=2)
        if wp_accuracy is None:
            raise ValueError()

        def to_alt_frame(alt, mav_frame):
            if mav_frame in ["MAV_FRAME_GLOBAL_RELATIVE_ALT",
                             "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT",
                             "MAV_FRAME_GLOBAL_TERRAIN_ALT",
                             "MAV_FRAME_GLOBAL_TERRAIN_ALT_INT"]:
                home = self.home_position_as_mav_location()
                return alt - home.alt
            else:
                return alt

        def send_target_position(lat, lng, alt, mav_frame):
            self.mav.mav.set_position_target_global_int_send(
                0,  # timestamp
                self.sysid_thismav(),  # target system_id
                1,  # target component id
                mav_frame,
                MAV_POS_TARGET_TYPE_MASK.VEL_IGNORE |
                MAV_POS_TARGET_TYPE_MASK.ACC_IGNORE |
                MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE |
                MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE,
                int(lat * 1.0e7),  # lat
                int(lng * 1.0e7),  # lon
                alt,  # alt
                0,  # vx
                0,  # vy
                0,  # vz
                0,  # afx
                0,  # afy
                0,  # afz
                0,  # yaw
                0,  # yawrate
            )

        for frame in MAV_FRAMES_TO_TEST:
            frame_name = mavutil.mavlink.enums["MAV_FRAME"][frame].name
            self.start_subtest("Testing Set Position in %s" % frame_name)
            self.start_subtest("Changing Latitude")
            targetpos.lat += 0.0001
            if test_alt:
                targetpos.alt += 5
            send_target_position(targetpos.lat, targetpos.lng, to_alt_frame(targetpos.alt, frame_name), frame)
            self.wait_location(targetpos, accuracy=wp_accuracy, timeout=timeout,
                               target_altitude=(targetpos.alt if test_alt else None),
                               height_accuracy=2, minimum_duration=2)

            self.start_subtest("Changing Longitude")
            targetpos.lng += 0.0001
            if test_alt:
                targetpos.alt -= 5
            send_target_position(targetpos.lat, targetpos.lng, to_alt_frame(targetpos.alt, frame_name), frame)
            self.wait_location(targetpos, accuracy=wp_accuracy, timeout=timeout,
                               target_altitude=(targetpos.alt if test_alt else None),
                               height_accuracy=2, minimum_duration=2)

            self.start_subtest("Revert Latitude")
            targetpos.lat -= 0.0001
            if test_alt:
                targetpos.alt += 5
            send_target_position(targetpos.lat, targetpos.lng, to_alt_frame(targetpos.alt, frame_name), frame)
            self.wait_location(targetpos, accuracy=wp_accuracy, timeout=timeout,
                               target_altitude=(targetpos.alt if test_alt else None),
                               height_accuracy=2, minimum_duration=2)

            self.start_subtest("Revert Longitude")
            targetpos.lng -= 0.0001
            if test_alt:
                targetpos.alt -= 5
            send_target_position(targetpos.lat, targetpos.lng, to_alt_frame(targetpos.alt, frame_name), frame)
            self.wait_location(targetpos, accuracy=wp_accuracy, timeout=timeout,
                               target_altitude=(targetpos.alt if test_alt else None),
                               height_accuracy=2, minimum_duration=2)

            if test_heading:
                self.start_subtest("Testing Yaw targetting in %s" % frame_name)
                self.progress("Changing Latitude and Heading")
                targetpos.lat += 0.0001
                if test_alt:
                    targetpos.alt += 5
                self.mav.mav.set_position_target_global_int_send(
                    0,  # timestamp
                    self.sysid_thismav(),  # target system_id
                    1,  # target component id
                    frame,
                    MAV_POS_TARGET_TYPE_MASK.VEL_IGNORE |
                    MAV_POS_TARGET_TYPE_MASK.ACC_IGNORE |
                    MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE,
                    int(targetpos.lat * 1.0e7),  # lat
                    int(targetpos.lng * 1.0e7),  # lon
                    to_alt_frame(targetpos.alt, frame_name),  # alt
                    0,  # vx
                    0,  # vy
                    0,  # vz
                    0,  # afx
                    0,  # afy
                    0,  # afz
                    math.radians(42),  # yaw
                    0,  # yawrate
                )
                self.wait_location(targetpos, accuracy=wp_accuracy, timeout=timeout,
                                   target_altitude=(targetpos.alt if test_alt else None),
                                   height_accuracy=2, minimum_duration=2)
                self.wait_heading(42, minimum_duration=5, timeout=timeout)

                self.start_subtest("Revert Latitude and Heading")
                targetpos.lat -= 0.0001
                if test_alt:
                    targetpos.alt -= 5
                self.mav.mav.set_position_target_global_int_send(
                    0,  # timestamp
                    self.sysid_thismav(),  # target system_id
                    1,  # target component id
                    frame,
                    MAV_POS_TARGET_TYPE_MASK.VEL_IGNORE |
                    MAV_POS_TARGET_TYPE_MASK.ACC_IGNORE |
                    MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE,
                    int(targetpos.lat * 1.0e7),  # lat
                    int(targetpos.lng * 1.0e7),  # lon
                    to_alt_frame(targetpos.alt, frame_name),  # alt
                    0,  # vx
                    0,  # vy
                    0,  # vz
                    0,  # afx
                    0,  # afy
                    0,  # afz
                    math.radians(0),  # yaw
                    0,  # yawrate
                )
                self.wait_location(targetpos, accuracy=wp_accuracy, timeout=timeout,
                                   target_altitude=(targetpos.alt if test_alt else None),
                                   height_accuracy=2, minimum_duration=2)
                self.wait_heading(0, minimum_duration=5, timeout=timeout)

            if test_yaw_rate:
                self.start_subtest("Testing Yaw Rate targetting in %s" % frame_name)

                def send_yaw_rate(rate, target=None):
                    self.mav.mav.set_position_target_global_int_send(
                        0,  # timestamp
                        self.sysid_thismav(),  # target system_id
                        1,  # target component id
                        frame,
                        MAV_POS_TARGET_TYPE_MASK.VEL_IGNORE |
                        MAV_POS_TARGET_TYPE_MASK.ACC_IGNORE |
                        MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE,
                        int(targetpos.lat * 1.0e7),  # lat
                        int(targetpos.lng * 1.0e7),  # lon
                        to_alt_frame(targetpos.alt, frame_name),  # alt
                        0,  # vx
                        0,  # vy
                        0,  # vz
                        0,  # afx
                        0,  # afy
                        0,  # afz
                        0,  # yaw
                        rate,  # yawrate in rad/s
                    )

                self.start_subtest("Changing Latitude and Yaw rate")
                target_rate = 1.0  # in rad/s
                targetpos.lat += 0.0001
                if test_alt:
                    targetpos.alt += 5
                self.wait_yaw_speed(target_rate, timeout=timeout,
                                    called_function=lambda plop, empty: send_yaw_rate(
                                        target_rate, None), minimum_duration=5)
                self.wait_location(targetpos, accuracy=wp_accuracy, timeout=timeout,
                                   target_altitude=(targetpos.alt if test_alt else None),
                                   height_accuracy=2)

                self.start_subtest("Revert Latitude and invert Yaw rate")
                target_rate = -1.0
                targetpos.lat -= 0.0001
                if test_alt:
                    targetpos.alt -= 5
                self.wait_yaw_speed(target_rate, timeout=timeout,
                                    called_function=lambda plop, empty: send_yaw_rate(
                                        target_rate, None), minimum_duration=5)
                self.wait_location(targetpos, accuracy=wp_accuracy, timeout=timeout,
                                   target_altitude=(targetpos.alt if test_alt else None),
                                   height_accuracy=2)
                self.start_subtest("Changing Yaw rate to zero")
                target_rate = 0.0
                self.wait_yaw_speed(target_rate, timeout=timeout,
                                    called_function=lambda plop, empty: send_yaw_rate(
                                        target_rate, None), minimum_duration=5)

        self.stop_mavproxy(mavproxy)

        self.progress("Getting back to home and disarm")
        self.do_RTL(distance_min=0, distance_max=wp_accuracy)
        self.disarm_vehicle()

    def test_set_velocity_global_int(self, timeout=30):
        """Test set position message in guided mode."""
        # Disable heading and yaw rate test on rover type
        if self.is_rover():
            test_vz = False
            test_heading = False
            test_yaw_rate = False
        else:
            test_vz = True
            test_heading = True
            test_yaw_rate = True

        # we must start mavproxy here as otherwise we can't get the
        # terrain database tiles - this leads to random failures in
        # CI!
        mavproxy = self.start_mavproxy()

        self.set_parameter("FS_GCS_ENABLE", 0)
        self.change_mode("GUIDED")
        self.wait_ready_to_arm()
        self.arm_vehicle()

        if self.is_copter() or self.is_heli():
            self.user_takeoff(alt_min=50)

        target_speed = Vector3(1.0, 0.0, 0.0)

        wp_accuracy = None
        if self.is_copter() or self.is_heli():
            wp_accuracy = self.get_parameter("WPNAV_RADIUS", attempts=2)
            wp_accuracy = wp_accuracy * 0.01  # cm to m
        if self.is_plane() or self.is_rover():
            wp_accuracy = self.get_parameter("WP_RADIUS", attempts=2)
        if wp_accuracy is None:
            raise ValueError()

        def send_speed_vector(vector, mav_frame):
            self.mav.mav.set_position_target_global_int_send(
                0,  # timestamp
                self.sysid_thismav(),  # target system_id
                1,  # target component id
                mav_frame,
                MAV_POS_TARGET_TYPE_MASK.POS_IGNORE |
                MAV_POS_TARGET_TYPE_MASK.ACC_IGNORE |
                MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE |
                MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE,
                0,
                0,
                0,
                vector.x,  # vx
                vector.y,  # vy
                vector.z,  # vz
                0,  # afx
                0,  # afy
                0,  # afz
                0,  # yaw
                0,  # yawrate
            )

        for frame in MAV_FRAMES_TO_TEST:
            frame_name = mavutil.mavlink.enums["MAV_FRAME"][frame].name
            self.start_subtest("Testing Set Velocity in %s" % frame_name)
            self.progress("Changing Vx speed")
            self.wait_speed_vector(
                target_speed,
                timeout=timeout,
                called_function=lambda plop, empty: send_speed_vector(target_speed, frame),
                minimum_duration=2
            )

            self.start_subtest("Add Vy speed")
            target_speed.y = 1.0
            self.wait_speed_vector(
                target_speed,
                timeout=timeout,
                called_function=lambda plop, empty: send_speed_vector(target_speed, frame),
                minimum_duration=2)

            self.start_subtest("Add Vz speed")
            if test_vz:
                target_speed.z = 1.0
            else:
                target_speed.z = 0.0
            self.wait_speed_vector(
                target_speed,
                timeout=timeout,
                called_function=lambda plop, empty: send_speed_vector(target_speed, frame),
                minimum_duration=2
            )

            self.start_subtest("Invert Vz speed")
            if test_vz:
                target_speed.z = -1.0
            else:
                target_speed.z = 0.0
            self.wait_speed_vector(
                target_speed,
                timeout=timeout,
                called_function=lambda plop, empty: send_speed_vector(target_speed, frame), minimum_duration=2
            )

            self.start_subtest("Invert Vx speed")
            target_speed.x = -1.0
            self.wait_speed_vector(
                target_speed,
                timeout=timeout,
                called_function=lambda plop, empty: send_speed_vector(target_speed, frame),
                minimum_duration=2
            )

            self.start_subtest("Invert Vy speed")
            target_speed.y = -1.0
            self.wait_speed_vector(
                target_speed,
                timeout=timeout,
                called_function=lambda plop, empty: send_speed_vector(target_speed, frame),
                minimum_duration=2
            )

            self.start_subtest("Set Speed to zero")
            target_speed.x = 0.0
            target_speed.y = 0.0
            target_speed.z = 0.0
            self.wait_speed_vector(
                target_speed,
                timeout=timeout,
                called_function=lambda plop, empty: send_speed_vector(target_speed, frame),
                minimum_duration=2
            )

            if test_heading:
                self.start_subtest("Testing Yaw targetting in %s" % frame_name)

                def send_yaw_target(yaw, mav_frame):
                    self.mav.mav.set_position_target_global_int_send(
                        0,  # timestamp
                        self.sysid_thismav(),  # target system_id
                        1,  # target component id
                        mav_frame,
                        MAV_POS_TARGET_TYPE_MASK.POS_IGNORE |
                        MAV_POS_TARGET_TYPE_MASK.ACC_IGNORE |
                        MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE,
                        0,
                        0,
                        0,
                        0,  # vx
                        0,  # vy
                        0,  # vz
                        0,  # afx
                        0,  # afy
                        0,  # afz
                        math.radians(yaw),  # yaw
                        0,  # yawrate
                    )

                target_speed.x = 1.0
                target_speed.y = 1.0
                if test_vz:
                    target_speed.z = -1.0
                else:
                    target_speed.z = 0.0

                def send_yaw_target_vel(yaw, vector, mav_frame):
                    self.mav.mav.set_position_target_global_int_send(
                        0,  # timestamp
                        self.sysid_thismav(),  # target system_id
                        1,  # target component id
                        mav_frame,
                        MAV_POS_TARGET_TYPE_MASK.POS_IGNORE |
                        MAV_POS_TARGET_TYPE_MASK.ACC_IGNORE |
                        MAV_POS_TARGET_TYPE_MASK.YAW_RATE_IGNORE,
                        0,
                        0,
                        0,
                        vector.x,  # vx
                        vector.y,  # vy
                        vector.z,  # vz
                        0,  # afx
                        0,  # afy
                        0,  # afz
                        math.radians(yaw),  # yaw
                        0,  # yawrate
                    )

                self.start_subtest("Target a fixed Heading")
                target_yaw = 42.0
                self.wait_heading(target_yaw, minimum_duration=5, timeout=timeout,
                                  called_function=lambda plop, empty: send_yaw_target(target_yaw, frame))

                self.start_subtest("Set target Heading")
                target_yaw = 0.0
                self.wait_heading(target_yaw, minimum_duration=5, timeout=timeout,
                                  called_function=lambda plop, empty: send_yaw_target(target_yaw, frame))

                self.start_subtest("Add Vx, Vy, Vz speed and target a fixed Heading")
                target_yaw = 42.0
                self.wait_heading(
                    target_yaw,
                    minimum_duration=5,
                    timeout=timeout,
                    called_function=lambda p, e: send_yaw_target_vel(target_yaw,
                                                                     target_speed,
                                                                     frame)
                )
                self.wait_speed_vector(
                    target_speed,
                    called_function=lambda p, e: send_yaw_target_vel(target_yaw,
                                                                     target_speed,
                                                                     frame)
                )

                self.start_subtest("Stop Vx, Vy, Vz speed and target zero Heading")
                target_yaw = 0.0
                target_speed.x = 0.0
                target_speed.y = 0.0
                target_speed.z = 0.0
                self.wait_heading(target_yaw, minimum_duration=5, timeout=timeout,
                                  called_function=lambda plop, empty: send_yaw_target_vel(target_yaw, target_speed, frame))
                self.wait_speed_vector(
                    target_speed,
                    timeout=timeout,
                    called_function=lambda p, ee: send_yaw_target_vel(target_yaw,
                                                                      target_speed,
                                                                      frame),
                    minimum_duration=2
                )

            if test_yaw_rate:
                self.start_subtest("Testing Yaw Rate targetting in %s" % frame_name)

                def send_yaw_rate(rate, mav_frame):
                    self.mav.mav.set_position_target_global_int_send(
                        0,  # timestamp
                        self.sysid_thismav(),  # target system_id
                        1,  # target component id
                        mav_frame,
                        MAV_POS_TARGET_TYPE_MASK.POS_IGNORE |
                        MAV_POS_TARGET_TYPE_MASK.ACC_IGNORE |
                        MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE,
                        0,
                        0,
                        0,
                        0,  # vx
                        0,  # vy
                        0,  # vz
                        0,  # afx
                        0,  # afy
                        0,  # afz
                        0,  # yaw
                        rate,  # yawrate in rad/s
                    )

                target_speed.x = 1.0
                target_speed.y = 1.0
                if test_vz:
                    target_speed.z = -1.0
                else:
                    target_speed.z = 0.0

                def send_yaw_rate_vel(rate, vector, mav_frame):
                    self.mav.mav.set_position_target_global_int_send(
                        0,  # timestamp
                        self.sysid_thismav(),  # target system_id
                        1,  # target component id
                        mav_frame,
                        MAV_POS_TARGET_TYPE_MASK.POS_IGNORE |
                        MAV_POS_TARGET_TYPE_MASK.ACC_IGNORE |
                        MAV_POS_TARGET_TYPE_MASK.YAW_IGNORE,
                        0,
                        0,
                        0,
                        vector.x,  # vx
                        vector.y,  # vy
                        vector.z,  # vz
                        0,  # afx
                        0,  # afy
                        0,  # afz
                        0,  # yaw
                        rate,  # yawrate in rad/s
                    )

                self.start_subtest("Set Yaw rate")
                target_rate = 1.0
                self.wait_yaw_speed(target_rate, timeout=timeout,
                                    called_function=lambda plop, empty: send_yaw_rate(target_rate, frame), minimum_duration=2)

                self.start_subtest("Invert Yaw rate")
                target_rate = -1.0
                self.wait_yaw_speed(target_rate, timeout=timeout,
                                    called_function=lambda plop, empty: send_yaw_rate(target_rate, frame), minimum_duration=2)

                self.start_subtest("Stop Yaw rate")
                target_rate = 0.0
                self.wait_yaw_speed(target_rate, timeout=timeout,
                                    called_function=lambda plop, empty: send_yaw_rate(target_rate, frame), minimum_duration=2)

                self.start_subtest("Set Yaw Rate and Vx, Vy, Vz speed")
                target_rate = 1.0
                self.wait_yaw_speed(
                    target_rate,
                    called_function=lambda p, e: send_yaw_rate_vel(target_rate,
                                                                   target_speed,
                                                                   frame),
                    minimum_duration=2
                )
                self.wait_speed_vector(
                    target_speed,
                    timeout=timeout,
                    called_function=lambda p, e: send_yaw_rate_vel(target_rate,
                                                                   target_speed,
                                                                   frame),
                    minimum_duration=2
                )

                target_rate = -1.0
                target_speed.x = -1.0
                target_speed.y = -1.0
                if test_vz:
                    target_speed.z = 1.0
                else:
                    target_speed.z = 0.0
                self.start_subtest("Invert Vx, Vy, Vz speed")
                self.wait_yaw_speed(
                    target_rate,
                    timeout=timeout,
                    called_function=lambda p, e: send_yaw_rate_vel(target_rate,
                                                                   target_speed,
                                                                   frame),
                    minimum_duration=2
                )
                self.wait_speed_vector(
                    target_speed,
                    timeout=timeout,
                    called_function=lambda p, e: send_yaw_rate_vel(target_rate,
                                                                   target_speed,
                                                                   frame),
                    minimum_duration=2
                )

                target_rate = 0.0
                target_speed.x = 0.0
                target_speed.y = 0.0
                target_speed.z = 0.0
                self.start_subtest("Stop Yaw rate and all speed")
                self.wait_yaw_speed(
                    target_rate,
                    timeout=timeout,
                    called_function=lambda p, e: send_yaw_rate_vel(target_rate,
                                                                   target_speed,
                                                                   frame),
                    minimum_duration=2
                )
                self.wait_speed_vector(
                    target_speed,
                    timeout=timeout,
                    called_function=lambda p, e: send_yaw_rate_vel(target_rate,
                                                                   target_speed,
                                                                   frame),
                    minimum_duration=2
                )

        self.stop_mavproxy(mavproxy)

        self.progress("Getting back to home and disarm")
        self.do_RTL(distance_min=0, distance_max=wp_accuracy)
        self.disarm_vehicle()

    def is_copter(self):
        return False

    def is_sub(self):
        return False

    def is_plane(self):
        return False

    def is_rover(self):
        return False

    def is_balancebot(self):
        return False

    def is_heli(self):
        return False

    def is_tracker(self):
        return False

    def initial_mode(self):
        '''return mode vehicle should start in with no RC inputs set'''
        return None

    def initial_mode_switch_mode(self):
        '''return mode vehicle should start in with default RC inputs set'''
        return None

    def upload_fences_from_locations(self,
                                     vertex_type,
                                     list_of_list_of_locs,
                                     target_system=1,
                                     target_component=1):
        seq = 0
        items = []
        for locs in list_of_list_of_locs:
            if type(locs) == dict:
                # circular fence
                if vertex_type == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    v = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
                else:
                    v = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
                item = self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    seq, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    v,
                    0, # current
                    0, # autocontinue
                    locs["radius"], # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    int(locs["loc"].lat * 1e7), # latitude
                    int(locs["loc"].lng * 1e7), # longitude
                    33.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
                seq += 1
                items.append(item)
                continue
            count = len(locs)
            for loc in locs:
                item = self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    seq, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    vertex_type,
                    0, # current
                    0, # autocontinue
                    count, # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    int(loc.lat * 1e7), # latitude
                    int(loc.lng * 1e7), # longitude
                    33.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
                seq += 1
                items.append(item)

        self.check_fence_upload_download(items)

    def wait_for_initial_mode(self):
        '''wait until we get a heartbeat with an expected initial mode (the
one specified in the vehicle constructor)'''
        want = self.initial_mode()
        if want is None:
            return
        self.progress("Waiting for initial mode %s" % want)
        self.wait_mode(want)

    def wait_for_mode_switch_poll(self):
        '''look for a transition from boot-up-mode (e.g. the flightmode
specificied in Copter's constructor) to the one specified by the mode
switch value'''
        want = self.initial_mode_switch_mode()
        if want is None:
            return
        self.progress("Waiting for mode-switch mode %s" % want)
        self.wait_mode(want)

    def start_subtest(self, description):
        self.progress("-")
        self.progress("---------- %s  ----------" % description)
        self.progress("-")

    def start_subsubtest(self, description):
        self.progress(".")
        self.progress(".......... %s  .........." % description)
        self.progress(".")

    def end_subtest(self, description):
        '''TODO: sanity checks?'''
        pass

    def end_subsubtest(self, description):
        '''TODO: sanity checks?'''
        pass

    def test_skipped(self, test, reason):
        self.progress("##### %s is skipped: %s" % (test, reason))
        self.skip_list.append((test, reason))

    def last_onboard_log(self):
        '''return number of last onboard log'''
        mavproxy = self.start_mavproxy()
        mavproxy.send("module load log\n")
        loaded_module = False
        mavproxy.expect(["Loaded module log", "module log already loaded"])
        if mavproxy.match.group(0) == "Loaded module log":
            loaded_module = True
        mavproxy.send("log list\n")
        mavproxy.expect(["lastLog ([0-9]+)", "No logs"])
        if mavproxy.match.group(0) == "No logs":
            num_log = None
        else:
            num_log = int(mavproxy.match.group(1))
        if loaded_module:
            mavproxy.send("module unload log\n")
            mavproxy.expect("Unloaded module log")
        self.stop_mavproxy(mavproxy)
        return num_log

    def current_onboard_log_filepath(self):
        '''return filepath to currently open dataflash log.  We assume that's
        the latest log...'''
        logs = self.log_list()
        latest = logs[-1]
        return latest

    def dfreader_for_path(self, path):
        return DFReader.DFReader_binary(path,
                                        zero_time_base=True)

    def dfreader_for_current_onboard_log(self):
        return self.dfreader_for_path(self.current_onboard_log_filepath())

    def current_onboard_log_contains_message(self, messagetype):
        self.progress("Checking (%s) for (%s)" %
                      (self.current_onboard_log_filepath(), messagetype))
        dfreader = self.dfreader_for_current_onboard_log()
        m = dfreader.recv_match(type=messagetype)
        print("m=%s" % str(m))
        return m is not None

    def run_tests(self, tests):
        """Autotest vehicle in SITL."""
        if self.run_tests_called:
            raise ValueError("run_tests called twice")
        self.run_tests_called = True

        self.fail_list = []

        try:
            self.init()

            self.progress("Waiting for a heartbeat with mavlink protocol %s"
                          % self.mav.WIRE_PROTOCOL_VERSION)
            self.wait_heartbeat()
            self.wait_for_initial_mode()
            self.progress("Setting up RC parameters")
            self.set_rc_default()
            self.wait_for_mode_switch_poll()
            if not self.is_tracker(): # FIXME - more to the point, fix Tracker's mission handling
                self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_ALL)

            for test in tests:
                self.drain_mav_unparsed()
                self.run_one_test(test)

        except pexpect.TIMEOUT:
            self.progress("Failed with timeout")
            self.fail_list.append(["Failed with timeout", None, None])
            if self.logs_dir:
                if glob.glob("core*") or glob.glob("ap-*.core"):
                    self.check_logs("FRAMEWORK")

        if self.rc_thread is not None:
            self.progress("Joining RC thread")
            self.rc_thread_should_quit = True
            self.rc_thread.join()
            self.rc_thread = None
        self.close()

        if len(self.skip_list):
            self.progress("Skipped tests:")
            for skipped in self.skip_list:
                (test, reason) = skipped
                print("  %s (see %s)" % (test.name, reason))

        if len(self.fail_list):
            self.progress("Failing tests:")
            for failure in self.fail_list:
                (desc, exception, debug_filename) = failure
                print("  %s (%s) (see %s)" % (desc, exception, debug_filename))
            return False

        return True

    def dictdiff(self, dict1, dict2):
        fred = copy.copy(dict1)
        for key in dict2.keys():
            try:
                del fred[key]
            except KeyError:
                pass
        return fred

    # download parameters tries to cope with its download being
    # interrupted or broken by simply retrying the download a few
    # times.
    def download_parameters(self, target_system, target_component):
        # try a simple fetch-all:
        last_parameter_received = 0
        attempt_count = 0
        start_done = False
        # make flake8 happy:
        count = 0
        expected_count = 0
        seen_ids = {}
        self.progress("Downloading parameters")
        while True:
            now = self.get_sim_time_cached()
            if not start_done or now - last_parameter_received > 10:
                start_done = True
                if attempt_count > 3:
                    raise AutoTestTimeoutException("Failed to download parameters  (have %s/%s) (seen_ids-count=%u)" %
                                                   (str(count), str(expected_count), len(seen_ids.keys())))
                elif attempt_count != 0:
                    self.progress("Download failed; retrying")
                    self.delay_sim_time(1)
                self.drain_mav()
                self.mav.mav.param_request_list_send(target_system, target_component)
                attempt_count += 1
                count = 0
                expected_count = None
                seen_ids = {}
                id_seq = {}
            m = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=10)
            if m is None:
                raise AutoTestTimeoutException("tardy PARAM_VALUE (have %s/%s)" % (
                    str(count), str(expected_count)))
            if m.param_index == 65535:
                self.progress("volunteered parameter: %s" % str(m))
                continue
            if False:
                self.progress("  received (%4u/%4u %s=%f" %
                              (m.param_index, m.param_count, m.param_id, m.param_value))
            if m.param_index >= m.param_count:
                raise ValueError("parameter index (%u) gte parameter count (%u)" %
                                 (m.param_index, m.param_count))
            if expected_count is None:
                expected_count = m.param_count
            else:
                if m.param_count != expected_count:
                    raise ValueError("expected count changed")
            if m.param_id not in seen_ids:
                count += 1
                seen_ids[m.param_id] = m.param_value
                last_parameter_received = now
                if count == expected_count:
                    break

        self.progress("Downloaded %u parameters OK (attempt=%u)" %
                      (count, attempt_count))
        return (seen_ids, id_seq)

    def test_parameters_download(self):
        self.start_subtest("parameter download")
        target_system = self.sysid_thismav()
        target_component = 1
        self.progress("First Download:")
        (parameters, seq_id) = self.download_parameters(target_system, target_component)
        self.reboot_sitl()
        self.progress("Second download:")
        (parameters2, seq2_id) = self.download_parameters(target_system, target_component)

        delta = self.dictdiff(parameters, parameters2)
        if len(delta) != 0:
            raise ValueError("Got %u fewer parameters when downloading second time (before=%u vs after=%u) (delta=%s)" %
                             (len(delta), len(parameters), len(parameters2), str(delta.keys())))

        delta = self.dictdiff(parameters2, parameters)
        if len(delta) != 0:
            raise ValueError("Got %u extra parameters when downloading second time (before=%u vs after=%u) (delta=%s)" %
                             (len(delta), len(parameters), len(parameters2), str(delta.keys())))

        self.end_subsubtest("parameter download")

    def test_enable_parameter(self):
        self.start_subtest("enable parameters")
        target_system = 1
        target_component = 1
        parameters = self.download_parameters(target_system, target_component)
        enable_parameter = self.sample_enable_parameter()
        if enable_parameter is None:
            self.progress("Skipping enable parameter check as no enable parameter supplied")
            return
        self.set_parameter(enable_parameter, 1)
        parameters2 = self.download_parameters(target_system, target_component)
        if len(parameters) == len(parameters2):
            raise NotAchievedException("Enable parameter did not increase no of parameters downloaded")
        self.end_subsubtest("enable download")

    def test_parameters_mis_total(self):
        self.start_subsubtest("parameter mis_total")
        if self.is_tracker():
            # uses CMD_TOTAL not MIS_TOTAL, and it's in a scalr not a
            # group and it's generally all bad.
            return
        self.start_subtest("Ensure GCS is not able to set MIS_TOTAL")
        old_mt = self.get_parameter("MIS_TOTAL", attempts=20) # retries to avoid seeming race condition with MAVProxy
        ex = None
        try:
            self.set_parameter("MIS_TOTAL", 17, attempts=1)
        except ValueError as e:
            ex = e
        if ex is None:
            raise NotAchievedException("Set parameter when I shouldn't have")
        if old_mt != self.get_parameter("MIS_TOTAL"):
            raise NotAchievedException("Total has changed")

        self.start_subtest("Ensure GCS is able to set other MIS_ parameters")
        self.set_parameter("MIS_OPTIONS", 1)
        if self.get_parameter("MIS_OPTIONS") != 1:
            raise NotAchievedException("Failed to set MIS_OPTIONS")

        mavproxy = self.start_mavproxy()
        from_mavproxy = self.get_parameter_mavproxy(mavproxy, "MIS_OPTIONS")
        if from_mavproxy != 1:
            raise NotAchievedException("MAVProxy failed to get parameter")
        self.stop_mavproxy(mavproxy)

    def test_parameter_documentation(self):
        '''ensure parameter documentation is valid'''
        self.start_subsubtest("Check all parameters are documented")
        self.test_parameter_documentation_get_all_parameters()

    def test_parameters(self):
        '''general small tests for parameter system'''
        self.test_parameter_documentation()
        self.test_parameters_mis_total()
        self.test_parameters_download()

    def disabled_tests(self):
        return {}

    def test_parameter_checks_poscontrol(self, param_prefix):
        self.wait_ready_to_arm()
        self.context_push()
        self.set_parameter("%s_POSXY_P" % param_prefix, -1)
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     1,  # ARM
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=4,
                     want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.context_pop()
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     1,  # ARM
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=4,
                     want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)
        self.disarm_vehicle()

    def assert_not_receiving_message(self, message, timeout=1, mav=None):
        self.progress("making sure we're not getting %s messages" % message)
        if mav is None:
            mav = self.mav
        m = mav.recv_match(type=message, blocking=True, timeout=timeout)
        if m is not None:
            raise PreconditionFailedException("Receiving %s messags" % message)

    def test_pid_tuning(self):
        self.assert_not_receiving_message('PID_TUNING', timeout=5)
        self.set_parameter("GCS_PID_MASK", 1)
        self.progress("making sure we are now getting PID_TUNING messages")
        self.assert_receive_message('PID_TUNING', timeout=5)

    def sample_mission_filename(self):
        return "flaps.txt"

    def test_advanced_failsafe(self):
        self.context_push()
        ex = None
        try:
            self.drain_mav()
            self.assert_no_capability(mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION)
            self.set_parameter("AFS_ENABLE", 1)
            self.set_parameter("SYSID_MYGCS", self.mav.source_system)
            self.drain_mav()
            self.assert_capability(mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION)
            self.set_parameter("AFS_TERM_ACTION", 42)
            self.load_sample_mission()
            self.context_collect("STATUSTEXT")
            self.change_mode("AUTO") # must go to auto for AFS to latch on
            self.wait_statustext("AFS State: AFS_AUTO", check_context=True)
            self.change_mode("MANUAL")
            self.start_subtest("RC Failure")
            self.set_parameter("AFS_RC_FAIL_TIME", 1)
            self.set_parameter("SIM_RC_FAIL", 1)
            self.wait_statustext("Terminating due to RC failure", check_context=True)
            self.set_parameter("AFS_RC_FAIL_TIME", 0)
            self.set_parameter("SIM_RC_FAIL", 0)
            self.set_parameter("AFS_TERMINATE", 0)

            if not self.is_plane():
                # plane requires a polygon fence...
                self.start_subtest("Altitude Limit breach")
                self.set_parameter("AFS_AMSL_LIMIT", 100)
                self.do_fence_enable()
                self.wait_statustext("Terminating due to fence breach", check_context=True)
                self.set_parameter("AFS_AMSL_LIMIT", 0)
                self.set_parameter("AFS_TERMINATE", 0)
                self.do_fence_disable()

            self.start_subtest("GPS Failure")
            self.set_parameter("AFS_MAX_GPS_LOSS", 1)
            self.set_parameter("SIM_GPS_DISABLE", 1)
            self.wait_statustext("AFS State: GPS_LOSS", check_context=True)
            self.set_parameter("SIM_GPS_DISABLE", 0)
            self.set_parameter("AFS_MAX_GPS_LOSS", 0)
            self.set_parameter("AFS_TERMINATE", 0)

            self.run_cmd(
                mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
                1,  # terminate
                0,
                0,
                0,
                0,
                0,
                0,
            )
            self.wait_statustext("Terminating due to GCS request", check_context=True)

        except Exception as e:
            ex = e
        try:
            self.do_fence_disable()
        except ValueError:
            # may not actually be enabled....
            pass
        self.context_pop()
        if ex is not None:
            raise ex

    def drain_mav_seconds(self, seconds):
        tstart = self.get_sim_time_cached()
        while self.get_sim_time_cached() - tstart < seconds:
            self.drain_mav()
            self.delay_sim_time(0.5)

    def wait_gps_fix_type_gte(self, fix_type, timeout=30, message_type="GPS_RAW_INT", verbose=False):
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise AutoTestTimeoutException("Did not get good GPS lock")
            m = self.mav.recv_match(type=message_type, blocking=True, timeout=0.1)
            if verbose:
                self.progress("Received: %s" % str(m))
            if m is None:
                continue
            if m.fix_type >= fix_type:
                break

    def nmea_output(self):
        self.set_parameter("SERIAL5_PROTOCOL", 20) # serial5 is NMEA output
        self.set_parameter("GPS_TYPE2", 5) # GPS2 is NMEA
        self.customise_SITL_commandline([
            "--uartE=tcp:6735", # GPS2 is NMEA....
            "--uartF=tcpclient:127.0.0.1:6735", # serial5 spews to localhost:6735
        ])
        self.drain_mav_unparsed()
        self.wait_gps_fix_type_gte(3)
        gps1 = self.mav.recv_match(type="GPS_RAW_INT", blocking=True, timeout=10)
        self.progress("gps1=(%s)" % str(gps1))
        if gps1 is None:
            raise NotAchievedException("Did not receive GPS_RAW_INT")
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 20:
                raise NotAchievedException("NMEA output not updating?!")
            gps2 = self.mav.recv_match(type="GPS2_RAW", blocking=True, timeout=1)
            self.progress("gps2=%s" % str(gps2))
            if gps2 is None:
                continue
            if gps2.time_usec != 0:
                break
        max_distance = 1
        distance = self.get_distance_int(gps1, gps2)
        if distance > max_distance:
            raise NotAchievedException("NMEA output inaccurate (dist=%f want<%f)" %
                                       (distance, max_distance))

    def mavproxy_load_module(self, mavproxy, module):
        mavproxy.send("module load %s\n" % module)
        mavproxy.expect("Loaded module %s" % module)

    def mavproxy_unload_module(self, mavproxy, module):
        mavproxy.send("module unload %s\n" % module)
        mavproxy.expect("Unloaded module %s" % module)

    def accelcal(self):
        ex = None
        mavproxy = self.start_mavproxy()
        try:
            # setup with pre-existing accel offsets, to show that existing offsets don't
            # adversely affect a new cal
            pre_aofs = [Vector3(2.8, 1.2, 1.7),
                        Vector3(0.2, -0.9, 2.9)]
            pre_ascale = [Vector3(0.95, 1.2, 0.98),
                          Vector3(1.1, 1.0, 0.93)]
            aofs = [Vector3(0.7, -0.3, 1.8),
                    Vector3(-2.1, 1.9, 2.3)]
            ascale = [Vector3(0.98, 1.12, 1.05),
                      Vector3(1.11, 0.98, 0.96)]
            atrim = Vector3(0.05, -0.03, 0)
            pre_atrim = Vector3(-0.02, 0.04, 0)
            param_map = [("INS_ACCOFFS", "SIM_ACC1_BIAS", pre_aofs[0], aofs[0]),
                         ("INS_ACC2OFFS", "SIM_ACC2_BIAS", pre_aofs[1], aofs[1]),
                         ("INS_ACCSCAL", "SIM_ACC1_SCAL", pre_ascale[0], ascale[0]),
                         ("INS_ACC2SCAL", "SIM_ACC2_SCAL", pre_ascale[1], ascale[1]),
                         ("AHRS_TRIM", "SIM_ACC_TRIM", pre_atrim, atrim)]
            axes = ['X', 'Y', 'Z']

            # form the pre-calibration params
            initial_params = {}
            for (ins_prefix, sim_prefix, pre_value, post_value) in param_map:
                for axis in axes:
                    initial_params[ins_prefix + "_" + axis] = getattr(pre_value, axis.lower())
                    initial_params[sim_prefix + "_" + axis] = getattr(post_value, axis.lower())
            self.set_parameters(initial_params)
            self.customise_SITL_commandline(["-M", "calibration"])
            self.mavproxy_load_module(mavproxy, "sitl_calibration")
            self.mavproxy_load_module(mavproxy, "calibration")
            self.mavproxy_load_module(mavproxy, "relay")
            mavproxy.send("sitl_accelcal\n")
            mavproxy.send("accelcal\n")
            mavproxy.expect("Calibrated")
            for wanted in [
                    "level",
                    "on its LEFT side",
                    "on its RIGHT side",
                    "nose DOWN",
                    "nose UP",
                    "on its BACK",
            ]:
                timeout = 2
                mavproxy.expect("Place vehicle %s and press any key." % wanted, timeout=timeout)
                mavproxy.expect("sitl_accelcal: sending attitude, please wait..", timeout=timeout)
                mavproxy.expect("sitl_accelcal: attitude detected, please press any key..", timeout=timeout)
                mavproxy.send("\n")
            mavproxy.expect(".*Calibration successful", timeout=timeout)
            self.drain_mav()

            self.progress("Checking results")
            accuracy_pct = 0.5
            for (ins_prefix, sim_prefix, pre_value, post_value) in param_map:
                for axis in axes:
                    pname = ins_prefix+"_"+axis
                    v = self.get_parameter(pname)
                    expected_v = getattr(post_value, axis.lower())
                    if v == expected_v:
                        continue
                    error_pct = 100.0 * abs(v - expected_v) / abs(expected_v)
                    if error_pct > accuracy_pct:
                        raise NotAchievedException(
                            "Incorrect value %.6f for %s should be %.6f error %.2f%%" %
                            (v, pname, expected_v, error_pct))
                    else:
                        self.progress("Correct value %.4f for %s error %.2f%%" % (v, pname, error_pct))
        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.mavproxy_unload_module(mavproxy, "relay")
        self.mavproxy_unload_module(mavproxy, "calibration")
        self.mavproxy_unload_module(mavproxy, "sitl_calibration")
        self.stop_mavproxy(mavproxy)
        if ex is not None:
            raise ex

    def ahrstrim_preflight_cal(self):
        # setup with non-zero accel offsets
        self.set_parameters({
            "INS_ACCOFFS_X": 0.7,
            "INS_ACCOFFS_Y": -0.3,
            "INS_ACCOFFS_Z": 1.8,
            "INS_ACC2OFFS_X": -2.1,
            "INS_ACC2OFFS_Y": 1.9,
            "INS_ACC2OFFS_Z": 2.3,
            "SIM_ACC1_BIAS_X": 0.7,
            "SIM_ACC1_BIAS_Y": -0.3,
            "SIM_ACC1_BIAS_Z": 1.8,
            "SIM_ACC2_BIAS_X": -2.1,
            "SIM_ACC2_BIAS_Y": 1.9,
            "SIM_ACC2_BIAS_Z": 2.3,
            "AHRS_TRIM_X": 0.05,
            "AHRS_TRIM_Y": -0.03,
            "SIM_ACC_TRIM_X": -0.04,
            "SIM_ACC_TRIM_Y": 0.05,
        })
        expected_parms = {
            "AHRS_TRIM_X": -0.04,
            "AHRS_TRIM_Y": 0.05,
        }

        self.progress("Starting ahrstrim")
        self.drain_mav()
        self.mav.mav.command_long_send(self.sysid_thismav(), 1,
                                       mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                       0, 0, 0, 0, 2, 0, 0)
        self.wait_statustext('Trim OK')
        self.drain_mav()

        self.progress("Checking results")
        accuracy_pct = 0.2
        for (pname, expected_v) in expected_parms.items():
            v = self.get_parameter(pname)
            if v == expected_v:
                continue
            error_pct = 100.0 * abs(v - expected_v) / abs(expected_v)
            if error_pct > accuracy_pct:
                raise NotAchievedException(
                    "Incorrect value %.6f for %s should be %.6f error %.2f%%" %
                    (v, pname, expected_v, error_pct))
            self.progress("Correct value %.4f for %s error %.2f%%" %
                          (v, pname, error_pct))

    def ahrstrim_attitude_correctness(self):
        self.wait_ready_to_arm()
        HOME = self.sitl_start_location()
        for heading in 0, 90:
            self.customise_SITL_commandline([
                "--home", "%s,%s,%s,%s" % (HOME.lat, HOME.lng, HOME.alt, heading)
            ])
            for ahrs_type in [0, 2, 3]:
                self.start_subsubtest("Testing AHRS_TYPE=%u" % ahrs_type)
                self.context_push()
                self.set_parameter("AHRS_EKF_TYPE", ahrs_type)
                self.reboot_sitl()
                self.wait_prearm_sys_status_healthy()
                for (r, p) in [(0, 0), (9, 0), (2, -6), (10, 10)]:
                    self.set_parameters({
                        'AHRS_TRIM_X': math.radians(r),
                        'AHRS_TRIM_Y': math.radians(p),
                        "SIM_ACC_TRIM_X": math.radians(r),
                        "SIM_ACC_TRIM_Y": math.radians(p),
                    })
                    self.wait_attitude(desroll=0, despitch=0, timeout=120, tolerance=1.5)
                    if ahrs_type != 0:  # we don't get secondary msgs while DCM is primary
                        self.wait_attitude(desroll=0, despitch=0, message_type='AHRS2', tolerance=1, timeout=120)
                    self.wait_attitude_quaternion(desroll=0, despitch=0, tolerance=1, timeout=120)

                self.context_pop()
                self.reboot_sitl()

    def ahrstrim(self):
        self.start_subtest("Attitude Correctness")
        self.ahrstrim_attitude_correctness()
        self.start_subtest("Preflight Calibration")
        self.ahrstrim_preflight_cal()

    def test_button(self):
        self.set_parameter("SIM_PIN_MASK", 0)
        self.set_parameter("BTN_ENABLE", 1)
        self.drain_mav()
        self.do_heartbeats(force=True)
        btn = 4
        pin = 3
        self.set_parameter("BTN_PIN%u" % btn, pin, verbose=True)
        m = self.mav.recv_match(type='BUTTON_CHANGE', blocking=True, timeout=1)
        self.progress("### m: %s" % str(m))
        if m is not None:
            # should not get a button-changed event here.  The pins
            # are simulated pull-down
            raise NotAchievedException("Received BUTTON_CHANGE event")
        mask = 1 << pin
        self.set_parameter("SIM_PIN_MASK", mask)
        m = self.assert_receive_message('BUTTON_CHANGE', timeout=1, verbose=True)
        if not (m.state & mask):
            raise NotAchievedException("Bit not set in mask (got=%u want=%u)" % (m.state, mask))
        m2 = self.mav.recv_match(type='BUTTON_CHANGE', blocking=True, timeout=10)
        if m2 is None:
            raise NotAchievedException("Did not get repeat message")
        self.progress("### m2: %s" % str(m2))
        # wait for messages to stop coming:
        self.drain_mav_seconds(15)

        new_mask = 0
        self.send_set_parameter("SIM_PIN_MASK", new_mask, verbose=True)
        m3 = self.mav.recv_match(type='BUTTON_CHANGE', blocking=True, timeout=1)
        if m3 is None:
            raise NotAchievedException("Did not get 'off' message")
        self.progress("### m3: %s" % str(m3))

        if m.last_change_ms == m3.last_change_ms:
            raise NotAchievedException("last_change_ms same as first message")
        if m3.state != new_mask:
            raise NotAchievedException("Unexpected mask (want=%u got=%u)" %
                                       (new_mask, m3.state))
        self.progress("correct BUTTON_CHANGE event received")

        if self.is_tracker():
            # tracker starts armed, which is annoying
            self.progress("Skipping arm/disarm tests for tracker")
            return

        self.wait_ready_to_arm()
        self.set_parameter("BTN_FUNC%u" % btn, 153)  # ARM/DISARM
        self.set_parameter("SIM_PIN_MASK", mask)
        self.wait_armed()
        self.set_parameter("SIM_PIN_MASK", 0)
        self.wait_disarmed()

        if self.is_rover():
            self.context_push()
            # arming should be inhibited while e-STOP is in use:
            # set the function:
            self.set_parameter("BTN_FUNC%u" % btn, 31)
            # invert the sense of the pin, so eStop is asserted when pin is low:
            self.set_parameter("BTN_OPTIONS%u" % btn, 1 << 1)
            self.reboot_sitl()
            # assert the pin:
            self.set_parameter("SIM_PIN_MASK", mask)
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.disarm_vehicle()
            # de-assert the pin:
            self.set_parameter("SIM_PIN_MASK", 0)
            self.delay_sim_time(1)  # 5Hz update rate on Button library
            self.context_collect("STATUSTEXT")
            # try to arm the vehicle:
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                1,  # ARM
                0,
                0,
                0,
                0,
                0,
                0,
                want_result=mavutil.mavlink.MAV_RESULT_FAILED
            )
            self.wait_statustext("PreArm: Motors Emergency Stopped", check_context=True)
            self.reboot_sitl()
            self.delay_sim_time(10)
            self.assert_prearm_failure("Motors Emergency Stopped")
            self.context_pop()
            self.reboot_sitl()

        if self.is_rover():
            self.start_subtest("Testing using buttons for changing modes")
            self.context_push()
            if not self.mode_is('MANUAL'):
                raise NotAchievedException("Bad mode")
            self.set_parameter("BTN_FUNC%u" % btn, 53)  # steering mode
            # press button:
            self.set_parameter("SIM_PIN_MASK", mask)
            self.wait_mode('STEERING')
            # release button:
            self.set_parameter("SIM_PIN_MASK", 0)
            self.wait_mode('MANUAL')
            self.context_pop()

    def compare_number_percent(self, num1, num2, percent):
        if num1 == 0 and num2 == 0:
            return True
        if abs(num1 - num2) / max(abs(num1), abs(num2)) <= percent * 0.01:
            return True
        return False

    def bit_extract(self, number, offset, length):
        mask = 0
        for i in range(offset, offset+length):
            mask |= 1 << i
        return (number & mask) >> offset

    def tf_encode_gps_latitude(self, lat):
        value = 0
        if lat < 0:
            value = ((abs(lat)//100)*6) | 0x40000000
        else:
            value = ((abs(lat)//100)*6)
        return value

    def tf_validate_gps(self, value): # shared by proto 4 and proto 10
        self.progress("validating gps (0x%02x)" % value)
        lat = value
        gpi = self.mav.recv_match(
            type='GLOBAL_POSITION_INT',
            blocking=True,
            timeout=1
        )
        if gpi is None:
            raise NotAchievedException("Did not get GLOBAL_POSITION_INT message")
        gpi_lat = self.tf_encode_gps_latitude(gpi.lat)
        self.progress("GLOBAL_POSITION_INT lat==%f frsky==%f" % (gpi_lat, lat))
        if gpi_lat == lat:
            return True
        return False

    def tfp_prep_number(self, number, digits, power):
        res = 0
        abs_number = abs(number)
        if digits == 2 and power == 1: # number encoded on 8 bits: 7 bits for digits + 1 for 10^power
            if abs_number < 100:
                res = abs_number << 1
            elif abs_number < 1270:
                res = (round(abs_number * 0.1) << 1) | 0x1
            else: # transmit max possible value (0x7F x 10^1 = 1270)
                res = 0xFF
            if number < 0:  # if number is negative, add sign bit in front
                res |= 0x1 << 8
        elif digits == 2 and power == 2: # number encoded on 9 bits: 7 bits for digits + 2 for 10^power
            if abs_number < 100:
                res = abs_number << 2
            elif abs_number < 1000:
                res = (round(abs_number * 0.1) << 2) | 0x1
            elif abs_number < 10000:
                res = (round(abs_number * 0.01) << 2) | 0x2
            elif abs_number < 127000:
                res = (round(abs_number * 0.001) << 2) | 0x3
            else: # transmit max possible value (0x7F x 10^3 = 127000)
                res = 0x1FF
            if number < 0: # if number is negative, add sign bit in front
                res |= 0x1 << 9
        elif digits == 3 and power == 1: # number encoded on 11 bits: 10 bits for digits + 1 for 10^power
            if abs_number < 1000:
                res = abs_number << 1
            elif abs_number < 10240:
                res = (round(abs_number * 0.1) << 1) | 0x1
            else: # transmit max possible value (0x3FF x 10^1 = 10240)
                res = 0x7FF
            if number < 0: # if number is negative, add sign bit in front
                res |= 0x1 << 11
        elif digits == 3 and power == 2: # number encoded on 12 bits: 10 bits for digits + 2 for 10^power
            if abs_number < 1000:
                res = abs_number << 2
            elif abs_number < 10000:
                res = (round(abs_number * 0.1) << 2) | 0x1
            elif abs_number < 100000:
                res = (round(abs_number * 0.01) << 2) | 0x2
            elif abs_number < 1024000:
                res = (round(abs_number * 0.001) << 2) | 0x3
            else: # transmit max possible value (0x3FF x 10^3 = 127000)
                res = 0xFFF
            if number < 0: # if number is negative, add sign bit in front
                res |= 0x1 << 12
        return res

    def tfp_validate_ap_status(self, value): # 0x5001
        self.progress("validating ap_status(0x%02x)" % value)
        flight_mode = self.bit_extract(value, 0, 5) - 1 # first mode is 1 not 0 :-)
        # simple_mode = self.bit_extract(value, 5, 2)
        # is_flying = not self.bit_extract(value, 7, 1)
        # status_armed = self.bit_extract(value, 8, 1)
        # batt_failsafe = self.bit_extract(value, 9, 1)
        # ekf_failsafe = self.bit_extract(value, 10, 2)
        # imu_temp = self.bit_extract(value, 26, 6) + 19 # IMU temperature: 0 means temp =< 19, 63 means temp => 82
        heartbeat = self.wait_heartbeat()
        mav_flight_mode = heartbeat.custom_mode
        self.progress(" mode=%u heartbeat=%u" % (flight_mode, mav_flight_mode))
        if mav_flight_mode == flight_mode:
            self.progress("flight mode match")
            return True
            # FIXME: need to check other values as well
        return False

    def tfp_validate_attitude(self, value):
        self.progress("validating attitude(0x%02x)" % value)
        roll = (min(self.bit_extract(value, 0, 11), 1800) - 900) * 0.2 # roll [0,1800] ==> [-180,180]
        pitch = (min(self.bit_extract(value, 11, 10), 900) - 450) * 0.2 # pitch [0,900] ==> [-90,90]
#        rng_cm = self.bit_extract(value, 22, 10) * (10 ^ self.bit_extract(value, 21, 1)) # cm
        atti = self.mav.recv_match(
            type='ATTITUDE',
            blocking=True,
            timeout=1
        )
        if atti is None:
            raise NotAchievedException("Did not get ATTITUDE message")
        atti_roll = round(atti.roll)
        self.progress("ATTITUDE roll==%f frsky==%f" % (atti_roll, roll))
        if abs(atti_roll - roll) >= 5:
            return False
        atti_pitch = round(atti.pitch)
        self.progress("ATTITUDE pitch==%f frsky==%f" % (atti_pitch, pitch))
        if abs(atti_pitch - pitch) >= 5:
            return False
            # FIXME: need to check other values as well
        return True

    def tfp_validate_home_status(self, value):
        self.progress("validating home status(0x%02x)" % value)
#        home_dist_m = self.bit_extract(value,2,10) * (10^self.bit_extract(value,0,2))
        home_alt_dm = self.bit_extract(value, 14, 10) * (10 ^ self.bit_extract(value, 12, 2)) * 0.1 * (self.bit_extract(value, 24, 1) == 1 and -1 or 1)  # noqa
        # home_angle_d = self.bit_extract(value, 25,  7) * 3
        gpi = self.mav.recv_match(
            type='GLOBAL_POSITION_INT',
            blocking=True,
            timeout=1
        )
        if gpi is None:
            raise NotAchievedException("Did not get GLOBAL_POSITION_INT message")
        gpi_relative_alt_dm = gpi.relative_alt/100.0
        self.progress("GLOBAL_POSITION_INT rel_alt==%fm frsky_home_alt==%fm" % (gpi_relative_alt_dm, home_alt_dm))
        if abs(gpi_relative_alt_dm - home_alt_dm) < 10:
            return True
            # FIXME: need to check other values as well
        return False

    def tfp_validate_gps_status(self, value):
        self.progress("validating gps status(0x%02x)" % value)
#        num_sats = self.bit_extract(value, 0, 4)
        gps_status = self.bit_extract(value, 4, 2) + self.bit_extract(value, 14, 2)
#        gps_hdop = self.bit_extract(value, 7, 7) * (10 ^ self.bit_extract(value, 6, 1)) # dm
#        gps_alt = self.bit_extract(value, 24, 7) * (10 ^ self.bit_extract(value, 22, 2)) * (self.bit_extract(value, 31, 1) == 1 and -1 or 1) # dm  # noqa
        gri = self.mav.recv_match(
            type='GPS_RAW_INT',
            blocking=True,
            timeout=1
        )
        if gri is None:
            raise NotAchievedException("Did not get GPS_RAW_INT message")
        gri_status = gri.fix_type
        self.progress("GPS_RAW_INT fix_type==%f frsky==%f" % (gri_status, gps_status))
        if gps_status == gri_status:
            return True
            # FIXME: need to check other values as well
        return False

    def tfp_validate_vel_and_yaw(self, value): # 0x5005
        self.progress("validating vel_and_yaw(0x%02x)" % value)
        z_vel_dm_per_second = self.bit_extract(value, 1, 7) * (10 ^ self.bit_extract(value, 0, 1)) * (self.bit_extract(value, 8, 1) == 1 and -1 or 1)  # noqa
        xy_vel = self.bit_extract(value, 10, 7) * (10 ^ self.bit_extract(value, 9, 1))
        yaw = self.bit_extract(value, 17, 11) * 0.2
        gpi = self.mav.recv_match(
            type='GLOBAL_POSITION_INT',
            blocking=True,
            timeout=1
        )
        if gpi is None:
            return
        self.progress(" yaw=%u gpi=%u" % (yaw, gpi.hdg*0.01))
        self.progress(" xy_vel=%u" % xy_vel)
        self.progress(" z_vel_dm_per_second=%u" % z_vel_dm_per_second)
        if self.compare_number_percent(gpi.hdg*0.01, yaw, 10):
            self.progress("Yaw match")
            return True
        # FIXME: need to be under way to check the velocities, really....
        return False

    def tfp_validate_battery1(self, value):
        self.progress("validating battery1 (0x%02x)" % value)
        voltage = self.bit_extract(value, 0, 9)  # dV
        # current = self.bit_extract(value, 10, 7) * (10 ^ self.bit_extract(value, 9, 1))
        # mah = self.bit_extract(value, 17, 15)
        voltage = value * 0.1
        batt = self.mav.recv_match(
            type='BATTERY_STATUS',
            blocking=True,
            timeout=5,
            condition="BATTERY_STATUS.id==0"
        )
        if batt is None:
            raise NotAchievedException("Did not get BATTERY_STATUS message")
        battery_status_value = batt.voltages[0]*0.001
        self.progress("BATTERY_STATUS voltage==%f frsky==%f" % (battery_status_value, voltage))
        if abs(battery_status_value - voltage) > 0.1:
            return False
        # FIXME: need to check other values as well
        return True

    def tfp_validate_params(self, value):
        param_id = self.bit_extract(value, 24, 4)
        param_value = self.bit_extract(value, 0, 24)
        self.progress("received param (0x%02x) (id=%u value=%u)" %
                      (value, param_id, param_value))
        frame_type = param_value
        hb = self.mav.messages['HEARTBEAT']
        hb_type = hb.type
        self.progress("validate_params: HEARTBEAT type==%f frsky==%f param_id=%u" % (hb_type, frame_type, param_id))
        if param_id != 1:
            return False
        if hb_type == frame_type:
            return True
            # FIXME: need to check other values as well
        return False

    def tfp_validate_rpm(self, value):
        self.progress("validating rpm (0x%02x)" % value)
        tf_rpm = self.bit_extract(value, 0, 16)
        rpm = self.mav.recv_match(
            type='RPM',
            blocking=True,
            timeout=5
        )
        if rpm is None:
            raise NotAchievedException("Did not get RPM message")
        rpm_value = round(rpm.rpm1 * 0.1)
        self.progress("RPM rpm==%f frsky==%f" % (rpm_value, tf_rpm))
        if rpm_value != tf_rpm:
            return False
        return True

    def tfp_validate_terrain(self, value):
        self.progress("validating terrain(0x%02x)" % value)
        alt_above_terrain_dm = self.bit_extract(value, 2, 10) * (10 ^ self.bit_extract(value, 0, 2)) * 0.1 * (self.bit_extract(value, 12, 1) == 1 and -1 or 1)  # noqa
        terrain = self.mav.recv_match(
            type='TERRAIN_REPORT',
            blocking=True,
            timeout=1
        )
        if terrain is None:
            raise NotAchievedException("Did not get TERRAIN_REPORT message")
        altitude_terrain_dm = round(terrain.current_height*10)
        self.progress("TERRAIN_REPORT terrain_alt==%fdm frsky_terrain_alt==%fdm" % (altitude_terrain_dm, alt_above_terrain_dm))
        if abs(altitude_terrain_dm - alt_above_terrain_dm) < 1:
            return True
        return False

    def tfp_validate_wind(self, value):
        self.progress("validating wind(0x%02x)" % value)
        speed_m = self.bit_extract(value, 8, 7) * (10 ^ self.bit_extract(value, 7, 1)) * 0.1 # speed in m/s
        wind = self.mav.recv_match(
            type='WIND',
            blocking=True,
            timeout=1
        )
        if wind is None:
            raise NotAchievedException("Did not get WIND message")
        self.progress("WIND mav==%f frsky==%f" % (speed_m, wind.speed))
        if abs(speed_m - wind.speed) < 0.5:
            return True
        return False

    def test_frsky_passthrough_do_wants(self, frsky, wants):

        tstart = self.get_sim_time_cached()
        while len(wants):
            self.progress("Still wanting (%s)" % ",".join([("0x%02x" % x) for x in wants.keys()]))
            wants_copy = copy.copy(wants)
            self.drain_mav()
            t2 = self.get_sim_time_cached()
            if t2 - tstart > 300:
                self.progress("Failed to get frsky passthrough data")
                self.progress("Counts of sensor_id polls sent:")
                frsky.dump_sensor_id_poll_counts_as_progress_messages()
                self.progress("Counts of dataids received:")
                frsky.dump_dataid_counts_as_progress_messages()
                raise AutoTestTimeoutException("Failed to get frsky passthrough data")
            frsky.update()
            for want in wants_copy:
                data = frsky.get_data(want)
                if data is None:
                    continue
                self.progress("Checking 0x%x" % (want,))
                if wants[want](data):
                    self.progress("  Fulfilled")
                    del wants[want]

    def test_frsky_passthrough(self):
        self.set_parameter("SERIAL5_PROTOCOL", 10) # serial5 is FRSky passthrough
        self.set_parameter("RPM1_TYPE", 10) # enable RPM output
        self.customise_SITL_commandline([
            "--uartF=tcp:6735" # serial5 spews to localhost:6735
        ])
        frsky = FRSkyPassThrough(("127.0.0.1", 6735),
                                 get_time=self.get_sim_time_cached)

        # waiting until we are ready to arm should ensure our wanted
        # statustext doesn't get blatted out of the ArduPilot queue by
        # random messages.
        self.wait_ready_to_arm()

        # test we get statustext strings.  This relies on ArduPilot
        # emitting statustext strings when we fetch parameters. (or,
        # now, an updating-barometer statustext)
        tstart = self.get_sim_time()
        old_data = None
        text = ""
        self.context_collect('STATUSTEXT')
        self.run_cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                     0, # p1
                     0, # p2
                     1, # p3, baro
                     0, # p4
                     0, # p5
                     0, # p6
                     0) # p7

        received_frsky_texts = []
        last_len_received_statustexts = 0
        while True:
            self.drain_mav()
            now = self.get_sim_time_cached()
            if now - tstart > 60: # it can take a *long* time to get these messages down!
                raise NotAchievedException("Did not get statustext in time")
            frsky.update()
            data = frsky.get_data(0x5000) # no timestamping on this data, so we can't catch legitimate repeats.
            if data is None:
                continue
            # frsky sends each quartet three times; skip the suplicates.
            if old_data is not None and old_data == data:
                continue
            old_data = data
            self.progress("Got (0x%x)" % data)
            severity = 0
            last = False
            for i in 3, 2, 1, 0:
                x = (data >> i*8) & 0xff
                text += chr(x & 0x7f)
                self.progress("  x=0x%02x" % x)
                if x & 0x80:
                    severity += 1 << i
                self.progress("Text sev=%u: %s" % (severity, str(text)))
                if (x & 0x7f) == 0x00:
                    last = True
            if last:
                m = None
                text = text.rstrip("\0")
                self.progress("Received frsky text (%s)" % (text,))
                self.progress("context texts: %s" % str([x.text for x in self.context_collection('STATUSTEXT')]))
                m = self.statustext_in_collections(text)
                if m is not None:
                    want_sev = m.severity
                    if severity != want_sev:
                        raise NotAchievedException("Incorrect severity; want=%u got=%u" % (want_sev, severity))
                    self.progress("Got statustext (%s)" % m.text)
                    break
                received_frsky_texts.append((severity, text))
                text = ""
            received_statustexts = self.context_collection('STATUSTEXT')
            if len(received_statustexts) != last_len_received_statustexts:
                last_len_received_statustexts = len(received_statustexts)
                self.progress("received statustexts: %s" % str([x.text for x in received_statustexts]))
                self.progress("received frsky texts: %s" % str(received_frsky_texts))
                for (want_sev, text) in received_frsky_texts:
                    for m in received_statustexts:
                        if m.text == text:
                            if want_sev != m.severity:
                                raise NotAchievedException("Incorrect severity; want=%u got=%u" % (want_sev, severity))
                            self.progress("Got statustext (%s)" % text)
                            break
        self.context_stop_collecting('STATUSTEXT')

        self.wait_ready_to_arm()

        # we need to start the engine to get some RPM readings, we do it for plane only
        self.drain_mav_unparsed()
        # anything with a lambda in here needs a proper test written.
        # This, at least makes sure we're getting some of each
        # message.  These are ordered according to the wfq scheduler
        wants = {
            0x5000: lambda xx: True,
            0x5006: self.tfp_validate_attitude,
            0x0800: self.tf_validate_gps,
            0x5005: self.tfp_validate_vel_and_yaw,
            0x5001: self.tfp_validate_ap_status,
            0x5002: self.tfp_validate_gps_status,
            0x5004: self.tfp_validate_home_status,
            # 0x5008: lambda x : True, # no second battery, so this doesn't arrive
            0x5003: self.tfp_validate_battery1,
            0x5007: self.tfp_validate_params,
            0x500B: self.tfp_validate_terrain,
            0x500C: self.tfp_validate_wind,
        }
        self.test_frsky_passthrough_do_wants(frsky, wants)

        # now check RPM:
        if self.is_plane():
            self.set_autodisarm_delay(0)
            if not self.arm_vehicle():
                raise NotAchievedException("Failed to ARM")
            self.set_rc(3, 1050)
            wants = {
                0x500A: self.tfp_validate_rpm,
            }
            self.test_frsky_passthrough_do_wants(frsky, wants)
            self.zero_throttle()
            if not self.disarm_vehicle():
                raise NotAchievedException("Failed to DISARM")

        self.progress("Counts of sensor_id polls sent:")
        frsky.dump_sensor_id_poll_counts_as_progress_messages()
        self.progress("Counts of dataids received:")
        frsky.dump_dataid_counts_as_progress_messages()

    def decode_mavlite_param_value(self, message):
        '''returns a tuple of parameter name, value'''
        (value,) = struct.unpack("<f", message[0:4])
        name = message[4:]
        return (name, value)

    def decode_mavlite_command_ack(self, message):
        '''returns a tuple of parameter name, value'''
        (command, result) = struct.unpack("<HB", message)
        return (command, result)

    def read_message_via_mavlite(self, frsky, sport_to_mavlite):
        '''read bytes from frsky mavlite stream, trying to form up a mavlite
        message'''
        self.drain_mav(quiet=True)
        tstart = self.get_sim_time()
        while True:
            self.drain_mav(quiet=True)
            tnow = self.get_sim_time_cached()
            timeout = 30
            if self.valgrind or self.callgrind:
                timeout *= 10
            if tnow - tstart > timeout:
                raise NotAchievedException("Did not get parameter via mavlite")
            frsky.update()
            if sport_to_mavlite.state == sport_to_mavlite.state_MESSAGE_RECEIVED:
                message = sport_to_mavlite.get_message()
                sport_to_mavlite.reset()
#                self.progress("############ message received (type=%u)" % message.msgid)
                return message

    def read_parameter_via_mavlite(self, frsky, sport_to_mavlite, name):
        self.drain_mav(quiet=True)
        tstart = self.get_sim_time()
        while True:
            tnow = self.get_sim_time_cached()
            if tnow - tstart > 30:
                raise NotAchievedException("Did not get parameter via mavlite")
            message = self.read_message_via_mavlite(frsky, sport_to_mavlite)
            if message.msgid != mavutil.mavlink.MAVLINK_MSG_ID_PARAM_VALUE:
                raise NotAchievedException("Unexpected msgid %u received" % message.msgid)
            (got_name, value) = self.decode_mavlite_param_value(message.body)
            #                self.progress("Received parameter: %s=%f" % (name, value))
            got_name = got_name.decode('ascii')
            if got_name != name:
                raise NotAchievedException("Incorrect name received (want=%s) (got=%s)" % (name, got_name))
            return value

    def get_parameter_via_mavlite(self, frsky, sport_to_mavlite, name):
        # self.progress("########## Sending request")
        frsky.send_mavlite_param_request_read(name)
        return self.read_parameter_via_mavlite(frsky, sport_to_mavlite, name)

    def set_parameter_via_mavlite(self, frsky, sport_to_mavlite, name, value):
        # self.progress("########## Sending request")
        frsky.send_mavlite_param_set(name, value)
        # new value is echoed back immediately:
        got_val = self.read_parameter_via_mavlite(frsky, sport_to_mavlite, name)
        if abs(got_val - value) > 0.00001:
            raise NotAchievedException("Returned value not same as set value (want=%f got=%f)" % (value, got_val))

    def run_cmd_via_mavlite(self,
                            frsky,
                            sport_to_mavlite,
                            command,
                            p1=None,
                            p2=None,
                            p3=None,
                            p4=None,
                            p5=None,
                            p6=None,
                            p7=None,
                            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED):
        frsky.send_mavlite_command_long(
            command,
            p1=p1,
            p2=p2,
            p3=p3,
            p4=p4,
            p5=p5,
            p6=p6,
            p7=p7,
        )
        self.run_cmd_via_mavlite_get_ack(
            frsky,
            sport_to_mavlite,
            command,
            want_result
        )

    def run_cmd_via_mavlite_get_ack(self, frsky, sport_to_mavlite, command, want_result):
        '''expect and read a command-ack from frsky sport passthrough'''
        msg = self.read_message_via_mavlite(frsky, sport_to_mavlite)
        if msg.msgid != mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_ACK:
            raise NotAchievedException("Expected a command-ack, got a %u" % msg.msgid)
        (got_command, got_result) = self.decode_mavlite_command_ack(msg.body)
        if got_command != command:
            raise NotAchievedException(
                "Did not receive expected command in command_ack; want=%u got=%u" %
                (command, got_command))
        if got_result != want_result:
            raise NotAchievedException(
                "Did not receive expected result in command_ack; want=%u got=%u" %
                (want_result, got_result))

    def test_frsky_mavlite(self):
        self.set_parameter("SERIAL5_PROTOCOL", 10) # serial5 is FRSky passthrough
        self.customise_SITL_commandline([
            "--uartF=tcp:6735" # serial5 spews to localhost:6735
        ])
        frsky = FRSkyPassThrough(("127.0.0.1", 6735))
        frsky.connect()

        sport_to_mavlite = SPortToMAVlite()
        frsky.data_downlink_handler = sport_to_mavlite.downlink_handler

        self.start_subtest("Get parameter via MAVlite")
        param_name = "STAB_PITCH_DOWN"  # FIXME: want common across vehicles
        set_value = 97.21
        self.set_parameter(param_name, set_value)  # DO NOT FLY
        got_value = self.get_parameter_via_mavlite(frsky,
                                                   sport_to_mavlite,
                                                   param_name)
        if abs(got_value - set_value) > 0.00001:
            raise NotAchievedException("Incorrect value retrieved via mavlite (want=%f got=%f)" % (set_value, got_value))
        self.progress("Got value OK")
        self.end_subtest("Get parameter via MAVlite")

        self.start_subtest("Set parameter via MAVlite")
        param_name = "STAB_PITCH_DOWN"  # FIXME: want common across vehicles
        set_value = 91.67
#        frsky.verbose = True
        self.set_parameter_via_mavlite(frsky, sport_to_mavlite, param_name, set_value)  # DO NOT FLY
        got_value = self.get_parameter(param_name)
        if abs(got_value - set_value) > 0.00001:
            raise NotAchievedException("Incorrect value retrieved via mavlink (want=%f got=%f)" % (set_value, got_value))
        self.progress("Set value OK")
        self.end_subtest("Set parameter via MAVlite")

        self.start_subtest("Calibrate Baro via MAVLite")
        self.context_push()
        self.context_collect("STATUSTEXT")
        self.run_cmd_via_mavlite(
            frsky,
            sport_to_mavlite,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p1=0,
            p2=0,
            p3=1.0,
        )
        self.wait_statustext("Updating barometer calibration", check_context=True)
        self.context_pop()
        self.end_subtest("Calibrate Baro via MAVLite")

        self.start_subtest("Change mode via MAVLite")
        #  FIXME: currently plane-specific
        self.run_cmd_via_mavlite(
            frsky,
            sport_to_mavlite,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.PLANE_MODE_MANUAL,
        )
        self.wait_mode("MANUAL")
        self.run_cmd_via_mavlite(
            frsky,
            sport_to_mavlite,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            p1=mavutil.mavlink.PLANE_MODE_FLY_BY_WIRE_A,
        )
        self.wait_mode("FBWA")
        self.end_subtest("Change mode via MAVLite")

        self.start_subtest("Enable fence via MAVlite")
        #  Fence can be enabled using MAV_CMD
        self.run_cmd_via_mavlite(
            frsky,
            sport_to_mavlite,
            mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,
            p1=1,
            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
        )
        self.end_subtest("Enable fence via MAVlite")

    def tfs_validate_gps_alt(self, value):
        self.progress("validating gps altitude (0x%02x)" % value)
        alt_m = value * 0.01 # cm -> m
        gpi = self.mav.recv_match(
            type='GLOBAL_POSITION_INT',
            blocking=True,
            timeout=1
        )
        if gpi is None:
            raise NotAchievedException("Did not get GLOBAL_POSITION_INT message")
        gpi_alt_m = round(gpi.alt * 0.001) # mm-> m
        self.progress("GLOBAL_POSITION_INT alt==%f frsky==%f" % (gpi_alt_m, alt_m))
        if self.compare_number_percent(gpi_alt_m, alt_m, 10):
            return True
        return False

    def tfs_validate_baro_alt(self, value):
        self.progress("validating baro altitude (0x%02x)" % value)
        alt_m = value * 0.01 # cm -> m
        gpi = self.mav.recv_match(
            type='GLOBAL_POSITION_INT',
            blocking=True,
            timeout=1
        )
        if gpi is None:
            raise NotAchievedException("Did not get GLOBAL_POSITION_INT message")
        gpi_alt_m = round(gpi.relative_alt * 0.001) # mm -> m
        self.progress("GLOBAL_POSITION_INT relative_alt==%f frsky==%f" % (gpi_alt_m, alt_m))
        if abs(gpi_alt_m - alt_m) < 1:
            return True
        return False

    def tfs_validate_gps_speed(self, value):
        self.progress("validating gps speed (0x%02x)" % value)
        speed_ms = value * 0.001 # mm/s -> m/s
        vfr_hud = self.mav.recv_match(
            type='VFR_HUD',
            blocking=True,
            timeout=1
        )
        if vfr_hud is None:
            raise NotAchievedException("Did not get VFR_HUD message")
        vfr_hud_speed_ms = round(vfr_hud.groundspeed)
        self.progress("VFR_HUD groundspeed==%f frsky==%f" % (vfr_hud_speed_ms, speed_ms))
        if self.compare_number_percent(vfr_hud_speed_ms, speed_ms, 10):
            return True
        return False

    def tfs_validate_yaw(self, value):
        self.progress("validating yaw (0x%02x)" % value)
        yaw_deg = value * 0.01 # cd -> deg
        vfr_hud = self.mav.recv_match(
            type='VFR_HUD',
            blocking=True,
            timeout=1
        )
        if vfr_hud is None:
            raise NotAchievedException("Did not get VFR_HUD message")
        vfr_hud_yaw_deg = round(vfr_hud.heading)
        self.progress("VFR_HUD heading==%f frsky==%f" % (vfr_hud_yaw_deg, yaw_deg))
        if self.compare_number_percent(vfr_hud_yaw_deg, yaw_deg, 10):
            return True
        return False

    def tfs_validate_vspeed(self, value):
        self.progress("validating vspeed (0x%02x)" % value)
        vspeed_ms = value * 0.01 # cm/s -> m/s
        vfr_hud = self.mav.recv_match(
            type='VFR_HUD',
            blocking=True,
            timeout=1
        )
        if vfr_hud is None:
            raise NotAchievedException("Did not get VFR_HUD message")
        vfr_hud_vspeed_ms = round(vfr_hud.climb)
        self.progress("VFR_HUD climb==%f frsky==%f" % (vfr_hud_vspeed_ms, vspeed_ms))
        if self.compare_number_percent(vfr_hud_vspeed_ms, vspeed_ms, 10):
            return True
        return False

    def tfs_validate_battery1(self, value):
        self.progress("validating battery1 (0x%02x)" % value)
        voltage_v = value * 0.01 # cV -> V
        batt = self.mav.recv_match(
            type='BATTERY_STATUS',
            blocking=True,
            timeout=5,
            condition="BATTERY_STATUS.id==0"
        )
        if batt is None:
            raise NotAchievedException("Did not get BATTERY_STATUS message")
        battery_status_voltage_v = batt.voltages[0] * 0.001 # mV -> V
        self.progress("BATTERY_STATUS volatge==%f frsky==%f" % (battery_status_voltage_v, voltage_v))
        if self.compare_number_percent(battery_status_voltage_v, voltage_v, 10):
            return True
        return False

    def tfs_validate_current1(self, value):
        # test frsky current vs BATTERY_STATUS
        self.progress("validating battery1 (0x%02x)" % value)
        current_a = value * 0.1 # dA -> A
        batt = self.mav.recv_match(
            type='BATTERY_STATUS',
            blocking=True,
            timeout=5,
            condition="BATTERY_STATUS.id==0"
        )
        if batt is None:
            raise NotAchievedException("Did not get BATTERY_STATUS message")
        battery_status_current_a = batt.current_battery * 0.01 # cA -> A
        self.progress("BATTERY_STATUS current==%f frsky==%f" % (battery_status_current_a, current_a))
        if self.compare_number_percent(battery_status_current_a, current_a, 10):
            return True
        return False

    def tfs_validate_fuel(self, value):
        self.progress("validating fuel (0x%02x)" % value)
        fuel = value
        batt = self.mav.recv_match(
            type='BATTERY_STATUS',
            blocking=True,
            timeout=5,
            condition="BATTERY_STATUS.id==0"
        )
        if batt is None:
            raise NotAchievedException("Did not get BATTERY_STATUS message")
        battery_status_fuel = batt.battery_remaining
        self.progress("BATTERY_STATUS fuel==%f frsky==%f" % (battery_status_fuel, fuel))
        if self.compare_number_percent(battery_status_fuel, fuel, 10):
            return True
        return False

    def tfs_validate_tmp1(self, value):
        self.progress("validating tmp1 (0x%02x)" % value)
        tmp1 = value
        heartbeat = self.wait_heartbeat()
        heartbeat_tmp1 = heartbeat.custom_mode
        self.progress("GLOBAL_POSITION_INT custom_mode==%f frsky==%f" % (heartbeat_tmp1, tmp1))
        if heartbeat_tmp1 == tmp1:
            return True
        return False

    def tfs_validate_tmp2(self, value):
        self.progress("validating tmp2 (0x%02x)" % value)
        tmp2 = value
        gps_raw = self.mav.recv_match(
            type='GPS_RAW_INT',
            blocking=True,
            timeout=1
        )
        if gps_raw is None:
            raise NotAchievedException("Did not get GPS_RAW_INT message")
        gps_raw_tmp2 = gps_raw.satellites_visible*10 + gps_raw.fix_type
        self.progress("GPS_RAW_INT tmp2==%f frsky==%f" % (gps_raw_tmp2, tmp2))
        if gps_raw_tmp2 == tmp2:
            return True
        return False

    def tfs_validate_rpm(self, value):
        self.progress("validating rpm (0x%02x)" % value)
        tfs_rpm = value
        rpm = self.mav.recv_match(
            type='RPM',
            blocking=True,
            timeout=5
        )
        if rpm is None:
            raise NotAchievedException("Did not get RPM message")
        rpm_value = round(rpm.rpm1)
        self.progress("RPM rpm==%f frsky==%f" % (rpm_value, tfs_rpm))
        if rpm_value == tfs_rpm:
            return True
        return False

    def wait_rpm1(self, min_rpm=None, timeout=10):
        '''wait for mavlink RPM message to indicate valid RPM'''
        tstart = self.get_sim_time()
        while True:
            t = self.get_sim_time_cached()
            if t - tstart > 10:
                raise AutoTestTimeoutException("Failed to do get valid RPM")
            rpm = self.mav.recv_match(
                type='RPM',
                blocking=True,
                timeout=1
            )
            self.progress("rpm: (%s)" % str(rpm))
            if rpm is None:
                continue
            if min_rpm is None:
                return
            if rpm.rpm1 >= min_rpm:
                return

    def test_frsky_sport(self):
        self.set_parameter("SERIAL5_PROTOCOL", 4) # serial5 is FRSky sport
        self.set_parameter("RPM1_TYPE", 10) # enable SITL RPM sensor
        self.customise_SITL_commandline([
            "--uartF=tcp:6735" # serial5 spews to localhost:6735
        ])
        frsky = FRSkySPort(("127.0.0.1", 6735), verbose=True)
        self.wait_ready_to_arm()

        # we need to start the engine to get some RPM readings, we do it for plane only
        if self.is_plane():
            self.set_autodisarm_delay(0)
            if not self.arm_vehicle():
                raise NotAchievedException("Failed to ARM")
            self.set_rc(3, 1050)
            self.wait_rpm1(timeout=10, min_rpm=200)

        self.drain_mav_unparsed()
        # anything with a lambda in here needs a proper test written.
        # This, at least makes sure we're getting some of each
        # message.
        wants = {
            0x082F: self.tfs_validate_gps_alt, # gps altitude integer cm
            0x040F: self.tfs_validate_tmp1, # Tmp1
            0x060F: self.tfs_validate_fuel, # fuel % 0-100
            0x041F: self.tfs_validate_tmp2, # Tmp2
            0x010F: self.tfs_validate_baro_alt, # baro alt cm
            0x083F: self.tfs_validate_gps_speed, # gps speed integer mm/s
            0x084F: self.tfs_validate_yaw, # yaw in cd
            0x020F: self.tfs_validate_current1, # current dA
            0x011F: self.tfs_validate_vspeed, # vertical speed cm/s
            0x021F: self.tfs_validate_battery1, # battery 1 voltage cV
            0x0800: self.tf_validate_gps, # gps lat/lon
            0x050E: self.tfs_validate_rpm, # rpm 1
        }
        tstart = self.get_sim_time_cached()
        last_wanting_print = 0

        last_data_time = None
        while len(wants):
            now = self.get_sim_time()
            if now - last_wanting_print > 1:
                self.progress("Still wanting (%s)" %
                              ",".join([("0x%02x" % x) for x in wants.keys()]))
                last_wanting_print = now
            wants_copy = copy.copy(wants)
            if now - tstart > 300:
                self.progress("Failed to get frsky passthrough data")
                self.progress("Counts of sensor_id polls sent:")
                frsky.dump_sensor_id_poll_counts_as_progress_messages()
                self.progress("Counts of dataids received:")
                frsky.dump_dataid_counts_as_progress_messages()
                raise AutoTestTimeoutException("Failed to get frsky sport data")
            frsky.update()
            if frsky.last_data_time == last_data_time:
                continue
            last_data_time = frsky.last_data_time
            for want in wants_copy:
                data = frsky.get_data(want)
                if data is None:
                    continue
                self.progress("Checking 0x%x" % (want,))
                if wants[want](data):
                    self.progress("  Fulfilled")
                    del wants[want]
        # ok done, stop the engine
        if self.is_plane():
            self.zero_throttle()
            if not self.disarm_vehicle(force=True):
                raise NotAchievedException("Failed to DISARM")

    def test_frsky_d(self):
        self.set_parameter("SERIAL5_PROTOCOL", 3) # serial5 is FRSky output
        self.customise_SITL_commandline([
            "--uartF=tcp:6735" # serial5 spews to localhost:6735
        ])
        frsky = FRSkyD(("127.0.0.1", 6735))
        self.wait_ready_to_arm()
        self.drain_mav_unparsed()
        m = self.assert_receive_message('GLOBAL_POSITION_INT', timeout=1)
        gpi_abs_alt = int((m.alt+500) / 1000) # mm -> m

        # grab a battery-remaining percentage
        self.run_cmd(mavutil.mavlink.MAV_CMD_BATTERY_RESET,
                     255,  # battery mask
                     96,  # percentage
                     0,
                     0,
                     0,
                     0,
                     0,
                     0)
        m = self.assert_receive_message('BATTERY_STATUS', timeout=1)
        want_battery_remaining_pct = m.battery_remaining

        tstart = self.get_sim_time_cached()
        have_alt = False
        have_battery = False
        while True:
            t2 = self.get_sim_time_cached()
            if t2 - tstart > 10:
                raise AutoTestTimeoutException("Failed to get frsky D data")
            frsky.update()

            alt = frsky.get_data(frsky.dataid_GPS_ALT_BP)
            self.progress("Got alt (%s) mav=%s" % (str(alt), str(gpi_abs_alt)))
            if alt is None:
                continue
            if alt == gpi_abs_alt:
                have_alt = True

            batt = frsky.get_data(frsky.dataid_FUEL)
            self.progress("Got batt (%s) mav=%s" % (str(batt), str(want_battery_remaining_pct)))
            if batt is None:
                continue
            if batt == want_battery_remaining_pct:
                have_battery = True

            if have_alt and have_battery:
                break

            self.drain_mav_unparsed()

    def test_ltm_g(self, ltm):
        g = ltm.g()
        if g is None:
            return
        m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        print("m: %s" % str(m))

        print("g.lat=%s m.lat=%s" % (str(g.lat()), str(m.lat)))
        if abs(m.lat - g.lat()) > 10:
            return False

        print("g.lon:%s m.lon:%s" % (str(g.lon()), str(m.lon)))
        if abs(m.lon - g.lon()) > 10:
            return False

        print("gndspeed: %s" % str(g.gndspeed()))
        if g.gndspeed() != 0:
            # FIXME if we start the vehicle moving.... check against VFR_HUD?
            return False

        print("g.alt=%s m.alt=%s" % (str(g.alt()/100.0), str(m.relative_alt/1000.0)))
        if abs(m.relative_alt/1000.0 - g.alt()/100.0) > 1:
            return False

        print("sats: %s" % str(g.sats()))
        m = self.mav.recv_match(type='GPS_RAW_INT', blocking=True)
        if m.satellites_visible != g.sats():
            return False

        constrained_fix_type = m.fix_type
        if constrained_fix_type > 3:
            constrained_fix_type = 3
        print("fix_type: %s" % g.fix_type())
        if constrained_fix_type != g.fix_type():
            return False

        return True

    def test_ltm_a(self, ltm):
        a = ltm.a()
        if a is None:
            return
        m = self.mav.recv_match(type='ATTITUDE', blocking=True)

        pitch = a.pitch()
        print("pitch: %s" % str(pitch))
        if abs(math.degrees(m.pitch) - pitch) > 1:
            return False

        roll = a.roll()
        print("roll: %s" % str(roll))
        if abs(math.degrees(m.roll) - roll) > 1:
            return False

        hdg = a.hdg()
        myaw = math.degrees(m.yaw)
        myaw += 360
        myaw %= 360
        print("a.hdg=%s m.hdg=%s" % (str(hdg), str(myaw)))
        if abs(myaw - hdg) > 1:
            return False

        return True

    def test_ltm_s(self, ltm):
        s = ltm.s()
        if s is None:
            return
        # FIXME.  Actually check the field values are correct :-)
        return True

    def test_ltm(self):
        self.set_parameter("SERIAL5_PROTOCOL", 25) # serial5 is LTM output
        self.customise_SITL_commandline([
            "--uartF=tcp:6735" # serial5 spews to localhost:6735
        ])
        ltm = LTM(("127.0.0.1", 6735))
        self.wait_ready_to_arm()
        self.drain_mav_unparsed()

        wants = {
            "g": self.test_ltm_g,
            "a": self.test_ltm_a,
            "s": self.test_ltm_s,
        }

        tstart = self.get_sim_time_cached()
        while True:
            self.progress("Still wanting (%s)" %
                          ",".join([("%s" % x) for x in wants.keys()]))
            if len(wants) == 0:
                break
            now = self.get_sim_time_cached()
            if now - tstart > 10:
                raise AutoTestTimeoutException("Failed to get ltm data")

            ltm.update()

            wants_copy = copy.copy(wants)
            for want in wants_copy:
                self.progress("Checking %s" % (want,))
                if wants[want](ltm):
                    self.progress("  Fulfilled")
                    del wants[want]

    def convertDmsToDdFormat(self, dms):
        deg = math.trunc(dms * 1e-7)
        dd = deg + (((dms * 1.0e-7) - deg) * 100.0 / 60.0)
        if dd < -180.0 or dd > 180.0:
            dd = 0.0
        return math.trunc(dd * 1.0e7)

    def DEVO(self):
        self.context_push()
        self.set_parameter("SERIAL5_PROTOCOL", 17) # serial5 is DEVO output
        self.customise_SITL_commandline([
            "--uartF=tcp:6735" # serial5 spews to localhost:6735
        ])
        devo = DEVO(("127.0.0.1", 6735))
        self.wait_ready_to_arm()
        self.drain_mav_unparsed()
        m = self.assert_receive_message('GLOBAL_POSITION_INT', timeout=1)

        tstart = self.get_sim_time_cached()
        while True:
            self.drain_mav()
            now = self.get_sim_time_cached()
            if now - tstart > 10:
                if devo.frame is not None:
                    # we received some frames but could not find correct values
                    raise AutoTestTimeoutException("Failed to get correct data")
                else:
                    # No frames received. Devo telemetry is compiled out?
                    break

            devo.update()
            frame = devo.frame
            if frame is None:
                continue

            m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

            loc = LocationInt(self.convertDmsToDdFormat(frame.lat()), self.convertDmsToDdFormat(frame.lon()), 0, 0)

            print("received lat:%s expected lat:%s" % (str(loc.lat), str(m.lat)))
            print("received lon:%s expected lon:%s" % (str(loc.lon), str(m.lon)))
            dist_diff = self.get_distance_int(loc, m)
            print("Distance:%s" % str(dist_diff))
            if abs(dist_diff) > 2:
                continue

            gpi_rel_alt = int(m.relative_alt / 10) # mm -> cm, since driver send alt in cm
            print("received alt:%s expected alt:%s" % (str(frame.alt()), str(gpi_rel_alt)))
            if abs(gpi_rel_alt - frame.alt()) > 10:
                continue

            print("received gndspeed: %s" % str(frame.speed()))
            if frame.speed() != 0:
                # FIXME if we start the vehicle moving.... check against VFR_HUD?
                continue

            print("received temp:%s expected temp:%s" % (str(frame.temp()), str(self.mav.messages['HEARTBEAT'].custom_mode)))
            if frame.temp() != self.mav.messages['HEARTBEAT'].custom_mode:
                # currently we receive mode as temp. This should be fixed when driver is updated
                continue

            # we match the received voltage with the voltage of primary instance
            batt = self.mav.recv_match(
                type='BATTERY_STATUS',
                blocking=True,
                timeout=5,
                condition="BATTERY_STATUS.id==0"
            )
            if batt is None:
                raise NotAchievedException("Did not get BATTERY_STATUS message")
            volt = batt.voltages[0]*0.001
            print("received voltage:%s expected voltage:%s" % (str(frame.volt()*0.1), str(volt)))
            if abs(frame.volt()*0.1 - volt) > 0.1:
                continue
            # if we reach here, exit
            break
        self.context_pop()
        self.reboot_sitl()

    def test_msp_dji(self):
        self.set_parameter("SERIAL5_PROTOCOL", 33) # serial5 is MSP DJI output
        self.set_parameter("MSP_OPTIONS", 1) # telemetry (unpolled) mode
        self.customise_SITL_commandline([
            "--uartF=tcp:6735" # serial5 spews to localhost:6735
        ])
        msp = MSP_DJI(("127.0.0.1", 6735))
        self.wait_ready_to_arm()
        self.drain_mav_unparsed()

        tstart = self.get_sim_time_cached()
        while True:
            self.drain_mav()
            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException("Did not get location")
            msp.update()
            self.drain_mav_unparsed(quiet=True)
            try:
                f = msp.get_frame(msp.FRAME_GPS_RAW)
            except KeyError:
                continue
            dist = self.get_distance_int(f.LocationInt(), self.sim_location_int())
            print("lat=%f lon=%f dist=%f" % (f.lat(), f.lon(), dist))
            if dist < 1:
                break

    def test_crsf(self):
        self.context_push()
        ex = None
        try:
            self.set_parameter("SERIAL5_PROTOCOL", 23) # serial5 is RCIN input
            self.customise_SITL_commandline([
                "--uartF=tcp:6735" # serial5 reads from to localhost:6735
            ])
            crsf = CRSF(("127.0.0.1", 6735))
            crsf.connect()

            self.progress("Writing vtx_frame")
            crsf.write_data_id(crsf.dataid_vtx_frame)
            self.delay_sim_time(5)
            self.progress("Writing vtx_telem")
            crsf.write_data_id(crsf.dataid_vtx_telem)
            self.delay_sim_time(5)
            self.progress("Writing vtx_unknown")
            crsf.write_data_id(crsf.dataid_vtx_unknown)
            self.delay_sim_time(5)
        except Exception as e:
            self.print_exception_caught(e)
            ex = e
        self.context_pop()
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def AHRS_ORIENTATION(self):
        '''test AHRS_ORIENTATION parameter works'''
        self.context_push()
        self.wait_ready_to_arm()
        original_imu = self.assert_receive_message("RAW_IMU", verbose=True)
        self.set_parameter("AHRS_ORIENTATION", 16)  # roll-90
        self.delay_sim_time(2)  # we update this on a timer
        new_imu = self.assert_receive_message("RAW_IMU", verbose=True)
        delta_zacc = original_imu.zacc - new_imu.zacc
        delta_z_g = delta_zacc/1000.0  # milligravities -> gravities
        if delta_z_g - 1 > 0.1:  # milligravities....
            raise NotAchievedException("Magic AHRS_ORIENTATION update did not work (delta_z_g=%f)" % (delta_z_g,))
        delta_yacc = original_imu.yacc - new_imu.yacc
        delta_y_g = delta_yacc/1000.0  # milligravities -> gravities
        if delta_y_g + 1 > 0.1:
            raise NotAchievedException("Magic AHRS_ORIENTATION update did not work (delta_y_g=%f)" % (delta_y_g,))
        self.context_pop()
        self.reboot_sitl()
        self.delay_sim_time(2)  # we update orientation on a timer

    def GPSTypes(self):
        '''check each simulated GPS works'''
        self.reboot_sitl()
        orig = self.poll_home_position(timeout=60)
        # (sim_gps_type, name, gps_type, detection name)
        # if gps_type is None we auto-detect
        sim_gps = [
            # (0, "NONE"),
            (1, "UBLOX", None, "u-blox"),
            (5, "NMEA", 5, "NMEA"),
            (6, "SBP", None, "SBP"),
            # (7, "SBP2", 9, "SBP2"),  # broken, "waiting for config data"
            (8, "NOVA", 15, "NOVA"),  # no attempt to auto-detect this in AP_GPS
            # (9, "FILE"),
        ]
        for (sim_gps_type, name, gps_type, detect_name) in sim_gps:
            self.start_subtest("Checking GPS type %s" % name)
            self.set_parameter("SIM_GPS_TYPE", sim_gps_type)
            if gps_type is None:
                gps_type = 1  # auto-detect
            self.set_parameter("GPS_TYPE", gps_type)
            self.context_collect("STATUSTEXT")
            self.reboot_sitl()
            self.wait_statustext("detected as %s" % detect_name, check_context=True)
            n = self.poll_home_position(timeout=120)
            distance = self.get_distance_int(orig, n)
            if distance > 1:
                raise NotAchievedException("gps type %u misbehaving" % name)

    def assert_gps_satellite_count(self, messagename, count):
        m = self.assert_receive_message(messagename)
        if m.satellites_visible != count:
            raise NotAchievedException("Expected %u sats, got %u" %
                                       (count, m.satellites_visible))

    def MultipleGPS(self):
        '''check ArduPilot behaviour across multiple GPS units'''
        self.assert_message_rate_hz('GPS2_RAW', 0)

        # we start sending GPS_TYPE2 - but it will never actually be
        # filled in as _port[1] is only filled in in AP_GPS::init()
        self.start_subtest("Get GPS2_RAW as soon as we're configured for a second GPS")
        self.set_parameter("GPS_TYPE2", 1)
        self.assert_message_rate_hz('GPS2_RAW', 5)

        self.start_subtest("Ensure correct fix type when no connected GPS")
        m = self.assert_receive_message("GPS2_RAW")
        self.progress(self.dump_message_verbose(m))
        if m.fix_type != mavutil.mavlink.GPS_FIX_TYPE_NO_GPS:
            raise NotAchievedException("Incorrect fix type")

        self.start_subtest("Ensure detection when sim gps connected")
        self.set_parameter("SIM_GPS2_TYPE", 1)
        self.set_parameter("SIM_GPS2_DISABLE", 0)
        # a reboot is required after setting GPS_TYPE2.  We start
        # sending GPS2_RAW out, once the parameter is set, but a
        # reboot is required because _port[1] is only set in
        # AP_GPS::init() at boot time, so it will never be detected.
        self.context_collect("STATUSTEXT")
        self.reboot_sitl()
        self.wait_statustext("GPS 1: detected as u-blox", check_context=True)
        self.wait_statustext("GPS 2: detected as u-blox", check_context=True)
        m = self.assert_receive_message("GPS2_RAW")
        self.progress(self.dump_message_verbose(m))
        # would be nice for it to take some time to get a fix....
        if m.fix_type != mavutil.mavlink.GPS_FIX_TYPE_RTK_FIXED:
            raise NotAchievedException("Incorrect fix type")

        self.start_subtest("Check parameters are per-GPS")
        self.assert_parameter_value("SIM_GPS_NUMSATS", 10)
        self.assert_gps_satellite_count("GPS_RAW_INT", 10)
        self.set_parameter("SIM_GPS_NUMSATS", 13)
        self.assert_gps_satellite_count("GPS_RAW_INT", 13)

        self.assert_parameter_value("SIM_GPS2_NUMSATS", 10)
        self.assert_gps_satellite_count("GPS2_RAW", 10)
        self.set_parameter("SIM_GPS2_NUMSATS", 12)
        self.assert_gps_satellite_count("GPS2_RAW", 12)

        self.start_subtest("check that GLOBAL_POSITION_INT fails over")
        m = self.assert_receive_message("GLOBAL_POSITION_INT")
        gpi_alt = m.alt
        for msg in ["GPS_RAW_INT", "GPS2_RAW"]:
            m = self.assert_receive_message(msg)
            if abs(m.alt - gpi_alt) > 100:  # these are in mm
                raise NotAchievedException("Alt (%s) discrepancy; %d vs %d" %
                                           (msg, m.alt, gpi_alt))
        introduced_error = 10  # in metres
        self.set_parameter("SIM_GPS2_ALT_OFS", introduced_error)
        m = self.assert_receive_message("GPS2_RAW")
        if abs((m.alt-introduced_error*1000) - gpi_alt) > 100:
            raise NotAchievedException("skewed Alt (%s) discrepancy; %d+%d vs %d" %
                                       (msg, introduced_error*1000, m.alt, gpi_alt))
        m = self.assert_receive_message("GLOBAL_POSITION_INT")
        new_gpi_alt = m.alt
        if abs(gpi_alt - new_gpi_alt) > 100:
            raise NotAchievedException("alt moved unexpectedly")
        self.progress("Killing first GPS")
        self.set_parameter("SIM_GPS_DISABLE", 1)
        self.delay_sim_time(1)
        self.progress("Checking altitude now matches second GPS")
        m = self.assert_receive_message("GLOBAL_POSITION_INT")
        new_gpi_alt2 = m.alt
        m = self.assert_receive_message("GPS2_RAW")
        if abs(new_gpi_alt2 - m.alt) > 100:
            raise NotAchievedException("Failover not detected")

    def fetch_file_via_ftp(self, path):
        '''returns the content of the FTP'able file at path'''
        mavproxy = self.start_mavproxy()
        ex = None
        tmpfile = tempfile.NamedTemporaryFile(mode='r', delete=False)
        try:
            mavproxy.send("module load ftp\n")
            mavproxy.expect(["Loaded module ftp", "module ftp already loaded"])
            mavproxy.send("ftp get %s %s\n" % (path, tmpfile.name))
            mavproxy.expect("Getting")
            self.delay_sim_time(2)
            mavproxy.send("ftp status\n")
            mavproxy.expect("No transfer in progress")
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.stop_mavproxy(mavproxy)

        if ex is not None:
            raise ex

        return tmpfile.read()

    def MAVFTP(self):
        '''ensure MAVProxy can do MAVFTP to ardupilot'''
        mavproxy = self.start_mavproxy()
        ex = None
        try:
            mavproxy.send("module load ftp\n")
            mavproxy.expect(["Loaded module ftp", "module ftp already loaded"])
            mavproxy.send("ftp list\n")
            some_directory = None
            for entry in sorted(os.listdir(".")):
                if os.path.isdir(entry):
                    some_directory = entry
                    break
            if some_directory is None:
                raise NotAchievedException("No directories?!")
            expected_line = " D %s" % some_directory
            mavproxy.expect(expected_line)  # one line from the ftp list output
        except Exception as e:
            self.print_exception_caught(e)
            ex = e

        self.stop_mavproxy(mavproxy)

        if ex is not None:
            raise ex

    def tests(self):
        return [
            Test("PIDTuning",
                 "Test PID Tuning",
                 self.test_pid_tuning),

            Test("ArmFeatures", "Arm features", self.test_arm_feature),

            Test("SetHome",
                 "Test Set Home",
                 self.fly_test_set_home),

            Test("ConfigErrorLoop",
                 "Test Config Error Loop",
                 self.test_config_error_loop),

            Test("CPUFailsafe",
                 "Ensure we do something appropriate when the main loop stops",
                 self.CPUFailsafe),

            Test("Parameters",
                 "Test Parameter Set/Get",
                 self.test_parameters),

            Test("LoggerDocumentation",
                 "Test Onboard Logging Generation",
                 self.test_onboard_logging_generation),

            Test("Logging",
                 "Test Onboard Logging",
                 self.test_onboard_logging),

            Test("GetCapabilities",
                 "Get Capabilities",
                 self.test_get_autopilot_capabilities),

            Test("InitialMode",
                 "Test initial mode switching",
                 self.test_initial_mode),
        ]

    def post_tests_announcements(self):
        if self._show_test_timings:
            if self.waiting_to_arm_count == 0:
                avg = None
            else:
                avg = self.total_waiting_to_arm_time/self.waiting_to_arm_count
            self.progress("Spent %f seconds waiting to arm. count=%u avg=%s" %
                          (self.total_waiting_to_arm_time,
                           self.waiting_to_arm_count,
                           str(avg)))
            self.show_test_timings()
        if self.forced_post_test_sitl_reboots != 0:
            print("Had to force-reset SITL %u times" %
                  (self.forced_post_test_sitl_reboots,))

    def autotest(self):
        """Autotest used by ArduPilot autotest CI."""
        all_tests = []
        for test in self.tests():
            if type(test) == Test:
                all_tests.append(test)
                continue
            (name, desc, func) = test
            actual_test = Test(name, desc, func)
            all_tests.append(actual_test)

        disabled = self.disabled_tests()
        tests = []
        for test in all_tests:
            if test.name in disabled:
                self.test_skipped(test, disabled[test.name])
                continue
            tests.append(test)

        ret = self.run_tests(tests)
        self.post_tests_announcements()
        return ret

    def mavfft_fttd(self, sensor_type, sensor_instance, since, until):
        '''display fft for raw ACC data in current logfile'''

        '''object to store data about a single FFT plot'''
        class MessageData(object):
            def __init__(self, ffth):
                self.seqno = -1
                self.fftnum = ffth.N
                self.sensor_type = ffth.type
                self.instance = ffth.instance
                self.sample_rate_hz = ffth.smp_rate
                self.multiplier = ffth.mul
                self.sample_us = ffth.SampleUS
                self.data = {}
                self.data["X"] = []
                self.data["Y"] = []
                self.data["Z"] = []
                self.holes = False
                self.freq = None

            def add_fftd(self, fftd):
                self.seqno += 1
                self.data["X"].extend(fftd.x)
                self.data["Y"].extend(fftd.y)
                self.data["Z"].extend(fftd.z)

        mlog = self.dfreader_for_current_onboard_log()

        # see https://holometer.fnal.gov/GH_FFT.pdf for a description of the techniques used here
        messages = []
        messagedata = None
        while True:
            m = mlog.recv_match()
            if m is None:
                break
            msg_type = m.get_type()
            if msg_type == "ISBH":
                if messagedata is not None:
                    if (messagedata.sensor_type == sensor_type and
                            messagedata.instance == sensor_instance and
                            messagedata.sample_us > since and
                            messagedata.sample_us < until):
                        messages.append(messagedata)
                messagedata = MessageData(m)
                continue

            if msg_type == "ISBD":
                if (messagedata is not None and
                        messagedata.sensor_type == sensor_type and
                        messagedata.instance == sensor_instance):
                    messagedata.add_fftd(m)

        fft_len = len(messages[0].data["X"])
        sum_fft = {
            "X": numpy.zeros(int(fft_len / 2 + 1)),
            "Y": numpy.zeros(int(fft_len / 2 + 1)),
            "Z": numpy.zeros(int(fft_len / 2 + 1)),
        }
        sample_rate = 0
        counts = 0
        window = numpy.hanning(fft_len)
        freqmap = numpy.fft.rfftfreq(fft_len, 1.0 / messages[0].sample_rate_hz)

        # calculate NEBW constant
        S2 = numpy.inner(window, window)

        for message in messages:
            for axis in ["X", "Y", "Z"]:
                # normalize data and convert to dps in order to produce more meaningful magnitudes
                if message.sensor_type == 1:
                    d = numpy.array(numpy.degrees(message.data[axis])) / float(message.multiplier)
                else:
                    d = numpy.array(message.data[axis]) / float(message.multiplier)

                # apply window to the input
                d *= window
                # perform RFFT
                d_fft = numpy.fft.rfft(d)
                # convert to squared complex magnitude
                d_fft = numpy.square(abs(d_fft))
                # remove DC component
                d_fft[0] = 0
                d_fft[-1] = 0
                # accumulate the sums
                sum_fft[axis] += d_fft

            sample_rate = message.sample_rate_hz
            counts += 1

        numpy.seterr(divide='ignore')
        psd = {}
        for axis in ["X", "Y", "Z"]:
            # normalize output to averaged PSD
            psd[axis] = 2 * (sum_fft[axis] / counts) / (sample_rate * S2)
            psd[axis] = 10 * numpy.log10(psd[axis])

        psd["F"] = freqmap

        return psd

    def model_defaults_filepath(self, model):
        vehicle = self.vehicleinfo_key()
        vinfo = vehicleinfo.VehicleInfo()
        defaults_filepath = vinfo.options[vehicle]["frames"][model]["default_params_filename"]
        if isinstance(defaults_filepath, str):
            defaults_filepath = [defaults_filepath]
        defaults_list = []
        for d in defaults_filepath:
            defaults_list.append(util.reltopdir(os.path.join(testdir, d)))
        return defaults_list

    def load_default_params_file(self, filename):
        '''load a file from Tools/autotest/default_params'''
        filepath = util.reltopdir(os.path.join("Tools", "autotest", "default_params", filename))
        self.repeatedly_apply_parameter_file(filepath)

    def send_pause_command(self):
        '''pause AUTO/GUIDED modes'''
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
                     0, # 0: pause, 1: continue
                     0, # param2
                     0, # param3
                     0, # param4
                     0, # param5
                     0, # param6
                     0) # param7

    def send_resume_command(self):
        '''resume AUTO/GUIDED modes'''
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
                     1, # 0: pause, 1: continue
                     0, # param2
                     0, # param3
                     0, # param4
                     0, # param5
                     0, # param6
                     0) # param7
