#!/usr/bin/env python
############################################################################
#
#   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

#
# Serial firmware uploader for the PX4FMU bootloader
#
# The PX4 firmware file is a JSON-encoded Python object, containing
# metadata fields and a zlib-compressed base64-encoded firmware image.
#
# The uploader uses the following fields from the firmware file:
#
# image
#       The firmware that will be uploaded.
# image_size
#       The size of the firmware in bytes.
# board_id
#       The board for which the firmware is intended.
# board_revision
#       Currently only used for informational purposes.
#

# for python2.7 compatibility
from __future__ import print_function

import sys
import argparse
import binascii
import serial
import struct
import json
import zlib
import base64
import time
import array
import os
import platform
import re

from sys import platform as _platform

is_WSL = bool("Microsoft" in platform.uname()[2])

# default list of port names to look for autopilots
default_ports = ['/dev/serial/by-id/usb-Ardu*',
                 '/dev/serial/by-id/usb-3D*',
                 '/dev/serial/by-id/usb-APM*',
                 '/dev/serial/by-id/usb-Radio*',
                 '/dev/serial/by-id/usb-*_3DR_*',
                 '/dev/serial/by-id/usb-Hex_Technology_Limited*',
                 '/dev/serial/by-id/usb-Hex_ProfiCNC*',
                 '/dev/serial/by-id/usb-Holybro*',
                 '/dev/serial/by-id/usb-mRo*',
                 '/dev/tty.usbmodem*']

if "cygwin" in _platform or is_WSL:
    default_ports += ['/dev/ttyS*']

# Detect python version
if sys.version_info[0] < 3:
    runningPython3 = False
else:
    runningPython3 = True

# dictionary of bootloader {boardID: (firmware boardID, boardname), ...}
# designating firmware builds compatible with multiple boardIDs
compatible_IDs = {33: (9, 'AUAVX2.1')}


# CRC equivalent to crc_crc32() in AP_Math/crc.cpp
crctab = array.array('I', [
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
    0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
    0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
    0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
    0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
    0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
    0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
    0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
    0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
    0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
    0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
    0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
    0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
    0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
    0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
    0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
    0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
    0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
    0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
    0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
    0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
    0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d])

def crc32(bytes, state=0):
    '''crc32 exposed for use by chibios.py'''
    for byte in bytes:
        index = (state ^ byte) & 0xff
        state = crctab[index] ^ (state >> 8)
    return state

class firmware(object):
    '''Loads a firmware file'''

    desc = {}
    image = bytes()
    crcpad = bytearray(b'\xff\xff\xff\xff')

    def __init__(self, path):

        # read the file
        f = open(path, "r")
        self.desc = json.load(f)
        f.close()

        self.image = bytearray(zlib.decompress(base64.b64decode(self.desc['image'])))

        # pad image to 4-byte length
        while ((len(self.image) % 4) != 0):
            self.image.append('\xff')

    def property(self, propname):
        return self.desc[propname]

    def crc(self, padlen):
        state = crc32(self.image, int(0))
        for i in range(len(self.image), (padlen - 1), 4):
            state = crc32(self.crcpad, state)
        return state


class uploader(object):
    '''Uploads a firmware file to the PX FMU bootloader'''

    # protocol bytes
    INSYNC          = b'\x12'
    EOC             = b'\x20'

    # reply bytes
    OK              = b'\x10'
    FAILED          = b'\x11'
    INVALID         = b'\x13'     # rev3+
    BAD_SILICON_REV = b'\x14'     # rev5+

    # command bytes
    NOP             = b'\x00'     # guaranteed to be discarded by the bootloader
    GET_SYNC        = b'\x21'
    GET_DEVICE      = b'\x22'
    CHIP_ERASE      = b'\x23'
    CHIP_VERIFY     = b'\x24'     # rev2 only
    PROG_MULTI      = b'\x27'
    READ_MULTI      = b'\x28'     # rev2 only
    GET_CRC         = b'\x29'     # rev3+
    GET_OTP         = b'\x2a'     # rev4+  , get a word from OTP area
    GET_SN          = b'\x2b'     # rev4+  , get a word from SN area
    GET_CHIP        = b'\x2c'     # rev5+  , get chip version
    SET_BOOT_DELAY  = b'\x2d'     # rev5+  , set boot delay
    GET_CHIP_DES    = b'\x2e'     # rev5+  , get chip description in ASCII
    MAX_DES_LENGTH  = 20

    REBOOT          = b'\x30'
    SET_BAUD        = b'\x33'     # set baud

    INFO_BL_REV     = b'\x01'        # bootloader protocol revision
    BL_REV_MIN      = 2              # minimum supported bootloader protocol
    BL_REV_MAX      = 5              # maximum supported bootloader protocol
    INFO_BOARD_ID   = b'\x02'        # board type
    INFO_BOARD_REV  = b'\x03'        # board revision
    INFO_FLASH_SIZE = b'\x04'        # max firmware size in bytes

    PROG_MULTI_MAX  = 252            # protocol max is 255, must be multiple of 4
    READ_MULTI_MAX  = 252            # protocol max is 255

    NSH_INIT        = bytearray(b'\x0d\x0d\x0d')
    NSH_REBOOT_BL   = b"reboot -b\n"
    NSH_REBOOT      = b"reboot\n"

    def __init__(self, portname, baudrate_bootloader, baudrate_flightstack, baudrate_bootloader_flash=None, target_system=None, target_component=None, source_system=None, source_component=None):
        self.MAVLINK_REBOOT_ID1 = bytearray(b'\xfe\x21\x72\xff\x00\x4c\x00\x00\x40\x40\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xf6\x00\x01\x00\x00\x53\x6b')
        self.MAVLINK_REBOOT_ID0 = bytearray(b'\xfe\x21\x45\xff\x00\x4c\x00\x00\x40\x40\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xf6\x00\x00\x00\x00\xcc\x37')
        if target_component is None:
            target_component = 1
        if source_system is None:
            source_system = 255
        if source_component is None:
            source_component = 1

        # open the port, keep the default timeout short so we can poll quickly
        self.port = serial.Serial(portname, baudrate_bootloader, timeout=1.0)
        self.baudrate_bootloader = baudrate_bootloader
        if baudrate_bootloader_flash is not None:
            self.baudrate_bootloader_flash = baudrate_bootloader_flash
        else:
            self.baudrate_bootloader_flash = self.baudrate_bootloader
        self.baudrate_flightstack = baudrate_flightstack
        self.baudrate_flightstack_idx = -1
        # generate mavlink reboot message:
        if target_system is not None:
            from pymavlink import mavutil
            m = mavutil.mavlink.MAVLink_command_long_message(
                target_system,
                target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                1, # confirmation
                3, # remain in bootloader
                0,
                0,
                0,
                0,
                0,
                0)
            mav = mavutil.mavlink.MAVLink(self,
                                          srcSystem=source_system,
                                          srcComponent=source_component)
            self.MAVLINK_REBOOT_ID1 = m.pack(mav)
            self.MAVLINK_REBOOT_ID0 = None

    def close(self):
        if self.port is not None:
            self.port.close()

    def open(self):
        timeout = time.time() + 0.2

        # Attempt to open the port while it exists and until timeout occurs
        while self.port is not None:
            portopen = True
            try:
                portopen = self.port.is_open
            except AttributeError:
                portopen = self.port.isOpen()

            if not portopen and time.time() < timeout:
                try:
                    self.port.open()
                except OSError:
                    # wait for the port to be ready
                    time.sleep(0.04)
                except serial.SerialException:
                    # if open fails, try again later
                    time.sleep(0.04)

            else:
                break

    def __send(self, c):
        self.port.write(c)

    def __recv(self, count=1):
        c = self.port.read(count)
        if len(c) < 1:
            raise RuntimeError("timeout waiting for data (%u bytes)" % count)
        # print("recv " + binascii.hexlify(c))
        return c

    def __recv_int(self):
        raw = self.__recv(4)
        val = struct.unpack("<I", raw)
        return val[0]

    def __getSync(self):
        self.port.flush()
        c = bytes(self.__recv())
        if c != self.INSYNC:
            raise RuntimeError("unexpected %s instead of INSYNC" % c)
        c = self.__recv()
        if c == self.INVALID:
            raise RuntimeError("bootloader reports INVALID OPERATION")
        if c == self.FAILED:
            raise RuntimeError("bootloader reports OPERATION FAILED")
        if c != self.OK:
            raise RuntimeError("unexpected response 0x%x instead of OK" % ord(c))

    # attempt to get back into sync with the bootloader
    def __sync(self):
        # send a stream of ignored bytes longer than the longest possible conversation
        # that we might still have in progress
        # self.__send(uploader.NOP * (uploader.PROG_MULTI_MAX + 2))
        self.port.flushInput()
        self.__send(uploader.GET_SYNC +
                    uploader.EOC)
        self.__getSync()

    def __trySync(self):
        try:
            self.port.flush()
            if (self.__recv() != self.INSYNC):
                # print("unexpected 0x%x instead of INSYNC" % ord(c))
                return False
            c = self.__recv()
            if (c == self.BAD_SILICON_REV):
                raise NotImplementedError()
            if (c != self.OK):
                # print("unexpected 0x%x instead of OK" % ord(c))
                return False
            return True

        except NotImplementedError:
            raise RuntimeError("Programing not supported for this version of silicon!\n"
                               "See https://pixhawk.org/help/errata")
        except RuntimeError:
            # timeout, no response yet
            return False

    # send the GET_DEVICE command and wait for an info parameter
    def __getInfo(self, param):
        self.__send(uploader.GET_DEVICE + param + uploader.EOC)
        value = self.__recv_int()
        self.__getSync()
        return value

    # send the GET_OTP command and wait for an info parameter
    def __getOTP(self, param):
        t = struct.pack("I", param)  # int param as 32bit ( 4 byte ) char array.
        self.__send(uploader.GET_OTP + t + uploader.EOC)
        value = self.__recv(4)
        self.__getSync()
        return value

    # send the GET_SN command and wait for an info parameter
    def __getSN(self, param):
        t = struct.pack("I", param)  # int param as 32bit ( 4 byte ) char array.
        self.__send(uploader.GET_SN + t + uploader.EOC)
        value = self.__recv(4)
        self.__getSync()
        return value

    # send the GET_CHIP command
    def __getCHIP(self):
        self.__send(uploader.GET_CHIP + uploader.EOC)
        value = self.__recv_int()
        self.__getSync()
        return value

    # send the GET_CHIP command
    def __getCHIPDes(self):
        self.__send(uploader.GET_CHIP_DES + uploader.EOC)
        length = self.__recv_int()
        value = self.__recv(length)
        self.__getSync()
        if runningPython3:
            value = value.decode('ascii')
        peices = value.split(",")
        return peices

    def __drawProgressBar(self, label, progress, maxVal):
        if maxVal < progress:
            progress = maxVal

        percent = (float(progress) / float(maxVal)) * 100.0

        sys.stdout.write("\r%s: [%-20s] %.1f%%" % (label, '='*int(percent/5.0), percent))
        sys.stdout.flush()

    # send the CHIP_ERASE command and wait for the bootloader to become ready
    def __erase(self, label):
        print("\n", end='')
        self.__send(uploader.CHIP_ERASE +
                    uploader.EOC)

        # erase is very slow, give it 20s
        deadline = time.time() + 20.0
        while time.time() < deadline:

            # Draw progress bar (erase usually takes about 9 seconds to complete)
            estimatedTimeRemaining = deadline-time.time()
            if estimatedTimeRemaining >= 9.0:
                self.__drawProgressBar(label, 20.0-estimatedTimeRemaining, 9.0)
            else:
                self.__drawProgressBar(label, 10.0, 10.0)
                sys.stdout.write(" (timeout: %d seconds) " % int(deadline-time.time()))
                sys.stdout.flush()

            if self.__trySync():
                self.__drawProgressBar(label, 10.0, 10.0)
                return

        raise RuntimeError("timed out waiting for erase")

    # send a PROG_MULTI command to write a collection of bytes
    def __program_multi(self, data):

        if runningPython3:
            length = len(data).to_bytes(1, byteorder='big')
        else:
            length = chr(len(data))

        self.__send(uploader.PROG_MULTI)
        self.__send(length)
        self.__send(data)
        self.__send(uploader.EOC)
        self.__getSync()

    # verify multiple bytes in flash
    def __verify_multi(self, data):

        if runningPython3:
            length = len(data).to_bytes(1, byteorder='big')
        else:
            length = chr(len(data))

        self.__send(uploader.READ_MULTI)
        self.__send(length)
        self.__send(uploader.EOC)
        self.port.flush()
        programmed = self.__recv(len(data))
        if programmed != data:
            print("got    " + binascii.hexlify(programmed))
            print("expect " + binascii.hexlify(data))
            return False
        self.__getSync()
        return True

    # read multiple bytes from flash
    def __read_multi(self, length):

        if runningPython3:
            clength = length.to_bytes(1, byteorder='big')
        else:
            clength = chr(length)

        self.__send(uploader.READ_MULTI)
        self.__send(clength)
        self.__send(uploader.EOC)
        self.port.flush()
        ret = self.__recv(length)
        self.__getSync()
        return ret
    
    # send the reboot command
    def __reboot(self):
        self.__send(uploader.REBOOT +
                    uploader.EOC)
        self.port.flush()

        # v3+ can report failure if the first word flash fails
        if self.bl_rev >= 3:
            self.__getSync()

    # split a sequence into a list of size-constrained pieces
    def __split_len(self, seq, length):
        return [seq[i:i+length] for i in range(0, len(seq), length)]

    # upload code
    def __program(self, label, fw):
        print("\n", end='')
        code = fw.image
        groups = self.__split_len(code, uploader.PROG_MULTI_MAX)

        uploadProgress = 0
        for bytes in groups:
            self.__program_multi(bytes)

            # Print upload progress (throttled, so it does not delay upload progress)
            uploadProgress += 1
            if uploadProgress % 256 == 0:
                self.__drawProgressBar(label, uploadProgress, len(groups))
        self.__drawProgressBar(label, 100, 100)

    # download code
    def __download(self, label, fw):
        print("\n", end='')
        f = open(fw, 'wb')

        downloadProgress = 0
        readsize = uploader.READ_MULTI_MAX
        total = 0
        while True:
            n = min(self.fw_maxsize - total, readsize)
            bb = self.__read_multi(n)
            f.write(bb)

            total += len(bb)
            # Print download progress (throttled, so it does not delay download progress)
            downloadProgress += 1
            if downloadProgress % 256 == 0:
                self.__drawProgressBar(label, total, self.fw_maxsize)
            if len(bb) < readsize:
                break
        f.close()
        self.__drawProgressBar(label, total, self.fw_maxsize)
        print("\nReceived %u bytes to %s" % (total, fw))
        
    # verify code
    def __verify_v2(self, label, fw):
        print("\n", end='')
        self.__send(uploader.CHIP_VERIFY +
                    uploader.EOC)
        self.__getSync()
        code = fw.image
        groups = self.__split_len(code, uploader.READ_MULTI_MAX)
        verifyProgress = 0
        for bytes in groups:
            verifyProgress += 1
            if verifyProgress % 256 == 0:
                self.__drawProgressBar(label, verifyProgress, len(groups))
            if (not self.__verify_multi(bytes)):
                raise RuntimeError("Verification failed")
        self.__drawProgressBar(label, 100, 100)

    def __verify_v3(self, label, fw):
        print("\n", end='')
        self.__drawProgressBar(label, 1, 100)
        expect_crc = fw.crc(self.fw_maxsize)
        self.__send(uploader.GET_CRC +
                    uploader.EOC)
        report_crc = self.__recv_int()
        self.__getSync()
        if report_crc != expect_crc:
            print("Expected 0x%x" % expect_crc)
            print("Got      0x%x" % report_crc)
            raise RuntimeError("Program CRC failed")
        self.__drawProgressBar(label, 100, 100)

    def __set_boot_delay(self, boot_delay):
        self.__send(uploader.SET_BOOT_DELAY +
                    struct.pack("b", boot_delay) +
                    uploader.EOC)
        self.__getSync()

    def __setbaud(self, baud):
        self.__send(uploader.SET_BAUD +
                    struct.pack("I", baud) +
                    uploader.EOC)
        self.__getSync()

    # get basic data about the board
    def identify(self):
        # make sure we are in sync before starting
        self.__sync()

        # get the bootloader protocol ID first
        self.bl_rev = self.__getInfo(uploader.INFO_BL_REV)
        if (self.bl_rev < uploader.BL_REV_MIN) or (self.bl_rev > uploader.BL_REV_MAX):
            print("Unsupported bootloader protocol %d" % self.bl_rev)
            raise RuntimeError("Bootloader protocol mismatch")

        self.board_type = self.__getInfo(uploader.INFO_BOARD_ID)
        self.board_rev = self.__getInfo(uploader.INFO_BOARD_REV)
        self.fw_maxsize = self.__getInfo(uploader.INFO_FLASH_SIZE)

    def dump_board_info(self):
        # OTP added in v4:
        print("Bootloader Protocol: %u" % self.bl_rev)
        if self.bl_rev > 3:
            otp = b''
            for byte in range(0, 32*6, 4):
                x = self.__getOTP(byte)
                otp = otp + x
#                print(binascii.hexlify(x).decode('Latin-1') + ' ', end='')
            # see src/modules/systemlib/otp.h in px4 code:
            otp_id = otp[0:4]
            otp_idtype = otp[4:5]
            otp_vid = otp[8:4:-1]
            otp_pid = otp[12:8:-1]
            otp_coa = otp[32:160]
            # show user:
            try:
                print("OTP:")
                print("  type: " + otp_id.decode('Latin-1'))
                print("  idtype: " + binascii.b2a_qp(otp_idtype).decode('Latin-1'))
                print("  vid: " + binascii.hexlify(otp_vid).decode('Latin-1'))
                print("  pid: " + binascii.hexlify(otp_pid).decode('Latin-1'))
                print("  coa: " + binascii.b2a_base64(otp_coa).decode('Latin-1'), end='')
                print("  sn: ", end='')
                for byte in range(0, 12, 4):
                    x = self.__getSN(byte)
                    x = x[::-1]  # reverse the bytes
                    print(binascii.hexlify(x).decode('Latin-1'), end='')  # show user
                print('')
            except Exception:
                # ignore bad character encodings
                pass

        if self.bl_rev >= 5:
            des = self.__getCHIPDes()
            if (len(des) == 2):
                print("ChipDes:")
                print("  family: %s" % des[0])
                print("  revision: %s" % des[1])
        print("Chip:")
        if self.bl_rev > 4:
            chip = self.__getCHIP()
            mcu_id = chip & 0xfff
            revs = {}

            F4_IDS = {
                0x413: "STM32F40x_41x",
                0x419: "STM32F42x_43x",
                0x421: "STM32F42x_446xx",
            }
            F7_IDS = {
                0x449: "STM32F74x_75x",
                0x451: "STM32F76x_77x",
            }
            H7_IDS = {
                0x450: "STM32H74x_75x",
            }

            family = mcu_id & 0xfff
            chip_s = "%x [unknown family/revision]" % (chip)

            if family in F4_IDS:
                mcu = F4_IDS[family]
                MCU_REV_STM32F4_REV_A = 0x1000
                MCU_REV_STM32F4_REV_Z = 0x1001
                MCU_REV_STM32F4_REV_Y = 0x1003
                MCU_REV_STM32F4_REV_1 = 0x1007
                MCU_REV_STM32F4_REV_3 = 0x2001
                revs = {
                    MCU_REV_STM32F4_REV_A: ("A", True),
                    MCU_REV_STM32F4_REV_Z: ("Z", True),
                    MCU_REV_STM32F4_REV_Y: ("Y", True),
                    MCU_REV_STM32F4_REV_1: ("1", True),
                    MCU_REV_STM32F4_REV_3: ("3", False),
                }
                rev = (chip & 0xFFFF0000) >> 16

                if rev in revs:
                    (label, flawed) = revs[rev]
                    if flawed and family == 0x419:
                        print("  %x %s rev%s (flawed; 1M limit, see STM32F42XX Errata sheet sec. 2.1.10)" % (chip, mcu, label,))
                    elif family == 0x419:
                        print("  %x %s rev%s (no 1M flaw)" % (chip, mcu, label,))
                    else:
                        print("  %x %s rev%s" % (chip, mcu, label,))
            elif family in F7_IDS:
                print("  %s %08x" % (F7_IDS[family], chip))
            elif family in H7_IDS:
                print("  %s %08x" % (H7_IDS[family], chip))
        else:
            print("  [unavailable; bootloader too old]")

        print("Info:")
        print("  flash size: %u" % self.fw_maxsize)
        name = self.board_name_for_board_id(self.board_type)
        if name is not None:
            print("  board_type: %u (%s)" % (self.board_type, name))
        else:
            print("  board_type: %u" % self.board_type)
        print("  board_rev: %u" % self.board_rev)

        print("Identification complete")

    def board_name_for_board_id(self, board_id):
        '''return name for board_id, None if it can't be found'''
        shared_ids = {
            9: "fmuv3",
            50: "fmuv5",
        }
        if board_id in shared_ids:
            return shared_ids[board_id]

        try:
            ret = []

            hwdef_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                     "..", "..", "libraries", "AP_HAL_ChibiOS", "hwdef")
            # uploader.py is swiped into other places, so if the dir
            # doesn't exist then fail silently
            if os.path.exists(hwdef_dir):
                dirs = [x if (x not in ["scripts","common","STM32CubeConf"] and os.path.isdir(os.path.join(hwdef_dir, x))) else None for x in os.listdir(hwdef_dir)]
                for adir in dirs:
                    if adir is None:
                        continue
                    filepath = os.path.join(hwdef_dir, adir, "hwdef.dat")
                    if not os.path.exists(filepath):
                        continue
                    fh = open(filepath)
                    if fh is None:
#                        print("Failed to open (%s)" % filepath)
                        continue
                    text = fh.readlines()
                    for line in text:
                        m = re.match("^\s*APJ_BOARD_ID\s+(\d+)\s*$", line)
                        if m is None:
                            continue
                        if int(m.group(1)) == board_id:
                            ret.append(adir)
            if len(ret) == 0:
                return None
            return " or ".join(ret)
        except Exception as e:
            print("Failed to get name: %s" % str(e))
        return None

    # upload the firmware
    def upload(self, fw, force=False, boot_delay=None):
        # Make sure we are doing the right thing
        if self.board_type != fw.property('board_id'):
            # ID mismatch: check compatibility
            incomp = True
            if self.board_type in compatible_IDs:
                comp_fw_id = compatible_IDs[self.board_type][0]
                board_name = compatible_IDs[self.board_type][1]
                if comp_fw_id == fw.property('board_id'):
                    msg = "Target %s (board_id: %d) is compatible with firmware for board_id=%u)" % (
                         board_name, self.board_type, fw.property('board_id'))
                    print("INFO: %s" % msg)
                    incomp = False
            if incomp:                        
                msg = "Firmware not suitable for this board (board_type=%u (%s) board_id=%u (%s))" % (
                    self.board_type,
                    self.board_name_for_board_id(self.board_type),
                    fw.property('board_id'),
                    self.board_name_for_board_id(fw.property('board_id')))
                print("WARNING: %s" % msg)
                
                if force:
                    print("FORCED WRITE, FLASHING ANYWAY!")
                else:
                    raise IOError(msg)

        self.dump_board_info()

        if self.fw_maxsize < fw.property('image_size'):
            raise RuntimeError("Firmware image is too large for this board")

        if self.baudrate_bootloader_flash != self.baudrate_bootloader:
            print("Setting baudrate to %u" % self.baudrate_bootloader_flash)
            self.__setbaud(self.baudrate_bootloader_flash)
            self.port.baudrate = self.baudrate_bootloader_flash
            self.__sync()

        self.__erase("Erase  ")
        self.__program("Program", fw)

        if self.bl_rev == 2:
            self.__verify_v2("Verify ", fw)
        else:
            self.__verify_v3("Verify ", fw)

        if boot_delay is not None:
            self.__set_boot_delay(boot_delay)

        print("\nRebooting.\n")
        self.__reboot()
        self.port.close()

    def __next_baud_flightstack(self):
        self.baudrate_flightstack_idx = self.baudrate_flightstack_idx + 1
        if self.baudrate_flightstack_idx >= len(self.baudrate_flightstack):
            return False

        try:
            self.port.baudrate = self.baudrate_flightstack[self.baudrate_flightstack_idx]
        except Exception:
            return False

        return True

    def send_reboot(self):
        if (not self.__next_baud_flightstack()):
            return False

        print("Attempting reboot on %s with baudrate=%d..." % (self.port.port, self.port.baudrate), file=sys.stderr)
        print("If the board does not respond, unplug and re-plug the USB connector.", file=sys.stderr)

        try:
            # try MAVLINK command first
            self.port.flush()
            if self.MAVLINK_REBOOT_ID1 is not None:
                self.__send(self.MAVLINK_REBOOT_ID1)
            if self.MAVLINK_REBOOT_ID0 is not None:
                self.__send(self.MAVLINK_REBOOT_ID0)
            # then try reboot via NSH
            self.__send(uploader.NSH_INIT)
            self.__send(uploader.NSH_REBOOT_BL)
            self.__send(uploader.NSH_INIT)
            self.__send(uploader.NSH_REBOOT)
            self.port.flush()
            self.port.baudrate = self.baudrate_bootloader
        except Exception:
            try:
                self.port.flush()
                self.port.baudrate = self.baudrate_bootloader
            except Exception:
                pass

        return True

    # upload the firmware
    def download(self, fw):
        if self.baudrate_bootloader_flash != self.baudrate_bootloader:
            print("Setting baudrate to %u" % self.baudrate_bootloader_flash)
            self.__setbaud(self.baudrate_bootloader_flash)
            self.port.baudrate = self.baudrate_bootloader_flash
            self.__sync()

        self.__download("Download", fw)
        self.port.close()
    
def ports_to_try(args):
    portlist = []
    if args.port is None:
        patterns = default_ports
    else:
        patterns = args.port.split(",")
    # use glob to support wildcard ports. This allows the use of
    # /dev/serial/by-id/usb-ArduPilot on Linux, which prevents the
    # upload from causing modem hangups etc
    if "linux" in _platform or "darwin" in _platform or "cygwin" in _platform:
        import glob
        for pattern in patterns:
            portlist += glob.glob(pattern)
    else:
        portlist = patterns

    # filter ports based on platform:
    if "cygwin" in _platform:
        # Cygwin, don't open MAC OS and Win ports, we are more like
        # Linux. Cygwin needs to be before Windows test
        pass
    elif "darwin" in _platform:
        # OS X, don't open Windows and Linux ports
        portlist = [port for port in portlist if "COM" not in port and "ACM" not in port]
    elif "win" in _platform:
        # Windows, don't open POSIX ports
        portlist = [port for port in portlist if "/" not in port]

    return portlist

def modemmanager_check():
    if os.path.exists("/usr/sbin/ModemManager"):
        print("""
==========================================================================================================
WARNING: You should uninstall ModemManager as it conflicts with any non-modem serial device (like Pixhawk)
==========================================================================================================
""")

def find_bootloader(up, port):
    while (True):
        up.open()

        # port is open, try talking to it
        try:
            # identify the bootloader
            up.identify()
            print("Found board %x,%x bootloader rev %x on %s" % (up.board_type, up.board_rev, up.bl_rev, port))
            return True

        except Exception as e:
            pass

        reboot_sent = up.send_reboot()

        # wait for the reboot, without we might run into Serial I/O Error 5
        time.sleep(0.25)

        # always close the port
        up.close()

        # wait for the close, without we might run into Serial I/O Error 6
        time.sleep(0.3)

        if not reboot_sent:
            return False


def main():

    # Parse commandline arguments
    parser = argparse.ArgumentParser(description="Firmware uploader for the PX autopilot system.")
    parser.add_argument('--port', action="store", help="Comma-separated list of serial port(s) to which the FMU may be attached",
                        default=None)
    parser.add_argument('--baud-bootloader', action="store", type=int, default=115200, help="Baud rate of the serial port (default is 115200) when communicating with bootloader, only required for true serial ports.")
    parser.add_argument('--baud-bootloader-flash', action="store", type=int, default=None, help="Attempt to negotiate this baudrate with bootloader for flashing.")
    parser.add_argument('--baud-flightstack', action="store", default="57600", help="Comma-separated list of baud rate of the serial port (default is 57600) when communicating with flight stack (Mavlink or NSH), only required for true serial ports.")
    parser.add_argument('--force', action='store_true', default=False, help='Override board type check and continue loading')
    parser.add_argument('--boot-delay', type=int, default=None, help='minimum boot delay to store in flash')
    parser.add_argument('--target-system', type=int, action="store", help="System ID to update")
    parser.add_argument('--target-component', type=int, action="store", help="Component ID to update")
    parser.add_argument('--source-system', type=int, action="store", help="Source system to send reboot mavlink packets from", default=255)
    parser.add_argument('--source-component', type=int, action="store", help="Source component to send reboot mavlink packets from", default=0)
    parser.add_argument('--download', action='store_true', default=False, help='download firmware from board')
    parser.add_argument('--identify', action="store_true", help="Do not flash firmware; simply dump information about board")
    parser.add_argument('firmware', nargs="?", action="store", default=None, help="Firmware file to be uploaded")
    args = parser.parse_args()

    # warn people about ModemManager which interferes badly with Pixhawk
    modemmanager_check()

    if args.firmware is None and not args.identify:
        parser.error("Firmware filename required for upload or download")
        sys.exit(1)

    # Load the firmware file
    if not args.download and not args.identify:
        fw = firmware(args.firmware)
        print("Loaded firmware for %x,%x, size: %d bytes, waiting for the bootloader..." % (fw.property('board_id'), fw.property('board_revision'), fw.property('image_size')))
    print("If the board does not respond within 1-2 seconds, unplug and re-plug the USB connector.")

    baud_flightstack = [int(x) for x in args.baud_flightstack.split(',')]

    # Spin waiting for a device to show up
    try:
        while True:

            for port in ports_to_try(args):

                # print("Trying %s" % port)

                # create an uploader attached to the port
                try:
                    up = uploader(port,
                                  args.baud_bootloader,
                                  baud_flightstack,
                                  args.baud_bootloader_flash,
                                  args.target_system,
                                  args.target_component,
                                  args.source_system,
                                  args.source_component)

                except Exception as e:
                    if not is_WSL:
                        # open failed, WSL must cycle through all ttyS* ports quickly but rate limit everything else
                        print("Exception creating uploader: %s" % str(e))
                        time.sleep(0.05)

                    # and loop to the next port
                    continue

                if not find_bootloader(up, port):
                    # Go to the next port
                    continue

                try:
                    # ok, we have a bootloader, try flashing it
                    if args.identify:
                        up.dump_board_info()
                    elif args.download:
                        up.download(args.firmware)
                    else:
                        up.upload(fw, force=args.force, boot_delay=args.boot_delay)

                except RuntimeError as ex:
                    # print the error
                    print("\nERROR: %s" % ex.args)

                except IOError:
                    up.close()
                    continue

                finally:
                    # always close the port
                    up.close()

                # we could loop here if we wanted to wait for more boards...
                sys.exit(0)

            # Delay retries to < 20 Hz to prevent spin-lock from hogging the CPU
            time.sleep(0.05)

    # CTRL+C aborts the upload/spin-lock by interrupt mechanics
    except KeyboardInterrupt:
        print("\n Upload aborted by user.")
        sys.exit(0)

if __name__ == '__main__':
    main()

# vim: tabstop=4 expandtab shiftwidth=4 softtabstop=4
