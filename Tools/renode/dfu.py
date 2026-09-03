#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Minimal STM32 factory-ROM compatible DfuSe device over USB/IP."""

import errno
import os
import socket
import struct
import threading

from pathlib import Path

USBIP_VERSION = 0x0111
OP_REQ_DEVLIST = 0x8005
OP_REP_DEVLIST = 0x0005
OP_REQ_IMPORT = 0x8003
OP_REP_IMPORT = 0x0003
CMD_SUBMIT = 1
CMD_UNLINK = 2
RET_SUBMIT = 3
RET_UNLINK = 4
DIR_OUT = 0
SPEED_FULL = 2
BUS_ID = '1-0'

STATUS_OK = 0
STATUS_ERASE = 4
STATUS_ADDRESS = 8
USB_OK = 0
USB_STALL = -errno.EPIPE
USB_UNLINKED = -errno.ECONNRESET

DFU_DETACH = 0
DFU_DNLOAD = 1
DFU_UPLOAD = 2
DFU_GETSTATUS = 3
DFU_CLRSTATUS = 4
DFU_GETSTATE = 5
DFU_ABORT = 6

STATE_DFU_IDLE = 2
STATE_DFU_DNLOAD_SYNC = 3
STATE_DFU_DNBUSY = 4
STATE_DFU_DNLOAD_IDLE = 5
STATE_DFU_MANIFEST_SYNC = 6
STATE_DFU_MANIFEST = 7
STATE_DFU_UPLOAD_IDLE = 9
STATE_DFU_ERROR = 10

STM32_VENDOR_ID = 0x0483
STM32_DFU_PRODUCT_ID = 0xDF11
FLASH_BASE = 0x08000000
TRANSFER_SIZE = 2048


def flash_sectors(family, flash_size):
    """Return DfuSe erase sectors for supported STM32 families."""
    if family in ('f405', 'f407', 'f427'):
        bank_size = min(flash_size, 1024 * 1024)
        bank_geometry = (
            (4, 16 * 1024),
            (1, 64 * 1024),
            ((bank_size - 128 * 1024) // (128 * 1024), 128 * 1024),
        )
        banks = (flash_size + bank_size - 1) // bank_size
        geometry = bank_geometry * banks
    elif family in ('h743', 'h757'):
        geometry = ((flash_size // (128 * 1024), 128 * 1024),)
    else:
        raise ValueError('USB DFU is unsupported for STM32 family %s' % family)
    if sum(count * size for count, size in geometry) != flash_size:
        raise ValueError('unsupported %s flash size %u' % (family, flash_size))
    sectors = []
    address = FLASH_BASE
    for count, size in geometry:
        for _index in range(count):
            sectors.append((address, size))
            address += size
    return tuple(sectors)


def memory_layout(family, flash_size):
    sectors = flash_sectors(family, flash_size)
    runs = []
    for _address, size in sectors:
        if runs and runs[-1][1] == size:
            runs[-1] = (runs[-1][0] + 1, size)
        else:
            runs.append((1, size))
    descriptions = []
    for count, size in runs:
        unit = 'K' if size % 1024 == 0 else 'B'
        value = size // 1024 if unit == 'K' else size
        descriptions.append('%02u*%03u%sg' % (count, value, unit))
    return '@Internal Flash /0x%08X/%s' % (
        FLASH_BASE, ','.join(descriptions))


def device_descriptor():
    return struct.pack(
        '<BBHBBBBHHHBBBB',
        18, 1, 0x0200, 0, 0, 0, 64,
        STM32_VENDOR_ID, STM32_DFU_PRODUCT_ID, 0x2200,
        1, 2, 3, 1)


DFU_CONFIG_DESCRIPTOR = b''.join((
    struct.pack('<BBHBBBBB', 9, 2, 27, 1, 1, 0, 0x80, 50),
    struct.pack('<BBBBBBBBB', 9, 4, 0, 0, 0, 0xFE, 0x01, 0x02, 4),
    struct.pack('<BBBHHH', 9, 0x21, 0x0B, 255, TRANSFER_SIZE, 0x011A),
))


class DfuDevice:
    """Serve a DfuSe endpoint which modifies a persistent STM32 flash file."""

    def __init__(self, flash, family, flash_size, host='127.0.0.1', port=3240,
                 log=None):
        self.flash = Path(flash)
        self.family = family
        self.flash_size = flash_size
        self.sectors = flash_sectors(family, flash_size)
        if not self.flash.is_file() or self.flash.stat().st_size != flash_size:
            raise ValueError('%s is not a %u-byte flash image' %
                             (self.flash, flash_size))
        self.host = host
        self.port = port
        self.log = log or (lambda _message: None)
        self.address = FLASH_BASE
        self.state = STATE_DFU_IDLE
        self.status = STATUS_OK
        self.manifested = threading.Event()
        self._busy_once = False
        self._manifest_scheduled = False
        self._flash_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._connection = None
        self._running = True
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind((host, port))
        self.port = self._socket.getsockname()[1]
        self._socket.listen(1)
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def close(self):
        self._running = False
        try:
            self._socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._socket.close()
        with self._send_lock:
            connection = self._connection
            self._connection = None
            if connection is not None:
                try:
                    # A manifested USB device disappears abruptly. Reset the
                    # USB/IP connection so this TCP port can be rebound by
                    # Renode immediately instead of entering TIME_WAIT.
                    connection.setsockopt(
                        socket.SOL_SOCKET, socket.SO_LINGER,
                        struct.pack('ii', 1, 0))
                    # Wake the receive thread without sending a FIN; close()
                    # then resets the connection and avoids TCP TIME_WAIT.
                    connection.shutdown(socket.SHUT_RD)
                except OSError:
                    pass
                connection.close()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=2)

    def wait_for_manifest(self, stop=None):
        while not self.manifested.wait(0.2):
            if stop is not None and stop():
                return False
        return True

    def _serve(self):
        while self._running:
            try:
                connection, address = self._socket.accept()
            except OSError:
                return
            if not self._running:
                connection.close()
                return
            self.log('USB/IP connection from %s' % (address,))
            with self._send_lock:
                self._connection = connection
            try:
                self._session(connection)
            except (OSError, struct.error, UnicodeError) as error:
                if self._running:
                    self.log('USB/IP session ended: %s' % error)
            finally:
                with self._send_lock:
                    if self._connection is connection:
                        self._connection = None
                connection.close()

    @staticmethod
    def _receive(connection, length):
        result = b''
        while len(result) < length:
            data = connection.recv(length - len(result))
            if not data:
                raise OSError('connection closed')
            result += data
        return result

    def _session(self, connection):
        while True:
            version, command, _status = struct.unpack(
                '>HHI', self._receive(connection, 8))
            if version != USBIP_VERSION:
                raise OSError('unsupported USB/IP version 0x%04x' % version)
            if command == OP_REQ_DEVLIST:
                connection.sendall(
                    struct.pack('>HHII', USBIP_VERSION, OP_REP_DEVLIST, 0, 1) +
                    self._usb_device() + bytes((0xFE, 0x01, 0x02, 0)))
                continue
            if command != OP_REQ_IMPORT:
                raise OSError('unsupported USB/IP request 0x%04x' % command)
            bus_id = self._receive(connection, 32).split(b'\0')[0].decode()
            if bus_id != BUS_ID:
                connection.sendall(
                    struct.pack('>HHI', USBIP_VERSION, OP_REP_IMPORT, 1))
                return
            connection.sendall(
                struct.pack('>HHI', USBIP_VERSION, OP_REP_IMPORT, 0) +
                self._usb_device())
            break

        while self._running:
            header = self._receive(connection, 48)
            command, sequence, _device, direction, endpoint = struct.unpack(
                '>IIIII', header[:20])
            if command == CMD_UNLINK:
                self._return_unlink(
                    connection, sequence, struct.unpack('>I', header[20:24])[0])
                continue
            if command != CMD_SUBMIT:
                raise OSError('unsupported USB/IP command %u' % command)
            _flags, length, _start, _packets, _interval = struct.unpack(
                '>Iiiii', header[20:40])
            data = (self._receive(connection, length)
                    if direction == DIR_OUT and length else b'')
            self._submit(connection, sequence, direction, endpoint,
                         length, header[40:48], data)

    def _usb_device(self):
        path = '/sys/devices/platform/vhci_hcd.0/usb1/%s' % BUS_ID
        return struct.pack(
            '>256s32sIIIHHHBBBBBB',
            path.encode(), BUS_ID.encode(), 1, 1, SPEED_FULL,
            STM32_VENDOR_ID, STM32_DFU_PRODUCT_ID, 0x2200,
            0, 0, 0, 1, 1, 1)

    def _submit(self, connection, sequence, direction, endpoint, length,
                setup, data):
        if endpoint != 0:
            self._return_submit(connection, sequence, USB_STALL)
            return
        status, reply = self._control(setup, data)
        actual = len(data) if direction == DIR_OUT else min(len(reply), length)
        self._return_submit(connection, sequence, status, reply[:length], actual)

    @staticmethod
    def _return_submit(connection, sequence, status, data=b'', actual=None):
        if actual is None:
            actual = len(data)
        header = struct.pack('>IIIII', RET_SUBMIT, sequence, 0, 0, 0)
        header += struct.pack('>iiiii8s', status, actual, 0, 0, 0, b'')
        connection.sendall(header + data)

    @staticmethod
    def _return_unlink(connection, sequence, _victim):
        header = struct.pack('>IIIII', RET_UNLINK, sequence, 0, 0, 0)
        connection.sendall(header + struct.pack('>i24s', USB_UNLINKED, b''))

    def _string_descriptor(self, index):
        strings = (
            'STMicroelectronics',
            'STM32 BOOTLOADER',
            'RENODE-STM32-DFU',
            memory_layout(self.family, self.flash_size),
        )
        if index == 0:
            return bytes((4, 3, 0x09, 0x04))
        if not 1 <= index <= len(strings):
            return None
        body = strings[index - 1].encode('utf-16-le')
        return bytes((len(body) + 2, 3)) + body

    def _control(self, setup, data):
        request_type, request, value, _index, length = struct.unpack(
            '<BBHHH', setup)
        if (request_type & 0x60) == 0:
            if request == 0x06:
                descriptor_type, descriptor_index = value >> 8, value & 0xFF
                if descriptor_type == 1:
                    return USB_OK, device_descriptor()[:length]
                if descriptor_type == 2:
                    return USB_OK, DFU_CONFIG_DESCRIPTOR[:length]
                if descriptor_type == 3:
                    descriptor = self._string_descriptor(descriptor_index)
                    return ((USB_STALL, b'') if descriptor is None else
                            (USB_OK, descriptor[:length]))
                return USB_STALL, b''
            if request in (0x01, 0x03, 0x05, 0x09, 0x0B):
                return USB_OK, b''
            if request == 0x00:
                return USB_OK, b'\0\0'[:length]
            if request == 0x08:
                return USB_OK, b'\1'[:length]
            if request == 0x0A:
                return USB_OK, b'\0'[:length]
            return USB_STALL, b''

        if request == DFU_DNLOAD:
            return self._download(value, data)
        if request == DFU_UPLOAD:
            return self._upload(value, length)
        if request == DFU_GETSTATUS:
            return USB_OK, self._get_status()
        if request == DFU_GETSTATE:
            return USB_OK, bytes((self.state,))
        if request == DFU_CLRSTATUS:
            self.status = STATUS_OK
            self.state = STATE_DFU_IDLE
            return USB_OK, b''
        if request == DFU_ABORT:
            self.state = STATE_DFU_IDLE
            return USB_OK, b''
        if request == DFU_DETACH:
            self._schedule_manifest()
            return USB_OK, b''
        return USB_STALL, b''

    def _download(self, block, data):
        if not data:
            self.state = STATE_DFU_MANIFEST_SYNC
            self._busy_once = False
            return USB_OK, b''
        try:
            if block == 0:
                self._dfuse_command(data)
            elif block >= 2:
                self._write(self.address + (block - 2) * TRANSFER_SIZE, data)
            else:
                raise ValueError('unsupported DfuSe block')
        except ValueError as error:
            self.log('DFU download rejected: %s' % error)
            self.status = STATUS_ADDRESS
            self.state = STATE_DFU_ERROR
            return USB_OK, b''
        self.state = STATE_DFU_DNLOAD_SYNC
        self._busy_once = True
        return USB_OK, b''

    def _upload(self, block, length):
        if block < 2:
            return USB_STALL, b''
        try:
            result = self._read(
                self.address + (block - 2) * TRANSFER_SIZE,
                min(length, TRANSFER_SIZE))
        except ValueError:
            self.status = STATUS_ADDRESS
            self.state = STATE_DFU_ERROR
            return USB_OK, b''
        self.state = STATE_DFU_UPLOAD_IDLE
        return USB_OK, result

    def _dfuse_command(self, data):
        if data[0] == 0x21 and len(data) == 5:
            address = struct.unpack('<I', data[1:])[0]
            self._validate(address, 0)
            self.address = address
            return
        if data[0] == 0x41 and len(data) == 5:
            self._erase_sector(struct.unpack('<I', data[1:])[0])
            return
        raise ValueError('unsupported DfuSe command 0x%02x' % data[0])

    def _get_status(self):
        state = self.state
        timeout_ms = 0
        if state == STATE_DFU_DNLOAD_SYNC:
            state = STATE_DFU_DNBUSY if self._busy_once else STATE_DFU_DNLOAD_IDLE
            timeout_ms = 1 if self._busy_once else 0
            self._busy_once = False
            self.state = state
        elif state == STATE_DFU_DNBUSY:
            state = STATE_DFU_DNLOAD_IDLE
            self.state = state
        elif state == STATE_DFU_MANIFEST_SYNC:
            state = STATE_DFU_MANIFEST
            self.state = state
            self._schedule_manifest()
        return bytes((self.status, timeout_ms, 0, 0, state, 0))

    def _validate(self, address, size):
        if (address < FLASH_BASE or
                address + size > FLASH_BASE + self.flash_size):
            raise ValueError('address 0x%08x is outside internal flash' % address)

    def _write(self, address, data):
        self._validate(address, len(data))
        with self._flash_lock, self.flash.open('r+b') as stream:
            stream.seek(address - FLASH_BASE)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())

    def _read(self, address, length):
        self._validate(address, length)
        with self._flash_lock, self.flash.open('rb') as stream:
            stream.seek(address - FLASH_BASE)
            return stream.read(length)

    def _erase_sector(self, address):
        try:
            _start, size = next(sector for sector in self.sectors
                                if sector[0] == address)
        except StopIteration as error:
            self.status = STATUS_ERASE
            raise ValueError('0x%08x is not a sector boundary' % address) from error
        self._write(address, b'\xff' * size)

    def _schedule_manifest(self):
        if self._manifest_scheduled:
            return
        self._manifest_scheduled = True
        timer = threading.Timer(0.2, self.manifested.set)
        timer.daemon = True
        timer.start()
