# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Read SITL thread diagnostics over MAVFTP, including Rust worker reloads."""

import os
import re
import struct
import sys

import apmod_info
import pack
import pytest

from pymavlink import mavftp_op as ftp
from pymavlink.mavftp import FtpError
from test_runtime import Vehicle  # noqa: F401
from test_runtime import images  # noqa: F401


class FTPReader:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.seq = 0
        self.session = 0

    def request(self, opcode, payload=b'', size=None, offset=0):
        op = ftp.FTP_OP(self.seq, self.session, opcode, len(payload) if size is None else size,
                        0, 0, offset, bytearray(payload))
        packet = op.pack()
        packet.extend(bytes(251 - len(packet)))
        self.vehicle.link.mav.file_transfer_protocol_send(0, 1, 1, packet)
        message = self.vehicle.until(lambda m: m.get_type() == 'FILE_TRANSFER_PROTOCOL'
                                     and m.payload[5] == opcode
                                     and struct.unpack_from('<H', bytes(m.payload))[0] == (self.seq + 1) % 65536)
        seq, self.session, result, length, _, _, _, offset = struct.unpack('<HBBBBBBI', bytes(message.payload[:12]))
        self.seq = (seq + 1) % 65536
        return result, bytes(message.payload[12:12 + length])

    def read(self, path):
        status, _ = self.request(ftp.OP_ResetSessions)
        assert status == ftp.OP_Ack
        status, payload = self.request(ftp.OP_OpenFileRO, path.encode() + b'\0')
        assert status == ftp.OP_Ack, payload
        length, = struct.unpack('<I', payload)
        contents = bytearray()
        try:
            while True:
                status, payload = self.request(ftp.OP_ReadFile, size=239, offset=len(contents))
                if status == ftp.OP_Nack:
                    assert payload == bytes([FtpError.EndOfFile]), payload
                    break
                assert status == ftp.OP_Ack and payload, (status, payload)
                contents.extend(payload)
                assert len(contents) <= length
        finally:
            status, _ = self.request(ftp.OP_TerminateSession)
            assert status == ftp.OP_Ack
        # @SYS advertises a placeholder stat size; EOF terminates the snapshot.
        assert 0 < len(contents) <= length
        return contents.decode()


@pytest.mark.parametrize('rust', [False, True])
def test_threads_over_ftp(tmp_path, images, rust):  # noqa: F811
    modules = {'diag.apmod': images['cleanup'], 'gone.apmod': images['return']} if rust else None
    vehicle = Vehicle(tmp_path / 'vehicle', modules, {'SCR_ENABLE': 1, 'MOD_ENABLE': int(rust)})
    try:
        vehicle.start()
        if rust:
            vehicle.text('Module: exited gone.apmod')
        reader = FTPReader(vehicle)

        def snapshot():
            listing = reader.read('@SYS/threads.txt')
            print(listing, flush=True)
            assert listing.startswith('ThreadsV2\n')
            names = [line.split()[0] for line in listing.splitlines()[1:]]
            assert 'main' in names and 'Scripting' in names and 'FTP' in names
            assert 'gone.apmod' not in names
            # Default SITL libc stacks have no high-water mark instrumentation.
            assert re.search(r'^Scripting\s+PRI=\s*\d+ sp=\S+ STACK=\?/\d+', listing, re.M)
            if rust:
                assert names.count('ModMgr') == names.count('diag.apmod') == 1
                match = re.search(r'^diag.apmod\s+PRI=\s*0 sp=\S+ STACK=(\d+)/(\d+)$', listing, re.M)
                assert match, listing
                free, total = map(int, match.groups())
                required = pack.unpack(images['cleanup'])['contract']['stack_size'] + 16384  # x86-64 host-call margin
                page = os.sysconf('SC_PAGESIZE')
                expected = (required + page - 1) // page * page
                assert 0 < free < total == expected
            else:
                assert 'ModMgr' not in names
            return listing

        snapshot()
        if rust:
            for generation in range(1, 6):
                vehicle.restart(generation, expected=2)
                vehicle.pump(0.1)
                snapshot()
            (vehicle.modules / 'diag.apmod').unlink()
            (vehicle.modules / 'gone.apmod').unlink()
            vehicle.restart(6, expected=0)
            assert 'diag.apmod' not in reader.read('@SYS/threads.txt')
    finally:
        vehicle.stop()


@pytest.mark.parametrize('count', [0, 1, 12])
def test_modules_over_ftp(tmp_path, images, count):  # noqa: F811
    names = [f'module_{index:02d}_with_a_long_diagnostic_name.apmod' for index in range(count)]
    heaps = {name: 4096 if index % 2 == 0 else 8192 for index, name in enumerate(names)}
    modules = {name: apmod_info.resize_resources(images['cleanup'], heap_size=heaps[name]) for name in names}
    vehicle = Vehicle(tmp_path / 'vehicle', modules, {'SCR_ENABLE': 0, 'MOD_ENABLE': int(count > 0), 'MOD_HEAP_MAX': 8192})
    try:
        vehicle.start()
        reader = FTPReader(vehicle)

        def snapshot(complete):
            listing = reader.read('@SYS/modules.txt')
            assert listing.startswith('ModulesV1\n'), listing
            rows = {}
            for line in listing.splitlines()[1:]:
                name, *fields = line.split()
                assert name in heaps and name not in rows, listing
                values = dict(field.split('=', 1) for field in fields)
                rows[name] = values
                assert values['SRC'] == 'FILE' and int(values['PRI']) == 1, listing
                assert not {'SOURCE', 'STACK', 'USED', 'FREE'} & values.keys(), listing
                free, capacity = map(int, values['HEAP'].split('/'))
                assert 0 <= free <= capacity, listing
                if values['STATE'] == 'RUN':
                    assert capacity == heaps[name] and int(values['ALLOC']) >= 512, listing
                    assert int(values['PEAK']) >= int(values['ALLOC']) and int(values['ALLOCS']) >= 1, listing
                    assert int(values['STK']) > 0 and int(values['RX']) > 0, listing
                    if 'LOAD' in values or sys.platform.startswith('linux'):
                        assert re.fullmatch(r'\d+(?:\.\d+)?%', values['LOAD']), listing
                        assert 0 <= float(values['LOAD'][:-1]) <= 100, listing
            if complete:
                assert set(rows) == set(names), listing
                assert all(row['STATE'] == 'RUN' for row in rows.values()), listing
            return listing

        snapshot(True)
        if count:
            for generation in range(1, 6):
                mark = len(vehicle.texts)
                vehicle.parameter('MOD_RESTART', generation)
                snapshot(False)  # read while handles may be loading or reaping
                vehicle.text(f'Module: loaded {count} modules', after=mark)
                vehicle.pump(0.1)
                snapshot(True)
            for name in names:
                (vehicle.modules / name).unlink()
            vehicle.restart(6, expected=0)
            assert reader.read('@SYS/modules.txt') == 'ModulesV1\n'
    finally:
        vehicle.stop()


@pytest.mark.parametrize('mode', ['panic', 'late_stop'])
def test_retained_module_diagnostics(tmp_path, images, mode):  # noqa: F811
    vehicle = Vehicle(tmp_path / 'vehicle', {'retained.apmod': images[mode]}, {'SCR_ENABLE': 0})
    try:
        vehicle.start()
        if mode == 'panic':
            vehicle.text('panic; reboot required')
        reader = FTPReader(vehicle)
        listing = reader.read('@SYS/modules.txt')
        assert 'retained.apmod STATE=' + ('PANICKED' if mode == 'panic' else 'RUN') in listing, listing
        if mode == 'late_stop' and sys.platform.startswith('linux'):
            match = re.search(r'LOAD=(\d+(?:\.\d+)?)%', listing)
            assert match and float(match.group(1)) > 0, listing
        (vehicle.modules / 'retained.apmod').unlink()
        mark = len(vehicle.texts)
        vehicle.parameter('MOD_RESTART', 1)
        vehicle.text('stop timeout', after=mark)
        listing = reader.read('@SYS/modules.txt')
        assert 'retained.apmod STATE=' in listing, listing
        if mode == 'late_stop':
            vehicle.pump(4)  # the delayed worker eventually exits and is reaped
            assert reader.read('@SYS/modules.txt') == 'ModulesV1\n'
    finally:
        vehicle.stop()
