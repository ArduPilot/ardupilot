#!/usr/bin/env python3
# AP_FLAKE8_CLEAN

"""Run the slow secure-boot regression suite on emulated flight controllers."""

import argparse
import base64
import importlib.util
import os
import re
import shutil
import signal
import socket
import struct
import subprocess
import sys
import time

from pathlib import Path

import monocypher

ROOT = Path(__file__).resolve().parents[3]
OUTPUT = ROOT / 'build' / 'renode-signing-security'
FORGED_ZERO_KEY_SIGNATURE = bytes.fromhex('58' + '66' * 31 + '01' + '00' * 31)
GET_SESSION_KEY = 0
GET_PUBLIC_KEYS = 3
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_DENIED = 2
MAV_RESULT_FAILED = 4


def run_checked(command):
    print('+', ' '.join(str(value) for value in command), flush=True)
    subprocess.run([str(value) for value in command], cwd=ROOT, check=True)


def write_key_pair(name, private_key):
    public_key = monocypher.compute_signing_public_key(private_key)
    prefix = OUTPUT / name
    private_path = prefix.with_name(prefix.name + '_private_key.dat')
    public_path = prefix.with_name(prefix.name + '_public_key.dat')
    private_path.write_text(
        'PRIVATE_KEYV1:' + base64.b64encode(private_key).decode('ascii'))
    public_path.write_text(
        'PUBLIC_KEYV1:' + base64.b64encode(public_key).decode('ascii'))
    return private_path, public_path


def configure_and_build(board, target, *options):
    run_checked([
        sys.executable, ROOT / 'waf', 'configure', '--board', board, *options,
    ])
    # The weekly job builds SITL first. Clear generated DroneCAN headers from
    # that configuration before changing Python versions or ChibiOS boards.
    run_checked([sys.executable, ROOT / 'waf', 'clean'])
    run_checked([sys.executable, ROOT / 'waf', target])


def build_bootloader(board):
    configure_and_build(board, 'bootloader', '--bootloader', '--signed-fw')
    return ROOT / 'build' / board / 'bin' / 'AP_Bootloader.bin'


def provision_bootloader(source, public_key, destination):
    shutil.copyfile(source, destination)
    run_checked([
        ROOT / 'Tools' / 'scripts' / 'signing' / 'make_secure_bl.py',
        '--omit-ardupilot-keys', destination, public_key,
    ])


def build_signed_copter(board, private_key, extra_hwdef=None,
                        artifact_name=None, configure=True, elf=False):
    options = ['--signed-fw', '--private-key', str(private_key)]
    if extra_hwdef is not None:
        options += ['--extra-hwdef', str(extra_hwdef)]
    if configure:
        configure_and_build(board, 'copter', *options)
    else:
        run_checked([sys.executable, ROOT / 'waf', 'copter'])
    extension = 'elf' if elf else 'apj'
    source_name = 'arducopter' if elf else 'arducopter.apj'
    destination = OUTPUT / (artifact_name or '%s-arducopter.%s' % (board, extension))
    shutil.copyfile(ROOT / 'build' / board / 'bin' / source_name, destination)
    return destination


def build_unsigned_copter(board, extra_hwdef, artifact_name):
    configure_and_build(
        board, 'copter', '--extra-hwdef', str(extra_hwdef))
    destination = OUTPUT / artifact_name
    shutil.copyfile(ROOT / 'build' / board / 'bin' / 'arducopter.apj', destination)
    return destination


def check_embedded_bootloader(candidate):
    firmware = (ROOT / 'build' / 'CubeOrangePlus' / 'bin' /
                'arducopter.bin').read_bytes()
    if firmware.count(candidate.read_bytes()) != 1:
        raise AssertionError('embedded bootloader changed during application build')


def generate_dialect():
    output = OUTPUT / 'ardupilotmega_security'
    run_checked([
        sys.executable,
        ROOT / 'modules' / 'mavlink' / 'pymavlink' / 'tools' / 'mavgen.py',
        '--lang', 'Python3', '--wire-protocol', '2.0', '--output', output,
        ROOT / 'modules' / 'mavlink' / 'message_definitions' / 'v1.0' /
        'ardupilotmega.xml',
    ])
    module_path = output.with_suffix('.py')
    spec = importlib.util.spec_from_file_location('ardupilotmega_security', module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError('failed to load generated MAVLink dialect')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def free_port():
    sock = socket.socket()
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


class SocketWriter:
    def __init__(self, sock):
        self.sock = sock

    def write(self, data):
        self.sock.sendall(data)


class RenodeBoard:
    def __init__(self, dialect, renode, board, firmware, bootloader,
                 name, timeout, expect_heartbeat=True):
        self.dialect = dialect
        self.timeout = timeout
        self.uart_port = free_port()
        self.monitor_port = free_port()
        state_dir = OUTPUT / name
        self.state_dir = state_dir
        log_path = OUTPUT / ('%s.log' % name)
        command = [
            sys.executable, ROOT / 'Tools' / 'renode' / 'run.py', board,
            '--firmware', firmware,
            '--bootloader', bootloader,
            '--renode', renode,
            '--state-dir', state_dir,
            '--uart-port', str(self.uart_port),
            '--port', str(self.monitor_port),
            '--unthrottled',
            '--no-device-sidecars',
        ]
        print('+', ' '.join(str(value) for value in command), flush=True)
        self.log = log_path.open('wb')
        self.process = subprocess.Popen(
            [str(value) for value in command], cwd=ROOT,
            stdout=self.log, stderr=subprocess.STDOUT,
            start_new_session=True)
        self.sock = None
        self.mav = None
        self.pending_messages = []
        try:
            self._connect()
            if expect_heartbeat:
                self.wait_message('HEARTBEAT', timeout)
        except BaseException:
            self.close()
            raise

    def _connect(self):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError('Renode exited before opening its UART')
            sock = socket.socket()
            sock.settimeout(1)
            try:
                sock.connect(('127.0.0.1', self.uart_port))
            except OSError:
                sock.close()
                time.sleep(0.25)
                continue
            self.sock = sock
            self.mav = self.dialect.MAVLink(
                SocketWriter(sock), srcSystem=255, srcComponent=0)
            self.mav.robust_parsing = True
            return
        raise TimeoutError('timed out connecting to Renode UART')

    def wait_message(self, message_type, timeout, predicate=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for index, message in enumerate(self.pending_messages):
                if (message.get_type() == message_type and
                        (predicate is None or predicate(message))):
                    return self.pending_messages.pop(index)
            if self.process.poll() is not None:
                raise RuntimeError('Renode exited while waiting for %s' % message_type)
            self.sock.settimeout(min(1, max(0.01, deadline - time.monotonic())))
            try:
                data = self.sock.recv(4096)
            except socket.timeout:
                continue
            if not data:
                raise RuntimeError('Renode UART closed while waiting for %s' % message_type)
            for value in data:
                message = self.mav.parse_char(bytes((value,)))
                if message is not None and message.get_type() != 'BAD_DATA':
                    self.pending_messages.append(message)
        for index, message in enumerate(self.pending_messages):
            if (message.get_type() == message_type and
                    (predicate is None or predicate(message))):
                return self.pending_messages.pop(index)
        raise TimeoutError('timed out waiting for %s' % message_type)

    def secure_command(self, sequence, operation, data, signature):
        payload = data + signature
        if len(payload) > 220:
            raise ValueError('secure command payload is too large')
        payload += bytes(220 - len(payload))
        self.mav.secure_command_send(
            1, 1, sequence, operation, len(data), len(signature), payload)
        return self.wait_message(
            'SECURE_COMMAND_REPLY', self.timeout,
            lambda message: message.sequence == sequence and
            message.operation == operation)

    def pause(self):
        monitor = socket.create_connection(
            ('127.0.0.1', self.monitor_port), timeout=5)
        monitor.settimeout(1)
        try:
            try:
                monitor.recv(4096)
            except socket.timeout:
                pass
            monitor.sendall(b'pause\n')
            deadline = time.monotonic() + 10
            response = b''
            while time.monotonic() < deadline:
                try:
                    response += monitor.recv(4096)
                except socket.timeout:
                    continue
                if b'Pausing emulation' in response:
                    return
            raise TimeoutError('timed out pausing Renode')
        finally:
            monitor.close()

    def monitor_value(self, command):
        return self.monitor_values([command])[0]

    def monitor_values(self, commands):
        monitor = socket.create_connection(
            ('127.0.0.1', self.monitor_port), timeout=5)
        monitor.settimeout(1)
        try:
            deadline = time.monotonic() + 10
            response = b''
            while time.monotonic() < deadline and b'(ardupilot)' not in response:
                try:
                    response += monitor.recv(4096)
                except socket.timeout:
                    continue
            results = []
            for command in commands:
                monitor.sendall(command.encode('ascii') + b'\n')
                deadline = time.monotonic() + 10
                response = b''
                while time.monotonic() < deadline:
                    try:
                        response += monitor.recv(4096)
                    except socket.timeout:
                        continue
                    values = re.findall(rb'0x[0-9A-Fa-f]+', response)
                    if values and b'(ardupilot)' in response:
                        results.append(int(values[-1], 16))
                        break
                else:
                    raise TimeoutError(
                        'timed out reading Renode monitor value for %s' % command)
            return results
        finally:
            monitor.close()

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        self.log.close()

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()


def signed_command(private_key, sequence, operation, data=b'', session_key=b''):
    transcript = struct.pack('<II', sequence, operation) + data + session_key
    return monocypher.signature_sign(private_key, transcript)


def external_elf_image(path):
    data = path.read_bytes()
    phoff, = struct.unpack_from('<I', data, 0x1C)
    phentsize, phnum = struct.unpack_from('<HH', data, 0x2A)
    for index in range(phnum):
        offset = phoff + index * phentsize
        p_type, p_offset, p_vaddr, _p_paddr, p_filesz = struct.unpack_from(
            '<IIIII', data, offset)
        if p_type == 1 and p_vaddr == 0x90000000 and p_filesz:
            return data[p_offset:p_offset + p_filesz], p_offset
    raise ValueError('%s has no external-flash ELF segment' % path)


def fnv1a64(data):
    value = 14695981039346656037
    for byte in data:
        value = ((value ^ byte) * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return value


def tamper_external_elf(source, destination):
    data = bytearray(source.read_bytes())
    external, offset = external_elf_image(source)
    tamper_offset = len(external) // 2
    data[offset + tamper_offset] ^= 1
    destination.write_bytes(data)
    return bytes(data[offset:offset + len(external)])


def test_cubeorange_commands(dialect, renode, firmware, bootloader,
                             private_key, timeout):
    with RenodeBoard(
            dialect, renode, 'CubeOrange', firmware, bootloader,
            'cubeorange-commands', timeout) as board:
        forged_transcript = struct.pack('<II', 14, GET_SESSION_KEY)
        if not monocypher.signature_check(
                FORGED_ZERO_KEY_SIGNATURE, bytes(32), forged_transcript):
            raise AssertionError('zero-key regression vector is invalid')
        reply = board.secure_command(
            14, GET_SESSION_KEY, b'', FORGED_ZERO_KEY_SIGNATURE)
        if reply.result != MAV_RESULT_DENIED:
            raise AssertionError('all-zero public-key slot accepted a forged command')

        sequence = 20
        reply = board.secure_command(
            sequence, GET_SESSION_KEY, b'',
            signed_command(private_key, sequence, GET_SESSION_KEY))
        if reply.result != MAV_RESULT_ACCEPTED or reply.data_length != 8:
            raise AssertionError('signed session request was not accepted')
        session_key = bytes(reply.data[:reply.data_length])
        reply = board.secure_command(
            sequence, GET_SESSION_KEY, b'',
            signed_command(private_key, sequence, GET_SESSION_KEY))
        if reply.result != MAV_RESULT_DENIED:
            raise AssertionError('replayed session request was accepted')

        sequence = 100
        data = bytes((0, 1))
        signature = signed_command(
            private_key, sequence, GET_PUBLIC_KEYS, data, session_key)
        reply = board.secure_command(sequence, GET_PUBLIC_KEYS, data, signature)
        if reply.result != MAV_RESULT_ACCEPTED:
            raise AssertionError('first signed command was not accepted')
        reply = board.secure_command(sequence, GET_PUBLIC_KEYS, data, signature)
        if reply.result != MAV_RESULT_DENIED:
            raise AssertionError('replayed signed command was accepted')

        board.mav.command_int_send(
            1, 1, dialect.MAV_FRAME_GLOBAL, dialect.MAV_CMD_FLASH_BOOTLOADER,
            0, 0, 0, 0, 0, 0, 290876, 0, 0)
        reply = board.wait_message(
            'COMMAND_ACK', timeout,
            lambda message: message.command == dialect.MAV_CMD_FLASH_BOOTLOADER)
        if reply.result != MAV_RESULT_FAILED:
            raise AssertionError('bootloader with a different trust set was accepted')
        board.wait_message(
            'STATUSTEXT', timeout,
            lambda message: 'Bootloader not signed' in message.text)


def test_matching_bootloader(dialect, renode, firmware, bootloader, timeout):
    with RenodeBoard(
            dialect, renode, 'CubeOrange', firmware, bootloader,
            'cubeorange-matching-bootloader', timeout) as board:
        board.mav.command_int_send(
            1, 1, dialect.MAV_FRAME_GLOBAL, dialect.MAV_CMD_FLASH_BOOTLOADER,
            0, 0, 0, 0, 0, 0, 290876, 0, 0)
        reply = board.wait_message(
            'COMMAND_ACK', timeout,
            lambda message: message.command == dialect.MAV_CMD_FLASH_BOOTLOADER)
        if reply.result != MAV_RESULT_ACCEPTED:
            raise AssertionError('bootloader with the same trust set was rejected')


def test_uncompressed_bootloader_update(dialect, renode, firmware,
                                        bootloader, candidate, timeout):
    with RenodeBoard(
            dialect, renode, 'CubeOrangePlus', firmware, bootloader,
            'cubeorangeplus-uncompressed-bootloader-update', timeout) as board:
        board.mav.command_int_send(
            1, 1, dialect.MAV_FRAME_GLOBAL, dialect.MAV_CMD_FLASH_BOOTLOADER,
            0, 0, 0, 0, 0, 0, 290876, 0, 0)
        reply = board.wait_message(
            'COMMAND_ACK', timeout,
            lambda message: message.command == dialect.MAV_CMD_FLASH_BOOTLOADER)
        if reply.result != MAV_RESULT_ACCEPTED:
            raise AssertionError('uncompressed bootloader update was rejected')
        board.pause()
        state_dir = board.state_dir
        candidate_data = candidate.read_bytes()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            flash = (state_dir / 'flash.img').read_bytes()
            if flash[:len(candidate_data)] == candidate_data:
                break
            time.sleep(0.1)
        else:
            raise AssertionError('uncompressed bootloader update corrupted flash')


def test_cubered_external_flash(dialect, renode, firmware, bootloader, timeout):
    external, _offset = external_elf_image(firmware)
    expected_hash = fnv1a64(external)
    with RenodeBoard(
            dialect, renode, 'CubeRedPrimary', firmware, bootloader,
            'cubered-signed-external-flash', timeout) as board:
        read_count, read_hash, mdma_bytes, workaround_count = board.monitor_values([
            'qspiModel.qspiFlash CurrentReadCount',
            'qspiModel.qspiFlash CurrentReadHash',
            'mdma BytesTransferred',
            'qspi WorkaroundCount',
        ])
        if read_count != len(external):
            raise AssertionError('secure boot did not read the complete external image')
        if read_hash != expected_hash:
            raise AssertionError('QuadSPI returned data different from the signed image')
        if mdma_bytes < len(external):
            raise AssertionError('external image did not pass through MDMA')
        if workaround_count == 0:
            raise AssertionError('H757 command-only QSPI workaround was not exercised')

    tampered = OUTPUT / 'CubeRedPrimary-external-tampered-arducopter.elf'
    tampered_external = tamper_external_elf(firmware, tampered)
    with RenodeBoard(
            dialect, renode, 'CubeRedPrimary', tampered, bootloader,
            'cubered-tampered-external-flash', timeout,
            expect_heartbeat=False) as board:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            read_count, read_hash = board.monitor_values([
                'qspiModel.qspiFlash CurrentReadCount',
                'qspiModel.qspiFlash CurrentReadHash',
            ])
            if read_count == len(tampered_external):
                break
            time.sleep(1)
        else:
            raise AssertionError('secure boot did not verify the tampered external image')
        if read_hash != fnv1a64(tampered_external):
            raise AssertionError('tampered external image was not read faithfully')
        try:
            board.wait_message('HEARTBEAT', 5)
        except TimeoutError:
            pass
        else:
            raise AssertionError('firmware with tampered external flash booted')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--renode', required=True)
    parser.add_argument('--timeout', type=int, default=180)
    args = parser.parse_args()
    if monocypher.__version__ != '3.1.3.2':
        parser.error('pymonocypher 3.1.3.2 is required')

    shutil.rmtree(OUTPUT, ignore_errors=True)
    OUTPUT.mkdir(parents=True)
    private_a = bytes(range(1, 33))
    private_b = bytes(range(33, 65))
    private_a_path, public_a_path = write_key_pair('key-a', private_a)
    _private_b_path, public_b_path = write_key_pair('key-b', private_b)
    dialect = generate_dialect()

    candidate_name = 'SigningSecurityCandidate'
    candidate_path = ROOT / 'Tools' / 'bootloaders' / (candidate_name + '_bl.bin')
    extra_hwdef = OUTPUT / 'candidate.hwdef'
    extra_hwdef.write_text('USE_BOOTLOADER_FROM_BOARD %s\n' % candidate_name)
    try:
        cubeorange_bootloader = OUTPUT / 'CubeOrange-key-a_bl.bin'
        cubeorange_base = build_bootloader('CubeOrange')
        cubeorange_base_copy = OUTPUT / 'CubeOrange-base_bl.bin'
        shutil.copyfile(cubeorange_base, cubeorange_base_copy)
        provision_bootloader(cubeorange_base_copy, public_a_path, cubeorange_bootloader)
        provision_bootloader(cubeorange_base_copy, public_b_path, candidate_path)
        cubeorange_firmware = build_signed_copter(
            'CubeOrange', private_a_path, extra_hwdef)
        test_cubeorange_commands(
            dialect, args.renode, cubeorange_firmware,
            cubeorange_bootloader, private_a, args.timeout)
        provision_bootloader(cubeorange_base_copy, public_a_path, candidate_path)
        matching_firmware = build_signed_copter(
            'CubeOrange', private_a_path, extra_hwdef,
            artifact_name='CubeOrange-matching-arducopter.apj', configure=False)
        test_matching_bootloader(
            dialect, args.renode, matching_firmware,
            cubeorange_bootloader, args.timeout)

        cubeorangeplus_base = build_bootloader('CubeOrangePlus')
        cubeorangeplus_base_copy = OUTPUT / 'CubeOrangePlus-base_bl.bin'
        shutil.copyfile(cubeorangeplus_base, cubeorangeplus_base_copy)
        provision_bootloader(
            cubeorangeplus_base_copy, public_a_path, candidate_path)
        extra_hwdef.write_text(
            'USE_BOOTLOADER_FROM_BOARD %s\n'
            'env ROMFS_UNCOMPRESSED True\n' % candidate_name)
        cubeorangeplus_firmware = build_unsigned_copter(
            'CubeOrangePlus', extra_hwdef,
            'CubeOrangePlus-unsigned-transition-arducopter.apj')
        check_embedded_bootloader(candidate_path)
        test_uncompressed_bootloader_update(
            dialect, args.renode, cubeorangeplus_firmware,
            ROOT / 'Tools' / 'bootloaders' / 'CubeOrangePlus_bl.bin',
            candidate_path, args.timeout)

        cubered_bootloader = OUTPUT / 'CubeRedPrimary-key-a_bl.bin'
        cubered_base = build_bootloader('CubeRedPrimary')
        cubered_base_copy = OUTPUT / 'CubeRedPrimary-base_bl.bin'
        shutil.copyfile(cubered_base, cubered_base_copy)
        provision_bootloader(
            cubered_base_copy, public_a_path, cubered_bootloader)
        cubered_firmware = build_signed_copter(
            'CubeRedPrimary', private_a_path, elf=True)
        test_cubered_external_flash(
            dialect, args.renode, cubered_firmware,
            cubered_bootloader, args.timeout)
    finally:
        candidate_path.unlink(missing_ok=True)

    print('all Renode signing security regressions passed')


if __name__ == '__main__':
    main()
