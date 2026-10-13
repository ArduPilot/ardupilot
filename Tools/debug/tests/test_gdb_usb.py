#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Host protocol tests: python3 -m unittest discover -s Tools/debug/tests."""

import argparse
import importlib.util
import io
import json
import queue
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest

from pathlib import Path
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('gdb_usb', Path(__file__).resolve().parents[1] / 'gdb_usb.py')
gdb_usb = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gdb_usb)


class Port:
    def __init__(self, data):
        self.data = bytearray(data)
        self.written = bytearray()

    def read(self, count):
        # Exercise fragmented checksum bytes too.
        result = self.data[:min(count, 1)]
        del self.data[:len(result)]
        return bytes(result)

    def write(self, data):
        self.written.extend(data)


class USBLauncherTests(unittest.TestCase):
    def read(self, port):
        return gdb_usb.read_packet(port, time.monotonic()+0.1)

    def test_fragmented_reply(self):
        port = Port(b'noise+' + gdb_usb.rsp_packet(b'PacketSize=200;ap-usb-debug+'))
        self.assertEqual(self.read(port), b'PacketSize=200;ap-usb-debug+')
        self.assertEqual(port.written, b'+')

    def test_checksum_retry(self):
        port = Port(b'$bad#00' + gdb_usb.rsp_packet(b'OK'))
        self.assertEqual(self.read(port), b'OK')
        self.assertEqual(port.written, b'-+')

    def test_escaped_reply(self):
        port = Port(gdb_usb.rsp_packet(b'a}\x03b'))
        self.assertEqual(self.read(port), b'a#b')

    def test_bad_checksum_digit(self):
        port = Port(b'$bad#zz' + gdb_usb.rsp_packet(b'OK'))
        self.assertEqual(self.read(port), b'OK')
        self.assertEqual(port.written, b'-+')

    def test_truncated_reply(self):
        port = Port(b'$partial#')
        self.assertIsNone(self.read(port))
        self.assertNotIn(ord('+'), port.written)

    def test_oversized_reply(self):
        with self.assertRaises(gdb_usb.DebugError):
            self.read(Port(b'$' + b'x'*4097))

    def test_elf_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'firmware'
            path.write_bytes(b'not an ELF')
            with self.assertRaises(gdb_usb.DebugError):
                gdb_usb.validate_elf(path)
            data = bytearray(52)
            data[:6] = b'\x7fELF\x01\x01'
            struct.pack_into('<HH', data, 16, 2, 40)
            path.write_bytes(data)
            gdb_usb.validate_elf(path)
            struct.pack_into('<H', data, 18, 62)
            path.write_bytes(data)
            with self.assertRaises(gdb_usb.DebugError):
                gdb_usb.validate_elf(path)

    def test_gdb_arguments(self):
        args = argparse.Namespace(port='/dev/serial/by-id/test-if02', elf=Path('/tmp/firmware elf'),
                                  gdb='arm-none-eabi-gdb', gdb_server=None, batch=True, ex=['bt', 'x/4wx $sp'])
        command = gdb_usb.gdb_command(args)
        self.assertIn(str(args.elf.resolve()), command)
        self.assertIn('target remote /dev/serial/by-id/test-if02', command)
        self.assertEqual(command[-4:], ['-ex', 'bt', '-ex', 'x/4wx $sp'])
        args.port = '/dev/ttyACM0\nquit'
        with self.assertRaises(gdb_usb.DebugError):
            gdb_usb.gdb_command(args)

    def test_windows_com_port(self):
        args = argparse.Namespace(port='COM12', elf=Path('firmware'), gdb='gdb', gdb_server=None, batch=False, ex=[])
        with patch.object(gdb_usb.os, 'name', 'nt'):
            # Resolve Path before mocking os.name (WindowsPath cannot run on POSIX).
            args.elf = unittest.mock.Mock()
            args.elf.resolve.return_value = 'firmware'
            self.assertIn('target remote COM12', gdb_usb.gdb_command(args))

    def test_vscode_workspace(self):
        with tempfile.TemporaryDirectory(prefix='usb debug ') as directory:
            args = argparse.Namespace(elf=Path(directory)/'plane elf', port='COM12',
                                      gdb_server=None, tcp_port=3333, timeout=30, no_break=True, gdb=sys.executable,
                                      vscode=Path(directory)/'debug.code-workspace')
            gdb_usb.vscode_workspace(args)
            config = json.loads(args.vscode.read_text())['launch']['configurations'][0]
            self.assertEqual(config['program'], str(args.elf.resolve()))
            self.assertEqual(config['debugServerPath'], sys.executable)
            self.assertEqual(config['launchCompleteCommand'], 'None')
            self.assertIn('--no-break', config['debugServerArgs'])
            self.assertIn('"'+str(args.elf.resolve())+'"', config['debugServerArgs'])
            self.assertIn('--port COM12', config['debugServerArgs'])
            with self.assertRaises(FileExistsError):
                gdb_usb.vscode_workspace(args)

            args.vscode = Path(directory)/'wsl.code-workspace'
            args.port = None
            args.gdb_server = '127.0.0.1:3333'
            gdb_usb.vscode_workspace(args)
            config = json.loads(args.vscode.read_text())['launch']['configurations'][0]
            self.assertEqual(config['miDebuggerServerAddress'], args.gdb_server)
            self.assertNotIn('debugServerPath', config)
            self.assertTrue(config['stopAtConnect'])
            self.assertEqual(config['miDebuggerArgs'], '-nx')

    def test_bridge_binary_interrupt_and_disconnect(self):
        class Serial:
            in_waiting = 0

            def __init__(self):
                self.received = queue.Queue()
                self.sent = queue.Queue()

            def read(self, count):
                try:
                    return self.received.get(timeout=.05)
                except queue.Empty:
                    return b''

            def write(self, data):
                self.sent.put(data)

        serial = Serial()
        host, target = socket.socketpair()
        host.settimeout(2)
        worker = threading.Thread(target=gdb_usb.bridge, args=(target, serial))
        worker.start()
        try:
            binary = b'\x03+$X20000000,4:\x00\xff}\x03#00'
            host.sendall(binary)
            received = b''
            while len(received) < len(binary):
                received += serial.sent.get(timeout=2)
            self.assertEqual(received, binary)
            reply = b'+$T05thread:123;#ab'
            serial.received.put(reply)
            self.assertEqual(host.recv(100), reply)
        finally:
            host.close()
            worker.join(3)
            target.close()
        self.assertFalse(worker.is_alive())

    def test_connect_keeps_verified_port_open(self):
        import types

        class Serial(Port):
            def __init__(self, *args, **kwargs):
                super().__init__(gdb_usb.rsp_packet(b'PacketSize=200;ap-usb-debug+'))
                self.closed = False

            def reset_input_buffer(self):
                pass

            def close(self):
                self.closed = True

        module = types.SimpleNamespace(Serial=Serial, SerialException=OSError)
        with patch.dict(sys.modules, serial=module):
            port = gdb_usb.connect('COM12', 1, trigger=False, keep_open=True)
        self.assertFalse(port.closed)
        self.assertTrue(port.written.startswith(b'\x03'))
        port.close()

    def test_bridge_connection_reset(self):
        class Serial:
            in_waiting = 0

            def read(self, count):
                time.sleep(.01)
                return b''

        client = unittest.mock.Mock()
        client.recv.side_effect = ConnectionAbortedError('peer closed')
        with patch('sys.stderr', new_callable=io.StringIO) as output:
            gdb_usb.bridge(client, Serial())
        self.assertIn('peer closed', output.getvalue())
        client.shutdown.assert_called_once()


if __name__ == '__main__':
    unittest.main()
