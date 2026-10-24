#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Host protocol tests: python3 -m unittest discover -s Tools/debug/tests."""

import argparse
import importlib.util
import io
import json
import os
import queue
import shutil
import socket
import struct
import subprocess
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

    def test_seer_arguments_and_deferred_setup(self):
        with tempfile.TemporaryDirectory(prefix='seer setup $ with spaces ') as directory:
            args = argparse.Namespace(port='/dev/ttyACM0', elf=Path('/tmp/firmware elf'), gdb='arm-gdb',
                                      gdb_server=None, ex=['print "$literal"'],
                                      apmod=[Path('/tmp/module symbols.apmod')])
            with patch.object(gdb_usb.shutil, 'which', return_value='/usr/bin/arm-gdb'):
                command = gdb_usb.seer_command(args, Path(directory))
            self.assertEqual(command[0], 'seergdb')
            self.assertIn('/usr/bin/arm-gdb', command)
            self.assertEqual(command[command.index('--connect')+1], args.port)
            self.assertIn('--interpreter=mi', command[command.index('--gdb-arguments')+1])
            self.assertNotIn(directory, command[command.index('--gdb-arguments')+1])
            startup = (Path(directory) / 'startup.gdb').read_text()
            self.assertIn('set serial baud 115200', startup)
            self.assertIn('gdb.events.new_objfile.connect', startup)
            self.assertIn('gdb.events.stop.connect', startup)
            self.assertIn('apmod symbols', startup)
            self.assertIn('$literal', startup)
            args.gdb_server = 'localhost:3333'
            with patch.object(gdb_usb.shutil, 'which', return_value='/usr/bin/arm-gdb'):
                command = gdb_usb.seer_command(args, Path(directory))
            self.assertEqual(command[command.index('--connect')+1], args.gdb_server)

    def test_seer_launch_lifetime(self):
        startup = []

        def launch(command, cwd=None):
            startup.append(Path(cwd) / 'startup.gdb')
            self.assertTrue(startup[-1].is_file())
            self.assertEqual(command[0], 'seergdb')
            child = unittest.mock.Mock()
            child.wait.return_value = 0
            return child

        with patch.object(gdb_usb, 'validate_elf'), \
                patch.object(gdb_usb.shutil, 'which', return_value='/tmp/gdb'), \
                patch.object(gdb_usb.subprocess, 'Popen', side_effect=launch), \
                patch.object(gdb_usb, 'connect') as connect:
            self.assertEqual(gdb_usb.main(['firmware', '--port', '/dev/fake', '--seer', '--no-break']), 0)
            connect.assert_called_once_with('/dev/fake', 30, trigger=False)
        self.assertFalse(startup[0].exists())

    def test_seer_startup_in_gdb(self):
        debugger = shutil.which(os.environ.get('AP_MODULE_ARM_GDB', 'gdb'))
        if debugger is None:
            self.skipTest('Requires Python-enabled GDB')
        probe = subprocess.run([debugger, '-nx', '--batch', '-ex', 'python print("PYTHON_OK")'],
                               capture_output=True, text=True, timeout=15)
        if probe.returncode != 0:
            self.skipTest('Requires Python-enabled GDB')
        with tempfile.TemporaryDirectory(prefix='seer startup with spaces ') as directory:
            args = argparse.Namespace(port='/dev/fake', elf=Path('/tmp/firmware'), gdb=debugger,
                                      gdb_server=None, ex=['print "$literal"'], apmod=[])
            gdb_usb.seer_command(args, Path(directory))
            result = subprocess.run([debugger, '-nx', '--batch', '-x', str(Path(directory) / 'startup.gdb'),
                                     '-ex', 'python assert callable(_usb_seer_event)'],
                                    capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_seer_preflight(self):
        for arguments, diagnostic in [(['--batch'], '--batch cannot'), ([], 'Seer executable not found')]:
            with self.subTest(arguments=arguments), patch.object(gdb_usb, 'validate_elf'), \
                    patch.object(gdb_usb.shutil, 'which', side_effect=lambda name: None if name == 'seergdb' else name), \
                    patch.object(gdb_usb, 'connect') as connect, \
                    patch('sys.stderr', new_callable=io.StringIO) as stderr:
                self.assertEqual(gdb_usb.main(['firmware', '--port', '/dev/fake', '--seer'] + arguments), 1)
                self.assertIn(diagnostic, stderr.getvalue())
                connect.assert_not_called()

    def test_rust_module_commands(self):
        args = argparse.Namespace(port='/dev/ttyACM0', elf=Path('/tmp/firmware'), gdb='python-arm-gdb',
                                  gdb_server=None, batch=False, ex=['bt'], apmod=[Path('/tmp/module symbols.apmod')])
        command = gdb_usb.gdb_command(args)
        self.assertNotIn('apmod printers', command)  # selected automatically from module DWARF
        self.assertIn('apmod symbols "/tmp/module symbols.apmod"', command)
        helper = Path(gdb_usb.__file__).resolve().parents[1] / 'modules/apmod_gdb.py'
        self.assertIn(f'source {helper}', command)
        self.assertEqual(command[-2:], ['-ex', 'bt'])
        args.apmod = [Path('/tmp/module\u00e9.apmod')]
        self.assertIn('apmod symbols "/tmp/module\u00e9.apmod"', gdb_usb.gdb_command(args))

    def test_rust_helper_source_in_gdb(self):
        debugger = shutil.which(os.environ.get('AP_MODULE_ARM_GDB', 'gdb'))
        if debugger is None:
            self.skipTest('Requires Python-enabled GDB and pyelftools')
        probe = subprocess.run([debugger, '-nx', '--batch', '-ex', 'python import elftools'],
                               capture_output=True, text=True, timeout=15)
        if probe.returncode != 0:
            self.skipTest('Requires Python-enabled GDB and pyelftools')
        with tempfile.TemporaryDirectory(prefix='usb rust debug ') as directory:
            helper_dir = Path(directory) / 'modules'
            helper_dir.mkdir()
            source_dir = Path(gdb_usb.__file__).resolve().parents[1] / 'modules'
            for name in ('apmod_gdb.py', 'apmod_debug.py', 'pack.py', 'targets.py'):
                shutil.copyfile(source_dir / name, helper_dir / name)
            args = argparse.Namespace(port='/dev/unused', elf=Path('/tmp/unused'), gdb=debugger,
                                      gdb_server=None, batch=True, ex=[], apmod=[Path('/tmp/unused.apmod')])
            with patch.object(gdb_usb, '__file__', str(Path(directory) / 'debug/gdb_usb.py')):
                command = gdb_usb.gdb_command(args)
            source = next(word for word in command if word.startswith('source '))
            result = subprocess.run([debugger, '-nx', '--batch', '-ex', source, '-ex', 'help apmod'],
                                    capture_output=True, text=True, timeout=15)
            output = result.stdout + result.stderr
            self.assertEqual(result.returncode, 0, output)
            self.assertIn('Native module symbols', output)
            self.assertNotIn('No such file or directory', output)

    def test_apmod_wildcards_sorted_and_deduplicated(self):
        with tempfile.TemporaryDirectory(prefix='module symbols ') as directory:
            root = Path(directory)
            for name in ('b.apmod', 'a.apmod', 'ignored.elf'):
                (root / name).touch()
            modules = gdb_usb.expand_apmod_paths([root / '*.apmod', root / 'a.apmod'])
            self.assertEqual(modules, [(root / name).resolve() for name in ('a.apmod', 'b.apmod')])
            self.assertEqual(gdb_usb.expand_apmod_paths([root / '*']), modules)
            literal = root / 'literal[1].apmod'
            literal.touch()
            self.assertEqual(gdb_usb.expand_apmod_paths([literal]), [literal.resolve()])

    def test_apmod_wildcard_command(self):
        with tempfile.TemporaryDirectory(prefix='module symbols ') as directory:
            root = Path(directory)
            for name in ('b.apmod', 'a.apmod'):
                module = root / name
                module.touch()
                module.with_suffix('.elf').touch()
            with patch.object(gdb_usb, 'validate_elf'), \
                    patch.object(gdb_usb.shutil, 'which', return_value='/tmp/gdb'), \
                    patch.object(gdb_usb.subprocess, 'run',
                                 return_value=argparse.Namespace(returncode=0, stdout='APMOD_PYTHON_OK')), \
                    patch.object(gdb_usb.subprocess, 'Popen') as child, \
                    patch.object(gdb_usb, 'connect') as connect:
                child.return_value.wait.return_value = 0
                arguments = ['firmware', '--port', '/dev/fake', '--apmod', str(root / '*.apmod'),
                             '--apmod', str(root / 'a.apmod')]
                self.assertEqual(gdb_usb.main(arguments), 0)
                command = child.call_args.args[0]
                paths = [json.dumps(str((root / name).resolve())) for name in ('a.apmod', 'b.apmod')]
                expected = 'apmod symbols ' + ' '.join(paths)
                self.assertIn(expected, command)
                connect.assert_called_once()

    def test_apmod_symlink_matches_debugger_resolution(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / 'build/module.apmod'
            target.parent.mkdir()
            target.touch()
            target.with_suffix('.elf').touch()
            alias = root / 'module.apmod'
            try:
                alias.symlink_to(target)
            except (OSError, NotImplementedError):
                self.skipTest('Requires symbolic-link support')
            # ModuleSymbols and gdb_command resolve the module before looking
            # for its ELF. Preflight must validate that same companion path.
            modules = gdb_usb.expand_apmod_paths([alias, target])
            self.assertEqual(modules, [target.resolve()])
            self.assertTrue(modules[0].with_suffix('.elf').is_file())
            self.assertFalse(alias.with_suffix('.elf').exists())

    def test_apmod_wildcard_errors_before_usb_attachment(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for error in ('no matches', 'missing companion'):
                with self.subTest(error=error):
                    if error == 'missing companion':
                        (root / 'module.apmod').touch()
                    with patch.object(gdb_usb, 'validate_elf'), \
                            patch.object(gdb_usb.shutil, 'which', return_value='/tmp/gdb'), \
                            patch.object(gdb_usb.subprocess, 'run') as probe, \
                            patch.object(gdb_usb, 'connect') as connect, \
                            patch('sys.stderr', new_callable=io.StringIO) as stderr:
                        self.assertEqual(gdb_usb.main(['firmware', '--port', '/dev/fake',
                                                      '--apmod', str(root / '*.apmod')]), 1)
                        diagnostic = 'matched no files' if error == 'no matches' else 'matching companion .elf'
                        self.assertIn(diagnostic, stderr.getvalue())
                        probe.assert_not_called()
                        connect.assert_not_called()

    def test_rust_requires_python_before_usb_attachment(self):
        with tempfile.TemporaryDirectory() as directory:
            elf = Path(directory) / 'firmware'
            header = bytearray(52)
            header[:6] = b'\x7fELF\x01\x01'
            struct.pack_into('<HH', header, 16, 2, 40)
            elf.write_bytes(header)
            module = Path(directory) / 'module.apmod'
            module.write_bytes(b'module')
            module.with_suffix('.elf').write_bytes(b'ELF')
            with patch.object(gdb_usb.shutil, 'which', return_value='/tmp/gdb'), \
                    patch.object(gdb_usb.subprocess, 'run', return_value=argparse.Namespace(returncode=1, stdout='')), \
                    patch.object(gdb_usb, 'connect') as connect:
                self.assertEqual(gdb_usb.main([str(elf), '--port', '/dev/fake', '--apmod', str(module)]), 1)
                connect.assert_not_called()

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
