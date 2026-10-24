#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Debug STM32H7 ArduPilot over USB with interactive GDB.

Requires firmware configured with --enable-USB-debug. Select the second CDC
interface (normally a /dev/serial/by-id/...-if02 symlink), and the matching ELF.
The monitor re-enumerates USB on entry and detach. Continue, breakpoints and
Ctrl-C work within the same session. The first CDC interface carries normal
GCS traffic while the target runs; telemetry pauses at debugger stops.
"""

import argparse
import json
import os
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

from pathlib import Path

TRIGGER = b"APUSBDBG\n"


class DebugError(Exception):
    """An actionable connection or configuration error."""


def rsp_packet(payload):
    return b"$" + payload + b"#%02x" % (sum(payload) & 255)


def read_packet(port, deadline):
    """Read a checksummed reply, acknowledging valid packets only."""
    payload = bytearray()
    checksum = 0
    active = False
    escaped = False
    while time.monotonic() < deadline:
        value = port.read(1)
        if not value:
            continue
        byte = value[0]
        if not active:
            if byte == ord('$'):
                active = True
                payload.clear()
                checksum = 0
            continue
        if byte == ord('#') and not escaped:
            trailer = bytearray()
            while len(trailer) < 2 and time.monotonic() < deadline:
                trailer.extend(port.read(2-len(trailer)))
            try:
                valid = len(trailer) == 2 and int(trailer, 16) == checksum
            except ValueError:
                valid = False
            port.write(b'+' if valid else b'-')
            if valid:
                return bytes(payload)
            active = False
            continue
        checksum = (checksum + byte) & 255
        if escaped:
            payload.append(byte ^ 0x20)
            escaped = False
        elif byte == ord('}'):
            escaped = True
        else:
            payload.append(byte)
        if len(payload) > 4096:
            raise DebugError('oversized USB debugger reply')
    return None


def validate_elf(path):
    try:
        with path.open('rb') as source:
            header = source.read(52)
    except OSError as error:
        raise DebugError(f'cannot read ELF {path}: {error}') from error
    if (len(header) < 52 or header[:6] != b'\x7fELF\x01\x01' or
            struct.unpack_from('<HH', header, 16) != (2, 40)):
        raise DebugError('expected an executable little-endian ARM ELF, not an APJ/BIN file')


def connect(port_name, timeout, trigger=True, keep_open=False):
    try:
        import serial
    except ImportError as error:
        raise DebugError('pyserial is required: python3 -m pip install pyserial') from error

    deadline = time.monotonic() + timeout
    if trigger:
        try:
            with serial.Serial(port_name, 115200, timeout=0.1, write_timeout=1, exclusive=True) as port:
                port.write(TRIGGER)
                port.flush()
        except (OSError, serial.SerialException) as error:
            raise DebugError(f'cannot request debug stop on {port_name}: {error}') from error
        # The firmware disconnects the controller before starting its polling
        # transport. Keep the by-id name, not the temporary ttyACM number.
        time.sleep(0.3)

    last_error = 'no GDB response'
    while time.monotonic() < deadline:
        try:
            port = serial.Serial(port_name, 115200, timeout=0.1, write_timeout=1, exclusive=True)
            connected = False
            try:
                port.reset_input_buffer()
                if not trigger:
                    # Reattach to a session left running by an earlier GDB.
                    port.write(b'\x03')
                port.write(rsp_packet(b'qSupported'))
                response = read_packet(port, min(deadline, time.monotonic()+1))
                if response is not None and response.startswith((b'T', b'S')):
                    response = read_packet(port, min(deadline, time.monotonic()+1))
                if response is not None:
                    if b'ap-usb-debug+' not in response.split(b';'):
                        raise DebugError('the selected port is not an ArduPilot USB debug monitor')
                    connected = True
                    return port if keep_open else None
            finally:
                if not (connected and keep_open):
                    port.close()
        except (OSError, serial.SerialException) as error:
            last_error = str(error)
        time.sleep(0.1)
    raise DebugError(
        f'USB debug monitor did not appear within {timeout:g}s ({last_error}). '
        'Use the second CDC interface and load firmware built with '
        '--enable-USB-debug. The stop is cooperative: the main loop must run.')


def bridge(client, port):
    """Relay RSP unchanged, including binary writes, acknowledgements and Ctrl-C.

    Windows cannot select() on a COM handle. A reader thread with a bounded
    serial timeout works on all three platforms and keeps USB ownership here.
    """
    finished = threading.Event()
    errors = []
    client.settimeout(1)

    def receive():
        try:
            while not finished.is_set():
                data = port.read(max(1, min(port.in_waiting, 4096)))
                if data:
                    client.sendall(data)
        except OSError as error:
            errors.append(error)
        finally:
            finished.set()
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    reader = threading.Thread(target=receive, daemon=True)
    reader.start()
    try:
        while not finished.is_set():
            try:
                data = client.recv(4096)
            except socket.timeout:
                continue
            if not data:
                break
            port.write(data)
    except OSError as error:
        # Windows may report a reset/abort instead of EOF when GDB detaches.
        # Like a USB disconnect in the reader, this ends the established bridge.
        errors.append(error)
    finally:
        finished.set()
        reader.join(2)
    if errors:
        # Detach/reset re-enumerates USB and may close it before TCP closes.
        print(f'USB connection closed: {errors[0]}', file=sys.stderr, flush=True)


def serve(args):
    """One local GDB connection per process; VS Code owns its lifetime."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        # Do not allow Windows SO_REUSEADDR to steal another debugger's port.
        if os.name == 'nt':
            server.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        else:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('127.0.0.1', args.tcp_port))
        server.listen(1)
        server.settimeout(args.timeout)
        print(f'USB GDB server listening on 127.0.0.1:{server.getsockname()[1]}', flush=True)
        client, _ = server.accept()
        with client:
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # Wait for the IDE before stopping firmware. This also avoids the
            # monitor's initial-packet timeout while the IDE starts GDB.
            with connect(args.port, args.timeout, trigger=not args.no_break, keep_open=True) as port:
                bridge(client, port)


def vscode_workspace(args):
    """Generate a separate workspace without replacing the user's launch.json."""
    root = Path(__file__).resolve().parents[2]
    server_args = [str(Path(__file__).resolve()), str(args.elf.resolve()),
                   '--port', args.port or '', '--serve', '--tcp-port', str(args.tcp_port),
                   '--timeout', str(args.timeout)]
    if args.no_break:
        server_args.append('--no-break')
    configuration = {
        'name': 'ArduPilot USB debug',
        'type': 'cppdbg',
        'request': 'launch',
        'program': str(args.elf.resolve()),
        'cwd': str(root),
        'MIMode': 'gdb',
        'targetArchitecture': 'arm',
        'miDebuggerPath': str(Path(shutil.which(args.gdb)).resolve()),
        'miDebuggerArgs': '-nx',
        'miDebuggerServerAddress': f'127.0.0.1:{args.tcp_port}',
        'debugServerPath': sys.executable,
        # OpenDebugAD7 uses ProcessStartInfo.Arguments (not a shell) on every
        # platform: double-quote rules also cover spaces in Windows paths.
        'debugServerArgs': subprocess.list2cmdline(server_args),
        'serverStarted': 'USB GDB server listening on',
        'filterStdout': True,
        'filterStderr': True,
        'serverLaunchTimeout': int((args.timeout+5)*1000),
        'launchCompleteCommand': 'None',
        'stopAtConnect': True,
        'setupCommands': [{'text': f'set remotetimeout {int(args.timeout)+5}'}],
        'externalConsole': False,
        'sourceFileMap': {},
    }
    if args.gdb_server:
        configuration['miDebuggerServerAddress'] = args.gdb_server
        for key in ('debugServerPath', 'debugServerArgs', 'serverStarted',
                    'filterStdout', 'filterStderr', 'serverLaunchTimeout'):
            del configuration[key]
    workspace = {'folders': [{'path': str(root)}],
                 'extensions': {'recommendations': ['ms-vscode.cpptools']},
                 'launch': {'version': '0.2.0', 'configurations': [configuration]}}
    args.vscode.parent.mkdir(parents=True, exist_ok=True)
    # Fail rather than discard source mappings or other user customizations.
    with args.vscode.open('x', encoding='utf-8') as output:
        json.dump(workspace, output, indent=4)
        output.write('\n')
    print(f'Open {args.vscode.resolve()} in VS Code, install the recommended C/C++ extension, then press F5.')


def gdb_command(args):
    # No shell is involved. Keep GDB command arguments on a single line.
    port = args.gdb_server or (args.port if os.name == 'nt' else os.path.abspath(args.port))
    if any(c in port for c in '\r\n\x00'):
        raise DebugError('USB device path must not contain newlines or NUL')
    command = [args.gdb, '-nx', '-quiet', str(args.elf.resolve()),
               '-ex', 'set pagination off',
               '-ex', 'set serial baud 115200',
               '-ex', 'set remotetimeout 10',
               '-ex', f'target remote {port}']
    if args.batch:
        command.append('--batch')
    for expression in args.ex:
        command.extend(['-ex', expression])
    return command


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=Path, help='matching build/<board>/bin/arducopter (or another vehicle ELF)')
    endpoint = parser.add_mutually_exclusive_group(required=True)
    endpoint.add_argument('--port', help='second USB CDC device: by-id path, /dev/cu.* or COM port')
    endpoint.add_argument('--gdb-server', metavar='HOST:PORT',
                          help='connect to an existing bridge (for example Windows USB with a WSL debugger)')
    parser.add_argument('--gdb', default='arm-none-eabi-gdb', help='GDB executable')
    parser.add_argument('--timeout', type=float, default=30, help='monitor connection deadline in seconds')
    parser.add_argument('--no-break', action='store_true',
                        help='reattach to an existing debug session, interrupting if running')
    parser.add_argument('--batch', action='store_true', help='run GDB non-interactively (for tests)')
    parser.add_argument('--ex', action='append', default=[], help='GDB command to run after connecting; repeatable')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--serve', action='store_true', help='serve one GDB connection over local TCP instead of starting GDB')
    mode.add_argument('--vscode', type=Path, metavar='WORKSPACE.code-workspace',
                      help='create a VS Code workspace for this ELF/port without attaching')
    parser.add_argument('--tcp-port', type=int, default=3333, help='local TCP port for --serve/--vscode (default: 3333)')
    args = parser.parse_args(argv)
    try:
        if not 0 < args.timeout <= 300:
            raise DebugError('--timeout must be greater than zero and at most 300 seconds')
        validate_elf(args.elf)
        if not 1 <= args.tcp_port <= 65535:
            raise DebugError('--tcp-port must be between 1 and 65535')
        if any(c in (args.port or args.gdb_server) for c in '\r\n\x00'):
            raise DebugError('USB device path must not contain newlines or NUL')
        if args.gdb_server and (args.serve or args.no_break):
            raise DebugError('--serve and --no-break require --port')
        if (args.serve or args.vscode) and (args.batch or args.ex):
            raise DebugError('--batch and --ex apply only to interactive/batch GDB, not --serve or --vscode')
        if not args.serve and shutil.which(args.gdb) is None:
            raise DebugError(f'GDB executable not found: {args.gdb}')
        if args.vscode:
            vscode_workspace(args)
            return 0
        if args.serve:
            serve(args)
            return 0
        command = gdb_command(args)
        if not args.gdb_server:
            print(f'Requesting USB debugging on {args.port}; use the matching firmware ELF.', flush=True)
            connect(args.port, args.timeout, trigger=not args.no_break)
        print('USB monitor ready. Use continue, Ctrl-C, break and info threads; detach restores normal USB.', flush=True)
        # GDB and the launcher share the terminal's foreground process group.
        # Leave SIGINT enabled in the child, but let GDB handle Ctrl-C itself.
        child = subprocess.Popen(command)
        previous_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            return child.wait()
        finally:
            signal.signal(signal.SIGINT, previous_sigint)
    except (DebugError, OSError) as error:
        print(f'gdb_usb.py: {error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    sys.exit(main())
