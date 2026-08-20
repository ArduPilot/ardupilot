#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Exercise a ChibiOS crashdump end to end under Renode."""

import argparse
import contextlib
import io
import os
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time

from pathlib import Path

from pymavlink import mavutil

CRASHDUMP_TEXT = 'CrashDump data detected'


def positive_float(value):
    result = float(value)
    if result <= 0:
        raise argparse.ArgumentTypeError('must be positive')
    return result


def unused_tcp_port():
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        return listener.getsockname()[1]


def pause_machine(port):
    with socket.create_connection(('127.0.0.1', port), timeout=5) as monitor:
        monitor.settimeout(2)
        try:
            monitor.recv(4096)
        except socket.timeout:
            pass
        monitor.sendall(b'pause\n')
        time.sleep(0.2)
        try:
            response = monitor.recv(4096).decode(errors='replace')
        except socket.timeout as error:
            raise RuntimeError('Renode monitor did not acknowledge pause') from error
        if 'error' in response.lower():
            raise RuntimeError('Renode monitor rejected pause: %s' % response.strip())


def log_tail(path, line_count=80):
    try:
        lines = path.read_text(errors='replace').splitlines()
    except OSError:
        return ''
    return '\n'.join(lines[-line_count:])


def stop_process_group(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


def check_process(process, log_path):
    returncode = process.poll()
    if returncode is None:
        return
    tail = log_tail(log_path)
    detail = 'Renode stopped with status %u' % returncode
    if tail:
        detail += '\n--- Renode output (tail) ---\n' + tail
    raise RuntimeError(detail)


def connect_mavlink(port, process, log_path, deadline):
    last_error = None
    while time.monotonic() < deadline:
        check_process(process, log_path)
        try:
            with contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()):
                return mavutil.mavlink_connection('tcp:127.0.0.1:%u' % port)
        except OSError as error:
            last_error = error
            time.sleep(0.1)
    raise RuntimeError('timed out connecting to MAVLink: %s' % last_error)


def wait_for_heartbeat(connection, process, log_path, deadline):
    while time.monotonic() < deadline:
        check_process(process, log_path)
        heartbeat = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if heartbeat is not None:
            return heartbeat
    raise RuntimeError('timed out waiting for the initial MAVLink heartbeat')


def connect_and_wait_for_heartbeat(port, process, log_path, deadline):
    last_error = None
    while time.monotonic() < deadline:
        connection = None
        try:
            connection = connect_mavlink(port, process, log_path, deadline)
            wait_for_heartbeat(connection, process, log_path, deadline)
            return connection
        except OSError as error:
            last_error = error
            if connection is not None:
                with contextlib.suppress(OSError):
                    connection.close()
            time.sleep(0.1)
    raise RuntimeError('timed out waiting for MAVLink after reconnect: %s' % last_error)


def send_crashdump(connection, fault):
    fault_magic = {
        'hardfault': 94,
        'lockup': 93,
    }[fault]
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        42,
        24,
        71,
        fault_magic,
        0,
        0,
        0,
    )


def wait_for_crashdump(connection, process, log_path, deadline):
    while time.monotonic() < deadline:
        check_process(process, log_path)
        message = connection.recv_match(type='STATUSTEXT', blocking=True, timeout=2)
        if message is not None and CRASHDUMP_TEXT in message.text:
            return
    raise RuntimeError('timed out waiting for the crashdump reboot')


def find_crashdump_info(elf):
    for parent in elf.parents:
        candidate = parent / 'Tools' / 'debug' / 'crashdump_info.py'
        if candidate.is_file():
            return candidate
    return None


def extract_fatfs_and_verify(sd_image, output, elf, verifier):
    mcopy = shutil.which('mcopy')
    if mcopy is None:
        raise RuntimeError('mcopy is required to extract CrashDump.DAT')
    subprocess.run(
        [mcopy, '-o', '-i', str(sd_image), '::APM/CrashDump.DAT', str(output)],
        check=True,
    )
    if verifier is None:
        print('warning: no Tools/debug/crashdump_info.py found beside the ELF; '
              'skipping firmware identity verification', file=sys.stderr)
        return None
    result = subprocess.run(
        [sys.executable, str(verifier), str(elf), str(output)],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    return result.stdout.strip()


def elf_symbols(elf, wanted):
    nm = shutil.which('arm-none-eabi-nm') or shutil.which('nm')
    if nm is None:
        raise RuntimeError('arm-none-eabi-nm or nm is required to locate the flash crashdump')
    result = subprocess.run(
        [nm, '-an', str(elf)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    )
    symbols = {}
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[2] in wanted:
            symbols[fields[2]] = int(fields[0], 16)
    missing = wanted - symbols.keys()
    if missing:
        raise RuntimeError('ELF lacks flash crashdump symbols: %s' %
                           ', '.join(sorted(missing)))
    return symbols


def extract_flash(flash_image, output, elf):
    symbols = elf_symbols(elf, {'__crash_log_base__', '__crash_log_end__'})
    region_start = symbols['__crash_log_base__']
    region_end = symbols['__crash_log_end__']
    flash_base = 0x08000000
    data = flash_image.read_bytes()
    start = region_start - flash_base
    end = region_end - flash_base
    if start < 0 or end <= start or end > len(data):
        raise RuntimeError(
            'ELF crashdump region 0x%08x-0x%08x is outside %s' %
            (region_start, region_end, flash_image))
    region = data[start:end]
    dump_size = struct.unpack_from('<I', region, len(region) - 4)[0]
    if dump_size == 0xFFFFFFFF:
        dump_size = len(region)
    if dump_size < 2 or dump_size > len(region):
        raise RuntimeError('invalid flash crashdump size %u (region is %u bytes)' %
                           (dump_size, len(region)))
    if region[:2] != b'\x63\x43':
        raise RuntimeError('flash crashdump lacks the CrashCatcher signature')
    output.write_bytes(region[:dump_size])
    return ('flash crashdump region 0x%08x-0x%08x, recorded size %u bytes' %
            (region_start, region_end, dump_size))


def run_test(args, root, state_dir, output):
    run_py = root / 'Tools' / 'renode' / 'run.py'
    uart_port = unused_tcp_port()
    monitor_port = unused_tcp_port()
    while monitor_port == uart_port:
        monitor_port = unused_tcp_port()
    log_path = state_dir / 'renode.log'
    command = [
        sys.executable,
        str(run_py),
        '--vehicle', args.vehicle,
        '--elf', str(args.elf),
        '--state-dir', str(state_dir),
        '--uart-port', str(uart_port),
        '--port', str(monitor_port),
        '--unthrottled',
    ]
    if args.backend == 'flash' or args.fault == 'lockup':
        command.append('--watchdog')
    if args.renode is not None:
        command.extend(['--renode', args.renode])
    if args.cpusel is not None:
        command.extend(['--cpusel', str(args.cpusel)])
    command.append(args.board)

    deadline = time.monotonic() + args.timeout
    with open(log_path, 'w') as log:
        process = subprocess.Popen(
            command,
            cwd=root,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    connection = None
    try:
        connection = connect_and_wait_for_heartbeat(
            uart_port, process, log_path, deadline)
        print('initial MAVLink heartbeat received')
        if args.fault == 'lockup':
            # The first heartbeat can precede the monitor thread's one-time
            # watchdog setup under unthrottled emulation. Let normal scheduler
            # iterations run before deliberately stopping the main loop.
            time.sleep(20)
        send_crashdump(connection, args.fault)
        if args.backend == 'fatfs':
            # Renode retains the serial terminal across the MCU reset, but
            # pymavlink's existing TCP connection does not recover reliably.
            # Reconnect while the fault handler is writing, then use the next
            # heartbeat as proof that the firmware rebooted.
            time.sleep(1)
            with contextlib.suppress(OSError):
                connection.close()
            connection = None
            deadline = time.monotonic() + args.timeout
            connection = connect_and_wait_for_heartbeat(
                uart_port, process, log_path, deadline)
        else:
            # A full watchdog reset invalidates Renode's UART TCP session. Stop
            # after the dump and use a clean process to test persistent flash
            # and the firmware's next-boot detection path.
            time.sleep(10)
            with contextlib.suppress(OSError):
                connection.close()
            connection = None
            pause_machine(monitor_port)
            stop_process_group(process)
            with open(log_path, 'a') as log:
                process = subprocess.Popen(
                    command,
                    cwd=root,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            deadline = time.monotonic() + args.timeout
            connection = connect_and_wait_for_heartbeat(
                uart_port, process, log_path, deadline)
        status_timeout = min(args.timeout, 30)
        deadline = time.monotonic() + status_timeout
        try:
            wait_for_crashdump(connection, process, log_path, deadline)
            print('firmware rebooted and reported %s' % CRASHDUMP_TEXT)
        except RuntimeError as error:
            if args.backend != 'fatfs' or str(error) != 'timed out waiting for the crashdump reboot':
                raise
            print('post-reboot status text not received; verifying the SD image directly')
    except Exception as error:
        tail = log_tail(log_path)
        if tail:
            raise RuntimeError('%s\n--- Renode output (tail) ---\n%s' %
                               (error, tail)) from error
        raise
    finally:
        if connection is not None:
            with contextlib.suppress(OSError):
                connection.close()
        stop_process_group(process)

    if args.backend == 'fatfs':
        sd_image = state_dir / 'sdcard.img'
        if not sd_image.is_file():
            raise RuntimeError('Renode did not create %s' % sd_image)
        verifier = find_crashdump_info(args.elf)
        verified = extract_fatfs_and_verify(sd_image, output, args.elf, verifier)
    else:
        flash_image = state_dir / 'flash.img'
        if not flash_image.is_file():
            raise RuntimeError('Renode did not create %s' % flash_image)
        verified = extract_flash(flash_image, output, args.elf)
    if verified:
        print(verified)
    print('crashdump written to %s (%u bytes)' %
          (output, output.stat().st_size))


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('board', nargs='?', default='Pixhawk6X',
                        help='Renode board to test (default: Pixhawk6X)')
    parser.add_argument('--vehicle', default='arducopter',
                        help='vehicle name passed to run.py (default: arducopter)')
    parser.add_argument('--elf', type=Path,
                        help='firmware ELF (default: build/<board>/bin/<vehicle>)')
    parser.add_argument('--backend', choices=('fatfs', 'flash'), default='fatfs',
                        help='crashdump storage backend (default: fatfs)')
    parser.add_argument('--fault', choices=('hardfault', 'lockup'), default='hardfault',
                        help='failure type to trigger (default: hardfault)')
    parser.add_argument('--output', type=Path,
                        help='extracted dump path (default: CrashDump-<board>.DAT)')
    parser.add_argument('--state-dir', type=Path,
                        help='new or empty state directory to retain after the test')
    parser.add_argument('--timeout', type=positive_float, default=180,
                        help='timeout for each boot phase in seconds (default: 180)')
    parser.add_argument('--renode', help='Renode executable passed to run.py')
    parser.add_argument('--cpusel', type=int,
                        help='host CPU passed to run.py --cpusel')
    args = parser.parse_args()

    if args.elf is None:
        args.elf = root / 'build' / args.board / 'bin' / args.vehicle
    args.elf = args.elf.resolve()
    if not args.elf.is_file():
        parser.error('firmware ELF does not exist: %s' % args.elf)
    output = (args.output or Path('CrashDump-%s.DAT' % args.board)).resolve()
    if output.exists():
        parser.error('output already exists: %s' % output)

    temporary_state = None
    if args.state_dir is None:
        temporary_state = tempfile.TemporaryDirectory(
            prefix='ardupilot-renode-crashdump-')
        state_dir = Path(temporary_state.name)
    else:
        state_dir = args.state_dir.resolve()
        if state_dir.exists() and any(state_dir.iterdir()):
            parser.error('--state-dir must be new or empty: %s' % state_dir)
        state_dir.mkdir(parents=True, exist_ok=True)

    try:
        run_test(args, root, state_dir, output)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print('crashdump test failed: %s' % error, file=sys.stderr)
        return 1
    finally:
        if temporary_state is not None:
            temporary_state.cleanup()
    return 0


if __name__ == '__main__':
    sys.exit(main())
