#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Exercise serial passthrough with ChibiOS UART ownership checks in Renode.

Build MatekH743 Copter first, then run this script with --renode pointing
at the executable downloaded by fetch_renode.sh. This checks ownership and
forwarding; USB line coding and hardware parity need separate coverage.
"""

import argparse
import select
import socket
import subprocess
import sys
import time

from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import test_mission as common  # noqa: E402

from launch import MONITOR_COMMAND_ERROR_RE  # noqa: E402
from launch import MonitorClient  # noqa: E402


def run(args, root, output):
    log_path = output / 'renode.log'
    ports = [common.unused_tcp_port() for _ in range(5)]
    command = [
        sys.executable, str(root / 'Tools/renode/run.py'), 'MatekH743',
        '--firmware', str(args.firmware),
        '--state-dir', str(output / 'state'),
        '--serial', '1', '--uart-port', str(ports[0]), '--port', str(ports[4]),
    ]
    if args.renode:
        command += ['--renode', args.renode]
    with log_path.open('w') as log:
        process = subprocess.Popen(command, cwd=root, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
    client = MonitorClient('127.0.0.1', ports[4])
    links = []
    try:
        link = common.connect_mavlink(ports[0], process, log_path, time.monotonic() + args.timeout)
        links.append(link)
        link.setup_logfile(str(output / 'serial1.tlog'))
        texts = []

        def collect_text(connection, message):
            if message.get_type() == 'STATUSTEXT':
                texts.append(message.text)

        link.message_hooks.append(collect_text)
        client.connect()
        # MatekH743 SERIAL1/2 are MAVLink; SERIAL6/7 are unused UARTs.
        for index, uart in ((1, 'usart1'), (2, 'uart4'), (3, 'usart6')):
            for instruction in (
                'emulation CreateServerSocketTerminal %u "passthru%u" false' % (ports[index], index),
                'connector Connect sysbus.%sHost passthru%u' % (uart, index),
            ):
                response = client.command(instruction)
                if MONITOR_COMMAND_ERROR_RE.search(response):
                    raise RuntimeError(response)

        def set_parameter(name, value):
            common.set_parameter(link, process, log_path, name, value)

        def assert_parameter(connection, name, value):
            connection.mav.param_request_read_send(connection.target_system, connection.target_component,
                                                   name.encode('ascii'), -1)
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                message = common.recv_match(connection, process, log_path, 'PARAM_VALUE', deadline)
                if message.param_id == name:
                    if message.param_value != value:
                        raise RuntimeError('%s: expected %s, got %s' % (name, value, message.param_value))
                    return
            raise RuntimeError('no response for %s' % name)

        def wait_text(text):
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                if any(text in message for message in texts):
                    return
                common.recv_match(link, process, log_path, 'STATUSTEXT', deadline)
            raise RuntimeError('no STATUSTEXT: %s' % text)

        def check_forwarding(source, destination, data):
            source.sendall(data)
            received = b''
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                if not select.select([destination], [], [], 0.5)[0]:
                    continue
                chunk = destination.recv(4096)
                if not chunk:
                    raise RuntimeError('passthrough socket closed')
                received += chunk
                if data in received:
                    return
            raise RuntimeError('passthrough data missing: %r' % data)

        def wait_for_startup():
            # A slow host can take several wall-clock seconds per simulated second.
            client.command('pause')
            client.command('emulation RunFor "2"', timeout=args.timeout)
            client.command('start')

        def reboot():
            common.send_command(link, process, log_path, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, (1,))
            time.sleep(5)
            while link.recv_match(blocking=False) is not None:
                pass
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                heartbeat = common.recv_match(link, process, log_path, 'HEARTBEAT', deadline)
                if heartbeat.system_status >= mavutil.mavlink.MAV_STATE_STANDBY:
                    return
            raise RuntimeError('vehicle did not finish starting after reboot')

        set_parameter('SERIAL1_PROTOCOL', 2)
        set_parameter('SERIAL2_PROTOCOL', 2)
        reboot()
        peer = common.connect_mavlink(ports[1], process, log_path, time.monotonic() + args.timeout)
        links.append(peer)
        peer.setup_logfile(str(output / 'serial2.tlog'))
        sysid = link.target_system
        # Both ports must resume receiving MAVLink, not just emitting heartbeats.
        for _ in range(3):
            texts.clear()
            set_parameter('SERIAL_PASSTIMO', 5)
            set_parameter('SERIAL_PASS1', 1)
            set_parameter('SERIAL_PASS2', 2)
            wait_text('Passthru enabled')
            wait_for_startup()
            check_forwarding(link.port, peer.port, b'owner checked serial1 to serial2')
            check_forwarding(peer.port, link.port, b'owner checked serial2 to serial1')
            wait_text('Passthru timed out')
            assert_parameter(link, 'MAV_SYSID', sysid)
            assert_parameter(peer, 'MAV_SYSID', sysid)
        print('ChibiOS timeout recovery: both UARTs receive MAVLink after three cycles', flush=True)
        peer.close()
        peer.logfile.close()
        links.remove(peer)

        assert_parameter(link, 'SERIAL6_PROTOCOL', -1)
        assert_parameter(link, 'SERIAL7_PROTOCOL', -1)
        set_parameter('SERIAL_PASSTIMO', 0)
        set_parameter('SERIAL_PASS1', 6)
        texts.clear()
        set_parameter('SERIAL_PASS2', 7)
        wait_text('Passthru enabled')
        wait_for_startup()
        with socket.create_connection(('127.0.0.1', ports[2]), timeout=5) as serial6, \
                socket.create_connection(('127.0.0.1', ports[3]), timeout=5) as serial7:
            for source, destination, data in ((serial6, serial7, b'serial6 to serial7'),
                                              (serial7, serial6, b'serial7 to serial6')):
                source.sendall(data)
                received = b''
                while len(received) < len(data):
                    chunk = destination.recv(len(data) - len(received))
                    if not chunk:
                        raise RuntimeError('passthrough socket closed')
                    received += chunk
                if received != data:
                    raise RuntimeError('passthrough data differs: %r' % received)
            set_parameter('SERIAL_PASS2', -1)
            wait_text('Passthru disabled')
            for source, destination in ((serial6, serial7), (serial7, serial6)):
                destination.settimeout(0.5)
                source.sendall(b'after disable')
                try:
                    received = destination.recv(1)
                except socket.timeout:
                    continue
                if received:
                    raise RuntimeError('passthrough forwards after disable')
        assert_parameter(link, 'MAV_SYSID', sysid)
        print('ChibiOS manual disable: bidirectional forwarding starts and stops', flush=True)
    finally:
        client.close()
        for connection in links:
            connection.close()
            if connection.logfile is not None:
                connection.logfile.close()
        common.stop_process_group(process)


def main():
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--renode', help='Renode executable')
    parser.add_argument('--firmware', type=Path, default=root / 'build/MatekH743/bin/arducopter')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--timeout', type=common.positive_float, default=180)
    args = parser.parse_args()
    if not args.firmware.is_file():
        parser.error('firmware does not exist: %s' % args.firmware)
    output = (args.output_dir or root / 'build/renode-test' /
              ('serial-passthru-' + time.strftime('%Y%m%d-%H%M%S'))).resolve()
    output.mkdir(parents=True)
    run(args, root, output)
    print('Serial passthrough passed; artifacts in %s' % output, flush=True)


if __name__ == '__main__':
    main()
