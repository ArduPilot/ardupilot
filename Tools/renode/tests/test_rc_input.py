#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Check emulated SBUS R/C input and R/C failsafe under Renode.

Each scenario boots ArduPlane with the transmitter switched on, checks the
channels the firmware reports, moves a stick from the Renode monitor, then
switches the transmitter off and on again to enter and leave R/C failsafe.
"""

import argparse
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

SCENARIOS = {
    # board, extra run.py options, SBUS UART for boards without an IOMCU
    'iomcu': ('CubeOrangePlus', [], None),
    'real-iomcu': ('CubeOrangePlus', ['--real-iomcu'], None),
    'uart': ('MatekH743', [], 'SERIAL4'),
}
STICKS = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1500]
MOVED = [1700] + STICKS[1:]
RC_RECEIVER = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER


def wait_channels(connection, renode, log_path, expected, deadline):
    while time.monotonic() < deadline:
        message = common.recv_match(
            connection, renode, log_path, 'RC_CHANNELS', deadline)
        values = [getattr(message, 'chan%u_raw' % (i + 1))
                  for i in range(len(expected))]
        if (message.chancount >= len(expected) and
                all(abs(a - b) <= 2 for a, b in zip(values, expected))):
            return
    raise RuntimeError('R/C channels did not reach %s' % expected)


def wait_rc_health(connection, renode, log_path, healthy, deadline):
    while time.monotonic() < deadline:
        message = common.recv_match(
            connection, renode, log_path, ['SYS_STATUS', 'STATUSTEXT'], deadline)
        if message.get_type() == 'STATUSTEXT':
            print('vehicle: %s' % message.text, flush=True)
            continue
        if not message.onboard_control_sensors_present & RC_RECEIVER:
            continue
        if bool(message.onboard_control_sensors_health & RC_RECEIVER) == healthy:
            return
    raise RuntimeError('R/C receiver did not become %s' %
                       ('healthy' if healthy else 'failsafe'))


def monitor(client, command):
    response = client.command(command, timeout=15)
    if MONITOR_COMMAND_ERROR_RE.search(response):
        raise RuntimeError('%s: %s' % (command, response.strip()))


def reboot(connection, renode, log_path):
    common.send_command(
        connection, renode, log_path,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, (1,))
    time.sleep(5)


def run_scenario(args, root, name, output_dir):
    board, options, sbus_port = SCENARIOS[name]
    firmware = args.uart_firmware if sbus_port else args.firmware
    if not firmware.is_file():
        raise RuntimeError('firmware does not exist: %s' % firmware)
    state_dir = output_dir / name
    state_dir.mkdir(parents=True)
    log_path = output_dir / ('%s-renode.log' % name)
    uart_port = common.unused_tcp_port()
    monitor_port = common.unused_tcp_port()
    command = [
        sys.executable, str(root / 'Tools' / 'renode' / 'run.py'), board,
        '--vehicle', 'arduplane',
        '--firmware', str(firmware),
        '--state-dir', str(state_dir),
        '--uart-port', str(uart_port),
        '--port', str(monitor_port),
        '--rc-channels', ','.join(str(value) for value in STICKS),
        '--unthrottled',
    ] + options
    if sbus_port:
        command += ['--device', '{"device":"sbus-rc","port":"%s"}' % sbus_port]
    if args.renode:
        command += ['--renode', args.renode]
    deadline = time.monotonic() + args.timeout
    with log_path.open('w') as log:
        renode = subprocess.Popen(
            command, cwd=root, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
    connection = None
    client = MonitorClient('127.0.0.1', monitor_port)
    try:
        connection = common.connect_mavlink(uart_port, renode, log_path, deadline)
        connection.setup_logfile(str(output_dir / ('%s.tlog' % name)))
        connection.mav.request_data_stream_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
        parameters = [('RC_PROTOCOLS', 1), ('THR_FAILSAFE', 1)]
        if sbus_port:
            parameters.append(('%s_PROTOCOL' % sbus_port, 23))
        for parameter, value in parameters:
            common.set_parameter(connection, renode, log_path, parameter, value)
        reboot(connection, renode, log_path)
        connection.mav.request_data_stream_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

        wait_channels(connection, renode, log_path, STICKS, deadline)
        wait_rc_health(connection, renode, log_path, True, deadline)
        print('%s: SBUS channels received' % name, flush=True)

        while True:
            try:
                client.connect()
                break
            except (OSError, TimeoutError):
                client.close()
                common.check_process(renode, log_path)
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.5)
        monitor(client, 'rcInput SetChannels "%s"' %
                ','.join(str(value) for value in MOVED))
        wait_channels(connection, renode, log_path, MOVED, deadline)
        print('%s: stick moved' % name, flush=True)

        monitor(client, 'rcInput Enabled false')
        wait_rc_health(connection, renode, log_path, False, deadline)
        print('%s: R/C failsafe entered' % name, flush=True)
        monitor(client, 'rcInput Enabled true')
        wait_rc_health(connection, renode, log_path, True, deadline)
        print('%s: R/C failsafe cleared' % name, flush=True)
    finally:
        client.close()
        if connection is not None:
            connection.close()
        common.stop_process_group(renode)


def main():
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', nargs='*', choices=sorted(SCENARIOS),
                        help='scenarios to run (default: all)')
    parser.add_argument('--renode', help='Renode executable')
    parser.add_argument(
        '--firmware', type=Path,
        default=root / 'build' / 'CubeOrangePlus' / 'bin' / 'arduplane',
        help='CubeOrangePlus ArduPlane ELF for the IOMCU scenarios')
    parser.add_argument(
        '--uart-firmware', type=Path,
        default=root / 'build' / 'MatekH743' / 'bin' / 'arduplane',
        help='MatekH743 ArduPlane ELF for the UART scenario')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--timeout', type=common.positive_float, default=300)
    args = parser.parse_args()
    output_dir = (args.output_dir or root / 'build' / 'renode-test' /
                  ('rc-input-' + time.strftime('%Y%m%d-%H%M%S'))).resolve()
    if output_dir.exists():
        parser.error('--output-dir already exists: %s' % output_dir)
    output_dir.mkdir(parents=True)
    for name in args.scenario or sorted(SCENARIOS):
        run_scenario(args, root, name, output_dir)
    print('R/C input passed; artifacts in %s' % output_dir, flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
