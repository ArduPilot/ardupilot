#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Check IOMCU reset-cause and supply-dip reporting under Renode.

Boots ArduCopter on CubeOrangePlus with the real IOMCU firmware and its
independent watchdog enforced, then injects faults from the Renode
monitor and checks what the FMU tells the GCS:

- halting the IOMCU core until its watchdog fires gives
  "IOMCU reset: watchdog";
- a 0.2ms dip in the IOMCU supply, much shorter than the 50ms the IOMCU
  polls at, gives "IOMCU supply voltage dip detected (1)";
- the same dip on the FMU gives "MCU supply voltage dip detected (1)";
- power cycling the IOMCU gives "IOMCU reset: power on";
- a dip after that reset, before the FMU has read the IOMCU's restarted
  count, is reported as (1) again.

The FMU firmware must embed IO firmware built with reset-cause and
supply-dip reporting; the binaries in Tools/IO_Firmware predate it until
they are rebuilt.
"""

import argparse
import re
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

SUPPLY_DIP_RE = re.compile(r'supply voltage dip')
# F1 EXTI_IMR; the IO firmware unmasks the PVD line (16) on its first poll
F1_EXTI_IMR = 0x40010400
F1_EXTI_PVD = 1 << 16
# seconds of vehicle uptime before the IOMCU is reliably talking to the FMU
SETTLE_BOOT_S = 20


class Vehicle:
    """MAVLink connection that keeps the link active and records STATUSTEXT."""

    def __init__(self, connection, renode, log_path):
        self.connection = connection
        self.renode = renode
        self.log_path = log_path
        self.texts = []
        self.last_heartbeat = 0
        self.uptime_ms = 0

    def poll(self, deadline):
        now = time.monotonic()
        if now - self.last_heartbeat >= 1:
            # statustexts are only sent to links that have seen a GCS
            self.connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_heartbeat = now
        message = common.recv_match(
            self.connection, self.renode, self.log_path,
            ['STATUSTEXT', 'SYSTEM_TIME'], min(deadline, now + 1))
        if message.get_type() == 'STATUSTEXT':
            print('vehicle: %s' % message.text, flush=True)
            self.texts.append(message.text)
        else:
            self.uptime_ms = message.time_boot_ms
        return message

    def wait_text(self, pattern, deadline):
        start = len(self.texts)
        while time.monotonic() < deadline:
            try:
                self.poll(deadline)
            except RuntimeError:
                continue
            for text in self.texts[start:]:
                if re.search(pattern, text):
                    return text
        raise RuntimeError('no statustext matching %r' % pattern)

    def wait_uptime(self, seconds, deadline):
        while time.monotonic() < deadline:
            try:
                message = self.poll(deadline)
            except RuntimeError:
                continue
            if message.get_type() == 'SYSTEM_TIME' and message.time_boot_ms >= seconds * 1000:
                return
        raise RuntimeError('vehicle uptime did not reach %us' % seconds)


def monitor(client, command):
    response = client.command(command, timeout=15)
    if MONITOR_COMMAND_ERROR_RE.search(response):
        raise RuntimeError('%s: %s' % (command, response.strip()))
    return response


def on_machine(client, machine, *commands):
    monitor(client, 'mach set "%s"' % machine)
    try:
        return [monitor(client, command) for command in commands]
    finally:
        monitor(client, 'mach set "ardupilot"')


def dip_detector_armed(client):
    response = on_machine(
        client, 'iomcu', 'sysbus ReadDoubleWord 0x%08X' % F1_EXTI_IMR)[0]
    # the response echoes the command, so take the last number
    return bool(int(re.findall(r'0x[0-9A-Fa-f]+', response)[-1], 16) & F1_EXTI_PVD)


def connect_monitor(client, renode, log_path, deadline):
    while True:
        try:
            client.connect()
            return
        except (OSError, TimeoutError):
            client.close()
            common.check_process(renode, log_path)
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.5)


def run_test(args, root, output_dir):
    if not args.firmware.is_file():
        raise RuntimeError('firmware does not exist: %s' % args.firmware)
    state_dir = output_dir / 'state'
    state_dir.mkdir(parents=True)
    log_path = output_dir / 'renode.log'
    uart_port = common.unused_tcp_port()
    monitor_port = common.unused_tcp_port()
    command = [
        sys.executable, str(root / 'Tools' / 'renode' / 'run.py'),
        'CubeOrangePlus',
        '--vehicle', 'arducopter',
        '--firmware', str(args.firmware),
        '--state-dir', str(state_dir),
        '--uart-port', str(uart_port),
        '--port', str(monitor_port),
        '--real-iomcu', '--iomcu-watchdog',
        '--unthrottled',
    ]
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
        connection.setup_logfile(str(output_dir / 'test.tlog'))
        connection.mav.request_data_stream_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1)
        vehicle = Vehicle(connection, renode, log_path)
        connect_monitor(client, renode, log_path, deadline)
        vehicle.wait_uptime(SETTLE_BOOT_S, deadline)
        early = [text for text in vehicle.texts if SUPPLY_DIP_RE.search(text)]
        if early:
            raise RuntimeError('supply dip reported before any dip: %s' % early)

        # a debugger-style halt stops the main loop and every interrupt,
        # like the in-flight lockup; the IWDG keeps counting
        on_machine(client, 'iomcu', 'cpu IsHalted true')
        vehicle.wait_text(r'^IOMCU reset: watchdog$', deadline)
        print('watchdog reset reported', flush=True)

        on_machine(client, 'iomcu', 'pwr Dip 2.5 0.2')
        vehicle.wait_text(
            r'^IOMCU supply voltage dip detected \(1\)$', deadline)
        print('IOMCU supply dip reported', flush=True)

        monitor(client, 'pwr Dip 2.5 0.2')
        vehicle.wait_text(
            r'^MCU supply voltage dip detected \(1\)$', deadline)
        print('FMU supply dip reported', flush=True)

        # a dip before the FMU's first status read after the reset makes
        # the restarted count equal the one already reported. Hold the FMU
        # across the reset, for less time than the IOMCU had been up, so
        # the IOMCU's clock still goes backwards and shows the reset
        vehicle.wait_uptime(vehicle.uptime_ms / 1000 + 5, deadline)
        monitor(client, 'pause')
        monitor(client, 'cpu IsHalted true')
        on_machine(client, 'iomcu', 'rcc PowerCycle')
        # the reset happens when the IOMCU next runs
        monitor(client, 'emulation RunFor "0.01"')
        if dip_detector_armed(client):
            raise RuntimeError('IOMCU supply dip detector armed before restart')
        for step in range(30):
            monitor(client, 'emulation RunFor "0.1"')
            if dip_detector_armed(client):
                print('IOMCU supply dip detector armed after %.1fs' % (0.1 * (step + 1)),
                      flush=True)
                break
        else:
            raise RuntimeError('IOMCU supply dip detector not armed after reset')
        on_machine(client, 'iomcu', 'pwr Dip 2.5 0.2')
        # let the IOMCU's 20Hz status update count it
        monitor(client, 'emulation RunFor "0.1"')
        monitor(client, 'cpu IsHalted false')
        monitor(client, 'start')
        vehicle.wait_text(r'^IOMCU reset: power on$', deadline)
        print('power-on reset reported', flush=True)
        vehicle.wait_text(
            r'^IOMCU supply voltage dip detected \(1\)$', deadline)
        print('IOMCU supply dip after reset reported', flush=True)
    finally:
        client.close()
        if connection is not None:
            connection.close()
        common.stop_process_group(renode)


def main():
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--renode', help='Renode executable')
    parser.add_argument(
        '--firmware', type=Path,
        default=root / 'build' / 'CubeOrangePlus' / 'bin' / 'arducopter',
        help='CubeOrangePlus ArduCopter ELF embedding the IO firmware')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--timeout', type=common.positive_float, default=600)
    args = parser.parse_args()
    output_dir = (args.output_dir or root / 'build' / 'renode-test' /
                  ('iomcu-faults-' + time.strftime('%Y%m%d-%H%M%S'))).resolve()
    if output_dir.exists():
        parser.error('--output-dir already exists: %s' % output_dir)
    output_dir.mkdir(parents=True)
    run_test(args, root, output_dir)
    print('IOMCU faults passed; artifacts in %s' % output_dir, flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
