#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Check bidirectional DShot telemetry under Renode.

Boots Copter on MatekH743-bdshot with its DShot outputs answered by ESCs:
the built-in AM32-style ESC model (default), scripted UDP ESCs from
Tools/renode/dshot_esc.py (--scripted), or AM32 SITL ESCs (--am32-sitl).
Some ESCs stay silent, and in the recover scenario one starts answering part
way through. The DataFlash log is then checked:

- ESCs answering throughout keep ESC.Err near zero
- silent ESCs never log, and each answering ESC reports its own rpm
- an ESC that recovers first logs a high ESC.Err, from the failures counted
  against it while it was silent, which then falls to near zero

The edt scenario enables extended DShot telemetry and checks the voltage and
temperature the ESCs report.
"""

import argparse
import os
import random
import re
import shutil
import signal
import socket
import subprocess
import sys
import time

from pathlib import Path

from pymavlink import DFReader
from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import test_mission as common  # noqa: E402

from dshot_esc import PORT_STRIDE  # noqa: E402
from dshot_esc import ESCBench  # noqa: E402
from launch import MONITOR_COMMAND_ERROR_RE  # noqa: E402
from launch import MonitorClient  # noqa: E402

BOARD = 'MatekH743-bdshot'
MOTORS = 4
ESC_ERPM = 21000
# ESC n reports ESC_ERPM + (n - 1) * ESC_ERPM_STEP, so crossed channels show
ESC_ERPM_STEP = 1400
PARAMETERS = {
    'FRAME_CLASS': 1,
    'FRAME_TYPE': 1,
    'MOT_PWM_TYPE': 5,  # DShot300
    'SERVO_BLH_BDMASK': 0xF,
    'SERVO_BLH_POLES': 14,
    'LOG_DISARMED': 1,
    'BRD_SAFETY_DEFLT': 0,
}
SCENARIOS = {
    # name: (ESCs silent at the start, of those the ones that later answer, EDT)
    'all': ((), (), False),
    'one': ((2, 3, 4), (), False),
    'recover': ((2,), (2,), False),
    'edt': ((4,), (), True),
}
EDT_ESC_TYPE = 3  # BLHeli32/AM32/Kiss+EDT
MAX_ERROR_RATE = 5.0
MIN_RECOVERY_ERROR_RATE = 50.0
MIN_EDT_UPDATES = 20
SILENT_SECONDS = 20
RECOVERED_SECONDS = 15
MOTOR_TEST_PERCENT = 15
ESC_ARM_SECONDS = 3


def reboot(link, process, log_path, timeout):
    common.send_command(link, process, log_path,
                        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, (1,))
    time.sleep(5)
    while link.recv_match(blocking=False) is not None:
        pass
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        heartbeat = common.recv_match(link, process, log_path, 'HEARTBEAT', deadline)
        if heartbeat.system_status >= mavutil.mavlink.MAV_STATE_STANDBY:
            request_streams(link)
            return
    raise RuntimeError('vehicle did not restart')


def set_parameter(link, process, log_path, name, value, timeout):
    # parameters are not answered until the emulated boot is far enough along
    deadline = time.monotonic() + timeout
    while True:
        try:
            common.set_parameter(link, process, log_path, name, value)
            return
        except RuntimeError:
            if time.monotonic() > deadline:
                raise


def request_streams(link):
    link.mav.request_data_stream_send(link.target_system, link.target_component,
                                      mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)


def free_port_base(count):
    '''A UDP port with the ports of count ESCs above it free.'''
    for _ in range(100):
        base = random.randrange(20000, 60000 - PORT_STRIDE * count)
        sockets = []
        try:
            for index in range(count):
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sockets.append(sock)
                sock.bind(('127.0.0.1', base + PORT_STRIDE * index))
            return base
        except OSError:
            continue
        finally:
            for sock in sockets:
                sock.close()
    raise RuntimeError('no free UDP port range for the ESCs')


def expected_rpm(esc):
    return (ESC_ERPM + (esc - 1) * ESC_ERPM_STEP) * 2 / PARAMETERS['SERVO_BLH_POLES']


class SimESCs:
    '''The bridge's built-in AM32-style ESCs; the motor test spins motor 1.'''

    motor_test = True
    edt = True
    edt_current = True

    def __init__(self, silent, output):
        self.silent = silent
        self.output = output
        self.monitor = None
        self.bridges = []

    def run_args(self):
        return ['--esc-sim']

    def attach(self, monitor):
        self.monitor = monitor
        repl = self.output / 'state' / 'generated' / ('%s.repl' % BOARD)
        self.bridges = re.findall(r'^(timer\d+BDShot):', repl.read_text(), re.M)
        for esc in self.silent:
            self.set_silent(esc, True)

    def set_silent(self, esc, silent):
        for bridge in self.bridges:
            response = self.monitor.command('sysbus.%s SetSilent %u %s' % (
                bridge, esc, 'true' if silent else 'false'))
            if MONITOR_COMMAND_ERROR_RE.search(response):
                raise RuntimeError(response)

    def start(self, esc):
        self.set_silent(esc, False)

    check_rpm = staticmethod(lambda rpm, answering: motor_test_rpm(rpm))

    def summary(self):
        for esc in range(1, MOTORS + 1):
            for bridge in self.bridges:
                state = self.monitor.command('sysbus.%s EscState %u' % (bridge, esc))
                if 'armed=' in state:
                    line = [part for part in state.splitlines() if 'armed=' in part][0]
                    print('ESC%u: %s' % (esc, line.strip()), flush=True)

    def close(self):
        pass


def motor_test_rpm(rpm):
    for esc in range(2, MOTORS + 1):
        if rpm[esc - 1] != 0:
            raise RuntimeError('idle ESC%u reports %u rpm' % (esc, rpm[esc - 1]))
    return rpm[0] > 500


class ScriptedESCs:
    motor_test = False
    edt = False
    edt_current = False

    def __init__(self, base_port, silent, output):
        self.base_port = base_port
        self.bench = ESCBench(base_port, MOTORS, ESC_ERPM, set(silent), ESC_ERPM_STEP)

    def run_args(self):
        return ['--esc-signal-port', str(self.base_port)]

    def attach(self, monitor):
        pass

    def start(self, esc):
        self.bench.escs[esc - 1].silent = False

    def check_rpm(self, rpm, answering):
        for esc in range(1, MOTORS + 1):
            if esc in answering:
                if abs(rpm[esc - 1] - expected_rpm(esc)) > expected_rpm(esc) * 0.02:
                    return False
            elif rpm[esc - 1] != 0:
                raise RuntimeError('silent ESC%u reports %u rpm' % (esc, rpm[esc - 1]))
        return True

    def summary(self):
        for index, esc in enumerate(self.bench.escs):
            print('ESC%u: %u frames, %u bidirectional, %u replies' % (
                index + 1, esc.frames, esc.bidirectional_frames, esc.replies), flush=True)
            if esc.bidirectional_frames == 0:
                raise RuntimeError('ESC%u received no bidirectional DShot' % (index + 1))

    def close(self):
        self.bench.close()


class AM32ESCs:
    '''AM32 SITL ESCs; the motor test spins only motor 1.'''

    motor_test = True
    edt = True
    # its simulated motor draws under 1A at the motor test throttle
    edt_current = False

    def __init__(self, base_port, silent, output, elf):
        self.base_port = base_port
        self.output = output
        self.elf = elf
        self.processes = {}
        for esc in range(1, MOTORS + 1):
            if esc not in silent:
                self.start(esc)

    def start(self, esc):
        self.processes[esc] = subprocess.Popen([
            str(self.elf), '--can-uri', 'none',
            '--input-port', str(self.base_port + PORT_STRIDE * (esc - 1)),
            '--state-port', '0', '--uid', 'esc%u' % esc,
            '--eeprom', str(self.output / ('esc%u.bin' % esc)),
            '--verbose', '--log-file', str(self.output / ('esc%u.log' % esc)),
        ], cwd=self.output, start_new_session=True)

    def run_args(self):
        return ['--esc-signal-port', str(self.base_port)]

    def attach(self, monitor):
        pass

    check_rpm = staticmethod(lambda rpm, answering: motor_test_rpm(rpm))

    def summary(self):
        for esc, process in sorted(self.processes.items()):
            if process.poll() is not None:
                raise RuntimeError('AM32 SITL ESC%u exited with %d' % (esc, process.returncode))

    def close(self):
        for process in self.processes.values():
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass


def wait_rpm(link, process, log_path, escs, answering, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = common.recv_match(link, process, log_path, 'ESC_TELEMETRY_1_TO_4', deadline)
        rpm = list(message.rpm)
        if escs.check_rpm(rpm, answering):
            return rpm
    raise RuntimeError('ESC telemetry rpm never matched the answering ESCs %s' % sorted(answering))


def wait_sim_seconds(link, process, log_path, seconds, timeout):
    '''Wait for simulated time to advance; returns the last boot time seen in ms.'''
    deadline = time.monotonic() + timeout
    start = None
    while time.monotonic() < deadline:
        message = common.recv_match(link, process, log_path, 'SYSTEM_TIME', deadline)
        start = start or message.time_boot_ms
        if message.time_boot_ms - start > seconds * 1000:
            return message.time_boot_ms
    raise RuntimeError('simulated time did not advance %us' % seconds)


def esc_logs(log_file):
    '''ESC number -> list of (time ms, Err).'''
    rows = {}
    reader = DFReader.DFReader_binary(str(log_file))
    while True:
        message = reader.recv_match(type='ESC')
        if message is None:
            return rows
        rows.setdefault(message.Instance + 1, []).append((message.TimeUS * 1e-3, message.Err))


def check_settled(esc, rows, failures):
    # skip startup, where frames arrive before the first window reset
    tail = [err for _, err in rows[len(rows) // 2:]]
    mean = sum(tail) / len(tail)
    print('ESC%u Err mean %.1f%% max %.1f%% over %u samples' % (
        esc, mean, max(tail), len(tail)), flush=True)
    if mean > MAX_ERROR_RATE:
        failures.append('ESC%u Err %.1f%%' % (esc, mean))


def check_log(log_file, silent, recover, recovered_ms):
    rows = esc_logs(log_file)
    failures = []
    for esc in range(1, MOTORS + 1):
        esc_rows = rows.get(esc, [])
        if esc in silent and esc not in recover:
            if esc_rows:
                failures.append('silent ESC%u has %u ESC log messages' % (esc, len(esc_rows)))
            continue
        if not esc_rows:
            failures.append('ESC%u has no ESC log messages' % esc)
            continue
        if esc in recover:
            early = [ms for ms, _ in esc_rows if ms < recovered_ms]
            if early:
                failures.append('ESC%u logged %u messages while silent' % (esc, len(early)))
            first = max(err for _, err in esc_rows[:5])
            print('ESC%u first Err after recovery %.1f%%' % (esc, first), flush=True)
            if first < MIN_RECOVERY_ERROR_RATE:
                failures.append('ESC%u first Err %.1f%%: failures while silent were not '
                                'counted against it' % (esc, first))
        check_settled(esc, esc_rows, failures)
    if failures:
        raise RuntimeError('; '.join(failures))


def check_edt(start, end, answering, spinning_current):
    '''EDT must keep updating through the run; ArduPilot holds old values.'''
    if start is None or end is None:
        raise RuntimeError('no ESC telemetry')
    failures = []
    for esc in range(1, MOTORS + 1):
        voltage = end.voltage[esc - 1] * 0.01
        temperature = end.temperature[esc - 1]
        current = end.current[esc - 1] * 0.01
        updates = (end.count[esc - 1] - start.count[esc - 1]) & 0xFFFF
        print('ESC%u EDT %.2fV %udegC %.2fA, %u updates' % (
            esc, voltage, temperature, current, updates), flush=True)
        if esc not in answering:
            if voltage or temperature or updates:
                failures.append('silent ESC%u reports EDT data' % esc)
            continue
        if voltage < 5 or temperature <= 0:
            failures.append('ESC%u reports no EDT voltage/temperature' % esc)
        if updates < MIN_EDT_UPDATES:
            failures.append('ESC%u EDT updated only %u times' % (esc, updates))
        if esc == 1 and spinning_current and current <= 0:
            failures.append('spinning ESC1 reports no EDT current')
    if failures:
        raise RuntimeError('; '.join(failures))


def make_escs(args, silent, output):
    if args.am32_sitl:
        return AM32ESCs(args.esc_port or free_port_base(MOTORS), silent, output, args.am32_sitl)
    if args.scripted:
        return ScriptedESCs(args.esc_port or free_port_base(MOTORS), silent, output)
    return SimESCs(silent, output)


def run(args, root, output, silent, recover, edt):
    answering = {esc for esc in range(1, MOTORS + 1) if esc not in silent}
    log_path = output / 'renode.log'
    uart_port = common.unused_tcp_port()
    monitor_port = common.unused_tcp_port()
    escs = make_escs(args, silent, output)
    process = None
    monitor = None
    try:
        command = [
            sys.executable, str(root / 'Tools/renode/run.py'), BOARD,
            '--firmware', str(args.firmware),
            '--state-dir', str(output / 'state'),
            '--uart-port', str(uart_port), '--port', str(monitor_port),
        ] + escs.run_args()
        if args.renode:
            command += ['--renode', args.renode]
        with log_path.open('w') as log:
            process = subprocess.Popen(command, cwd=root, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        print('waiting for the firmware to boot (Renode log %s)' % log_path, flush=True)
        link = common.connect_mavlink(uart_port, process, log_path,
                                      time.monotonic() + args.timeout)
        link.setup_logfile(str(output / 'bdshot.tlog'))
        monitor = MonitorClient('127.0.0.1', monitor_port)
        monitor.connect()
        escs.attach(monitor)
        latest = {}
        link.message_hooks.append(
            lambda connection, message: latest.__setitem__(message.get_type(), message))
        parameters = dict(PARAMETERS)
        if edt:
            parameters['SERVO_DSHOT_ESC'] = EDT_ESC_TYPE
        for name, value in parameters.items():
            set_parameter(link, process, log_path, name, value, args.timeout)
        reboot(link, process, log_path, args.timeout)
        if escs.motor_test:
            # AM32 only arms after a second of zero throttle
            wait_sim_seconds(link, process, log_path, ESC_ARM_SECONDS, args.timeout)
            common.send_command(link, process, log_path, mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                                (1, 0, MOTOR_TEST_PERCENT,
                                 SILENT_SECONDS + RECOVERED_SECONDS + 30, 1, 0))
        rpm = wait_rpm(link, process, log_path, escs, answering, args.timeout)
        print('ESC telemetry rpm %s' % rpm, flush=True)
        edt_start = latest.get('ESC_TELEMETRY_1_TO_4')
        # the vehicle is at least this far along when the ESCs start answering
        recovered_ms = wait_sim_seconds(link, process, log_path, SILENT_SECONDS, args.timeout)
        for esc in recover:
            escs.start(esc)
        wait_sim_seconds(link, process, log_path, RECOVERED_SECONDS, args.timeout)
        escs.summary()
        if edt:
            check_edt(edt_start, latest.get('ESC_TELEMETRY_1_TO_4'), answering,
                      escs.edt_current)
        log_file = common.download_log(link, process, log_path, output / 'bdshot.BIN',
                                       list_timeout=180)
    finally:
        if monitor is not None:
            monitor.close()
        if process is not None:
            common.stop_process_group(process)
        escs.close()
    check_log(log_file, silent, recover, recovered_ms)


def main():
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('scenario', nargs='*',
                        help='scenarios to run: %s (default all)' % ', '.join(sorted(SCENARIOS)))
    parser.add_argument('--renode', help='Renode executable')
    parser.add_argument('--firmware', type=Path,
                        default=root / 'build' / BOARD / 'bin' / 'arducopter',
                        help='%s ArduCopter ELF' % BOARD)
    escs = parser.add_mutually_exclusive_group()
    escs.add_argument('--am32-sitl', type=Path, metavar='ELF',
                      help='use AM32 SITL ESCs (make AM32_SITL_CAN in am32-firmware)')
    escs.add_argument('--scripted', action='store_true',
                      help='use scripted UDP ESCs from dshot_esc.py')
    parser.add_argument('--esc-port', type=int,
                        help='ESC 1 UDP signal port for --scripted and --am32-sitl '
                             '(default: a free range)')
    parser.add_argument('--timeout', type=float, default=600)
    parser.add_argument('--output', type=Path,
                        default=root / 'build' / 'renode-test' / 'bdshot')
    args = parser.parse_args()
    unknown = set(args.scenario) - set(SCENARIOS)
    if unknown:
        parser.error('unknown scenario %s' % ', '.join(sorted(unknown)))
    if not args.firmware.exists():
        parser.error('build %s Copter first: %s' % (BOARD, args.firmware))
    if args.am32_sitl is not None:
        args.am32_sitl = args.am32_sitl.resolve()
        if not args.am32_sitl.exists():
            parser.error('AM32 SITL not found: %s' % args.am32_sitl)
    failed = []
    backend = 'am32' if args.am32_sitl else 'scripted' if args.scripted else 'sim'
    for name in args.scenario or sorted(SCENARIOS):
        silent, recover, edt = SCENARIOS[name]
        if edt and args.scripted:
            print('scenario %s: skipped, scripted ESCs have no EDT' % name, flush=True)
            continue
        output = args.output / backend / name
        shutil.rmtree(output, ignore_errors=True)
        output.mkdir(parents=True)
        print('scenario %s (%s ESCs): silent %s, recovering %s, EDT %s' % (
            name, backend, list(silent), list(recover), edt), flush=True)
        try:
            run(args, root, output, silent, recover, edt)
            print('scenario %s: PASS' % name, flush=True)
        except (OSError, RuntimeError, ValueError) as error:
            print('scenario %s: FAIL: %s' % (name, error), flush=True)
            print(common.log_tail(output / 'renode.log'), flush=True)
            failed.append(name)
    if failed:
        sys.exit('failed: %s' % ', '.join(failed))


if __name__ == '__main__':
    main()
