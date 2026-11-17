#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Check bidirectional DShot telemetry error accounting under Renode.

Boots Copter on MatekH743-bdshot with its DShot outputs bridged over UDP to
ESCs: scripted ones from Tools/renode/dshot_esc.py, or AM32 SITL ESCs with
--am32-sitl. Some ESCs stay silent, and in the recover scenario one starts
answering part way through. The DataFlash log is then checked:

- ESCs answering throughout keep ESC.Err near zero
- silent ESCs never log, and each answering ESC reports its own rpm
- an ESC that recovers first logs a high ESC.Err, from the failures counted
  against it while it was silent, which then falls to near zero
"""

import argparse
import os
import random
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
    # name: (ESCs silent at the start, of those the ones that later answer)
    'all': ((), ()),
    'one': ((2, 3, 4), ()),
    'recover': ((2,), (2,)),
}
MAX_ERROR_RATE = 5.0
MIN_RECOVERY_ERROR_RATE = 50.0
SILENT_SECONDS = 20
RECOVERED_SECONDS = 15
MOTOR_TEST_PERCENT = 15


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


class ScriptedESCs:
    def __init__(self, base_port, silent, output):
        self.bench = ESCBench(base_port, MOTORS, ESC_ERPM, set(silent), ESC_ERPM_STEP)

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

    def check_rpm(self, rpm, answering):
        for esc in range(2, MOTORS + 1):
            if rpm[esc - 1] != 0:
                raise RuntimeError('idle ESC%u reports %u rpm' % (esc, rpm[esc - 1]))
        return rpm[0] > 500

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


def run(args, root, output, silent, recover):
    answering = {esc for esc in range(1, MOTORS + 1) if esc not in silent}
    log_path = output / 'renode.log'
    uart_port = common.unused_tcp_port()
    monitor_port = common.unused_tcp_port()
    esc_port = args.esc_port or free_port_base(MOTORS)
    if args.am32_sitl:
        escs = AM32ESCs(esc_port, silent, output, args.am32_sitl)
    else:
        escs = ScriptedESCs(esc_port, silent, output)
    process = None
    try:
        command = [
            sys.executable, str(root / 'Tools/renode/run.py'), BOARD,
            '--firmware', str(args.firmware),
            '--state-dir', str(output / 'state'),
            '--uart-port', str(uart_port), '--port', str(monitor_port),
            '--esc-signal-port', str(esc_port),
        ]
        if args.renode:
            command += ['--renode', args.renode]
        with log_path.open('w') as log:
            process = subprocess.Popen(command, cwd=root, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        print('waiting for the firmware to boot (Renode log %s)' % log_path, flush=True)
        link = common.connect_mavlink(uart_port, process, log_path,
                                      time.monotonic() + args.timeout)
        link.setup_logfile(str(output / 'bdshot.tlog'))
        for name, value in PARAMETERS.items():
            set_parameter(link, process, log_path, name, value, args.timeout)
        reboot(link, process, log_path, args.timeout)
        if args.am32_sitl:
            common.send_command(link, process, log_path, mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                                (1, 0, MOTOR_TEST_PERCENT,
                                 SILENT_SECONDS + RECOVERED_SECONDS + 30, 1, 0))
        rpm = wait_rpm(link, process, log_path, escs, answering, args.timeout)
        print('ESC telemetry rpm %s' % rpm, flush=True)
        # the vehicle is at least this far along when the ESCs start answering
        recovered_ms = wait_sim_seconds(link, process, log_path, SILENT_SECONDS, args.timeout)
        for esc in recover:
            escs.start(esc)
        wait_sim_seconds(link, process, log_path, RECOVERED_SECONDS, args.timeout)
        escs.summary()
        log_file = common.download_log(link, process, log_path, output / 'bdshot.BIN',
                                       list_timeout=180)
    finally:
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
    parser.add_argument('--am32-sitl', type=Path, metavar='ELF',
                        help='use AM32 SITL ESCs (make AM32_SITL_CAN in am32-firmware)')
    parser.add_argument('--esc-port', type=int,
                        help='ESC 1 UDP signal port (default: a free range)')
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
    for name in args.scenario or sorted(SCENARIOS):
        silent, recover = SCENARIOS[name]
        output = args.output / ('am32-' + name if args.am32_sitl else name)
        shutil.rmtree(output, ignore_errors=True)
        output.mkdir(parents=True)
        print('scenario %s: silent ESCs %s, recovering %s' % (
            name, list(silent), list(recover)), flush=True)
        try:
            run(args, root, output, silent, recover)
            print('scenario %s: PASS' % name, flush=True)
        except (OSError, RuntimeError, ValueError) as error:
            print('scenario %s: FAIL: %s' % (name, error), flush=True)
            print(common.log_tail(output / 'renode.log'), flush=True)
            failed.append(name)
    if failed:
        sys.exit('failed: %s' % ', '.join(failed))


if __name__ == '__main__':
    main()
