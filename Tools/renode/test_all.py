#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
'''
Boot built Renode targets and verify their parameter transport.

The optional pattern matches either a board name or ``board/vehicle`` and
defaults to every supported firmware found below ``build/*/bin``. Flight
controllers must produce a MAVLink heartbeat and complete MAVProxy's
``param ftp`` download. AP_Periph targets must appear on DroneCAN and answer
indexed GetSet requests through the end of their parameter table.
'''

import argparse
import errno
import fnmatch
import os
import queue
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

from collections import deque
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import as_completed
from dataclasses import dataclass
from pathlib import Path

import gen_board

FLIGHT_FIRMWARE = (
    'arducopter',
    'arduplane',
    'ardurover',
    'ardusub',
    'antennatracker',
)
# Leave mcast:0/1 available for an interactive Renode or hardware bridge while
# the automated matrix uses the remaining non-overlapping bus pairs.
CAN_BASES = (8, 6, 4, 2)
MAVPROXY_SUCCESS = re.compile(r'Received (\d+) parameters \(ftp\)')
MAVPROXY_HEARTBEAT = re.compile(r'Detected vehicle \d+:\d+ on link \d+')
RENODE_STARTED = re.compile(r'Machine started\.')


def dronecan_param_test(bus, timeout):
    # Keep this import in the client process. Flight-controller-only test
    # hosts do not need pydronecan installed.
    import dronecan

    from dronecan import uavcan

    deadline = time.monotonic() + timeout
    node = dronecan.make_node('mcast:%u' % bus, node_id=127, bitrate=1000000)
    monitor = dronecan.app.node_monitor.NodeMonitor(node)
    allocator = dronecan.app.dynamic_node_id.CentralizedServer(node, monitor)
    node.mode = uavcan.protocol.NodeStatus().MODE_OPERATIONAL

    try:
        target = None
        while target is None and time.monotonic() < deadline:
            node.spin(timeout=0.1)
            remote_nodes = [node_id for node_id in monitor.get_all_node_id()
                            if node_id != node.node_id]
            if remote_nodes:
                target = remote_nodes[0]
        if target is None:
            raise RuntimeError('timed out waiting for an AP_Periph DroneCAN node')

        count = 0
        while time.monotonic() < deadline:
            response = []

            def callback(event):
                response.append(event)

            request = uavcan.protocol.param.GetSet.Request(index=count)
            node.request(request, target, callback, timeout=2.0)
            while not response and time.monotonic() < deadline:
                node.spin(timeout=0.1)
            if not response:
                raise RuntimeError('timed out fetching DroneCAN parameter %u' % count)
            if response[0] is None:
                raise RuntimeError('no response for DroneCAN parameter %u' % count)
            name = response[0].response.name.decode('utf-8')
            if not name:
                break
            count += 1
            if count > 20000:
                raise RuntimeError('DroneCAN parameter table did not terminate')
        else:
            raise RuntimeError('timed out fetching DroneCAN parameters')

        if count == 0:
            raise RuntimeError('AP_Periph returned an empty parameter table')
        print('DRONECAN_PARAMS %u NODE %u' % (count, target), flush=True)
    finally:
        allocator.close()
        monitor.close()
        node.close()


@dataclass(frozen=True)
class Build:
    board: str
    vehicle: str
    is_periph: bool

    @property
    def name(self):
        return '%s/%s' % (self.board, self.vehicle)


@dataclass
class Result:
    build: Build
    duration: float
    detail: str
    log: str = ''
    failed: bool = False

    @property
    def passed(self):
        return not self.failed


class ClientFailure(RuntimeError):
    def __init__(self, message, output):
        super().__init__(message)
        self.output = output


class ProcessOutput:
    '''Drain a child pipe so verbose Renode output cannot block the child.'''

    def __init__(self, process, maximum_lines=100):
        self.process = process
        self.lines = deque(maxlen=maximum_lines)
        self.condition = threading.Condition()
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        try:
            for line in self.process.stdout:
                with self.condition:
                    self.lines.append(line.rstrip())
                    self.condition.notify_all()
        finally:
            with self.condition:
                self.condition.notify_all()

    def text(self):
        with self.condition:
            return '\n'.join(self.lines)

    def wait(self, timeout):
        with self.condition:
            self.condition.wait(timeout)


class PortAllocator:
    def __init__(self):
        self.lock = threading.Lock()
        self.used = set()

    def allocate(self):
        with self.lock:
            while True:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.bind(('127.0.0.1', 0))
                    port = sock.getsockname()[1]
                if port not in self.used:
                    self.used.add(port)
                    return port


def positive_integer(value):
    value = int(value)
    if value < 1:
        raise argparse.ArgumentTypeError('must be at least 1')
    return value


def positive_float(value):
    value = float(value)
    if value <= 0:
        raise argparse.ArgumentTypeError('must be greater than zero')
    return value


def discover_builds(root):
    supported = gen_board.supported_boards(root)
    builds = []
    for board in sorted(supported):
        binary_dir = root / 'build' / board / 'bin'
        is_periph = gen_board.is_periph_board(root, board)
        vehicles = ('AP_Periph',) if is_periph else FLIGHT_FIRMWARE
        for vehicle in vehicles:
            if (binary_dir / vehicle).is_file():
                builds.append(Build(board, vehicle, is_periph))
    return builds


def listener_started(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            sock.bind(('127.0.0.1', port))
        except OSError as error:
            if error.errno == errno.EADDRINUSE:
                return True
            raise
    return False


def wait_for_listener(process, output, port, deadline):
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError('Renode stopped with status %u' % process.returncode)
        if listener_started(port):
            return
        output.wait(0.1)
    raise RuntimeError('timed out waiting for MAVLink TCP port %u' % port)


def wait_for_pattern(process, output, pattern, deadline, description,
                     companion=None):
    while time.monotonic() < deadline:
        match = pattern.search(output.text())
        if match is not None:
            return match
        if process.poll() is not None:
            raise RuntimeError('%s stopped with status %u before %s' %
                               (os.path.basename(process.args[0]), process.returncode, description))
        if companion is not None and companion.poll() is not None:
            raise RuntimeError('Renode stopped with status %u before %s' %
                               (companion.returncode, description))
        output.wait(0.1)
    raise RuntimeError('timed out waiting for %s' % description)


def terminate_process_group(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    process.wait()


def start_process(command, cwd, env=None):
    return subprocess.Popen(
        command,
        cwd=cwd,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        stdin=subprocess.PIPE,
        text=True,
        bufsize=1,
        start_new_session=True,
    )


def test_mavlink(mavproxy, uart_port, state_dir, deadline):
    environment = dict(os.environ)
    environment['MPLCONFIGDIR'] = str(state_dir / 'matplotlib')
    environment['PYTHONUNBUFFERED'] = '1'
    command = [
        mavproxy,
        '--master=tcp:127.0.0.1:%u' % uart_port,
        '--no-state',
        '--default-modules=ftp',
    ]
    process = start_process(command, state_dir, environment)
    output = ProcessOutput(process)
    try:
        wait_for_pattern(process, output, MAVPROXY_HEARTBEAT, deadline,
                         'a MAVLink heartbeat')
        # Parameter enumeration can still change during vehicle setup. Delay
        # loading the param module so its automatic first fetch, and our
        # explicit FTP fetch, both see the stable table.
        time.sleep(min(5, max(0, deadline - time.monotonic())))
        process.stdin.write('module load param\nparam ftp\n')
        process.stdin.flush()
        match = wait_for_pattern(process, output, MAVPROXY_SUCCESS, deadline,
                                 'MAVLink parameters via FTP')
        count = int(match.group(1))
        if count == 0:
            raise RuntimeError('MAVProxy downloaded an empty parameter table')
        return '%u MAVLink parameters via FTP' % count
    except Exception as error:
        raise ClientFailure(str(error), output.text()) from error
    finally:
        terminate_process_group(process)


def test_dronecan(can_base, state_dir, deadline, renode_process):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise RuntimeError('timed out before starting the DroneCAN client')
    command = [
        sys.executable,
        str(Path(__file__).resolve()),
        '--dronecan-client', str(can_base), str(remaining),
    ]
    process = start_process(command, state_dir)
    output = ProcessOutput(process)
    pattern = re.compile(r'DRONECAN_PARAMS (\d+) NODE (\d+)')
    try:
        match = wait_for_pattern(process, output, pattern, deadline,
                                 'all DroneCAN parameters', renode_process)
        return ('%u DroneCAN parameters from node %u' %
                (int(match.group(1)), int(match.group(2))))
    except Exception as error:
        raise ClientFailure(str(error), output.text()) from error
    finally:
        terminate_process_group(process)


def test_build(build, root, run_py, mavproxy, state_root, timeout, renode,
               board_locks, can_bases, ports, print_lock):
    started = time.monotonic()
    renode_process = None
    renode_output = None
    can_base = None
    safe_name = build.name.replace('/', '__')
    state_dir = state_root / safe_name
    state_dir.mkdir(parents=True)
    with print_lock:
        print('[ RUN  ] %s' % build.name, flush=True)

    try:
        with board_locks[build.board]:
            if build.is_periph:
                can_base = can_bases.get()
            uart_port = ports.allocate()
            monitor_port = ports.allocate()
            command = [
                sys.executable,
                str(run_py),
                build.board,
                '--vehicle', build.vehicle,
                '--state-dir', str(state_dir),
                '--uart-port', str(uart_port),
                '--port', str(monitor_port),
                '--unthrottled',
            ]
            if renode:
                command += ['--renode', renode]
            if can_base is not None:
                command += ['--can-base', str(can_base)]

            renode_environment = dict(os.environ)
            renode_environment['PYTHONUNBUFFERED'] = '1'
            renode_environment['XDG_CONFIG_HOME'] = str(state_dir / 'xdg')
            renode_temp = state_dir / 'tmp'
            renode_temp.mkdir()
            renode_environment['TMPDIR'] = str(renode_temp)
            renode_process = start_process(command, root, renode_environment)
            renode_output = ProcessOutput(renode_process)
            deadline = time.monotonic() + timeout
            if build.is_periph:
                wait_for_pattern(renode_process, renode_output,
                                 RENODE_STARTED, deadline,
                                 'the Renode machine to start')
                # Starting the allocator while firmware is still resetting
                # FDCAN can leave an anonymous node midway through allocation.
                time.sleep(min(3, max(0, deadline - time.monotonic())))
                detail = test_dronecan(can_base, state_dir, deadline,
                                       renode_process)
            else:
                wait_for_listener(renode_process, renode_output, uart_port, deadline)
                detail = test_mavlink(mavproxy, uart_port, state_dir, deadline)
        return Result(build, time.monotonic() - started, detail)
    except Exception as error:  # noqa: BLE001
        logs = []
        if isinstance(error, ClientFailure) and error.output:
            logs += ['--- client output (tail) ---', error.output]
        if renode_output is not None and renode_output.text():
            logs += ['--- Renode output (tail) ---', renode_output.text()]
        log = '\n'.join(logs)
        return Result(build, time.monotonic() - started, str(error), log,
                      failed=True)
    finally:
        terminate_process_group(renode_process)
        if can_base is not None:
            can_bases.put(can_base)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('pattern', nargs='?', default='*',
                        help='wildcard matching a board or board/vehicle (default: *)')
    parser.add_argument('--parallel', type=positive_integer, default=1,
                        help='number of tests to run in parallel (default: 1)')
    parser.add_argument('--timeout', type=positive_float, default=180,
                        help='timeout for each running test in seconds (default: 180)')
    parser.add_argument('--renode', help='Renode executable passed to run.py')
    parser.add_argument('--list', action='store_true', help='list matching built firmware and exit')
    parser.add_argument('--dronecan-client', nargs=2, metavar=('BUS', 'TIMEOUT'),
                        help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.dronecan_client is not None:
        dronecan_param_test(int(args.dronecan_client[0]),
                            float(args.dronecan_client[1]))
        return 0

    root = Path(__file__).resolve().parents[2]
    builds = [build for build in discover_builds(root)
              if fnmatch.fnmatchcase(build.board, args.pattern) or
              fnmatch.fnmatchcase(build.name, args.pattern)]
    if args.list:
        print('\n'.join(build.name for build in builds))
        return 0
    if not builds:
        parser.error("pattern '%s' matched no supported builds" % args.pattern)

    mavproxy = shutil.which('mavproxy.py')
    if mavproxy is None and any(not build.is_periph for build in builds):
        parser.error('mavproxy.py is required for flight-controller tests')

    run_py = root / 'Tools' / 'renode' / 'run.py'
    board_locks = {build.board: threading.Lock() for build in builds}
    can_bases = queue.Queue()
    for can_base in CAN_BASES:
        can_bases.put(can_base)
    ports = PortAllocator()
    print_lock = threading.Lock()
    failures = []

    print('Testing %u build(s) with %u worker(s)' %
          (len(builds), min(args.parallel, len(builds))), flush=True)
    with tempfile.TemporaryDirectory(prefix='ardupilot-renode-tests-') as temporary:
        state_root = Path(temporary)
        with ThreadPoolExecutor(max_workers=args.parallel) as executor:
            futures = [
                executor.submit(
                    test_build, build, root, run_py, mavproxy, state_root,
                    args.timeout, args.renode, board_locks, can_bases, ports,
                    print_lock)
                for build in builds
            ]
            for future in as_completed(futures):
                result = future.result()
                with print_lock:
                    if result.passed:
                        print('[ PASS ] %s (%.1fs): %s' %
                              (result.build.name, result.duration, result.detail), flush=True)
                    else:
                        failures.append(result)
                        print('[ FAIL ] %s (%.1fs): %s' %
                              (result.build.name, result.duration, result.detail), flush=True)
                        if result.log:
                            print(result.log)

    passed = len(builds) - len(failures)
    print('\n%u passed, %u failed' % (passed, len(failures)))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
