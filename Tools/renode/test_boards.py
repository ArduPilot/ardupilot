#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
'''
Boot every supported Renode target and record the results, resumably.

Flight controllers run ArduPlane and pass once the full parameter table has
been downloaded over MAVLink. AP_Periph targets pass once their parameters
have been fetched over DroneCAN. Firmware is the latest ELF published on
firmware.ardupilot.org, downloaded once into the results directory, or a local
build of this tree when no ELF can be downloaded. Boards whose hwdef sets
AUTOBUILD_TARGETS None are not meant for general use and are skipped unless
named exactly.

Results are kept per board, so the script can be interrupted with control-C
(or by a reboot) and re-run to continue where it stopped. Use --fetch while
online to download all firmware and model data, after which testing needs no
network unless a board has to be built from a missing submodule.

  test_boards.py                    test every board not yet tested
  test_boards.py --status           show progress and failures
  test_boards.py --fetch            download firmware and model data only
  test_boards.py --retry-failed     test failed boards again
  test_boards.py 'Matek*' --rerun   test matching boards again
'''

import argparse
import datetime
import fcntl
import fnmatch
import json
import os
import random
import shutil
import signal
import subprocess
import sys
import termios
import threading
import time
import traceback

from concurrent.futures import FIRST_COMPLETED
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import wait
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

import gen_board  # noqa: E402
import renode_data  # noqa: E402
import renode_firmware  # noqa: E402

from launch import MonitorClient  # noqa: E402
from test_all import PortAllocator  # noqa: E402
from test_all import listener_started  # noqa: E402

DEFAULT_RESULTS = ROOT / 'build' / 'renode_tests'
# mcast:0/1 stay free for interactive use, as in test_all.py
CAN_BASES = (8, 6, 4, 2)
PASSED = 'pass'
FAILED = 'fail'
BUILD_FAILED = 'build-fail'
NO_FIRMWARE = 'no-firmware'
# a machine not running by then has usually failed to load its platform
PLATFORM_CHECK_S = 90
# MAVLink is only on USB, which plane runs leave down
NO_MAVLINK = 'no-mavlink-uart'
NO_MAVLINK_NOTE = 'serial:  no UART speaks MAVLink by default'
NO_USB_NOTE = 'does not have a supported Renode USB controller'
VHCI = Path('/sys/devices/platform/vhci_hcd.0')


class NeedsUsb(Exception):
    '''MAVLink is only on USB, so the board must be tested over USB.'''


def usb_tty(helper_log):
    '''The ACM tty of the device usbip_attach.py last attached, if any.'''
    ports = [line.split()[-1] for line in helper_log.read_text(errors='replace').splitlines()
             if line.startswith('attached ')]
    if not ports:
        return None
    try:
        status = (VHCI / 'status').read_text().splitlines()[1:]
    except OSError:
        return None
    for line in status:
        fields = line.split()
        if len(fields) >= 7 and fields[1].isdigit() and int(fields[1]) == int(ports[-1]):
            # interface 0 is the MAVLink console, SERIAL0
            for tty in Path('/sys/bus/usb/devices').glob('%s:1.0/tty/tty*' % fields[6]):
                device = Path('/dev') / tty.name
                # udev sets the group after the node appears
                if os.access(device, os.R_OK | os.W_OK):
                    return device
    return None


class Interrupted(Exception):
    '''The run is stopping; leave the board untested.'''


class TestFailure(Exception):
    def __init__(self, message, log='', status=FAILED):
        super().__init__(message)
        self.log = log
        self.status = status


def process_tree(root):
    '''The live process IDs below root, including root, from procfs.'''
    children = {}
    for stat in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = stat.read_text().rsplit(')', 1)[1].split()
        except (OSError, IndexError):
            continue
        children.setdefault(int(fields[1]), []).append(int(stat.parent.name))
    found = []
    pending = [root]
    while pending:
        pid = pending.pop()
        found.append(pid)
        pending += children.get(pid, [])
    return found


class Runner:
    '''Owns every child process so control-C can stop them all.'''

    def __init__(self):
        self.stopping = threading.Event()
        self.lock = threading.Lock()
        self.processes = set()
        self.kill_locks = {}

    def start(self, command, cwd, env=None, log=None):
        # holding the lock means stop_all() either sees this child or has
        # already set stopping
        with self.lock:
            if self.stopping.is_set():
                raise Interrupted()
            process = subprocess.Popen(
                command, cwd=cwd, env=env,
                stdout=log if log is not None else subprocess.PIPE,
                stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                text=True, bufsize=1, start_new_session=True)
            self.processes.add(process)
        return process

    def _kill_group(self, process, timeout):
        '''SIGTERM the process, then SIGKILL every process it started.'''
        with self.lock:
            lock = self.kill_locks.setdefault(process.pid, threading.Lock())
        # one signal sequence per process: a repeated SIGTERM would interrupt
        # run.py while it stops its own Renode process group
        with lock:
            tree = process_tree(process.pid) if process.poll() is None else []
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                pass
            # descendants can outlive the leader or sit in other groups
            for pid in tree + process_tree(process.pid):
                try:
                    os.kill(pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                pass

    def stop(self, process, timeout=5):
        if process is None:
            return
        self._kill_group(process, timeout)
        with self.lock:
            self.processes.discard(process)
            self.kill_locks.pop(process.pid, None)

    def stop_all(self):
        with self.lock:
            self.stopping.set()
            processes = list(self.processes)
        threads = [threading.Thread(target=self._kill_group, args=(process, 5))
                   for process in processes]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

    def check(self):
        if self.stopping.is_set():
            raise Interrupted()


def write_text(path, text):
    temporary = path.with_name('.%s.tmp' % path.name)
    with temporary.open('w') as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def write_json(path, data):
    write_text(path, json.dumps(data, indent=2, sort_keys=True) + '\n')


def read_json(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


class Results:
    '''Per-board result files below the results directory.'''

    def __init__(self, directory):
        self.directory = directory
        self.results = directory / 'results'
        self.logs = directory / 'logs'
        self.work = directory / 'work'
        self.firmware = directory / 'firmware'
        self.data = directory / 'data'
        self.running = directory / 'running'
        for path in (self.results, self.logs, self.work, self.firmware,
                     self.data, self.running):
            path.mkdir(parents=True, exist_ok=True)

    def get(self, board):
        return read_json(self.results / ('%s.json' % board))

    def begin(self, board):
        '''Forget an earlier result, so an interrupted test is redone.'''
        current = self.results / ('%s.json' % board)
        if current.exists():
            current.replace(current.with_suffix('.prev'))

    def record(self, board, status, detail, duration, firmware, log='', harness=None):
        previous = read_json(self.results / ('%s.prev' % board)) or {}
        # the log goes first, so a recorded failure always has its log
        log_path = self.logs / ('%s.log' % board)
        if status == PASSED:
            log_path.unlink(missing_ok=True)
        else:
            write_text(log_path, log)
        write_json(self.results / ('%s.json' % board), {
            'board': board,
            'status': status,
            'detail': detail,
            'duration_s': round(duration, 1),
            'firmware': firmware,
            'finished': datetime.datetime.now().isoformat(timespec='seconds'),
            'attempts': previous.get('attempts', 0) + 1,
            'harness': harness,
        })
        (self.results / ('%s.prev' % board)).unlink(missing_ok=True)


class Firmware:
    '''Downloads published ELF images, else builds this tree, serially.'''

    def __init__(self, results, runner, offline):
        self.results = results
        self.runner = runner
        self.offline = offline
        self.manifest = None
        self.manifest_error = None
        self.manifest_lock = threading.Lock()
        self.build_lock = threading.Lock()

    @staticmethod
    def vehicle(is_periph):
        return 'AP_Periph' if is_periph else 'Plane'

    def index_path(self, board):
        return self.results.firmware / ('%s.json' % board)

    def cached(self, board):
        record = read_json(self.index_path(board))
        if record is None:
            return None
        path = Path(record['path'])
        if not path.is_file() or path.stat().st_size != record.get('size'):
            return None
        return record

    def get_manifest(self):
        with self.manifest_lock:
            if self.manifest is None and self.manifest_error is None:
                if self.offline:
                    self.manifest_error = 'offline'
                else:
                    try:
                        self.manifest = renode_firmware.fetch_manifest()
                    except Exception as error:  # noqa: BLE001
                        self.manifest_error = repr(error)
            return self.manifest

    def download(self, board, is_periph):
        manifest = self.get_manifest()
        if manifest is None:
            return None, 'manifest unavailable: %s' % self.manifest_error
        try:
            entry = renode_firmware.select_firmware(
                manifest, board, self.vehicle(is_periph), 'latest')
            path, _ = renode_firmware.install_firmware(
                self.results.firmware / 'download', entry)
        except ValueError as error:
            return None, str(error)
        except Exception as error:  # noqa: BLE001
            # network trouble of any kind: build instead
            return None, 'download failed: %r' % error
        record = {
            'source': 'download',
            'url': entry['url'],
            'git-sha': entry['git-sha'],
            'path': str(path),
            'size': path.stat().st_size,
        }
        write_json(self.index_path(board), record)
        return record, None

    def build(self, board, is_periph, timeout):
        '''Build ArduPlane or AP_Periph from this tree, one board at a time.'''
        target = 'AP_Periph' if is_periph else 'plane'
        binary = ROOT / 'build' / board / 'bin' / (
            'AP_Periph' if is_periph else 'arduplane')
        log_path = self.results.work / ('%s-build.log' % board)
        destination = self.results.firmware / 'local' / board / binary.name
        (ROOT / 'build').mkdir(exist_ok=True)
        # the tree has one waf configuration, shared by every invocation
        with self.build_lock, (ROOT / 'build' / '.test_boards.lock').open('w') as tree_lock:
            while True:
                self.runner.check()
                try:
                    fcntl.flock(tree_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    time.sleep(1)
            binary.unlink(missing_ok=True)
            with log_path.open('w') as log:
                for command in (['./waf', 'configure', '--board', board],
                                ['./waf', target]):
                    log.write('$ %s\n' % ' '.join(command))
                    log.flush()
                    process = self.runner.start(command, ROOT, log=log)
                    try:
                        returncode = process.wait(timeout=timeout)
                    except subprocess.TimeoutExpired:
                        self.runner.stop(process)
                        raise TestFailure('%s timed out' % ' '.join(command),
                                          log_path.read_text(errors='replace'))
                    finally:
                        self.runner.stop(process)
                    self.runner.check()
                    if returncode != 0:
                        raise TestFailure('%s failed with status %d' %
                                          (' '.join(command), returncode),
                                          log_path.read_text(errors='replace'))
            if not binary.is_file():
                raise TestFailure('the build produced no %s' % binary,
                                  log_path.read_text(errors='replace'))
            # keep a copy so later builds of the tree cannot change it
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(binary, destination)
        log_path.unlink(missing_ok=True)
        record = {
            'source': 'local',
            'git-sha': git_head(),
            'path': str(destination),
            'size': destination.stat().st_size,
        }
        write_json(self.index_path(board), record)
        return record

    def get(self, board, is_periph, build_timeout, allow_build=True):
        record = self.cached(board)
        if record is not None:
            return record, None
        record, reason = self.download(board, is_periph)
        if record is not None or not allow_build:
            return record, reason
        return self.build(board, is_periph, build_timeout), reason


def git_head():
    try:
        return subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, check=True,
                              capture_output=True, text=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


# the code that turns a hwdef into a Renode platform and runs it
HARNESS_PATHS = ('Tools/renode', 'libraries/AP_HAL_ChibiOS/hwdef')


def harness_version():
    '''This tree's commit, and whether the harness has uncommitted changes.'''
    try:
        changes = subprocess.run(
            ['git', 'status', '--porcelain', '--untracked-files=no', '--'] +
            list(HARNESS_PATHS), cwd=ROOT, check=True, capture_output=True,
            text=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None
    return {'commit': git_head(), 'dirty': bool(changes)}


def harness_label(harness):
    if not harness:
        return 'unknown'
    return '%s%s' % ((harness.get('commit') or 'unknown')[:11],
                     '-dirty' if harness.get('dirty') else '')


def find_renode(explicit):
    if explicit:
        return explicit
    # the launcher's managed download
    cache = Path.home() / '.cache' / 'ardupilot' / 'renode'
    selected = read_json(cache / 'selected.json')
    if selected:
        path = cache / selected['install'] / selected['executable']
        if path.is_file():
            return str(path)
    return None


def wait_until(condition, deadline, process, what, runner):
    while time.monotonic() < deadline:
        runner.check()
        if condition():
            return
        if process.poll() is not None:
            raise TestFailure('Renode stopped with status %d before %s' %
                              (process.returncode, what))
        time.sleep(0.2)
    raise TestFailure('timed out waiting for %s' % what)


def uds_socket(state_dir):
    for path in state_dir.glob('APM-UDS-serial*'):
        if path.suffix != '.lock' and path.is_socket():
            return path
    return None


def mavlink_parameters(path, deadline, runner, usb=False):
    '''Wait for a heartbeat, then download the complete parameter table.'''
    from pymavlink import mavutil

    connection = None
    while connection is None:
        runner.check()
        try:
            connection = mavutil.mavlink_connection(
                str(path) if usb else 'uds:%s' % path, baud=115200,
                source_system=250, source_component=190)
        except OSError:
            if time.monotonic() >= deadline:
                raise TestFailure('could not connect to %s' % path)
            time.sleep(0.5)
        except ValueError:
            # 'uds:' needs pymavlink 2.4.50 or later
            raise TestFailure('this pymavlink cannot open uds:%s' % path)
    if usb:
        # a device that stops reading would otherwise block a write forever
        connection.port.write_timeout = 2

    def closed():
        # pymavlink would otherwise print and retry on every read
        raise TestFailure('the MAVLink socket closed')

    connection.handle_eof = closed
    connection.handle_disconnect = closed
    try:
        while True:
            runner.check()
            if time.monotonic() >= deadline:
                raise TestFailure('no MAVLink heartbeat')
            message = connection.recv_match(type='HEARTBEAT', blocking=True,
                                            timeout=1)
            if message is not None and message.get_srcComponent() == 1:
                break
        connection.mav.param_request_list_send(
            connection.target_system, connection.target_component)
        values = {}
        count = None
        last_progress = time.monotonic()
        while count is None or len(values) < count:
            runner.check()
            if time.monotonic() >= deadline:
                raise TestFailure('received %u of %s parameters' %
                                  (len(values), count))
            message = connection.recv_match(type='PARAM_VALUE', blocking=True,
                                            timeout=1)
            if message is not None:
                count = message.param_count
                if message.param_index < count:
                    values[message.param_index] = message.param_id
                last_progress = time.monotonic()
            elif count is not None and time.monotonic() - last_progress > 3:
                # ask again for anything the stream dropped
                for index in sorted(set(range(count)) - set(values))[:20]:
                    connection.mav.param_request_read_send(
                        connection.target_system, connection.target_component,
                        b'', index)
                last_progress = time.monotonic()
            elif count is None and time.monotonic() - last_progress > 5:
                # the request may have been sent before the link was ready
                connection.mav.param_request_list_send(
                    connection.target_system, connection.target_component)
                last_progress = time.monotonic()
        if count == 0:
            raise TestFailure('empty parameter table')
        return '%u MAVLink parameters' % count
    finally:
        if usb:
            close_tty(connection)
        else:
            connection.close()


def close_tty(connection):
    '''Close a serial MAVLink connection without waiting on a stalled device.'''
    def close():
        try:
            # a tty close waits for queued output the device may never read
            termios.tcflush(connection.port.fd, termios.TCOFLUSH)
        except (AttributeError, OSError, termios.error):
            pass
        connection.close()
    # a stalled device can block this thread in the kernel until Renode,
    # which the caller stops next, goes away
    closer = threading.Thread(target=close, daemon=True)
    closer.start()
    closer.join(5)


def dronecan_parameters(can_base, deadline, state_dir, runner):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise TestFailure('timed out before the DroneCAN client started')
    output_path = state_dir / 'dronecan.log'
    with output_path.open('w') as output:
        process = runner.start([sys.executable, str(HERE / 'test_all.py'),
                                '--dronecan-client', str(can_base), str(remaining)],
                               state_dir, log=output)
    try:
        while process.poll() is None:
            runner.check()
            if time.monotonic() > deadline + 5:
                raise TestFailure('DroneCAN client did not finish',
                                  output_path.read_text(errors='replace'))
            time.sleep(0.5)
        text = output_path.read_text(errors='replace')
        for line in text.splitlines():
            if line.startswith('DRONECAN_PARAMS '):
                fields = line.split()
                return '%s DroneCAN parameters from node %s' % (fields[1], fields[3])
        raise TestFailure('DroneCAN parameter fetch failed', text)
    finally:
        runner.stop(process)


def platform_failure(board, monitor_port, state_dir):
    '''The platform load error when the Renode machine never started, else None.'''
    monitor = MonitorClient('127.0.0.1', monitor_port)
    try:
        monitor.connect(timeout=30)
        if 'False' in monitor.command('machine IsPaused', timeout=30).splitlines():
            return None
        # the start script stops at the failed load, and Renode reports the
        # error only to the monitor: repeat the load in a scratch machine
        monitor.command('mach create "platform_check"', timeout=30)
        output = monitor.command('machine LoadPlatformDescription @%s' % (
            state_dir / 'generated' / ('%s.repl' % board)), timeout=120)
        monitor.command('mach set 0', timeout=30)
        monitor.command('mach rem "platform_check"', timeout=30)
        # a platform that loads is just slow to start
        return '\n'.join(output.splitlines()[1:-1]) or None
    except (OSError, TimeoutError):
        return None
    finally:
        monitor.close()


class CanBuses:
    '''CAN multicast bus pairs, locked so no other run shares one.'''

    def __init__(self, directory):
        directory.mkdir(parents=True, exist_ok=True)
        self.directory = directory

    def acquire(self, runner):
        while True:
            for base in CAN_BASES:
                lock = (self.directory / ('can-%u.lock' % base)).open('w')
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    return base, lock
                except BlockingIOError:
                    lock.close()
            runner.check()
            time.sleep(1)


def test_board(board, is_periph, firmware, args, results, runner, ports,
               can_buses, usb=False):
    state_dir = results.work / board
    shutil.rmtree(state_dir, ignore_errors=True)
    state_dir.mkdir(parents=True)
    (state_dir / 'tmp').mkdir()
    monitor_port = ports.allocate()
    command = [
        sys.executable, str(HERE / 'run.py'), board,
        '--vehicle', 'AP_Periph' if is_periph else 'arduplane',
        '--firmware', firmware['path'],
        '--state-dir', str(state_dir),
        '--data-cache', str(results.data),
        '--port', str(monitor_port),
        '--unthrottled',
    ]
    if args.renode:
        command += ['--renode', args.renode]
    can_lock = None
    if is_periph:
        can_base, can_lock = can_buses.acquire(runner)
        # an unused TCP port for any UART, so runs cannot collide
        command += ['--can-base', str(can_base),
                    '--uart-port', str(ports.allocate())]
    else:
        # MAVLink on a per-run socket; no CAN or real IOMCU, and USB only
        # for boards with no MAVLink UART
        command.append('--uds')
        if usb:
            usbip_port = ports.allocate()
            command += ['--usb', '--usbip-port', str(usbip_port)]
    environment = dict(os.environ)
    environment['PYTHONUNBUFFERED'] = '1'
    environment['XDG_CONFIG_HOME'] = str(state_dir / 'xdg')
    environment['TMPDIR'] = str(state_dir / 'tmp')
    process = None
    helper = None
    helper_log = state_dir / 'usbip.log'
    renode_log = state_dir / 'renode.log'
    try:
        with renode_log.open('w') as log:
            log.write('$ %s\n' % ' '.join(command))
            log.flush()
            process = runner.start(command, ROOT, environment, log=log)
        deadline = time.monotonic() + args.timeout
        try:
            if is_periph:
                wait_until(lambda: listener_started(monitor_port), deadline,
                           process, 'the Renode monitor', runner)
                monitor = MonitorClient('127.0.0.1', monitor_port)
                try:
                    monitor.connect(timeout=max(1, deadline - time.monotonic()))
                    # the monitor is up before the start command has run

                    def running():
                        return 'False' in monitor.command(
                            'machine IsPaused', timeout=30).splitlines()
                    try:
                        wait_until(running, min(deadline, time.monotonic() + PLATFORM_CHECK_S),
                                   process, 'the Renode machine to start', runner)
                    except TestFailure:
                        monitor.close()
                        error = platform_failure(board, monitor_port, state_dir)
                        if error:
                            raise TestFailure('Renode could not build the platform', error)
                        monitor.connect(timeout=30)
                        wait_until(running, deadline, process,
                                   'the Renode machine to start', runner)
                finally:
                    monitor.close()
                # let FDCAN finish its reset before node allocation starts
                time.sleep(3)
                return dronecan_parameters(can_base, deadline, state_dir, runner)
            try:
                wait_until(lambda: uds_socket(state_dir) is not None,
                           min(deadline, time.monotonic() + PLATFORM_CHECK_S),
                           process, 'the MAVLink socket', runner)
            except TestFailure:
                error = platform_failure(board, monitor_port, state_dir)
                if error:
                    raise TestFailure('Renode could not build the platform', error)
                wait_until(lambda: uds_socket(state_dir) is not None, deadline,
                           process, 'the MAVLink socket', runner)
            if NO_MAVLINK_NOTE not in renode_log.read_text(errors='replace'):
                return mavlink_parameters(uds_socket(state_dir), deadline, runner)
            if not usb:
                raise NeedsUsb()
            with helper_log.open('w') as log:
                helper = runner.start([sys.executable, str(HERE / 'usbip_attach.py'),
                                       '--port', str(usbip_port)], state_dir, log=log)
            tty = None

            def attached():
                nonlocal tty
                tty = usb_tty(helper_log)
                return tty is not None
            wait_until(attached, deadline, process, 'the USB serial device', runner)
            return mavlink_parameters(tty, deadline, runner, usb=True) + ' over USB'
        except (Interrupted, NeedsUsb):
            raise
        except Exception as error:  # noqa: BLE001
            runner.stop(process)
            runner.stop(helper)
            client = error.log if isinstance(error, TestFailure) else traceback.format_exc()
            log = 'FAILED: %s\n' % error
            if client:
                log += '\n--- client output ---\n%s\n' % client
            if helper_log.exists():
                log += '\n--- usbip_attach.py output ---\n%s\n' % helper_log.read_text(
                    errors='replace')
            renode_output = renode_log.read_text(errors='replace')
            log += '\n--- Renode output ---\n' + renode_output
            if usb and NO_USB_NOTE in renode_output:
                raise TestFailure('MAVLink is only on USB, which Renode does not '
                                  'model on this MCU', log, NO_MAVLINK) from None
            raise TestFailure(str(error) or repr(error), log,
                              getattr(error, 'status', FAILED)) from None
    finally:
        # detach the USB device before its Renode goes away
        runner.stop(helper)
        runner.stop(process)
        if can_lock is not None:
            can_lock.close()
        shutil.rmtree(state_dir, ignore_errors=True)


def model_data_ready(mcu, results, offline):
    '''The MCU's Renode model data is cached, fetching it if online.'''
    files = renode_data.MCU_DATA_FILES.get(mcu, {}).values()
    if all(renode_data.valid_cached_file(results.data / name,
                                         renode_data.DATA_FILES[name])
           for name in files):
        return True
    if offline:
        return False
    try:
        renode_data.ensure_mcu_data(mcu, results.data)
        return True
    except Exception:  # noqa: BLE001
        return False


def run_one(board, mcu, is_periph, args, results, runner, firmware, ports,
            can_buses, print_lock, counters):
    if not model_data_ready(mcu, results, args.offline):
        with print_lock:
            print('[ SKIP ] %s: no Renode model data for %s; run --fetch online'
                  % (board, mcu), flush=True)
        return
    marker = results.running / board
    marker.write_text('%d\n' % os.getpid())
    results.begin(board)
    harness = harness_version()
    started = time.monotonic()
    record = None
    try:
        with print_lock:
            print('[ RUN  ] %s' % board, flush=True)
        try:
            record, reason = firmware.get(board, is_periph, args.build_timeout)
        except TestFailure as error:
            status, detail, log = BUILD_FAILED, str(error), error.log
        except Interrupted:
            raise
        except Exception as error:  # noqa: BLE001
            status, detail, log = BUILD_FAILED, repr(error), traceback.format_exc()
        else:
            if record is None:
                status, detail, log = NO_FIRMWARE, reason, ''
            else:
                try:
                    try:
                        detail = test_board(board, is_periph, record, args, results,
                                            runner, ports, can_buses)
                    except NeedsUsb:
                        detail = test_board(board, is_periph, record, args, results,
                                            runner, ports, can_buses, usb=True)
                    status, log = PASSED, ''
                except TestFailure as error:
                    status, detail, log = error.status, str(error), error.log
                except Interrupted:
                    raise
                except Exception as error:  # noqa: BLE001
                    status, detail, log = FAILED, repr(error), traceback.format_exc()
        runner.check()
        results.record(board, status, detail, time.monotonic() - started,
                       record, log, harness)
        with print_lock:
            counters[status] = counters.get(status, 0) + 1
            label = {PASSED: 'PASS', NO_MAVLINK: 'USB'}.get(status, 'FAIL')
            print('[ %-4s ] %s (%.0fs): %s' % (
                label, board, time.monotonic() - started, detail), flush=True)
    except Interrupted:
        pass
    finally:
        marker.unlink(missing_ok=True)


def unbuilt_boards():
    '''Boards whose hwdef sets AUTOBUILD_TARGETS None.'''
    scripts = str(ROOT / 'Tools' / 'scripts')
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    import board_list
    return {board.name for board in board_list.BoardList().boards
            if board.autobuild_targets == []}


def boards(patterns):
    supported = gen_board.supported_boards(ROOT)
    unbuilt = unbuilt_boards()
    selected = sorted(board for board in supported
                      if board in patterns or board not in unbuilt and (
                          not patterns or any(fnmatch.fnmatchcase(board, pattern)
                                              for pattern in patterns)))
    return {board: (supported[board], gen_board.is_periph_board(ROOT, board))
            for board in selected}


def run_active(results):
    '''True while a test_boards.py run holds the results directory.'''
    with (results.directory / '.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return True
        fcntl.flock(lock, fcntl.LOCK_UN)
    return False


def show_status(results, targets, verbose, active=None):
    counts = {}
    by_mcu = {}
    failures = []
    # markers left by a reboot are not runs
    active = run_active(results) if active is None else active
    for board, (mcu, _) in targets.items():
        result = results.get(board)
        status = result['status'] if result else 'pending'
        if active and (results.running / board).exists():
            status = 'running'
        counts[status] = counts.get(status, 0) + 1
        mcu_counts = by_mcu.setdefault(mcu, {})
        mcu_counts[status] = mcu_counts.get(status, 0) + 1
        if result and result['status'] != PASSED and status != 'running':
            failures.append((board, result))
    tested = sum(counts.get(status, 0) for status in
                 (PASSED, FAILED, BUILD_FAILED, NO_FIRMWARE, NO_MAVLINK))
    print('%u boards: %u passed, %u failed, %u build failed, %u no firmware, '
          '%u MAVLink only on USB, %u running, %u pending (%u%% done)' % (
              len(targets), counts.get(PASSED, 0), counts.get(FAILED, 0),
              counts.get(BUILD_FAILED, 0), counts.get(NO_FIRMWARE, 0),
              counts.get(NO_MAVLINK, 0), counts.get('running', 0),
              counts.get('pending', 0), 100 * tested // max(1, len(targets))))
    print('\n%-14s %6s %6s %6s %6s %6s' % ('MCU', 'boards', 'pass', 'fail', 'usb', 'todo'))
    for mcu in sorted(by_mcu):
        mcu_counts = by_mcu[mcu]
        failed = sum(mcu_counts.get(status, 0)
                     for status in (FAILED, BUILD_FAILED, NO_FIRMWARE))
        todo = mcu_counts.get('pending', 0) + mcu_counts.get('running', 0)
        print('%-14s %6u %6u %6u %6u %6u' % (
            mcu, sum(mcu_counts.values()), mcu_counts.get(PASSED, 0), failed,
            mcu_counts.get(NO_MAVLINK, 0), todo))
    running = sorted(path.name for path in results.running.iterdir()) if active else []
    if running:
        print('\nrunning: %s' % ', '.join(running))
    current = harness_version()
    older = {}
    for board in targets:
        result = results.get(board)
        if result and result.get('harness') != current:
            label = harness_label(result.get('harness'))
            older[label] = older.get(label, 0) + 1
    if older:
        print('\nresults from another harness than %s: %s' % (
            harness_label(current),
            ', '.join('%u from %s' % (count, label)
                      for label, count in sorted(older.items()))))
    if failures:
        print('\nfailures:')
        for board, result in sorted(failures):
            print('  %-30s %-11s %s' % (board, result['status'], result['detail']))
            if verbose:
                print('      harness %s, log %s' % (
                    harness_label(result.get('harness')),
                    results.logs / ('%s.log' % board)))


def fetch_all(results, firmware, targets):
    for mcu in sorted({mcu for mcu, _ in targets.values()}):
        try:
            renode_data.ensure_mcu_data(mcu, results.data)
        except (OSError, RuntimeError, ValueError) as error:
            print('model data for %s: %s' % (mcu, error), flush=True)
    downloaded = local = 0
    for board, (_, is_periph) in targets.items():
        record, reason = firmware.get(board, is_periph, 0, allow_build=False)
        if record is None:
            local += 1
            print('%-30s needs a local build: %s' % (board, reason), flush=True)
        else:
            downloaded += 1
    print('%u boards have firmware, %u will be built locally' %
          (downloaded, local))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('patterns', nargs='*', metavar='BOARD',
                        help='board name wildcards (default: every supported board)')
    parser.add_argument('--results', type=Path, default=DEFAULT_RESULTS,
                        help='results and data directory (default: %(default)s)')
    parser.add_argument('--parallel', type=int, default=4,
                        help='boards tested at once (default: %(default)s)')
    parser.add_argument('--timeout', type=float, default=600,
                        help='seconds allowed for each boot test (default: %(default)s)')
    parser.add_argument('--build-timeout', type=float, default=3600,
                        help='seconds allowed for each waf step (default: %(default)s)')
    parser.add_argument('--nice', type=int, default=10,
                        help='niceness for this script, builds and Renode '
                             '(default: %(default)s)')
    parser.add_argument('--renode', help='Renode executable (default: the '
                        "launcher's download, else run.py's search)")
    parser.add_argument('--offline', action='store_true',
                        help='never download; build boards without cached firmware')
    parser.add_argument('--status', action='store_true', help='show progress and exit')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='with --status, show the log of each failure')
    parser.add_argument('--fetch', action='store_true',
                        help='download all firmware and model data, then exit')
    parser.add_argument('--retry-failed', action='store_true',
                        help='also test boards that failed before')
    parser.add_argument('--rerun', action='store_true',
                        help='test the selected boards again even if they passed')
    parser.add_argument('--sorted', action='store_true',
                        help='test boards in name order instead of a random order')
    parser.add_argument('--list', action='store_true', help='list the selected boards')
    args = parser.parse_args()
    if args.parallel < 1:
        parser.error('--parallel must be at least 1')
    if args.fetch and args.offline:
        parser.error('--fetch downloads, so cannot be used with --offline')

    results = Results(args.results.expanduser().resolve())
    targets = boards(args.patterns)
    if not targets:
        parser.error('no supported board matches %s' % ' '.join(args.patterns))
    if args.status:
        show_status(results, targets, args.verbose)
        return 0
    if args.list:
        for board, (mcu, is_periph) in targets.items():
            print('%-32s %-14s %s' % (board, mcu, 'AP_Periph' if is_periph else 'Plane'))
        return 0

    # one run per results directory
    lock = (results.directory / '.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('another test_boards.py is using %s' % results.directory)
    os.nice(args.nice)
    for prev in results.results.glob('*.prev'):
        # a .prev alone is an interrupted test, kept for its attempt count
        if (results.results / ('%s.json' % prev.stem)).exists():
            prev.unlink()
    for marker in results.running.iterdir():
        marker.unlink()
    for leftover in results.work.iterdir():
        shutil.rmtree(leftover, ignore_errors=True) if leftover.is_dir() \
            else leftover.unlink()

    runner = Runner()
    firmware = Firmware(results, runner, args.offline)
    if args.fetch:
        try:
            fetch_all(results, firmware, targets)
        except KeyboardInterrupt:
            print('\nstopped; downloads so far are kept', flush=True)
            return 130
        return 0

    args.renode = find_renode(args.renode)
    # unix socket paths are limited to 107 bytes
    longest = max(len(board) for board in targets)
    if len(str(results.work)) + longest + len('/APM-UDS-serial10') > 107:
        parser.error('--results path is too long for the MAVLink sockets')
    # control-C even if started with SIGINT ignored; a reboot or kill too
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    retry = ({FAILED, BUILD_FAILED, NO_FIRMWARE, NO_MAVLINK} if args.retry_failed
             else set())
    pending = []
    for board, (mcu, is_periph) in targets.items():
        result = results.get(board)
        if args.rerun or result is None or result['status'] in retry:
            pending.append((board, mcu, is_periph))
    # a random order spreads MCU families and slow boards across the run
    if not args.sorted:
        random.shuffle(pending)
    print('%u of %u boards to test, %u at a time; results in %s' % (
        len(pending), len(targets), args.parallel, results.directory), flush=True)
    if not pending:
        return exit_status(results, targets)

    ports = PortAllocator()
    # shared with other invocations so no two runs use one CAN bus
    can_buses = CanBuses(Path.home() / '.cache' / 'ardupilot' / 'renode' / 'locks')
    print_lock = threading.Lock()
    counters = {}
    executor = ThreadPoolExecutor(max_workers=args.parallel)
    futures = set()
    work = iter(pending)
    try:
        while True:
            while len(futures) < args.parallel:
                item = next(work, None)
                if item is None:
                    break
                futures.add(executor.submit(
                    run_one, *item, args, results, runner, firmware,
                    ports, can_buses, print_lock, counters))
            if not futures:
                break
            done, futures = wait(futures, timeout=1, return_when=FIRST_COMPLETED)
            for future in done:
                future.result()
    except KeyboardInterrupt:
        # finish stopping the children even if control-C is pressed again
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        print('\nstopping; re-run to continue', flush=True)
        runner.stop_all()
        executor.shutdown(wait=False, cancel_futures=True)
        # every worker thread, including one started as control-C arrived
        deadline = time.monotonic() + 20
        for thread in list(executor._threads):
            thread.join(timeout=max(0, deadline - time.monotonic()))
        stuck = sum(thread.is_alive() for thread in executor._threads)
        if stuck:
            # e.g. a stalled download; nothing of it is recorded
            print('%u worker(s) did not stop; exiting anyway' % stuck, flush=True)
        sys.stdout.flush()
        os._exit(130)
    executor.shutdown(wait=True)
    print('\nthis run: %s' % ', '.join('%u %s' % (count, status)
                                       for status, count in sorted(counters.items())))
    show_status(results, targets, False, active=False)
    return exit_status(results, targets)


def exit_status(results, targets):
    '''Non-zero while any selected board has failed or is still untested.'''
    return 1 if any((results.get(board) or {}).get('status') not in (PASSED, NO_MAVLINK)
                    for board in targets) else 0


if __name__ == '__main__':
    sys.exit(main())
