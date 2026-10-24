# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Vehicle-backed SITL checks for Rust loading, restart and Lua-style arming."""

import os
import socket
import subprocess
import time
import zlib

from pathlib import Path

import apmod_info
import pack
import pytest

from pymavlink import mavutil

import build

ROOT = Path(__file__).resolve().parents[3]
SYSTEM_CHECK = 1 << 13
CHECKSUM_MASK = 0x007FFFFF


@pytest.fixture(scope='module')
def images(tmp_path_factory):
    directory = tmp_path_factory.mktemp('runtime-modules')
    result = {}
    for mode in ('cooperative', 'cleanup', 'return', 'panic', 'late_stop', 'flood'):
        previous = os.environ.get('AP_MODULE_LIFECYCLE')
        os.environ['AP_MODULE_LIFECYCLE'] = mode
        try:
            path = build.build(ROOT / 'libraries/AP_Module/tests/modules/lifecycle.rs', directory / f'{mode}.apmod',
                               'x86_64-unknown-linux-gnu', 8192)
        finally:
            if previous is None:
                del os.environ['AP_MODULE_LIFECYCLE']
            else:
                os.environ['AP_MODULE_LIFECYCLE'] = previous
        result[mode] = path.read_bytes()
    return result


class Vehicle:
    def __init__(self, directory, files, extra, model='plane'):
        self.directory = directory
        self.directory.mkdir()
        self.modules = directory / 'modules'
        if files is not None:
            self.modules.mkdir()
            for name, data in files.items():
                (self.modules / name).write_bytes(data)
        settings = {'MOD_ENABLE': 1, 'MOD_HEAP_MAX': 4096, 'MOD_STACK_MAX': 65536,
                    'ARMING_REQUIRE': 1, 'LOG_BACKEND_TYPE': 0}
        settings.update(extra)
        self.defaults = directory / 'test.parm'
        self.defaults.write_text(''.join(f'{name} {value}\n' for name, value in settings.items()))
        self.process = None
        self.link = None
        self.log = None
        self.texts = []
        self.text_chunks = {}
        self.last_heartbeat = 0
        self.model = model

    def start(self):
        with socket.socket() as available:
            available.bind(('127.0.0.1', 0))
            port = available.getsockname()[1]
        binary = Path(os.environ.get('AP_MODULE_SITL_BINARY', ROOT / 'build/sitl/bin/arduplane'))
        if not binary.is_file():
            pytest.skip('Build SITL Plane with AP_MODULE_ENABLED=1')
        self.log = (self.directory / 'sitl.log').open('ab')
        self.process = subprocess.Popen([str(binary.resolve()), '--model', self.model, '--speedup', '5',
                                         '--base-port', str(port), '-I', '0', '--home', '-35.363261,149.165230,584,353',
                                         '--serial1', 'none', '--serial2', 'none',
                                         '--defaults', str(self.defaults)], cwd=self.directory,
                                        stdout=self.log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            self.check_alive()
            try:
                self.link = mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}', source_system=255)
                break
            except (ConnectionError, OSError):
                time.sleep(0.05)
        assert self.link is not None, self.diagnostic()
        self.until(lambda message: message.get_type() == 'HEARTBEAT', 20)
        assert self.parameter('MOD_ENABLE') in (0, 1)
        self.pump(5)  # let simulated home, sensors and startup calibration settle

    def stop(self):
        if self.link is not None:
            self.link.close()
            self.link = None
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
            self.process = None
        if self.log is not None:
            self.log.close()
            self.log = None

    def diagnostic(self):
        return '\n'.join(self.texts[-30:]) + '\n' + (self.directory / 'sitl.log').read_text(errors='replace')[-5000:]

    def check_alive(self):
        assert self.process.poll() is None, self.diagnostic()

    def receive(self):
        self.check_alive()
        if time.monotonic() - self.last_heartbeat > 1:
            self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                         0, 0, 0)
            self.last_heartbeat = time.monotonic()
        message = self.link.recv_match(blocking=True, timeout=0.1)
        if message is not None and message.get_type() == 'STATUSTEXT':
            identity = getattr(message, 'id', 0)
            sequence = getattr(message, 'chunk_seq', 0)
            if identity == 0:
                self.texts.append(message.text)
            else:
                key = (message.get_srcSystem(), message.get_srcComponent(), identity)
                previous = self.text_chunks.pop(key, None)
                if sequence == 0:
                    text = message.text
                elif previous is not None and previous[0] + 1 == sequence:
                    text = previous[1] + message.text
                else:
                    text = None
                if text is not None:
                    if len(message.text.encode('utf8')) < 50:
                        self.texts.append(text)
                    else:
                        self.text_chunks[key] = (sequence, text)
        elif message is not None and message.get_type() == 'PARAM_ERROR':
            self.texts.append(str(message))
        return message

    def until(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self.receive()
            if message is not None and predicate(message):
                return message
        raise AssertionError(self.diagnostic())

    def pump(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.receive()

    def parameter(self, name, value=None):
        deadline = time.monotonic() + 10
        sent = 0
        while time.monotonic() < deadline:
            if time.monotonic() - sent > 1:
                if value is None:
                    self.link.mav.param_request_read_send(1, 1, name.encode(), -1)
                else:
                    self.link.mav.param_set_send(1, 1, name.encode(), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                sent = time.monotonic()
            message = self.receive()
            if message is not None and message.get_type() == 'PARAM_VALUE' and message.param_id == name:
                return int(message.param_value)
        raise AssertionError(f'No parameter response for {name}\n{self.diagnostic()}')

    def wait_parameter(self, name, expected):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if self.parameter(name) == expected:
                return
            self.pump(0.1)
        raise AssertionError(f'{name} != {expected}\n{self.diagnostic()}')

    def text(self, fragment, after=0, timeout=10):
        deadline = time.monotonic() + timeout
        while not any(fragment in item for item in self.texts[after:]):
            if time.monotonic() >= deadline:
                raise AssertionError(self.diagnostic())
            self.receive()

    def restart(self, value, expected=1):
        mark = len(self.texts)
        self.parameter('MOD_RESTART', value)
        self.text(f'Module: loaded {expected} modules', after=mark)

    def checksums(self):
        self.parameter('MOD_DEBUG_OPTS', 32)
        self.wait_parameter('MOD_DEBUG_OPTS', 0)
        return self.parameter('MOD_LD_CSUM'), self.parameter('MOD_RUN_CSUM')

    def arm(self, armed=True, force=False, accepted=True):
        self.link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                        int(armed), 21196 if force else 0, 0, 0, 0, 0, 0)
        result = self.until(lambda msg: msg.get_type() == 'COMMAND_ACK' and
                            msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM).result
        if (result == mavutil.mavlink.MAV_RESULT_ACCEPTED) != accepted:
            self.pump(0.5)  # failure STATUSTEXT may follow the command acknowledgement
        assert (result == mavutil.mavlink.MAV_RESULT_ACCEPTED) == accepted, self.diagnostic()
        if accepted:
            self.until(lambda msg: msg.get_type() == 'HEARTBEAT' and
                       bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) == armed)

    def worker(self, name):
        for task in Path(f'/proc/{self.process.pid}/task').iterdir():
            try:
                if (task / 'comm').read_text().strip() == name:
                    fields = (task / 'stat').read_text().rsplit(')', 1)[1].split()
                    return int(task.name), (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')
            except FileNotFoundError:
                pass
        return None

    def anonymous_mappings(self):
        writable = executable = 0
        for line in Path(f'/proc/{self.process.pid}/maps').read_text().splitlines():
            fields = line.split()
            if len(fields) != 5:  # exclude file mappings and named process/stack areas
                continue
            low, high = (int(value, 16) for value in fields[0].split('-'))
            if 'w' in fields[1] and 'x' not in fields[1]:
                writable += high - low
            if 'x' in fields[1] and 'w' not in fields[1]:
                executable += high - low
        return writable, executable


@pytest.fixture
def launch(tmp_path):
    vehicles = []

    def start(files=None, **extra):
        vehicle = Vehicle(tmp_path / f'vehicle-{len(vehicles)}', files, extra)
        vehicles.append(vehicle)
        vehicle.start()
        return vehicle

    yield start
    for vehicle in vehicles:
        vehicle.stop()


def checksum(data):
    return zlib.crc32(data) & CHECKSUM_MASK


@pytest.mark.parametrize('stack_size', [24576, 24577, 65537])
def test_byte_granular_stack_size(tmp_path, launch, monkeypatch, stack_size):
    monkeypatch.setenv('AP_MODULE_LIFECYCLE', 'cooperative')
    module = build.build(ROOT / 'libraries/AP_Module/tests/modules/lifecycle.rs',
                         tmp_path / 'odd.apmod', 'x86_64-unknown-linux-gnu', 8193)
    vehicle = launch({'odd.apmod': module.read_bytes()}, SCR_ENABLE=0, MOD_STACK_MAX=stack_size)
    if stack_size < 8193 + 16384:
        # Keep the callback margin even when the HAL could round the stack up.
        vehicle.text('odd.apmod load failed: stack exceeds')
        vehicle.arm(accepted=False)
        return
    vehicle.text('ready')
    page = os.sysconf('SC_PAGESIZE')
    reserved = ((8193 + 16384 + page - 1) // page) * page
    vehicle.text(f'stack {reserved}')
    vehicle.arm()
    vehicle.arm(False)
    vehicle.restart(1)
    vehicle.text('Module: exited odd.apmod')


def test_per_module_heap_and_stack_requests(tmp_path, launch, monkeypatch):
    monkeypatch.setenv('AP_MODULE_LIFECYCLE', 'cleanup')
    files = {}
    for name, heap, stack in [('small', 2048, 1024), ('large', 8192, 8193)]:
        image = build.build(ROOT / 'libraries/AP_Module/tests/modules/lifecycle.rs',
                            tmp_path / f'{name}.apmod', 'x86_64-unknown-linux-gnu', stack, heap_size=heap)
        files[image.name] = image.read_bytes()
    vehicle = launch(files, SCR_ENABLE=0, MOD_HEAP_MAX=32768, MOD_STACK_MAX=65536)
    vehicle.text('Module: loaded 2 modules')
    vehicle.text('Module RAM: heap 2048 ')
    vehicle.text('Module RAM: heap 8192 ')
    for declared in (1024, 8193):
        page = os.sysconf('SC_PAGESIZE')
        reserved = ((declared + 16384 + page - 1) // page) * page
        vehicle.text(f'stack {reserved}')
    vehicle.arm()
    vehicle.arm(False)


def test_initial_heap_above_limit_rejected(tmp_path, launch):
    image = build.build(ROOT / 'libraries/AP_Module/examples/simple_loop.rs',
                        tmp_path / 'large.apmod', 'x86_64-unknown-linux-gnu', 8192, heap_size=8193)
    vehicle = launch({'large.apmod': image.read_bytes()}, SCR_ENABLE=0, MOD_HEAP_MAX=8192)
    vehicle.text('load failed: initial heap exceeds')
    assert not any('Module: started' in t for t in vehicle.texts)
    vehicle.arm(accepted=False)


@pytest.mark.parametrize('budget', ['negative', 'too_small', 'round_up', 'generous'])
def test_rounded_code_size(launch, images, budget):
    data = images['cooperative']
    image_size = pack.unpack(data)['memory_size']
    previous_page = ((image_size - 1) // 4096) * 4096
    assert 4096 <= previous_page < image_size - 1
    requested = {'negative': -1, 'too_small': previous_page,
                 'round_up': previous_page + 1, 'generous': 131071}[budget]
    vehicle = launch({'code.apmod': data}, SCR_ENABLE=0, MOD_CODE_MAXSIZE=requested)
    if budget in ('negative', 'too_small'):
        vehicle.text('invalid memory limits' if budget == 'negative' else
                     'code.apmod load failed: image exceeds')
        vehicle.arm(accepted=False)
        return
    vehicle.text('ready')
    # The parameter stays as entered; rounding affects the effective budget,
    # not how much image memory is actually allocated.
    assert vehicle.parameter('MOD_CODE_MAXSIZE') == requested
    page = os.sysconf('SC_PAGESIZE')
    reserved = ((image_size + page - 1) // page) * page
    vehicle.text('Module RAM: RX ')
    report = next(t for t in vehicle.texts if t.startswith('Module RAM: RX '))
    reported = int(report.split()[3])
    # The RAM report includes the executable-memory handle as well as pages.
    assert reserved <= reported < reserved + page, report
    vehicle.arm()
    vehicle.arm(False)
    vehicle.restart(1)


@pytest.mark.parametrize('offset,value,reason', [
    (0, 0, 'unsupported image'),
    (12, 40, 'wrong target'),
    (44, 0, 'ABI version mismatch'),
    (48, 1, 'CPU profile mismatch'),
    (48, 3, 'CPU profile mismatch'),
    (64, 0, 'API fingerprint'),
    (84, 0, 'image checksum mismatch'),
])
def test_load_failure_diagnostics(launch, images, offset, value, reason):
    data = bytearray(images['cooperative'])
    # Metadata is checked before CRC; the last case changes the CRC itself.
    data[offset:offset + 4] = value.to_bytes(4, 'little')
    vehicle = launch({'x.apmod': data}, SCR_ENABLE=0)
    vehicle.text(f'x.apmod load failed: {reason}')
    vehicle.arm(accepted=False)
    vehicle.text(f'Module: x.apmod: {reason}')


@pytest.mark.parametrize('priority,limit,accepted', [(1, 1, True), (2, 1, False), (3, 5, True), (6, 5, False)])
def test_module_thread_priority_limit(launch, images, priority, limit, accepted):
    data = apmod_info.resize_resources(images['cooperative'], thread_priority=priority)
    vehicle = launch({'p.apmod': data}, MOD_THD_PRIO_MAX=limit, SCR_ENABLE=0)
    if accepted:
        vehicle.text('Module: loaded 1 modules')
        assert vehicle.worker('p.apmod') is not None
        vehicle.restart(1)
        assert vehicle.worker('p.apmod') is not None
    else:
        vehicle.text('p.apmod load failed: thread priority exceeds MOD_THD_PRIO_MAX')
        vehicle.arm(accepted=False)
        assert vehicle.worker('p.apmod') is None


@pytest.mark.parametrize('limit', [0, 20, 130, 255])
def test_invalid_module_priority_limit(launch, limit):
    vehicle = launch({}, MOD_THD_PRIO_MAX=limit, SCR_ENABLE=0)
    assert vehicle.parameter('MOD_THD_PRIO_MAX') == limit
    vehicle.text('invalid MOD_THD_PRIO_MAX; reboot required')
    vehicle.arm(accepted=False)


def test_normal_exit_checksums_and_arming_bypasses(launch, images):
    vehicle = launch()
    assert vehicle.checksums() == (0, 0)
    vehicle.arm()
    vehicle.arm(False)
    vehicle.modules.mkdir()
    (vehicle.modules / 'one.apmod').write_bytes(images['return'])
    vehicle.restart(1)
    vehicle.text('Module: exited one.apmod')
    assert vehicle.checksums() == (checksum(images['return']), 0)
    vehicle.arm()
    vehicle.arm(False)
    vehicle.parameter('MOD_LD_CSUM', checksum(images['return']) ^ 1)
    vehicle.arm(accepted=False)
    vehicle.parameter('MOD_DEBUG_OPTS', 16)
    vehicle.arm()
    vehicle.arm(False)
    vehicle.parameter('MOD_DEBUG_OPTS', 0)
    vehicle.parameter('ARMING_SKIPCHK', SYSTEM_CHECK)
    vehicle.arm()
    vehicle.arm(False)
    vehicle.parameter('ARMING_SKIPCHK', 0)
    vehicle.arm(force=True)
    vehicle.arm(False)
    (vehicle.modules / 'two.apmod').write_bytes(images['return'])
    vehicle.restart(2, expected=2)
    vehicle.pump(0.5)
    assert vehicle.checksums() == (0, 0)  # duplicate files cancel, including after restart


def test_armed_restart_panic_retention_and_cpu(launch, images):
    vehicle = launch({'one.apmod': images['cleanup'], 'two.apmod': images['cooperative']})
    expected = checksum(images['cleanup']) ^ checksum(images['cooperative'])
    assert vehicle.checksums() == (expected, expected)
    vehicle.arm()
    for request in range(1, 6):
        old = [vehicle.worker(name)[0] for name in ('one.apmod', 'two.apmod')]
        vehicle.restart(request, expected=2)
        for name in ('one.apmod', 'two.apmod'):
            assert vehicle.worker(name)[0] not in old
        assert vehicle.checksums() == (expected, expected)
    (vehicle.modules / 'one.apmod').write_bytes(images['panic'])
    (vehicle.modules / 'two.apmod').unlink()
    mark = len(vehicle.texts)
    vehicle.parameter('MOD_RESTART', 6)
    vehicle.text('panic; reboot required', after=mark)
    vehicle.until(lambda msg: msg.get_type() == 'HEARTBEAT' and
                  bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
    before = vehicle.worker('one.apmod')
    assert before is not None
    vehicle.pump(2)
    after = vehicle.worker('one.apmod')
    assert before[0] == after[0] and after[1] - before[1] < 0.15
    vehicle.arm(False)
    vehicle.arm(accepted=False)
    vehicle.arm(force=True)
    mark = len(vehicle.texts)
    vehicle.parameter('MOD_RESTART', 7)
    vehicle.text('stop timeout', after=mark, timeout=15)
    assert vehicle.worker('one.apmod')[0] == before[0]
    assert not any('Module: loaded' in text for text in vehicle.texts[mark:])
    vehicle.arm(False)


def test_failed_generation_cleanup_and_recovery(launch, images):
    vehicle = launch({'one.apmod': images['cooperative'], 'two.apmod': b'bad image'})
    vehicle.arm(accepted=False)
    assert vehicle.worker('one.apmod') is None  # no worker starts before all images validate
    (vehicle.modules / 'two.apmod').write_bytes(images['return'])
    vehicle.restart(1, expected=2)
    vehicle.pump(0.5)
    expected = (checksum(images['cooperative']) ^ checksum(images['return']), checksum(images['cooperative']))
    assert vehicle.checksums() == expected
    vehicle.arm()
    vehicle.arm(False)
    (vehicle.modules / 'three.apmod').write_bytes(images['return'])
    vehicle.restart(2, expected=3)
    assert vehicle.worker('one.apmod') is not None
    too_long = vehicle.modules / ('x' * 58 + '.apmod')
    too_long.write_bytes(images['return'])
    mark = len(vehicle.texts)
    vehicle.parameter('MOD_RESTART', 3)
    vehicle.text('module filename exceeds 63 characters', after=mark)
    vehicle.arm(accepted=False)
    assert vehicle.worker('one.apmod') is None
    too_long.unlink()
    vehicle.restart(4, expected=3)


@pytest.fixture(scope='module')
def many_module_image(tmp_path_factory):
    directory = tmp_path_factory.mktemp('many-modules')
    path = build.build(ROOT / 'libraries/AP_Module/tests/modules/many_modules.rs', directory / 'many.apmod',
                       'x86_64-unknown-linux-gnu', 1024, heap_size=256)
    return path.read_bytes()


@pytest.mark.parametrize('count', [12, 32])
def test_many_modules_restart_and_partial_failure(launch, many_module_image, count):
    names = [f'm{i:03d}.apmod' for i in reversed(range(count))]
    vehicle = launch(dict.fromkeys(names, many_module_image), SCR_ENABLE=0)
    vehicle.text(f'Module: loaded {count} modules')
    for generation in range(4):
        vehicle.pump(0.5)
        workers = [vehicle.worker(name) for name in names]
        assert all(worker is not None for worker in workers), vehicle.diagnostic()
        assert len({worker[0] for worker in workers}) == count
        if generation < 3:
            vehicle.restart(generation + 1, expected=count)
            assert all(vehicle.worker(name)[0] != worker[0] for name, worker in zip(names, workers))
    # A late bad file must release every prepared image/context, without
    # starting a partial generation. Discovery order must not hide the file.
    (vehicle.modules / 'z-bad.apmod').write_bytes(b'bad image')
    mark = len(vehicle.texts)
    vehicle.parameter('MOD_RESTART', 4)
    vehicle.text('z-bad.apmod load failed', after=mark)
    assert all(vehicle.worker(name) is None for name in names)
    (vehicle.modules / 'z-bad.apmod').unlink()
    vehicle.restart(5, expected=count)
    for name in names:
        (vehicle.modules / name).unlink()
    vehicle.restart(6, expected=0)
    assert all(vehicle.worker(name) is None for name in names)
    assert not any('returned -' in text or 'panic' in text for text in vehicle.texts), vehicle.diagnostic()


def test_saved_checksum_action_does_not_repeat_on_reboot(launch, images):
    vehicle = launch({'one.apmod': images['cooperative']}, MOD_RESTART=23)
    expected = checksum(images['cooperative'])
    assert vehicle.checksums() == (expected, expected)
    vehicle.pump(2)
    worker = vehicle.worker('one.apmod')[0]
    vehicle.pump(2)
    assert vehicle.worker('one.apmod')[0] == worker  # saved request did not request an extra reload
    vehicle.stop()
    (vehicle.modules / 'one.apmod').write_bytes(images['cleanup'])
    vehicle.start()
    vehicle.pump(2)
    assert vehicle.parameter('MOD_DEBUG_OPTS') == 0
    assert vehicle.parameter('MOD_LD_CSUM') == expected
    assert vehicle.parameter('MOD_RUN_CSUM') == expected
    assert expected != checksum(images['cleanup'])
    vehicle.arm(accepted=False)


def test_disabled_runtime_ignores_module_errors(launch):
    vehicle = launch({'bad.apmod': b'bad image'}, MOD_ENABLE=0)
    vehicle.arm()
    vehicle.arm(False)


def test_late_exit_and_coalesced_restarts(launch, images):
    vehicle = launch({'one.apmod': images['late_stop']})
    worker = vehicle.worker('one.apmod')[0]
    (vehicle.modules / 'one.apmod').write_bytes(images['cooperative'])
    mark = len(vehicle.texts)
    started = time.monotonic()
    for request in range(1, 7):
        vehicle.parameter('MOD_RESTART', request)
        vehicle.pump(0.2)
    vehicle.text('stop timeout', after=mark)
    # At speedup 5 the deadline is about two wall-clock seconds. Repeated
    # requests during Stopping must not keep pushing the deadline back.
    assert time.monotonic() - started < 3.5, vehicle.diagnostic()
    assert vehicle.worker('one.apmod')[0] == worker
    vehicle.text('Module: exited one.apmod', after=mark)
    assert vehicle.worker('one.apmod') is None
    assert not any('Module: loaded' in text for text in vehicle.texts[mark:])
    vehicle.arm(accepted=False)  # a late exit does not clear the timeout error
    vehicle.restart(7)
    vehicle.arm()
    vehicle.arm(False)


def test_directory_errors_and_text_queue_overflow(launch, images):
    vehicle = launch()
    vehicle.modules.write_bytes(b'not a directory')
    mark = len(vehicle.texts)
    vehicle.parameter('MOD_RESTART', 1)
    vehicle.text('module directory unavailable', after=mark)
    vehicle.arm(accepted=False)
    vehicle.modules.unlink()
    vehicle.modules.mkdir()
    (vehicle.modules / 'one.apmod').write_bytes(images['flood'])
    mark = len(vehicle.texts)
    vehicle.restart(2)
    vehicle.text('Module: dropped', after=mark)
    vehicle.arm()
    vehicle.arm(False)


def test_restart_memory_mappings(launch, images):
    vehicle = launch({'one.apmod': images['cleanup'], 'two.apmod': images['cooperative']})
    request = 0
    for _ in range(10):  # warm the libc allocator and vehicle telemetry paths
        request += 1
        vehicle.restart(request, expected=2)
    baseline = vehicle.anonymous_mappings()
    for cycle in range(40):
        if cycle % 5 == 0:
            (vehicle.modules / 'two.apmod').write_bytes(b'bad image' * 32)
            request += 1
            mark = len(vehicle.texts)
            vehicle.parameter('MOD_RESTART', request)
            vehicle.text('load failed', after=mark)
            (vehicle.modules / 'two.apmod').write_bytes(images['cooperative'])
        request += 1
        vehicle.restart(request, expected=2)
        assert vehicle.anonymous_mappings() == baseline, vehicle.diagnostic()
    # This checks mapping reservations, not suballocations inside malloc arenas;
    # run it under Valgrind as well to catch leaked heap/metadata allocations.
