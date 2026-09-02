#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Graphical target/device launcher for ArduPilot Renode boards.

The Target tab selects a board, firmware and bootloader. The Config tab
attaches emulated devices to ports derived from the board hwdef. The Physics
tab manages a standalone vehicle model and location. Together they build and
control a normal Tools/renode/run.py session. Renode's telnet monitor supplies
live PC, configured MIPS, executed MIPS and virtual-time speedup.

With --control-port N the launcher can also be driven over localhost TCP,
one command per line:

  board NAME, firmware auto|PATH, bootloader auto|none|PATH,
  download-firmware VEHICLE stable|latest,
  cpu none|N, real-iomcu on|off, iomcu-force-update on|off,
  usb on|off, can on|off, can-base N, ethernet off|INTERFACE,
  attach PORT DEVICE, detach PORT DEVICE,
  physics on|off, physics-model NAME,
  physics-location LAT LON ALT HEADING, physics-rate HZ,
  download-renode, start, stop, status, quit

Replies are prefixed OK, ERR or STATUS.
"""

import argparse
import hashlib
import json
import math
import os
import platform
import queue
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import tarfile
import tempfile
import threading
import time
import urllib.parse
import urllib.request
import zipfile

from pathlib import Path

from driver_catalog import ATTACHABLE_DEVICES
from driver_catalog import i2c_endpoints
from process_utils import terminate_process_group

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

ANSI_RE = re.compile(r'\x1b\[[0-9;?]*[ -/]*[@-~]')
PROMPT_RE = re.compile(r'\([^)]+\)\s*$')
METRICS_COMMAND = (
    'cpu PC; cpu PerformanceInMips; cpu ExecutedInstructions; '
    'emulation GetTimeSourceInfo'
)
MONITOR_COMMAND_ERROR_RE = re.compile(
    r"There was an error executing command|\bError E\d+:")

RENODE_DOWNLOAD_BASE = 'https://firmware.ardupilot.org/Tools/Renode/'
RENODE_LATEST_URL = urllib.parse.urljoin(RENODE_DOWNLOAD_BASE, 'latest.json')
RENODE_SELECTION = 'selected.json'
HAS_TAR_DATA_FILTER = hasattr(tarfile, 'data_filter')


def runtime_device_commands(attachment, port, device, attach):
    '''Return Renode monitor commands for one live attachment change.'''
    runtime_id = attachment['runtime_id']
    if port['bus'] == 'can':
        if not attach or not attachment.get('configure_bridge', True):
            return []
        return ['sysbus.%sMcast Bus %u' %
                (port['can_bus'].lower(), attachment['can_bus_number'])]

    if attach:
        if port['bus'] == 'uart':
            description = '%s: %s @ sysbus 0x%08X' % (
                runtime_id, device['model'], attachment['runtime_address'])
        else:
            endpoints = i2c_endpoints(device)
            commands = []
            for index, endpoint in enumerate(endpoints):
                endpoint_id = runtime_id
                if len(endpoints) > 1:
                    endpoint_id += 'Part%u' % index
                description = '%s: %s @ %s 0x%02X' % (
                    endpoint_id, endpoint['model'],
                    port['peripheral'].lower(), endpoint['address'])
                physics = device.get('physics')
                if physics is not None:
                    description += '\n    %s: %u' % (
                        physics['property'], attachment['physics_index'])
                commands.append(
                    'machine LoadPlatformDescriptionFromString %s' %
                    json.dumps(description))
            return commands
        physics = device.get('physics')
        if physics is not None:
            description += '\n    %s: %u' % (
                physics['property'], attachment['physics_index'])
        commands = [
            'machine LoadPlatformDescriptionFromString %s' %
            json.dumps(description),
        ]
        if port['bus'] == 'uart':
            hub = '%sHub' % runtime_id
            # Externals present at emulation startup are started
            # automatically. A live-created UARTHub otherwise remains paused
            # and silently drops every byte routed through it.
            commands += [
                'emulation CreateUARTHub %s' % json.dumps(hub),
                'connector Connect sysbus.%s %s' % (port['target'], hub),
                'connector Connect sysbus.%s %s' % (runtime_id, hub),
                '%s Start' % hub,
            ]
        return commands

    if port['bus'] == 'uart':
        path = 'sysbus.%s' % runtime_id
        external = '%sHub' % runtime_id
    else:
        endpoints = i2c_endpoints(device)
        commands = []
        for index, _endpoint in enumerate(endpoints):
            endpoint_id = runtime_id
            if len(endpoints) > 1:
                endpoint_id += 'Part%u' % index
            path = 'sysbus.%s.%s' % (
                port['peripheral'].lower(), endpoint_id)
            commands.append('machine APHotUnplug %s ""' % json.dumps(path))
        return commands
    return ['machine APHotUnplug %s %s' %
            (json.dumps(path), json.dumps(external))]


def execute_monitor_command_batch(client, commands, rollback_commands=()):
    '''Execute monitor commands and compensate completed commands on failure.'''
    completed = 0
    try:
        for command in commands:
            response = client.command(command, timeout=15)
            if MONITOR_COMMAND_ERROR_RE.search(response):
                raise RuntimeError(response.strip())
            completed += 1
        return None, False
    except (OSError, RuntimeError, TimeoutError) as error:
        rollback_errors = []
        for command in reversed(rollback_commands[:completed]):
            try:
                response = client.command(command, timeout=15)
                if MONITOR_COMMAND_ERROR_RE.search(response):
                    raise RuntimeError(response.strip())
            except (OSError, RuntimeError, TimeoutError) as rollback_error:
                rollback_errors.append(str(rollback_error))
        if rollback_errors:
            return ('%s; rollback failed: %s' %
                    (error, '; '.join(rollback_errors))), True
        return str(error), False


def default_renode_cache():
    root = os.environ.get('XDG_CACHE_HOME')
    if root:
        return Path(root).expanduser() / 'ardupilot' / 'renode'
    return Path.home() / '.cache' / 'ardupilot' / 'renode'


def find_physics_binary(explicit=None):
    """Find an installed or locally-built standalone physics sidecar."""
    if explicit:
        path = Path(explicit).expanduser()
        return path.resolve() if path.is_file() else None
    windows_host = platform.system() == 'Windows' or sys.platform == 'cygwin'
    executable = 'renode-physics.exe' if windows_host else 'renode-physics'
    candidates = (
        HERE / executable,
        HERE / 'bin' / executable,
        ROOT / 'build' / 'sitl' / 'tool' / executable,
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    found = shutil.which(executable)
    return Path(found).resolve() if found else None


def host_target(system=None, machine=None):
    """Return the platform and architecture names used by latest.json."""
    system = (system or platform.system()).lower()
    machine = (machine or platform.machine()).lower()
    platforms = {'linux': 'linux', 'darwin': 'macos', 'windows': 'windows'}
    architectures = {
        'amd64': 'x86_64',
        'x64': 'x86_64',
        'x86_64': 'x86_64',
        'aarch64': 'aarch64' if system == 'linux' else 'arm64',
        'arm64': 'aarch64' if system == 'linux' else 'arm64',
    }
    if system not in platforms or machine not in architectures:
        raise RuntimeError('no ArduPilot Renode download for %s/%s' %
                           (system, machine))
    return platforms[system], architectures[machine]


def select_renode_package(latest, system=None, machine=None):
    """Select the portable package for this host from latest.json."""
    wanted_platform, wanted_architecture = host_target(system, machine)
    for artifact in latest.get('artifacts', []):
        target = artifact.get('target', {})
        if (target.get('platform') != wanted_platform or
                target.get('architecture') != wanted_architecture):
            continue
        packages = artifact.get('packages', [])
        if wanted_platform == 'windows':
            packages = [package for package in packages
                        if package.get('filename', '').endswith('.zip')]
        elif wanted_platform == 'linux':
            packages = [package for package in packages
                        if package.get('filename', '').endswith('.tar.gz')]
        else:
            raise RuntimeError(
                'automatic Renode installation is not yet supported on macOS')
        if len(packages) != 1:
            raise RuntimeError('latest.json has no unique portable package for '
                               '%s/%s' % (wanted_platform, wanted_architecture))
        package = dict(packages[0])
        filename = package.get('filename')
        digest = package.get('sha256')
        size = package.get('size')
        if (not isinstance(filename, str) or Path(filename).name != filename or
                not isinstance(digest, str) or
                not re.fullmatch(r'[0-9a-fA-F]{64}', digest) or
                not isinstance(size, int) or size <= 0):
            raise RuntimeError('latest.json has invalid portable package metadata')
        runtime_identifier = target.get('runtime_identifier')
        if not isinstance(runtime_identifier, str) or not runtime_identifier:
            raise RuntimeError('latest.json has no runtime identifier')
        package['runtime_identifier'] = runtime_identifier
        package['platform'] = wanted_platform
        package['architecture'] = wanted_architecture
        return package
    raise RuntimeError('latest.json has no package for %s/%s' %
                       (wanted_platform, wanted_architecture))


def fetch_renode_latest(opener=None):
    """Fetch uncached current-version metadata from firmware.ardupilot.org."""
    opener = opener or urllib.request.urlopen
    separator = '&' if '?' in RENODE_LATEST_URL else '?'
    url = '%s%st=%u' % (RENODE_LATEST_URL, separator, time.time_ns())
    request = urllib.request.Request(
        url, headers={'Cache-Control': 'no-cache', 'Pragma': 'no-cache'})
    with opener(request, timeout=30) as response:
        data = response.read()
    latest = json.loads(data.decode('utf-8'))
    if latest.get('schema_version') != 1:
        raise RuntimeError('unsupported Renode latest.json schema')
    revision = latest.get('source', {}).get('revision')
    if not revision or not re.fullmatch(r'[0-9a-fA-F]{7,64}', revision):
        raise RuntimeError('latest.json has no valid source revision')
    return latest


def renode_cache_key(latest, package):
    revision = latest['source']['revision']
    runtime = package['runtime_identifier']
    digest = package['sha256'][:12]
    return '%s-%s-%s' % (runtime, revision[:12], digest)


def cached_renode(cache, latest=None, package=None):
    """Return a verified cache executable, optionally requiring latest."""
    cache = Path(cache).expanduser()
    try:
        selection = json.loads((cache / RENODE_SELECTION).read_text())
        install = (cache / selection['install']).resolve()
        executable = (install / selection['executable']).resolve()
        if (not install.is_relative_to(cache.resolve()) or
                not executable.is_relative_to(install) or
                not executable.is_file()):
            return None
        manifest = json.loads((install / 'ardupilot-renode.json').read_text())
    except (KeyError, OSError, ValueError, json.JSONDecodeError):
        return None
    if latest is not None and package is not None:
        expected = {
            'revision': latest['source']['revision'],
            'filename': package['filename'],
            'sha256': package['sha256'],
            'runtime_identifier': package['runtime_identifier'],
        }
        if any(manifest.get(key) != value for key, value in expected.items()):
            return None
    return executable


def download_file(url, destination, size, sha256, progress=None, opener=None):
    opener = opener or urllib.request.urlopen
    request = urllib.request.Request(url, headers={'Cache-Control': 'no-cache'})
    digest = hashlib.sha256()
    received = 0
    with opener(request, timeout=60) as response, destination.open('wb') as output:
        while True:
            block = response.read(1024 * 1024)
            if not block:
                break
            output.write(block)
            digest.update(block)
            received += len(block)
            if progress:
                progress(received, size)
    if received != size:
        raise RuntimeError('Renode download is %u bytes; expected %u' %
                           (received, size))
    if digest.hexdigest().lower() != sha256.lower():
        raise RuntimeError('Renode download SHA-256 does not match latest.json')


def fallback_tar_members(bundle, destination):
    """Validate tar members on Python versions without data_filter."""
    destination = destination.resolve()
    members = bundle.getmembers()
    symlinks = set()
    for member in members:
        target = (destination / member.name).resolve()
        if (not target.is_relative_to(destination) or member.islnk() or
                member.isdev()):
            raise RuntimeError('unsafe path in Renode tar package')
        if member.issym():
            link_target = (target.parent / member.linkname).resolve()
            if not link_target.is_relative_to(destination):
                raise RuntimeError('unsafe path in Renode tar package')
            symlinks.add(target)

    # Do not let a later member traverse a symlink created by the archive, and
    # reject symlink chains whose final destination cannot be validated before
    # extraction. The published package only links to a real in-tree directory.
    for member in members:
        target = (destination / member.name).resolve()
        if any(parent in symlinks for parent in target.parents):
            raise RuntimeError('unsafe path in Renode tar package')
        if member.issym():
            link_target = (target.parent / member.linkname).resolve()
            if (link_target in symlinks or
                    any(parent in symlinks for parent in link_target.parents)):
                raise RuntimeError('unsafe path in Renode tar package')
        if member.mode is not None:
            member.mode &= 0o755
        member.uid = member.gid = None
        member.uname = member.gname = None
    return members


def extract_renode(archive, destination, package):
    destination.mkdir()
    if package['filename'].endswith('.tar.gz'):
        with tarfile.open(archive, 'r:gz') as bundle:
            try:
                members = fallback_tar_members(bundle, destination)
                if HAS_TAR_DATA_FILTER:
                    bundle.extractall(
                        destination, members=members, filter='data')
                else:
                    bundle.extractall(destination, members=members)
            except tarfile.TarError as error:
                raise RuntimeError('unsafe path in Renode tar package') from error
        executable_name = 'renode'
    elif package['filename'].endswith('.zip'):
        with zipfile.ZipFile(archive) as bundle:
            for member in bundle.infolist():
                target = (destination / member.filename).resolve()
                if not target.is_relative_to(destination.resolve()):
                    raise RuntimeError('unsafe path in Renode zip package')
            bundle.extractall(destination)
        executable_name = 'renode.exe'
    else:
        raise RuntimeError('unsupported Renode package %s' % package['filename'])
    candidates = [path for path in destination.rglob(executable_name)
                  if path.is_file()]
    if len(candidates) != 1:
        raise RuntimeError('downloaded package has %u %s executables' %
                           (len(candidates), executable_name))
    executable = candidates[0]
    if package['platform'] != 'windows':
        executable.chmod(executable.stat().st_mode | 0o111)
    return executable


def install_current_renode(cache, latest=None, progress=None, opener=None):
    """Ensure the cache holds the freshly queried current Renode package."""
    cache = Path(cache).expanduser().resolve()
    latest = latest or fetch_renode_latest(opener)
    package = select_renode_package(latest)
    executable = cached_renode(cache, latest, package)
    if executable is not None:
        return executable, latest, False

    cache.mkdir(parents=True, exist_ok=True)
    install_name = renode_cache_key(latest, package)
    install = cache / install_name
    with tempfile.TemporaryDirectory(prefix='.download-', dir=cache) as temporary:
        temporary = Path(temporary)
        archive = temporary / package['filename']
        filename = package['filename']
        if Path(filename).name != filename:
            raise RuntimeError('invalid Renode package filename')
        url = urllib.parse.urljoin(RENODE_DOWNLOAD_BASE,
                                   urllib.parse.quote(filename))
        download_file(url, archive, package['size'], package['sha256'],
                      progress, opener)
        payload = temporary / 'payload'
        executable = extract_renode(archive, payload, package)
        manifest = {
            'revision': latest['source']['revision'],
            'renode_version': latest.get('renode_version'),
            'filename': filename,
            'sha256': package['sha256'],
            'runtime_identifier': package['runtime_identifier'],
            'executable': str(executable.relative_to(payload)),
        }
        (payload / 'ardupilot-renode.json').write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + '\n')
        if install.exists():
            shutil.rmtree(install)
        payload.rename(install)

    selection = {
        'install': install_name,
        'executable': manifest['executable'],
    }
    selection_tmp = cache / (RENODE_SELECTION + '.tmp')
    selection_tmp.write_text(json.dumps(selection, indent=2) + '\n')
    selection_tmp.replace(cache / RENODE_SELECTION)
    return install / manifest['executable'], latest, True


def is_elf(path):
    """Return true for an existing ELF file without relying on its suffix."""
    try:
        with path.open('rb') as stream:
            return stream.read(4) == b'\x7fELF'
    except OSError:
        return False


def firmware_choices(board):
    """Built ELF firmware choices for a board, in useful vehicle order."""
    bindir = ROOT / 'build' / board / 'bin'
    if not bindir.is_dir():
        return []
    preferred = {
        name: index for index, name in enumerate((
            'arducopter', 'arduplane', 'ardurover', 'ardusub',
            'antennatracker', 'blimp', 'AP_Periph',
        ))
    }
    paths = [path for path in bindir.iterdir() if path.is_file() and is_elf(path)]
    return sorted(paths, key=lambda path: (preferred.get(path.name, 100), path.name))


def default_bootloader(board):
    path = ROOT / 'Tools' / 'bootloaders' / ('%s_bl.bin' % board)
    return path if path.is_file() else None


def parse_elapsed(value):
    """Convert Renode's [days.]HH:MM:SS.s timestamp to seconds."""
    fields = value.strip().split(':')
    if len(fields) != 3:
        raise ValueError('bad elapsed time %s' % value)
    days = 0
    hours = fields[0]
    if '.' in hours:
        days_text, hours = hours.split('.', 1)
        days = int(days_text)
    return (days * 86400 + int(hours) * 3600 + int(fields[1]) * 60 +
            float(fields[2]))


def clean_monitor_text(data):
    text = data.decode('utf-8', errors='replace')
    text = ANSI_RE.sub('', text).replace('\r', '')
    # Telnet negotiation bytes decode as replacement characters. They carry
    # no monitor content and make prompt/value matching less predictable.
    return text.replace('\ufffd', '')


def parse_metrics(text):
    """Parse the response to METRICS_COMMAND."""
    values = re.findall(r'(?m)^\s*(0x[0-9A-Fa-f]+)\s*$', text)
    virtual = re.search(r'(?m)^Elapsed Virtual Time:\s*(\S+)\s*$', text)
    host = re.search(r'(?m)^Elapsed Host Time:\s*(\S+)\s*$', text)
    if len(values) < 3 or virtual is None or host is None:
        raise ValueError('incomplete Renode monitor metrics')
    return {
        'pc': int(values[0], 16),
        'mips': int(values[1], 16),
        'instructions': int(values[2], 16),
        'virtual_seconds': parse_elapsed(virtual.group(1)),
        'host_seconds': parse_elapsed(host.group(1)),
    }


class MonitorClient:
    """Small, single-threaded client for Renode's telnet monitor."""

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self, timeout=45):
        self.close()
        self.sock = socket.create_connection((self.host, self.port), timeout=2)
        self.sock.settimeout(0.5)
        self._read_to_prompt(timeout)

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def command(self, command, timeout=5):
        if self.sock is None:
            raise OSError('monitor is not connected')
        self.sock.sendall((command + '\n').encode('ascii'))
        return self._read_to_prompt(timeout, expected=command)

    def _read_to_prompt(self, timeout, expected=None):
        deadline = time.monotonic() + timeout
        data = bytearray()
        while time.monotonic() < deadline:
            try:
                chunk = self.sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                raise OSError('Renode monitor disconnected')
            data.extend(chunk)
            text = clean_monitor_text(data)
            if ((expected is None or expected in text) and
                    PROMPT_RE.search(text)):
                return text
        raise TimeoutError('timed out waiting for the Renode monitor')


class ProcRunner:
    """A child process and its process group, with line-oriented output."""

    def __init__(self, name, out_q):
        self.name = name
        self.out_q = out_q
        self.proc = None

    def start(self, command, cwd):
        self.proc = subprocess.Popen(
            command, cwd=cwd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, errors='replace',
            start_new_session=True)
        proc = self.proc
        threading.Thread(target=self._pump, args=(proc,), daemon=True).start()

    def _pump(self, proc):
        for line in proc.stdout:
            self.out_q.put('[%s] %s' % (self.name, line.rstrip('\n')))
        self.out_q.put(('__exit__', self.name, proc, proc.wait()))

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def is_current(self, proc):
        return proc is self.proc

    def stop(self):
        proc = self.proc
        if proc is None:
            return
        try:
            terminate_process_group(proc)
        finally:
            self.proc = None


class Launcher:
    """UI-independent launch configuration, lifecycle and live metrics."""

    def __init__(self, args):
        self.args = args
        self.log_q = queue.Queue()
        self.monitor_q = queue.Queue()
        self.runner = ProcRunner('renode', self.log_q)
        self.usb_runner = ProcRunner('usb', self.log_q)
        self.physics_runner = ProcRunner('physics', self.log_q)
        self.device_runners = {}
        self.board = None
        self.firmware = 'auto'
        self.bootloader = 'auto'
        self.cpu = None
        self.real_iomcu = False
        self.iomcu_force_update = False
        self.usb = False
        self.can = False
        self.can_base = 0
        self.ethernet = ''
        self.attachments = []
        self.can_ports = []
        self.active_can_ports = set()
        self.status = 'stopped'
        self.usb_status = 'off'
        self.metrics = None
        self.speedup = None
        self.executed_mips = None
        self.hotplug_error = None
        self.generation = 0
        self.next_hotplug_id = 0
        self.next_node_id = 120
        self.physics_enabled = False
        self.physics_model = 'quad'
        self.physics_latitude = -35.363261
        self.physics_longitude = 149.165230
        self.physics_altitude = 584.0
        self.physics_heading = 353.0
        self.physics_rate = 400
        self.physics_port = getattr(args, 'physics_port', 9002)
        self.physics_binary = find_physics_binary(
            getattr(args, 'physics_binary', None))
        self.physics_status = 'off'

    def log(self, message):
        self.log_q.put(message)

    def selected_firmware(self):
        if self.firmware == 'auto':
            return None
        return Path(self.firmware).expanduser()

    def selected_bootloader(self):
        if self.bootloader == 'none':
            return None
        if self.bootloader == 'auto':
            return default_bootloader(self.board)
        return Path(self.bootloader).expanduser()

    def build_command(self):
        if not self.board:
            raise ValueError('pick a board first')
        firmware = self.selected_firmware()
        if firmware is not None and not firmware.is_file():
            raise ValueError('no firmware at %s' % firmware)
        bootloader = self.selected_bootloader()
        if self.bootloader != 'none' and self.bootloader != 'auto' and (
                bootloader is None or not bootloader.is_file()):
            raise ValueError('no bootloader at %s' % self.bootloader)
        if self.cpu is not None and self.cpu not in os.sched_getaffinity(0):
            raise ValueError('CPU %u is outside this process affinity' % self.cpu)
        if self.iomcu_force_update and not self.real_iomcu:
            raise ValueError('force IOMCU update requires real IOMCU')
        if self.ethernet and not Path('/sys/class/net', self.ethernet).exists():
            raise ValueError('no network interface %s' % self.ethernet)

        command = [
            sys.executable, str(HERE / 'run.py'), self.board,
            '--port', str(self.args.monitor_port),
            '--uart-port', str(self.args.uart_port),
            '--no-device-sidecars',
        ]
        if self.args.renode:
            command += ['--renode', self.args.renode]
        if getattr(self.args, 'data_cache', None):
            command += ['--data-cache', self.args.data_cache]
        if self.args.state_dir:
            command += ['--state-dir', self.args.state_dir]
        if firmware is not None:
            command += ['--elf', str(firmware)]
        if bootloader is not None:
            command += ['--bootloader', str(bootloader)]
        if self.cpu is not None:
            command += ['--cpusel', str(self.cpu)]
        if self.real_iomcu:
            command.append('--real-iomcu')
        if self.iomcu_force_update:
            command.append('--iomcu-force-update')
        if self.usb:
            command += ['--usb', '--usbip-port', str(self.args.usbip_port)]
        if self.can or any(attachment.get('bus') == 'can'
                           for attachment in self.attachments):
            command += ['--can', '--can-base', str(self.can_base)]
        for attachment in self.attachments:
            specification = {
                'device': attachment['device'],
                'port': attachment['port'],
            }
            if 'physics_index' in attachment:
                specification['physics_index'] = attachment['physics_index']
            command += ['--device', json.dumps(
                specification, sort_keys=True, separators=(',', ':'))]
        if self.ethernet:
            command += ['--ethernet-tap', self.ethernet]
        return command

    def physics_connect_command(self):
        values = (self.physics_latitude, self.physics_longitude,
                  self.physics_altitude, self.physics_heading)
        if (not self.physics_model or
                any(isinstance(value, bool) or
                    not isinstance(value, (int, float)) or
                    not math.isfinite(value) for value in values)):
            raise ValueError('invalid physics configuration')
        if not -90 <= self.physics_latitude <= 90:
            raise ValueError('physics latitude must be from -90 to 90')
        if not -180 <= self.physics_longitude <= 180:
            raise ValueError('physics longitude must be from -180 to 180')
        if not -10000 <= self.physics_altitude <= 83000:
            raise ValueError('physics altitude must be from -10000 to 83000')
        if not -360 <= self.physics_heading <= 360:
            raise ValueError('physics heading must be from -360 to 360')
        if (isinstance(self.physics_rate, bool) or
                not isinstance(self.physics_rate, int) or
                not 1 <= self.physics_rate <= 10000):
            raise ValueError('physics rate must be from 1 to 10000')
        if (isinstance(self.physics_port, bool) or
                not isinstance(self.physics_port, int) or
                not 1 <= self.physics_port <= 65535):
            raise ValueError('physics port must be from 1 to 65535')
        return 'physics Connect %u %s %.10g %.10g %.10g %.10g %u' % (
            self.physics_port, json.dumps(self.physics_model),
            self.physics_latitude, self.physics_longitude,
            self.physics_altitude, self.physics_heading, self.physics_rate)

    def _physics_ready_loop(self, proc):
        deadline = time.monotonic() + 10
        while (self.physics_runner.is_current(proc) and proc.poll() is None and
               time.monotonic() < deadline):
            if self.port_has_listener(self.physics_port):
                self.log_q.put(('__physics_ready__', proc))
                return
            time.sleep(0.05)
        if self.physics_runner.is_current(proc) and proc.poll() is None:
            self.log_q.put(('__physics_start_error__', proc,
                            'timed out waiting for the listener'))

    def _connect_physics_if_ready(self):
        if (self.status != 'running' or self.physics_status != 'listening' or
                not self.physics_runner.running()):
            return
        self.physics_status = 'connecting'
        if not self.queue_monitor_commands(
                [self.physics_connect_command()],
                {'action': 'physics_connect'}):
            self.physics_status = 'listening'

    def start_physics(self):
        if self.physics_runner.running():
            return 'already running'
        if self.physics_binary is None or not self.physics_binary.is_file():
            return 'no renode-physics binary; build it or select an installed copy'
        try:
            self.physics_connect_command()
        except ValueError as error:
            return str(error)
        if self.physics_port in (self.args.monitor_port, self.args.uart_port,
                                 self.args.usbip_port):
            return 'physics port conflicts with another launcher port'
        if not self.wait_port_free(self.physics_port):
            return 'TCP port %u is already in use' % self.physics_port
        command = [
            str(self.physics_binary), '--physics-port', str(self.physics_port),
            '--model', self.physics_model,
        ]
        self.log('$ ' + shlex.join(command))
        try:
            self.physics_runner.start(command, HERE)
        except OSError as error:
            self.physics_status = 'error: %s' % error
            return self.physics_status
        self.physics_status = 'starting'
        threading.Thread(target=self._physics_ready_loop,
                         args=(self.physics_runner.proc,), daemon=True).start()
        return None

    def stop_physics(self):
        if not self.physics_runner.running():
            self.physics_status = 'off'
            return None
        if self.status == 'running':
            self.physics_status = 'disconnecting'
            if self.queue_monitor_commands(
                    ['physics Disconnect'], {'action': 'physics_disconnect'}):
                return None
        self.physics_runner.stop()
        self.physics_status = 'off'
        return None

    def complete_physics_command(self, action, error):
        if action == 'physics_connect':
            if error is None:
                self.physics_status = 'connected'
            else:
                self.physics_runner.stop()
                self.physics_status = 'connect failed: %s' % error
        elif action == 'physics_disconnect':
            self.physics_runner.stop()
            self.physics_status = ('off' if error is None else
                                   'disconnect failed: %s' % error)

    @staticmethod
    def port_has_listener(port):
        """Return true only when a TCP server is listening on localhost.

        Trying to bind the port is not equivalent: a connection made by the
        metrics client can leave the old Renode monitor port in TIME_WAIT for
        a while after Stop, even though the listener has gone away.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.2)
        try:
            return sock.connect_ex(('127.0.0.1', port)) == 0
        finally:
            sock.close()

    @classmethod
    def wait_port_free(cls, port, timeout=5):
        deadline = time.monotonic() + timeout
        while cls.port_has_listener(port):
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.2)
        return True

    def start(self):
        if self.runner.running():
            return 'already running'
        try:
            command = self.build_command()
        except ValueError as error:
            return str(error)
        ports = [self.args.monitor_port, self.args.uart_port]
        if self.usb:
            ports.append(self.args.usbip_port)
        if len(ports) != len(set(ports)):
            return 'monitor, UART and USB/IP ports must differ'
        for port in ports:
            if not self.wait_port_free(port):
                return 'TCP port %u is already in use' % port

        physics_started = False
        if self.physics_enabled and not self.physics_runner.running():
            error = self.start_physics()
            if error:
                return error
            physics_started = True

        self.generation += 1
        generation = self.generation
        self.metrics = None
        self.speedup = None
        self.executed_mips = None
        self.hotplug_error = None
        self.status = 'starting'
        self.usb_status = 'waiting' if self.usb else 'off'
        self.log('$ ' + shlex.join(command))
        self.next_hotplug_id = 0
        self.next_node_id = 120
        can_enabled = self.can or any(
            attachment['bus'] == 'can' for attachment in self.attachments)
        self.active_can_ports = set(self.can_ports) if can_enabled else set()
        try:
            for index, attachment in enumerate(self.attachments):
                attachment['runtime_id'] = 'configDevice%u' % index
                attachment['state'] = 'starting'
                if attachment['bus'] == 'can':
                    attachment.setdefault(
                        'can_bus_number', self.can_base +
                        max(0, int(attachment['port'][3:]) - 1))
                    attachment['node_id'] = self._allocate_node_id()
        except (KeyError, ValueError) as error:
            if physics_started:
                self.physics_runner.stop()
                self.physics_status = 'off'
            self.status = 'stopped'
            return str(error)
        try:
            self.runner.start(command, ROOT)
        except OSError as error:
            if physics_started:
                self.physics_runner.stop()
                self.physics_status = 'off'
            self.status = 'stopped'
            return str(error)
        for attachment in self.attachments:
            if attachment['bus'] == 'can':
                self._start_device_sidecar(attachment)
        if self.usb:
            usb_command = [
                sys.executable, str(HERE / 'usbip_attach.py'),
                '--port', str(self.args.usbip_port),
            ]
            self.log('$ ' + shlex.join(usb_command))
            self.usb_runner.start(usb_command, ROOT)
        threading.Thread(target=self._monitor_loop, args=(generation,),
                         daemon=True).start()
        return None

    def _allocate_node_id(self):
        if self.next_node_id < 1:
            raise ValueError('too many emulated CAN devices')
        node_id = self.next_node_id
        self.next_node_id -= 1
        return node_id

    def allocate_runtime_device(self, attachment):
        attachment['runtime_id'] = 'configHotDevice%u' % self.next_hotplug_id
        if attachment['bus'] == 'uart':
            attachment['runtime_address'] = (
                0x6FFF0000 + self.next_hotplug_id * 0x100)
        self.next_hotplug_id += 1
        if attachment['bus'] == 'can':
            attachment['node_id'] = self._allocate_node_id()

    def allocate_physics_channel(self, attachment, device):
        physics = device.get('physics')
        if physics is None:
            return
        used = {
            candidate.get('physics_index')
            for candidate in self.attachments
            if (candidate is not attachment and
                ATTACHABLE_DEVICES[candidate['device']].get(
                    'physics', {}).get('source') == physics['source'])
        }
        physics_index = next((candidate for candidate in range(physics['count'])
                              if candidate not in used), None)
        if physics_index is None:
            raise ValueError('no free %s physics channels' % physics['source'])
        attachment['physics_index'] = physics_index

    def _start_device_sidecar(self, attachment):
        runtime_id = attachment['runtime_id']
        if runtime_id in self.device_runners:
            return
        sidecar = attachment.get('sidecar') or attachment['device']
        command = [
            sys.executable, '-u', str(HERE / 'device_emulator.py'),
            sidecar,
            '--can-bus', str(attachment['can_bus_number']),
            '--node-id', str(attachment['node_id']),
        ]
        runner = ProcRunner('device:%s' % runtime_id, self.log_q)
        self.device_runners[runtime_id] = runner
        self.log('$ ' + shlex.join(command))
        runner.start(command, ROOT)

    def _stop_device_sidecar(self, attachment):
        runner = self.device_runners.pop(attachment.get('runtime_id'), None)
        if runner is not None:
            runner.stop()

    def queue_monitor_commands(self, commands, context, rollback_commands=()):
        if self.status != 'running':
            return False
        if rollback_commands and len(rollback_commands) != len(commands):
            raise ValueError('rollback commands must match command count')
        self.monitor_q.put((commands, context, rollback_commands))
        return True

    def _monitor_loop(self, generation):
        client = MonitorClient('127.0.0.1', self.args.monitor_port)
        deadline = time.monotonic() + 120
        history = []
        try:
            while (generation == self.generation and self.runner.running() and
                   time.monotonic() < deadline):
                try:
                    client.connect()
                    break
                except (OSError, TimeoutError):
                    client.close()
                    time.sleep(0.5)
            else:
                if generation == self.generation and self.runner.running():
                    self.log_q.put(('__monitor_error__', generation,
                                    'monitor did not become ready'))
                return

            self.log_q.put(('__monitor_ready__', generation))
            while generation == self.generation and self.runner.running():
                try:
                    while True:
                        try:
                            commands, context, rollback_commands = (
                                self.monitor_q.get_nowait())
                        except queue.Empty:
                            break
                        error, rollback_failed = execute_monitor_command_batch(
                            client, commands, rollback_commands)
                        if rollback_failed:
                            context = dict(context, rollback_failed=True)
                        self.log_q.put(
                            ('__hotplug_done__', generation, context, error))
                    timeout = 60 if not history else 5
                    current = parse_metrics(client.command(
                        METRICS_COMMAND, timeout=timeout))
                except (OSError, TimeoutError, ValueError) as error:
                    self.log_q.put(('__monitor_error__', generation, str(error)))
                    return
                current['wall_seconds'] = time.monotonic()
                history.append(current)
                # Renode advances in bursts and then sleeps when paced. A
                # several-second wall-clock window reports useful realtime
                # speed instead of alternating above and below 1x.
                while (len(history) > 2 and
                       current['wall_seconds'] - history[1]['wall_seconds'] >= 8):
                    history.pop(0)
                if len(history) > 1:
                    base = history[0]
                    wall_delta = (current['wall_seconds'] -
                                  base['wall_seconds'])
                    virtual_delta = (current['virtual_seconds'] -
                                     base['virtual_seconds'])
                    instruction_delta = (current['instructions'] -
                                         base['instructions'])
                    if wall_delta > 0 and virtual_delta >= 0:
                        current['speedup'] = virtual_delta / wall_delta
                        current['executed_mips'] = instruction_delta / wall_delta / 1e6
                self.log_q.put(('__metrics__', generation, current))
                time.sleep(1)
        finally:
            client.close()

    def stop(self):
        self.generation += 1
        for runner in list(self.device_runners.values()):
            runner.stop()
        self.device_runners.clear()
        # Detach USB while Renode still has its export socket alive.
        self.usb_runner.stop()
        self.physics_runner.stop()
        self.runner.stop()
        self.status = 'stopped'
        self.usb_status = 'off'
        self.metrics = None
        self.speedup = None
        self.executed_mips = None
        self.physics_status = 'off'
        while True:
            try:
                self.monitor_q.get_nowait()
            except queue.Empty:
                break
        for attachment in self.attachments:
            attachment['state'] = 'configured'
            for key in ('runtime_id', 'runtime_address', 'node_id',
                        'configure_bridge'):
                attachment.pop(key, None)
        self.active_can_ports.clear()

    def handle_event(self, item):
        if not isinstance(item, tuple):
            if item.startswith('[usb] attached'):
                self.usb_status = 'attached'
            elif item.startswith('[usb] waiting'):
                self.usb_status = 'waiting'
            return item
        tag = item[0]
        if tag == '__monitor_ready__' and item[1] == self.generation:
            self.status = 'running'
            for attachment in self.attachments:
                if attachment.get('state') == 'starting':
                    attachment['state'] = 'connected'
            self._connect_physics_if_ready()
        elif (tag == '__physics_ready__' and
              self.physics_runner.is_current(item[1])):
            self.physics_status = 'listening'
            self._connect_physics_if_ready()
        elif (tag == '__physics_start_error__' and
              self.physics_runner.is_current(item[1])):
            self.physics_runner.stop()
            self.physics_status = 'failed to start: %s' % item[2]
            return '[physics] %s' % item[2]
        elif tag == '__metrics__' and item[1] == self.generation:
            self.metrics = item[2]
            self.speedup = self.metrics.get('speedup')
            self.executed_mips = self.metrics.get('executed_mips')
        elif tag == '__monitor_error__' and item[1] == self.generation:
            self.status = 'monitor error: %s' % item[2]
            return '[monitor] %s' % item[2]
        elif tag == '__exit__':
            _tag, name, proc, returncode = item
            if name.startswith('device:'):
                runtime_id = name.split(':', 1)[1]
                runner = self.device_runners.get(runtime_id)
                if runner is not None and runner.is_current(proc):
                    self.device_runners.pop(runtime_id, None)
                    attachment = next((
                        candidate for candidate in self.attachments
                        if candidate.get('runtime_id') == runtime_id), None)
                    if attachment is not None:
                        attachment['state'] = 'sidecar exited (%s)' % returncode
                    return '[%s exited, status %s]' % (name, returncode)
                return None
            if name == 'physics':
                if self.physics_runner.is_current(proc):
                    self.physics_status = 'exited with status %s' % returncode
                    return '[physics exited, status %s]' % returncode
                return None
            runner = self.runner if name == 'renode' else self.usb_runner
            if runner.is_current(proc):
                if name == 'renode' and self.status != 'stopped':
                    self.status = 'exited with status %s' % returncode
                    self.usb_runner.stop()
                    for device_runner in list(self.device_runners.values()):
                        device_runner.stop()
                    self.device_runners.clear()
                    self.physics_runner.stop()
                    self.physics_status = 'off'
                elif name == 'usb' and self.usb:
                    self.usb_status = 'exited with status %s' % returncode
                return '[%s exited, status %s]' % (name, returncode)
        return None

    def status_snapshot(self):
        result = {
            'state': self.status,
            'board': self.board,
            'firmware': self.firmware,
            'bootloader': self.bootloader,
            'renode': self.args.renode,
            'cpu': self.cpu,
            'real_iomcu': self.real_iomcu,
            'iomcu_force_update': self.iomcu_force_update,
            'usb': self.usb_status,
            'can': (self.can or any(attachment.get('bus') == 'can'
                                    for attachment in self.attachments)),
            'can_base': self.can_base,
            'ethernet': self.ethernet or None,
            'devices': [dict(attachment) for attachment in self.attachments],
            'hotplug_error': self.hotplug_error,
            'physics': {
                'enabled': self.physics_enabled,
                'status': self.physics_status,
                'binary': (str(self.physics_binary)
                           if self.physics_binary is not None else None),
                'port': self.physics_port,
                'model': self.physics_model,
                'rate_hz': self.physics_rate,
                'location': {
                    'latitude_deg': self.physics_latitude,
                    'longitude_deg': self.physics_longitude,
                    'altitude_m': self.physics_altitude,
                    'heading_deg': self.physics_heading,
                },
            },
        }
        if self.metrics is not None:
            result.update({
                'pc': '0x%08X' % self.metrics['pc'],
                'mips': self.metrics['mips'],
                'executed_mips': self.executed_mips,
                'speedup': self.speedup,
                'virtual_seconds': self.metrics['virtual_seconds'],
            })
        return result


def run_control_server(launcher, port, on_command):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', port))
    server.listen(4)

    def client(connection):
        stream = connection.makefile('rw')
        try:
            for line in stream:
                stream.write(on_command(line.strip()) + '\n')
                stream.flush()
        except OSError:
            pass
        finally:
            connection.close()

    def loop():
        while True:
            connection, _address = server.accept()
            threading.Thread(target=client, args=(connection,),
                             daemon=True).start()

    threading.Thread(target=loop, daemon=True).start()


def parse_bool(value):
    if value.lower() in ('1', 'on', 'true', 'yes'):
        return True
    if value.lower() in ('0', 'off', 'false', 'no'):
        return False
    raise ValueError('expected on or off')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--renode', help='Renode executable passed to run.py')
    parser.add_argument('--renode-cache',
                        help='Renode and model data download cache '
                             '(default: ~/.cache/ardupilot/renode)')
    parser.add_argument('--state-dir', help='persistent state directory passed to run.py')
    parser.add_argument('--monitor-port', type=int, default=12390)
    parser.add_argument('--uart-port', type=int, default=5762)
    parser.add_argument('--usbip-port', type=int, default=3240)
    parser.add_argument('--physics-binary',
                        help='standalone renode-physics executable')
    parser.add_argument('--physics-port', type=int, default=9002)
    parser.add_argument('--control-port', type=int, default=0,
                        help='localhost TCP port for scripted UI control')
    args = parser.parse_args()

    from PySide6.QtCore import Qt
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication
    from PySide6.QtWidgets import QCheckBox
    from PySide6.QtWidgets import QComboBox
    from PySide6.QtWidgets import QDoubleSpinBox
    from PySide6.QtWidgets import QFileDialog
    from PySide6.QtWidgets import QGridLayout
    from PySide6.QtWidgets import QGroupBox
    from PySide6.QtWidgets import QHBoxLayout
    from PySide6.QtWidgets import QLabel
    from PySide6.QtWidgets import QLineEdit
    from PySide6.QtWidgets import QPlainTextEdit
    from PySide6.QtWidgets import QPushButton
    from PySide6.QtWidgets import QSpinBox
    from PySide6.QtWidgets import QTabWidget
    from PySide6.QtWidgets import QTreeWidget
    from PySide6.QtWidgets import QTreeWidgetItem
    from PySide6.QtWidgets import QVBoxLayout
    from PySide6.QtWidgets import QWidget

    sys.path.insert(0, str(HERE))
    import gen_board
    import renode_firmware

    renode_cache = (Path(args.renode_cache).expanduser()
                    if args.renode_cache else default_renode_cache())
    args.data_cache = str(renode_cache / 'data')
    firmware_cache = renode_cache / 'firmware'

    app = QApplication(sys.argv)
    launcher = Launcher(args)
    boards = gen_board.supported_boards(ROOT)

    window = QWidget()
    window.setWindowTitle('ArduPilot Renode launcher')
    outer = QVBoxLayout(window)

    tabs = QTabWidget()
    target_tab = QWidget()
    target_layout = QVBoxLayout(target_tab)

    selection = QGroupBox('Target')
    selection_grid = QGridLayout(selection)
    selection_grid.addWidget(QLabel('Board'), 0, 0)
    board_filter = QLineEdit()
    board_filter.setPlaceholderText('filter boards...')
    selection_grid.addWidget(board_filter, 0, 1)
    board_combo = QComboBox()
    selection_grid.addWidget(board_combo, 0, 2, 1, 2)
    board_info = QLabel('pick a board')
    selection_grid.addWidget(board_info, 1, 2, 1, 2)

    selection_grid.addWidget(QLabel('Firmware'), 2, 0)
    firmware_combo = QComboBox()
    selection_grid.addWidget(firmware_combo, 2, 1, 1, 2)
    firmware_browse = QPushButton('Browse...')
    selection_grid.addWidget(firmware_browse, 2, 3)

    selection_grid.addWidget(QLabel('Catalog'), 3, 0)
    firmware_vehicle = QComboBox()
    for label, vehicle in (
            ('Copter', 'Copter'), ('Plane', 'Plane'), ('Rover', 'Rover'),
            ('Sub', 'Sub'), ('Tracker', 'AntennaTracker'),
            ('Blimp', 'Blimp'), ('AP_Periph', 'AP_Periph')):
        firmware_vehicle.addItem(label, vehicle)
    selection_grid.addWidget(firmware_vehicle, 3, 1)
    firmware_channel = QComboBox()
    firmware_channel.addItem('Latest (development)', 'latest')
    firmware_channel.addItem('Stable', 'stable')
    selection_grid.addWidget(firmware_channel, 3, 2)
    download_firmware = QPushButton('Download Firmware')
    selection_grid.addWidget(download_firmware, 3, 3)

    selection_grid.addWidget(QLabel('Bootloader'), 4, 0)
    bootloader_combo = QComboBox()
    selection_grid.addWidget(bootloader_combo, 4, 1, 1, 2)
    bootloader_browse = QPushButton('Browse...')
    selection_grid.addWidget(bootloader_browse, 4, 3)

    selection_grid.addWidget(QLabel('Renode'), 5, 0)
    renode_path = QLineEdit(args.renode or 'not selected')
    renode_path.setReadOnly(True)
    renode_path.setToolTip('Managed downloads are stored in %s' % renode_cache)
    selection_grid.addWidget(renode_path, 5, 1, 1, 2)
    download_renode = QPushButton('Download Renode')
    selection_grid.addWidget(download_renode, 5, 3)
    target_layout.addWidget(selection)

    options = QGroupBox('Emulation options')
    options_grid = QGridLayout(options)
    options_grid.addWidget(QLabel('Pin MCU thread'), 0, 0)
    cpu_combo = QComboBox()
    cpu_combo.addItem('No CPU pinning', None)
    for cpu in sorted(os.sched_getaffinity(0)):
        cpu_combo.addItem('CPU %u' % cpu, cpu)
    options_grid.addWidget(cpu_combo, 0, 1)

    real_iomcu = QCheckBox('Real IOMCU')
    iomcu_update = QCheckBox('Force IOMCU update')
    iomcu_update.setEnabled(False)
    options_grid.addWidget(real_iomcu, 0, 2)
    options_grid.addWidget(iomcu_update, 0, 3)

    usb = QCheckBox('USB')
    usb.setToolTip(
        'The launcher starts usbip_attach.py without sudo. One-time setup:\n'
        'sudo Tools/renode/usbip_attach.py --install-rules')
    options_grid.addWidget(usb, 1, 0)

    can = QCheckBox('CAN')
    can.setToolTip(
        'Bridge board CAN ports to multicast buses. Attaching a CAN device '
        'in Config enables this automatically.')
    options_grid.addWidget(can, 1, 1)
    can_base = QSpinBox()
    can_base.setRange(0, 9)
    can_base.setPrefix('bus base ')
    can_base.setEnabled(False)
    options_grid.addWidget(can_base, 1, 2)

    ethernet_enable = QCheckBox('Ethernet TAP')
    options_grid.addWidget(ethernet_enable, 2, 0)
    ethernet = QComboBox()
    ethernet.setEditable(True)
    ethernet.addItems(sorted(path.name for path in Path('/sys/class/net').iterdir()))
    ethernet.setEnabled(False)
    options_grid.addWidget(ethernet, 2, 1, 1, 2)
    target_layout.addWidget(options)
    target_layout.addStretch()
    tabs.addTab(target_tab, 'Target')

    config_tab = QWidget()
    config_layout = QVBoxLayout(config_tab)
    config_help = QLabel(
        'Attach emulated devices to ports from the selected board hwdef. '
        'This tab remains live while Renode runs: attach or remove devices '
        'to reproduce runtime probing. I2C and CAN buses support multiple '
        'devices. Firmware parameters must still enable the matching driver '
        'and port.')
    config_help.setWordWrap(True)
    config_layout.addWidget(config_help)
    config_status = QLabel('Pick a target board to load its ports.')
    config_layout.addWidget(config_status)
    port_tree = QTreeWidget()
    port_tree.setColumnCount(3)
    port_tree.setHeaderLabels(('Port / device', 'Bus', 'State'))
    port_tree.setRootIsDecorated(True)
    port_tree.setAlternatingRowColors(True)
    config_layout.addWidget(port_tree)
    attach_controls = QHBoxLayout()
    device_combo = QComboBox()
    device_combo.setEditable(True)
    device_combo.setInsertPolicy(QComboBox.NoInsert)
    device_combo.completer().setCaseSensitivity(Qt.CaseInsensitive)
    device_combo.completer().setFilterMode(Qt.MatchContains)
    device_combo.lineEdit().setPlaceholderText('search devices...')
    device_combo.setMinimumContentsLength(30)
    add_device = QPushButton('Attach')
    remove_device = QPushButton('Remove')
    add_device.setEnabled(False)
    remove_device.setEnabled(False)
    attach_controls.addWidget(QLabel('Device'))
    attach_controls.addWidget(device_combo, 1)
    attach_controls.addWidget(add_device)
    attach_controls.addWidget(remove_device)
    config_layout.addLayout(attach_controls)
    tabs.addTab(config_tab, 'Config')

    physics_tab = QWidget()
    physics_layout = QVBoxLayout(physics_tab)
    physics_help = QLabel(
        'Run an ArduPilot SITL vehicle model as a standalone lockstep '
        'sidecar. Physics can be connected and disconnected while Renode is '
        'running; reconnecting preserves the physical vehicle across a '
        'flight-controller reset. Stop the sidecar to change model or '
        'location.')
    physics_help.setWordWrap(True)
    physics_layout.addWidget(physics_help)
    physics_group = QGroupBox('Vehicle physics')
    physics_grid = QGridLayout(physics_group)
    physics_autostart = QCheckBox('Start with Renode')
    physics_grid.addWidget(physics_autostart, 0, 0, 1, 2)
    physics_grid.addWidget(QLabel('Sidecar'), 1, 0)
    physics_binary = QLineEdit(
        str(launcher.physics_binary) if launcher.physics_binary else 'not found')
    physics_binary.setReadOnly(True)
    physics_grid.addWidget(physics_binary, 1, 1, 1, 2)
    physics_browse = QPushButton('Browse...')
    physics_grid.addWidget(physics_browse, 1, 3)
    physics_grid.addWidget(QLabel('Model'), 2, 0)
    physics_model = QComboBox()
    for label, model in (
            ('Quadcopter (quad)', 'quad'),
            ('Fixed-wing plane (plane)', 'plane'),
            ('Rover (rover)', 'rover'),
            ('VTOL plane (quadplane)', 'quadplane')):
        physics_model.addItem(label, model)
    physics_grid.addWidget(physics_model, 2, 1)
    physics_rate = QSpinBox()
    physics_rate.setRange(1, 1200)
    physics_rate.setValue(launcher.physics_rate)
    physics_rate.setSuffix(' Hz')
    physics_grid.addWidget(QLabel('Exchange rate'), 2, 2)
    physics_grid.addWidget(physics_rate, 2, 3)

    def coordinate_spin(minimum, maximum, value, decimals):
        widget = QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setDecimals(decimals)
        widget.setValue(value)
        return widget

    physics_latitude = coordinate_spin(
        -90, 90, launcher.physics_latitude, 7)
    physics_longitude = coordinate_spin(
        -180, 180, launcher.physics_longitude, 7)
    physics_altitude = coordinate_spin(
        -10000, 83000, launcher.physics_altitude, 2)
    physics_altitude.setSuffix(' m AMSL')
    physics_heading = coordinate_spin(
        -360, 360, launcher.physics_heading, 2)
    physics_heading.setSuffix(' deg')
    physics_grid.addWidget(QLabel('Latitude'), 3, 0)
    physics_grid.addWidget(physics_latitude, 3, 1)
    physics_grid.addWidget(QLabel('Longitude'), 3, 2)
    physics_grid.addWidget(physics_longitude, 3, 3)
    physics_grid.addWidget(QLabel('Altitude'), 4, 0)
    physics_grid.addWidget(physics_altitude, 4, 1)
    physics_grid.addWidget(QLabel('Heading'), 4, 2)
    physics_grid.addWidget(physics_heading, 4, 3)
    physics_status = QLabel('off')
    physics_status.setTextInteractionFlags(Qt.TextSelectableByMouse)
    physics_grid.addWidget(QLabel('Status'), 5, 0)
    physics_grid.addWidget(physics_status, 5, 1, 1, 3)
    physics_buttons = QHBoxLayout()
    physics_start = QPushButton('Start Physics')
    physics_stop = QPushButton('Stop Physics')
    physics_stop.setEnabled(False)
    physics_buttons.addStretch()
    physics_buttons.addWidget(physics_start)
    physics_buttons.addWidget(physics_stop)
    physics_grid.addLayout(physics_buttons, 6, 0, 1, 4)
    physics_layout.addWidget(physics_group)
    physics_layout.addStretch()
    tabs.addTab(physics_tab, 'Physics')
    outer.addWidget(tabs)

    status_box = QGroupBox('Runtime')
    status_layout = QGridLayout(status_box)
    status_label = QLabel('stopped')
    status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
    status_layout.addWidget(status_label, 0, 0, 1, 4)
    metrics_label = QLabel('Speed -- | PC -- | MIPS --')
    metrics_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
    status_layout.addWidget(metrics_label, 1, 0, 1, 4)
    command_preview = QLineEdit()
    command_preview.setReadOnly(True)
    status_layout.addWidget(command_preview, 2, 0, 1, 4)
    start_button = QPushButton('Start')
    stop_button = QPushButton('Stop')
    quit_button = QPushButton('Quit')
    stop_button.setEnabled(False)
    buttons = QHBoxLayout()
    buttons.addStretch()
    buttons.addWidget(start_button)
    buttons.addWidget(stop_button)
    buttons.addWidget(quit_button)
    status_layout.addLayout(buttons, 3, 0, 1, 4)
    outer.addWidget(status_box)

    log_view = QPlainTextEdit()
    log_view.setReadOnly(True)
    log_view.setMaximumBlockCount(3000)
    log_view.setMinimumSize(760, 300)
    outer.addWidget(log_view)

    all_boards = sorted(boards)
    config_ports = []
    config_generation = 0
    config_compile_lock = threading.Lock()

    def selected_port():
        item = port_tree.currentItem()
        if item is None:
            return None
        port = item.data(0, Qt.UserRole)
        if port is None and item.parent() is not None:
            port = item.parent().data(0, Qt.UserRole)
        return port

    def tree_attachment(item=None):
        item = item or port_tree.currentItem()
        if item is None:
            return None
        index = item.data(0, Qt.UserRole + 1)
        if index is None or not 0 <= index < len(launcher.attachments):
            return None
        return launcher.attachments[index]

    def attachment_name(attachment):
        device = gen_board.ATTACHABLE_DEVICES[attachment['device']]
        return '%s — %s' % (device['category'], device['name'])

    def attachment_bus_detail(attachment):
        device = gen_board.ATTACHABLE_DEVICES[attachment['device']]
        if attachment['bus'] == 'i2c':
            addresses = ', '.join(
                '0x%02X' % endpoint['address']
                for endpoint in i2c_endpoints(device))
            label = 'address' if len(i2c_endpoints(device)) == 1 else 'addresses'
            return '%s %s' % (label, addresses)
        if attachment['bus'] == 'can':
            node_id = attachment.get('node_id')
            return 'node %u' % node_id if node_id is not None else 'auto node ID'
        return 'point-to-point'

    def populate_config_tree():
        port_tree.clear()
        for port in config_ports:
            attachments = [
                (index, attachment)
                for index, attachment in enumerate(launcher.attachments)
                if attachment['port'] == port['id']
            ]
            port_item = QTreeWidgetItem((
                port['name'], port['bus'].upper(),
                '%u attached' % len(attachments) if attachments else ''))
            port_item.setData(0, Qt.UserRole, port)
            port_tree.addTopLevelItem(port_item)
            for index, attachment in attachments:
                child = QTreeWidgetItem((
                    attachment_name(attachment),
                    attachment_bus_detail(attachment),
                    attachment.get('state', 'configured')))
                # Qt converts Python dictionaries through QVariant, yielding a
                # copy when read back. Keep an index so live state changes and
                # duplicate CAN nodes always address the original attachment.
                child.setData(0, Qt.UserRole + 1, index)
                port_item.addChild(child)
            port_item.setExpanded(True)
        for column in range(3):
            port_tree.resizeColumnToContents(column)

    def update_attachment_buttons():
        port = selected_port()
        selected = device_combo.currentData()
        if (device_combo.currentIndex() < 0 or
                device_combo.currentText() != device_combo.itemText(
                    device_combo.currentIndex())):
            selected = None
        occupied = [
            attachment for attachment in launcher.attachments
            if port is not None and attachment['port'] == port['id']
        ]
        can_add = (launcher.status in ('stopped', 'running') and
                   port is not None and selected is not None)
        if can_add and port['bus'] == 'uart' and occupied:
            can_add = False
        if can_add and port['bus'] == 'i2c':
            addresses = {
                endpoint['address'] for endpoint in i2c_endpoints(
                    gen_board.ATTACHABLE_DEVICES[selected])
            }
            occupied_addresses = {
                endpoint['address']
                for item in occupied
                for endpoint in i2c_endpoints(
                    gen_board.ATTACHABLE_DEVICES[item['device']])
            }
            can_add = addresses.isdisjoint(occupied_addresses)
        add_device.setEnabled(can_add)
        item = port_tree.currentItem()
        attachment = tree_attachment(item)
        remove_device.setEnabled(
            launcher.status in ('stopped', 'running') and item is not None and
            attachment is not None and attachment.get('state') not in (
                'attaching', 'detaching'))

    def refresh_device_choices():
        port = selected_port()
        current = device_combo.currentData()
        device_combo.blockSignals(True)
        device_combo.clear()
        if port is not None:
            choices = [
                (device_id, device) for device_id, device in
                gen_board.ATTACHABLE_DEVICES.items()
                if device['bus'] == port['bus']
            ]
            for device_id, device in sorted(
                    choices, key=lambda choice: (
                        choice[1]['category'], choice[1]['name'])):
                device_combo.addItem(
                    '%s — %s' % (device['category'], device['name']),
                    device_id)
        index = device_combo.findData(current)
        if index >= 0:
            device_combo.setCurrentIndex(index)
        device_combo.blockSignals(False)
        update_attachment_buttons()

    def attach_device():
        port = selected_port()
        device_id = device_combo.currentData()
        if port is None or device_id is None:
            return
        device = gen_board.ATTACHABLE_DEVICES[device_id]
        attachment = {
            'port': port['id'],
            'device': device_id,
            'bus': port['bus'],
            'port_index': port['index'],
        }
        if port['bus'] == 'can':
            attachment['can_bus_number'] = launcher.can_base + port['index']
            attachment['sidecar'] = device['sidecar']
        try:
            launcher.allocate_physics_channel(attachment, device)
        except ValueError as error:
            launcher.log('[attach] %s' % error)
            return
        launcher.attachments.append(attachment)
        if port['bus'] == 'can':
            can.setChecked(True)
        attachment = launcher.attachments[-1]
        if launcher.status == 'running':
            try:
                launcher.allocate_runtime_device(attachment)
            except ValueError as error:
                launcher.attachments.remove(attachment)
                launcher.log('[hotplug] %s' % error)
                return
            attachment['state'] = 'attaching'
            if port['bus'] == 'can':
                attachment['configure_bridge'] = (
                    port['id'] not in launcher.active_can_ports)
            commands = runtime_device_commands(
                attachment, port,
                gen_board.ATTACHABLE_DEVICES[device_id], True)
            rollback_commands = ()
            if (port['bus'] == 'i2c' and
                    len(i2c_endpoints(device)) > 1):
                rollback_commands = runtime_device_commands(
                    attachment, port, device, False)
            if not launcher.queue_monitor_commands(
                    commands, {'action': 'attach',
                               'attachment': attachment}, rollback_commands):
                launcher.attachments.remove(attachment)
                return
        populate_config_tree()
        for row in range(port_tree.topLevelItemCount()):
            item = port_tree.topLevelItem(row)
            if item.data(0, Qt.UserRole)['id'] == port['id']:
                port_tree.setCurrentItem(item)
                break
        refresh_device_choices()
        update_preview()

    def remove_selected_device():
        item = port_tree.currentItem()
        if item is None:
            return
        attachment = tree_attachment(item)
        if attachment is None:
            return
        if launcher.status == 'running':
            if attachment['bus'] == 'can':
                launcher._stop_device_sidecar(attachment)
                launcher.attachments.remove(attachment)
            else:
                port = next(port for port in config_ports
                            if port['id'] == attachment['port'])
                attachment['state'] = 'detaching'
                commands = runtime_device_commands(
                    attachment, port,
                    gen_board.ATTACHABLE_DEVICES[attachment['device']], False)
                device = gen_board.ATTACHABLE_DEVICES[attachment['device']]
                rollback_commands = ()
                if (port['bus'] == 'i2c' and
                        len(i2c_endpoints(device)) > 1):
                    rollback_commands = runtime_device_commands(
                        attachment, port, device, True)
                if not launcher.queue_monitor_commands(
                        commands, {'action': 'detach',
                                   'attachment': attachment}, rollback_commands):
                    attachment['state'] = 'connected'
        else:
            launcher.attachments.remove(attachment)
        populate_config_tree()
        refresh_device_choices()
        update_preview()

    def refresh_config_ports(board):
        nonlocal config_generation
        config_generation += 1
        generation = config_generation
        config_ports.clear()
        launcher.attachments.clear()
        populate_config_tree()
        config_status.setText('Loading ports for %s from hwdef.dat...' % board)
        device_combo.clear()
        add_device.setEnabled(False)

        def worker():
            if generation != config_generation:
                return
            try:
                with config_compile_lock:
                    if generation != config_generation:
                        return
                    with tempfile.TemporaryDirectory(
                            prefix='renode-hwdef-') as path:
                        ports = gen_board.configuration_ports(
                            ROOT, board, Path(path) / 'hwdef')
                launcher.log_q.put(
                    ('__config_ports__', generation, board, ports))
            except (OSError, ValueError) as error:
                launcher.log_q.put(
                    ('__config_error__', generation, board, str(error)))

        threading.Thread(target=worker, daemon=True).start()

    port_tree.currentItemChanged.connect(
        lambda _current, _previous: refresh_device_choices())
    device_combo.currentIndexChanged.connect(
        lambda _index: update_attachment_buttons())
    device_combo.editTextChanged.connect(
        lambda _text: update_attachment_buttons())
    add_device.clicked.connect(attach_device)
    remove_device.clicked.connect(remove_selected_device)

    def update_preview():
        try:
            command_preview.setText(shlex.join(launcher.build_command()))
        except ValueError as error:
            command_preview.setText(str(error))

    def refresh_firmware():
        firmware_combo.blockSignals(True)
        firmware_combo.clear()
        default_name = ('AP_Periph' if launcher.board and
                        gen_board.is_periph_board(ROOT, launcher.board)
                        else 'arducopter')
        firmware_combo.addItem('Auto (build/%s/bin/%s)' %
                               (launcher.board, default_name), 'auto')
        for path in firmware_choices(launcher.board):
            firmware_combo.addItem(path.name, str(path))
        firmware_combo.blockSignals(False)
        firmware_combo.setCurrentIndex(0)
        launcher.firmware = 'auto'

    def refresh_bootloader():
        bootloader_combo.blockSignals(True)
        bootloader_combo.clear()
        path = default_bootloader(launcher.board)
        label = ('Auto (%s)' % path.name if path else 'Auto (none found)')
        bootloader_combo.addItem(label, 'auto')
        bootloader_combo.addItem('None', 'none')
        bootloader_combo.blockSignals(False)
        bootloader_combo.setCurrentIndex(0)
        launcher.bootloader = 'auto'

    def board_changed():
        board = board_combo.currentText()
        if not board:
            launcher.board = None
            launcher.attachments.clear()
            config_ports.clear()
            populate_config_tree()
            config_status.setText('No matching board.')
            board_info.setText('no matching board')
            update_preview()
            return
        changed = launcher.board != board
        launcher.board = board
        board_info.setText('%s | %s' % (
            boards[board],
            '%u built firmware image(s)' % len(firmware_choices(board))))
        refresh_firmware()
        refresh_bootloader()
        if gen_board.is_periph_board(ROOT, board):
            index = firmware_vehicle.findData('AP_Periph')
            firmware_vehicle.setCurrentIndex(index)
        elif firmware_vehicle.currentData() == 'AP_Periph':
            firmware_vehicle.setCurrentIndex(0)
        if changed:
            refresh_config_ports(board)
        update_preview()

    def apply_filter():
        pattern = board_filter.text().strip().upper()
        current = launcher.board
        board_combo.blockSignals(True)
        board_combo.clear()
        board_combo.addItems([board for board in all_boards
                              if pattern in board.upper()])
        board_combo.blockSignals(False)
        index = board_combo.findText(current) if current else -1
        board_combo.setCurrentIndex(index if index >= 0 else 0)
        board_changed()

    def browse_firmware():
        directory = ROOT / 'build' / (launcher.board or '') / 'bin'
        path, _selected = QFileDialog.getOpenFileName(
            window, 'Firmware ELF', str(directory), 'ELF or executable (*)')
        if path:
            firmware_combo.insertItem(0, Path(path).name, path)
            firmware_combo.setCurrentIndex(0)

    def browse_bootloader():
        path, _selected = QFileDialog.getOpenFileName(
            window, 'Bootloader', str(ROOT / 'Tools' / 'bootloaders'),
            'Bootloader (*.bin *.elf *.hex);;All files (*)')
        if path:
            bootloader_combo.insertItem(0, Path(path).name, path)
            bootloader_combo.setCurrentIndex(0)

    download_active = False

    firmware_download_active = False

    def start_firmware_download():
        nonlocal firmware_download_active
        if firmware_download_active or not launcher.board:
            return False
        firmware_download_active = True
        board = launcher.board
        vehicle = firmware_vehicle.currentData()
        channel = firmware_channel.currentData()
        download_firmware.setEnabled(False)
        download_firmware.setText('Checking...')

        def progress(received, total):
            launcher.log_q.put(('__firmware_download_progress__',
                                board, received, total))

        def worker():
            try:
                manifest = renode_firmware.fetch_manifest()
                selected = renode_firmware.select_firmware(
                    manifest, board, vehicle, channel)
                path, downloaded = renode_firmware.install_firmware(
                    firmware_cache, selected, progress=progress)
                launcher.log_q.put(('__firmware_download_done__', board,
                                    path, selected, downloaded))
            except Exception as error:
                launcher.log_q.put(('__firmware_download_error__',
                                    board, str(error)))

        threading.Thread(target=worker, daemon=True).start()
        return True

    download_firmware.clicked.connect(start_firmware_download)

    def renode_version(latest):
        return '%s (%s)' % (latest.get('renode_version', '?'),
                            latest['source']['revision'][:9])

    def check_renode_download():
        try:
            latest = fetch_renode_latest()
            package = select_renode_package(latest)
            current = cached_renode(renode_cache, latest, package)
            launcher.log_q.put(('__renode_check__', current, latest))
        except (OSError, RuntimeError, ValueError,
                json.JSONDecodeError) as error:
            launcher.log_q.put(('__renode_download_error__',
                                'update check failed: %s' % error))

    def start_renode_download():
        nonlocal download_active
        if download_active:
            return False
        download_active = True
        download_renode.setEnabled(False)
        download_renode.setText('Checking...')

        def progress(received, total):
            launcher.log_q.put(('__renode_download_progress__', received, total))

        def worker():
            try:
                executable, latest, downloaded = install_current_renode(
                    renode_cache, progress=progress)
                launcher.log_q.put(('__renode_download_done__',
                                    executable, latest, downloaded))
            except (OSError, RuntimeError, ValueError, tarfile.TarError,
                    zipfile.BadZipFile, json.JSONDecodeError) as error:
                launcher.log_q.put(('__renode_download_error__', str(error)))

        threading.Thread(target=worker, daemon=True).start()
        return True

    download_renode.clicked.connect(start_renode_download)

    def sync_physics():
        launcher.physics_enabled = physics_autostart.isChecked()
        launcher.physics_model = physics_model.currentData()
        launcher.physics_rate = physics_rate.value()
        launcher.physics_latitude = physics_latitude.value()
        launcher.physics_longitude = physics_longitude.value()
        launcher.physics_altitude = physics_altitude.value()
        launcher.physics_heading = physics_heading.value()

    def refresh_physics_controls():
        running = launcher.physics_runner.running()
        physics_status.setText(launcher.physics_status)
        physics_start.setEnabled(not running and launcher.physics_binary is not None)
        physics_stop.setEnabled(running and launcher.physics_status != 'disconnecting')
        for widget in (physics_binary, physics_browse, physics_model,
                       physics_rate, physics_latitude, physics_longitude,
                       physics_altitude, physics_heading):
            widget.setEnabled(not running)

    def browse_physics_binary():
        path, _selected = QFileDialog.getOpenFileName(
            window, 'renode-physics executable', str(HERE),
            'Executable (*)')
        if path:
            launcher.physics_binary = Path(path)
            physics_binary.setText(path)
            refresh_physics_controls()

    def start_physics_now():
        sync_physics()
        error = launcher.start_physics()
        if error:
            launcher.log('[physics] %s' % error)
        refresh_physics_controls()

    def stop_physics_now():
        launcher.stop_physics()
        refresh_physics_controls()

    physics_browse.clicked.connect(browse_physics_binary)
    physics_start.clicked.connect(start_physics_now)
    physics_stop.clicked.connect(stop_physics_now)
    physics_autostart.toggled.connect(lambda _checked: sync_physics())
    physics_model.currentIndexChanged.connect(lambda _index: sync_physics())
    physics_rate.valueChanged.connect(lambda _value: sync_physics())
    physics_latitude.valueChanged.connect(lambda _value: sync_physics())
    physics_longitude.valueChanged.connect(lambda _value: sync_physics())
    physics_altitude.valueChanged.connect(lambda _value: sync_physics())
    physics_heading.valueChanged.connect(lambda _value: sync_physics())

    def sync_options():
        sync_physics()
        launcher.firmware = firmware_combo.currentData() or 'auto'
        launcher.bootloader = bootloader_combo.currentData() or 'auto'
        launcher.cpu = cpu_combo.currentData()
        launcher.real_iomcu = real_iomcu.isChecked()
        launcher.iomcu_force_update = iomcu_update.isChecked()
        launcher.usb = usb.isChecked()
        launcher.can = can.isChecked()
        launcher.can_base = can_base.value()
        port_by_id = {port['id']: port for port in config_ports}
        for attachment in launcher.attachments:
            port = port_by_id.get(attachment['port'])
            if port is not None and port['bus'] == 'can':
                attachment['can_bus_number'] = (
                    launcher.can_base + port['index'])
        launcher.ethernet = (ethernet.currentText().strip()
                             if ethernet_enable.isChecked() else '')
        update_preview()

    board_filter.textChanged.connect(apply_filter)
    board_combo.currentIndexChanged.connect(lambda _index: board_changed())
    firmware_combo.currentIndexChanged.connect(lambda _index: sync_options())
    bootloader_combo.currentIndexChanged.connect(lambda _index: sync_options())
    firmware_browse.clicked.connect(browse_firmware)
    bootloader_browse.clicked.connect(browse_bootloader)
    cpu_combo.currentIndexChanged.connect(lambda _index: sync_options())

    def real_iomcu_changed(checked):
        iomcu_update.setEnabled(checked)
        if not checked:
            iomcu_update.setChecked(False)
        sync_options()

    real_iomcu.toggled.connect(real_iomcu_changed)
    iomcu_update.toggled.connect(lambda _checked: sync_options())
    usb.toggled.connect(lambda _checked: sync_options())
    can.toggled.connect(can_base.setEnabled)
    can.toggled.connect(lambda _checked: sync_options())
    can_base.valueChanged.connect(lambda _value: sync_options())
    ethernet_enable.toggled.connect(ethernet.setEnabled)
    ethernet_enable.toggled.connect(lambda _checked: sync_options())
    ethernet.currentTextChanged.connect(lambda _text: sync_options())

    def do_start():
        sync_options()
        error = launcher.start()
        if error:
            launcher.status = error
            status_label.setText(error)
            return
        start_button.setEnabled(False)
        stop_button.setEnabled(True)
        target_tab.setEnabled(False)
        refresh_device_choices()
        refresh_physics_controls()

    def do_stop():
        launcher.stop()
        start_button.setEnabled(True)
        stop_button.setEnabled(False)
        target_tab.setEnabled(True)
        populate_config_tree()
        refresh_device_choices()
        refresh_physics_controls()

    start_button.clicked.connect(do_start)
    stop_button.clicked.connect(do_stop)
    quit_button.clicked.connect(app.quit)

    def update_status():
        status = launcher.status
        if launcher.usb:
            status += ' | USB %s' % launcher.usb_status
        if launcher.physics_status != 'off':
            status += ' | Physics %s' % launcher.physics_status
        status_label.setText(status)
        physics_status.setText(launcher.physics_status)
        if launcher.metrics is None:
            metrics_label.setText('Speed -- | PC -- | MIPS --')
            return
        speed = ('%.2fx realtime' % launcher.speedup
                 if launcher.speedup is not None else 'measuring...')
        executed = ('%.1f executed' % launcher.executed_mips
                    if launcher.executed_mips is not None else '-- executed')
        metrics_label.setText(
            'Speed %s | PC 0x%08X | MIPS %u configured / %s' %
            (speed, launcher.metrics['pc'], launcher.metrics['mips'], executed))

    def drain_log():
        nonlocal download_active, firmware_download_active
        lines = []
        while True:
            try:
                item = launcher.log_q.get_nowait()
            except queue.Empty:
                break
            if isinstance(item, tuple) and item[0] == '__config_ports__':
                _tag, generation, board, ports = item
                if generation == config_generation and board == launcher.board:
                    config_ports[:] = ports
                    launcher.can_ports = [
                        port['id'] for port in ports if port['bus'] == 'can'
                    ]
                    can_base.setMaximum(max(0, 10 - len(launcher.can_ports)))
                    config_status.setText(
                        '%u configurable ports from %s hwdef.dat' %
                        (len(ports), board))
                    populate_config_tree()
                    refresh_device_choices()
                continue
            if isinstance(item, tuple) and item[0] == '__config_error__':
                _tag, generation, board, error = item
                if generation == config_generation and board == launcher.board:
                    config_status.setText('Failed to load ports: %s' % error)
                continue
            if isinstance(item, tuple) and item[0] == '__hotplug_done__':
                _tag, generation, context, error = item
                if generation != launcher.generation:
                    continue
                action = context['action']
                if action.startswith('physics_'):
                    launcher.complete_physics_command(action, error)
                    if error is None:
                        lines.append('[physics] %s' % (
                            'connected' if action == 'physics_connect'
                            else 'disconnected'))
                    else:
                        lines.append('[physics] %s failed: %s' %
                                     (action.split('_', 1)[1], error))
                    refresh_physics_controls()
                    continue
                attachment = context['attachment']
                if error is not None:
                    launcher.hotplug_error = str(error)
                    if context.get('rollback_failed'):
                        attachment['state'] = 'error'
                    elif action == 'attach':
                        if attachment in launcher.attachments:
                            launcher.attachments.remove(attachment)
                    else:
                        attachment['state'] = 'connected'
                    lines.append('[hotplug] failed to %s %s: %s' % (
                        action, attachment_name(attachment), error))
                elif action == 'attach':
                    launcher.hotplug_error = None
                    attachment['state'] = 'connected'
                    if attachment['bus'] == 'can':
                        launcher.active_can_ports.add(attachment['port'])
                        launcher._start_device_sidecar(attachment)
                    lines.append('[hotplug] attached %s to %s' % (
                        attachment_name(attachment), attachment['port']))
                else:
                    launcher.hotplug_error = None
                    if attachment in launcher.attachments:
                        launcher.attachments.remove(attachment)
                    lines.append('[hotplug] detached %s from %s' % (
                        attachment_name(attachment), attachment['port']))
                populate_config_tree()
                refresh_device_choices()
                update_preview()
                continue
            if isinstance(item, tuple) and item[0].startswith('__renode_download'):
                tag = item[0]
                if tag == '__renode_download_progress__':
                    _tag, received, total = item
                    percent = min(100, received * 100 // max(total, 1))
                    download_renode.setText('Downloading %u%%' % percent)
                elif tag == '__renode_download_done__':
                    _tag, executable, latest, downloaded = item
                    download_active = False
                    args.renode = str(executable)
                    renode_path.setText(args.renode)
                    download_renode.setText('Renode current')
                    download_renode.setEnabled(True)
                    update_preview()
                    action = 'downloaded' if downloaded else 'using cached'
                    lines.append('[renode] %s %s: %s' %
                                 (action, renode_version(latest), executable))
                elif tag == '__renode_download_error__':
                    download_active = False
                    download_renode.setText('Retry Renode download')
                    download_renode.setEnabled(True)
                    lines.append('[renode] %s' % item[1])
                continue
            if (isinstance(item, tuple) and
                    item[0].startswith('__firmware_download')):
                tag = item[0]
                event_board = item[1]
                if tag == '__firmware_download_progress__':
                    _tag, _board, received, total = item
                    if event_board == launcher.board:
                        if total:
                            label = 'Downloading %u%%' % min(
                                100, received * 100 // total)
                        else:
                            label = 'Downloading %u KiB' % (received // 1024)
                        download_firmware.setText(label)
                elif tag == '__firmware_download_done__':
                    _tag, _board, path, selected, downloaded = item
                    firmware_download_active = False
                    download_firmware.setText('Download Firmware')
                    download_firmware.setEnabled(True)
                    if event_board == launcher.board:
                        label = '%s %s (%s)' % (
                            selected.get('mav-firmware-version-str', ''),
                            selected['vehicletype'],
                            'downloaded' if downloaded else 'cached')
                        existing = firmware_combo.findData(str(path))
                        if existing >= 0:
                            firmware_combo.removeItem(existing)
                        firmware_combo.insertItem(0, label, str(path))
                        firmware_combo.setCurrentIndex(0)
                        sync_options()
                    lines.append('[firmware] %s: %s' %
                                 ('downloaded' if downloaded else 'using cached',
                                  path))
                elif tag == '__firmware_download_error__':
                    firmware_download_active = False
                    download_firmware.setText('Retry Firmware')
                    download_firmware.setEnabled(True)
                    lines.append('[firmware] %s' % item[2])
                continue
            if isinstance(item, tuple) and item[0] == '__renode_check__':
                _tag, current, latest = item
                if current is None:
                    label = ('Update Renode' if cached_renode(renode_cache)
                             else 'Download Renode')
                    download_renode.setText(label)
                    download_renode.setToolTip(
                        'Current server version: %s' % renode_version(latest))
                else:
                    if args.renode is None:
                        args.renode = str(current)
                        renode_path.setText(args.renode)
                        update_preview()
                    selected = (args.renode and
                                Path(args.renode).expanduser() == current)
                    download_renode.setText(
                        'Renode current' if selected else 'Use cached current')
                    download_renode.setToolTip(
                        'Cached server version: %s' % renode_version(latest))
                continue
            line = launcher.handle_event(item)
            if line:
                lines.append(line)
            if (isinstance(item, tuple) and
                    item[0] in ('__monitor_ready__', '__physics_ready__',
                                '__physics_start_error__', '__exit__')):
                populate_config_tree()
                refresh_device_choices()
                refresh_physics_controls()
        if lines:
            log_view.appendPlainText('\n'.join(lines))
        if not launcher.runner.running() and stop_button.isEnabled():
            start_button.setEnabled(True)
            stop_button.setEnabled(False)
            target_tab.setEnabled(True)
            populate_config_tree()
            refresh_device_choices()
            refresh_physics_controls()
        update_status()

    timer = QTimer()
    timer.timeout.connect(drain_log)
    timer.start(150)

    pending = queue.Queue()

    def on_command(line):
        done = queue.Queue()
        pending.put((line, done))
        try:
            return done.get(timeout=30)
        except queue.Empty:
            return 'ERR timeout'

    def set_checkbox(widget, value):
        try:
            widget.setChecked(parse_bool(value))
        except ValueError as error:
            return 'ERR %s' % error
        return 'OK'

    def handle_command(line):
        parts = line.split(None, 1)
        if not parts:
            return 'ERR empty'
        command = parts[0].lower()
        value = parts[1].strip() if len(parts) > 1 else ''
        if command == 'board':
            index = board_combo.findText(value)
            if index < 0:
                board_filter.setText(value)
                index = board_combo.findText(value)
            if index < 0:
                return 'ERR no board %s' % value
            board_combo.setCurrentIndex(index)
            return 'OK'
        if command == 'firmware':
            if value in ('', 'auto'):
                firmware_combo.setCurrentIndex(firmware_combo.findData('auto'))
            else:
                firmware_combo.insertItem(0, Path(value).name, value)
                firmware_combo.setCurrentIndex(0)
            return 'OK'
        if command == 'bootloader':
            choice = value or 'auto'
            index = bootloader_combo.findData(choice)
            if index < 0:
                bootloader_combo.insertItem(0, Path(choice).name, choice)
                index = 0
            bootloader_combo.setCurrentIndex(index)
            return 'OK'
        if command == 'download-firmware':
            if launcher.runner.running():
                return 'ERR stop Renode before changing firmware'
            fields = value.split()
            if len(fields) != 2:
                return 'ERR download-firmware needs VEHICLE CHANNEL'
            vehicle, channel = fields
            vehicle_index = firmware_vehicle.findData(vehicle)
            channel_index = firmware_channel.findData(channel)
            if vehicle_index < 0:
                return 'ERR no firmware vehicle %s' % vehicle
            if channel_index < 0:
                return 'ERR no firmware channel %s' % channel
            firmware_vehicle.setCurrentIndex(vehicle_index)
            firmware_channel.setCurrentIndex(channel_index)
            return ('OK' if start_firmware_download()
                    else 'ERR firmware download is unavailable')
        if command == 'cpu':
            if value.lower() in ('none', 'off', ''):
                cpu_combo.setCurrentIndex(0)
                return 'OK'
            index = cpu_combo.findData(int(value))
            if index < 0:
                return 'ERR CPU %s is unavailable' % value
            cpu_combo.setCurrentIndex(index)
            return 'OK'
        if command == 'real-iomcu':
            return set_checkbox(real_iomcu, value)
        if command == 'iomcu-force-update':
            return set_checkbox(iomcu_update, value)
        if command == 'usb':
            return set_checkbox(usb, value)
        if command == 'can':
            return set_checkbox(can, value)
        if command == 'can-base':
            can_base.setValue(int(value))
            return 'OK'
        if command == 'ethernet':
            if value.lower() in ('', 'off', 'none'):
                ethernet_enable.setChecked(False)
            else:
                ethernet.setCurrentText(value)
                ethernet_enable.setChecked(True)
            return 'OK'
        if command in ('attach', 'detach'):
            fields = value.split()
            if len(fields) != 2:
                return 'ERR %s needs PORT DEVICE' % command
            port_id, device_id = fields
            port = next((candidate for candidate in config_ports
                         if candidate['id'] == port_id), None)
            if port is None:
                return 'ERR no port %s' % port_id
            if device_id not in gen_board.ATTACHABLE_DEVICES:
                return 'ERR no device %s' % device_id
            if command == 'attach':
                before = len(launcher.attachments)
                for row in range(port_tree.topLevelItemCount()):
                    port_item = port_tree.topLevelItem(row)
                    if port_item.data(0, Qt.UserRole)['id'] == port_id:
                        port_tree.setCurrentItem(port_item)
                        break
                index = device_combo.findData(device_id)
                if index < 0:
                    return 'ERR %s cannot attach to %s' % (device_id, port_id)
                device_combo.setCurrentIndex(index)
                if not add_device.isEnabled():
                    return 'ERR attachment conflicts with existing device'
                attach_device()
                return ('OK' if len(launcher.attachments) == before + 1
                        else 'ERR attachment failed')
            item = next((
                port_tree.topLevelItem(row).child(child)
                for row in range(port_tree.topLevelItemCount())
                for child in range(port_tree.topLevelItem(row).childCount())
                if (tree_attachment(
                    port_tree.topLevelItem(row).child(child))['port'] ==
                    port_id and tree_attachment(
                    port_tree.topLevelItem(row).child(child))['device'] ==
                    device_id)
            ), None)
            if item is None:
                return 'ERR device is not attached'
            port_tree.setCurrentItem(item)
            if not remove_device.isEnabled():
                return 'ERR device change is pending'
            remove_selected_device()
            return 'OK'
        if command == 'physics':
            enabled = parse_bool(value)
            physics_autostart.setChecked(enabled)
            sync_physics()
            if launcher.status == 'running':
                if enabled and not launcher.physics_runner.running():
                    error = launcher.start_physics()
                    refresh_physics_controls()
                    return 'ERR %s' % error if error else 'OK'
                if not enabled:
                    launcher.stop_physics()
                    refresh_physics_controls()
            return 'OK'
        if command == 'physics-model':
            if launcher.physics_runner.running():
                return 'ERR stop physics before changing model'
            index = physics_model.findData(value)
            if index < 0:
                return 'ERR no physics model %s' % value
            physics_model.setCurrentIndex(index)
            return 'OK'
        if command == 'physics-location':
            if launcher.physics_runner.running():
                return 'ERR stop physics before changing location'
            fields = value.split()
            if len(fields) != 4:
                return 'ERR physics-location needs LAT LON ALT HEADING'
            latitude, longitude, altitude, heading = map(float, fields)
            physics_latitude.setValue(latitude)
            physics_longitude.setValue(longitude)
            physics_altitude.setValue(altitude)
            physics_heading.setValue(heading)
            sync_physics()
            return 'OK'
        if command == 'physics-rate':
            if launcher.physics_runner.running():
                return 'ERR stop physics before changing rate'
            physics_rate.setValue(int(value))
            sync_physics()
            return 'OK'
        if command == 'physics-binary':
            if launcher.physics_runner.running():
                return 'ERR stop physics before changing binary'
            path = find_physics_binary(value)
            if path is None:
                return 'ERR no physics binary %s' % value
            launcher.physics_binary = path
            physics_binary.setText(str(path))
            refresh_physics_controls()
            return 'OK'
        if command == 'download-renode':
            return 'OK' if start_renode_download() else 'ERR download in progress'
        if command == 'start':
            do_start()
            return ('OK' if launcher.runner.running()
                    else 'ERR %s' % launcher.status)
        if command == 'stop':
            do_stop()
            return 'OK'
        if command == 'status':
            drain_log()
            return 'STATUS ' + json.dumps(launcher.status_snapshot(), sort_keys=True)
        if command == 'quit':
            QTimer.singleShot(100, app.quit)
            return 'OK'
        return 'ERR unknown %s' % command

    def poll_pending():
        try:
            line, done = pending.get_nowait()
        except queue.Empty:
            return
        try:
            done.put(handle_command(line))
        except (OSError, TypeError, ValueError) as error:
            done.put('ERR %s' % error)

    if args.control_port:
        run_control_server(launcher, args.control_port, on_command)
        control_timer = QTimer()
        control_timer.timeout.connect(poll_pending)
        control_timer.start(50)

    apply_filter()
    sync_physics()
    refresh_physics_controls()
    threading.Thread(target=check_renode_download, daemon=True).start()
    signal.signal(signal.SIGINT, lambda *_args: app.quit())
    signal.signal(signal.SIGTERM, lambda *_args: app.quit())
    window.show()
    try:
        app.exec()
    finally:
        launcher.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
