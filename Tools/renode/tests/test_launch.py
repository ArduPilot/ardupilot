#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Tests for the ArduPilot Renode launcher."""

import hashlib
import importlib.util
import io
import json
import os
import socket
import sys
import tarfile

from pathlib import Path
from types import SimpleNamespace

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / 'launch.py'
sys.path.insert(0, str(MODULE_PATH.parent))
import gen_board  # noqa: E402

SPEC = importlib.util.spec_from_file_location('renode_launch', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
launch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(launch)


def latest_metadata(package):
    return {
        'schema_version': 1,
        'renode_version': '1.16.1',
        'source': {'revision': '2a060779f4e2b87d1ae7238a041d858369818805'},
        'artifacts': [{
            'target': {
                'platform': 'linux',
                'architecture': 'x86_64',
                'runtime_identifier': 'linux-x64',
            },
            'packages': [package],
        }],
    }


def test_parse_metrics():
    metrics = launch.parse_metrics(
        'cpu PC; cpu PerformanceInMips; cpu ExecutedInstructions; '
        'emulation GetTimeSourceInfo\n'
        '0x08164B9C\n'
        '0x0000012C\n'
        '0x000000007F1BCE89\n'
        'Elapsed Virtual Time: 00:01:30.250000000\n'
        'Elapsed Host Time: 00:02:00.500000000\n'
        'Current load: 1.0\n'
        '(ardupilot) ')

    assert metrics == {
        'pc': 0x08164B9C,
        'mips': 300,
        'instructions': 0x7F1BCE89,
        'virtual_seconds': 90.25,
        'host_seconds': 120.5,
    }


def test_build_command_contains_selected_options(tmp_path):
    args = SimpleNamespace(
        monitor_port=12390,
        uart_port=5762,
        usbip_port=3240,
        renode='/opt/renode/renode',
        data_cache='/tmp/ardupilot-renode-cache/data',
        state_dir='/tmp/ardupilot-renode-launch-test',
    )
    launcher = launch.Launcher(args)
    launcher.board = 'CubeOrangePlus'
    launcher.firmware = str(tmp_path / 'arducopter')
    Path(launcher.firmware).touch()
    launcher.bootloader = 'none'
    launcher.cpu = min(os.sched_getaffinity(0))
    launcher.real_iomcu = True
    launcher.iomcu_force_update = True
    launcher.usb = True
    launcher.can = False
    launcher.can_base = 4
    launcher.ethernet = 'lo'
    launcher.attachments = [{
        'port': 'SERIAL2',
        'device': 'ublox-gps',
        'bus': 'uart',
    }, {
        'port': 'CAN2',
        'device': 'dronecan-airspeed',
        'bus': 'can',
    }]

    command = launcher.build_command()

    assert command[:3] == [
        os.sys.executable,
        str(launch.HERE / 'run.py'),
        'CubeOrangePlus',
    ]
    assert command[command.index('--cpusel') + 1] == str(launcher.cpu)
    assert command[command.index('--data-cache') + 1] == args.data_cache
    assert '--real-iomcu' in command
    assert '--iomcu-force-update' in command
    assert '--no-device-sidecars' in command
    assert '--usb' in command
    assert command[command.index('--usbip-port') + 1] == '3240'
    assert '--can' in command
    assert command[command.index('--can-base') + 1] == '4'
    assert command[command.index('--ethernet-tap') + 1] == 'lo'
    devices = [json.loads(command[index + 1])
               for index, value in enumerate(command) if value == '--device']
    assert devices == [
        {'device': 'ublox-gps', 'port': 'SERIAL2'},
        {'device': 'dronecan-airspeed', 'port': 'CAN2'},
    ]


def test_config_ports_follow_expanded_hwdef(tmp_path):
    ports = gen_board.configuration_ports(
        launch.ROOT, 'CubeOrangePlus', tmp_path / 'hwdef')

    assert [(port['id'], port['peripheral']) for port in ports] == [
        ('SERIAL1', 'USART2'),
        ('SERIAL2', 'USART3'),
        ('SERIAL3', 'UART4'),
        ('SERIAL4', 'UART8'),
        ('SERIAL5', 'UART7'),
        ('SERIAL7', 'USART6'),
        ('I2C0', 'I2C2'),
        ('I2C1', 'I2C1'),
        ('I2C2', 'I2C4'),
        ('CAN1', 'fdcan1'),
        ('CAN2', 'fdcan2'),
    ]


def test_generate_attached_devices(tmp_path):
    generated = gen_board.generate(
        launch.ROOT, 'CubeOrangePlus', tmp_path / 'generated',
        state_dir=tmp_path,
        attachments=[
            {'port': 'SERIAL2', 'device': 'ublox-gps'},
            {'port': 'I2C2', 'device': 'ist8310-compass'},
            {'port': 'I2C2', 'device': 'ms4525-airspeed'},
            {'port': 'CAN2', 'device': 'dronecan-airspeed'},
            {'port': 'CAN2', 'device': 'dronecan-airspeed'},
        ])

    repl = generated['repl'].read_text()
    resc = generated['resc'].read_text()
    assert 'configDevice0: Sensors.AP_UBlox @ sysbus' in repl
    assert 'configDevice1: Sensors.AP_IST8310 @ i2c4 0x0E' in repl
    assert 'configDevice2: Sensors.AP_Airspeed @ i2c4 0x28' in repl
    assert 'connector Connect sysbus.usart3Host configDevice0Hub' in resc
    assert 'connector Connect sysbus.configDevice0 configDevice0Hub' in resc
    assert generated['serial'] == 'USART2'
    assert [attachment['device']['sidecar']
            for attachment in generated['attachments'][3:]] == [
                'dronecan-airspeed', 'dronecan-airspeed']


def test_duplicate_i2c_address_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='already has a device at 0x0E'):
        gen_board.generate(
            launch.ROOT, 'CubeOrangePlus', tmp_path / 'generated',
            state_dir=tmp_path,
            attachments=[
                {'port': 'I2C2', 'device': 'ist8310-compass'},
                {'port': 'I2C2', 'device': 'ist8310-compass'},
            ])


def test_runtime_uart_device_commands():
    attachment = {
        'runtime_id': 'configHotDevice0',
        'runtime_address': 0x6FFF0000,
    }
    port = {'bus': 'uart', 'target': 'usart3Host'}
    device = {'model': 'Sensors.AP_UBlox'}

    assert launch.runtime_device_commands(attachment, port, device, True) == [
        'machine LoadPlatformDescriptionFromString '
        '"configHotDevice0: Sensors.AP_UBlox @ sysbus 0x6FFF0000"',
        'emulation CreateUARTHub "configHotDevice0Hub"',
        'connector Connect sysbus.usart3Host configHotDevice0Hub',
        'connector Connect sysbus.configHotDevice0 configHotDevice0Hub',
        'configHotDevice0Hub Start',
    ]
    assert launch.runtime_device_commands(attachment, port, device, False) == [
        'machine APHotUnplug "sysbus.configHotDevice0" '
        '"configHotDevice0Hub"',
    ]


def test_runtime_i2c_device_commands():
    attachment = {'runtime_id': 'configHotDevice1'}
    port = {'bus': 'i2c', 'peripheral': 'I2C4'}
    device = {'model': 'Sensors.AP_IST8310', 'address': 0x0E}

    assert launch.runtime_device_commands(attachment, port, device, True) == [
        'machine LoadPlatformDescriptionFromString '
        '"configHotDevice1: Sensors.AP_IST8310 @ i2c4 0x0E"',
    ]
    assert launch.runtime_device_commands(attachment, port, device, False) == [
        'machine APHotUnplug "sysbus.i2c4.configHotDevice1" ""',
    ]


def test_runtime_can_device_commands():
    attachment = {
        'runtime_id': 'configHotDevice2',
        'can_bus_number': 9,
        'configure_bridge': True,
    }
    port = {'bus': 'can', 'can_bus': 'CAN2'}

    assert launch.runtime_device_commands(attachment, port, {}, True) == [
        'sysbus.can2Mcast Bus 9',
    ]
    attachment['configure_bridge'] = False
    assert launch.runtime_device_commands(attachment, port, {}, True) == []
    assert launch.runtime_device_commands(attachment, port, {}, False) == []


def test_runtime_can_nodes_are_unique():
    args = SimpleNamespace(
        monitor_port=12390,
        uart_port=5762,
        usbip_port=3240,
        renode=None,
        state_dir=None,
    )
    launcher = launch.Launcher(args)
    first = {'bus': 'can'}
    second = {'bus': 'can'}

    launcher.allocate_runtime_device(first)
    launcher.allocate_runtime_device(second)

    assert first['node_id'] != second['node_id']
    assert first['runtime_id'] != second['runtime_id']


def test_force_iomcu_update_requires_real_iomcu():
    args = SimpleNamespace(
        monitor_port=12390,
        uart_port=5762,
        usbip_port=3240,
        renode=None,
        state_dir=None,
    )
    launcher = launch.Launcher(args)
    launcher.board = 'CubeOrangePlus'
    launcher.bootloader = 'none'
    launcher.iomcu_force_update = True

    with pytest.raises(ValueError, match='requires real IOMCU'):
        launcher.build_command()


def test_wait_port_free_ignores_closed_connection():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('127.0.0.1', 0))
    port = server.getsockname()[1]
    server.listen()

    client = socket.create_connection(('127.0.0.1', port))
    connection, _address = server.accept()
    try:
        assert not launch.Launcher.wait_port_free(port, timeout=0.01)
    finally:
        # Closing the server side first leaves its local port in TIME_WAIT on
        # Linux.  There is no listener, so this must not prevent a restart.
        connection.close()
        client.recv(1)
        client.close()
        server.close()
    assert launch.Launcher.wait_port_free(port, timeout=0.01)


def test_select_linux_renode_package():
    package = {
        'filename': 'renode-test.linux-portable-dotnet.tar.gz',
        'sha256': 'a' * 64,
        'size': 123,
    }

    selected = launch.select_renode_package(
        latest_metadata(package), system='linux', machine='amd64')

    assert selected['filename'] == package['filename']
    assert selected['runtime_identifier'] == 'linux-x64'


def test_download_cache_is_reused_only_for_current_version(tmp_path):
    executable_data = b'#!/bin/sh\nexit 0\n'
    archive_stream = io.BytesIO()
    with tarfile.open(fileobj=archive_stream, mode='w:gz') as bundle:
        info = tarfile.TarInfo('renode-test/renode')
        info.mode = 0o755
        info.size = len(executable_data)
        bundle.addfile(info, io.BytesIO(executable_data))
    archive = archive_stream.getvalue()
    package = {
        'filename': 'renode-test.linux-portable-dotnet.tar.gz',
        'sha256': hashlib.sha256(archive).hexdigest(),
        'size': len(archive),
    }
    latest = latest_metadata(package)

    def open_archive(_request, timeout):
        assert timeout == 60
        return io.BytesIO(archive)

    executable, _metadata, downloaded = launch.install_current_renode(
        tmp_path, latest=latest, opener=open_archive)

    assert downloaded
    assert executable.read_bytes() == executable_data

    def no_download(_request, _timeout):
        raise AssertionError('current cache should not be downloaded again')

    cached, _metadata, downloaded = launch.install_current_renode(
        tmp_path, latest=latest, opener=no_download)
    assert not downloaded
    assert cached == executable

    newer = latest_metadata(package)
    newer['source']['revision'] = '3' * 40
    selected = launch.select_renode_package(
        newer, system='linux', machine='x86_64')
    assert launch.cached_renode(tmp_path, newer, selected) is None


@pytest.mark.parametrize('fallback', [False, True])
@pytest.mark.parametrize('kind', ['path', 'hardlink', 'device'])
def test_tar_extraction_rejects_unsafe_members(tmp_path, monkeypatch, fallback,
                                               kind):
    if fallback:
        monkeypatch.setattr(launch, 'HAS_TAR_DATA_FILTER', False)
    archive = tmp_path / 'renode.tar.gz'
    with tarfile.open(archive, mode='w:gz') as bundle:
        if kind == 'path':
            info = tarfile.TarInfo('../outside')
            info.size = 1
            bundle.addfile(info, io.BytesIO(b'x'))
        elif kind == 'hardlink':
            info = tarfile.TarInfo('renode-test/link')
            info.type = tarfile.LNKTYPE
            info.linkname = 'renode-test/target'
            bundle.addfile(info)
        else:
            info = tarfile.TarInfo('renode-test/device')
            info.type = tarfile.CHRTYPE
            bundle.addfile(info)
    package = {
        'filename': 'renode-test.linux-portable-dotnet.tar.gz',
        'platform': 'linux',
    }

    with pytest.raises(RuntimeError, match='unsafe path'):
        launch.extract_renode(archive, tmp_path / 'payload', package)


@pytest.mark.parametrize('fallback', [False, True])
@pytest.mark.parametrize('kind', ['nested_member', 'symlink_chain'])
def test_tar_extraction_rejects_symlink_traversal(tmp_path, monkeypatch,
                                                  fallback, kind):
    if fallback:
        monkeypatch.setattr(launch, 'HAS_TAR_DATA_FILTER', False)
    archive = tmp_path / 'renode.tar.gz'
    with tarfile.open(archive, mode='w:gz') as bundle:
        target = tarfile.TarInfo('renode-test/target')
        target.type = tarfile.DIRTYPE
        bundle.addfile(target)
        link = tarfile.TarInfo('renode-test/link')
        link.type = tarfile.SYMTYPE
        link.linkname = 'target'
        bundle.addfile(link)
        if kind == 'nested_member':
            nested = tarfile.TarInfo('renode-test/link/file')
            nested.size = 1
            bundle.addfile(nested, io.BytesIO(b'x'))
        else:
            chained = tarfile.TarInfo('renode-test/chained')
            chained.type = tarfile.SYMTYPE
            chained.linkname = 'link'
            bundle.addfile(chained)
    package = {
        'filename': 'renode-test.linux-portable-dotnet.tar.gz',
        'platform': 'linux',
    }

    with pytest.raises(RuntimeError, match='unsafe path'):
        launch.extract_renode(archive, tmp_path / 'payload', package)


@pytest.mark.parametrize('fallback', [False, True])
def test_tar_extraction_allows_in_tree_symlink(tmp_path, monkeypatch, fallback):
    if fallback:
        monkeypatch.setattr(launch, 'HAS_TAR_DATA_FILTER', False)
    archive = tmp_path / 'renode.tar.gz'
    executable_data = b'#!/bin/sh\nexit 0\n'
    with tarfile.open(archive, mode='w:gz') as bundle:
        target = tarfile.TarInfo(
            'renode-test/plugins/IntegrationLibrary/libs/socket-cpp')
        target.type = tarfile.DIRTYPE
        bundle.addfile(target)
        link = tarfile.TarInfo(
            'renode-test/plugins/SystemCModule/lib/socket-cpp')
        link.type = tarfile.SYMTYPE
        link.linkname = '../../IntegrationLibrary/libs/socket-cpp'
        bundle.addfile(link)
        executable = tarfile.TarInfo('renode-test/renode')
        executable.mode = 0o755
        executable.size = len(executable_data)
        bundle.addfile(executable, io.BytesIO(executable_data))
    package = {
        'filename': 'renode-test.linux-portable-dotnet.tar.gz',
        'platform': 'linux',
    }

    launch.extract_renode(archive, tmp_path / 'payload', package)

    extracted_link = (tmp_path / 'payload/renode-test/plugins/'
                      'SystemCModule/lib/socket-cpp')
    extracted_target = (tmp_path / 'payload/renode-test/plugins/'
                        'IntegrationLibrary/libs/socket-cpp')
    assert extracted_link.is_symlink()
    assert extracted_link.resolve() == extracted_target.resolve()


@pytest.mark.parametrize('fallback', [False, True])
def test_tar_extraction_rejects_escaping_symlink(tmp_path, monkeypatch, fallback):
    if fallback:
        monkeypatch.setattr(launch, 'HAS_TAR_DATA_FILTER', False)
    archive = tmp_path / 'renode.tar.gz'
    with tarfile.open(archive, mode='w:gz') as bundle:
        link = tarfile.TarInfo('renode-test/link')
        link.type = tarfile.SYMTYPE
        link.linkname = '../../outside'
        bundle.addfile(link)
    package = {
        'filename': 'renode-test.linux-portable-dotnet.tar.gz',
        'platform': 'linux',
    }

    with pytest.raises(RuntimeError, match='unsafe path'):
        launch.extract_renode(archive, tmp_path / 'payload', package)
