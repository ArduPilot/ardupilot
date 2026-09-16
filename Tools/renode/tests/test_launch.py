#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Tests for the ArduPilot Renode launcher."""

import hashlib
import importlib.util
import io
import json
import os
import signal
import socket
import subprocess
import sys
import tarfile
import time

from pathlib import Path
from types import SimpleNamespace

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / 'launch.py'
sys.path.insert(0, str(MODULE_PATH.parent))
import gen_board  # noqa: E402
import process_utils  # noqa: E402

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


@pytest.mark.skipif(os.name != 'posix', reason='requires POSIX process groups')
def test_process_group_cleanup_after_leader_exits():
    script = (
        'import subprocess,sys; '
        'child=subprocess.Popen([sys.executable,"-c",'
        '"import time; time.sleep(60)"]); '
        'print(child.pid,flush=True)'
    )
    process = subprocess.Popen(
        [sys.executable, '-c', script],
        stdout=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    child_pid = int(process.stdout.readline())
    process.wait(timeout=5)
    try:
        os.kill(child_pid, 0)
        process_utils.terminate_process_group(
            process, graceful_timeout=0.2)
        assert not process_utils._process_group_exists(process.pid)
    finally:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def test_dronecan_device_emulator_has_bounded_runtime():
    result = subprocess.run(
        [
            sys.executable,
            str(launch.HERE / 'device_emulator.py'),
            'dronecan-airspeed',
            '--can-bus', '9',
            '--node-id', '127',
            '--run-seconds', '0.1',
        ],
        cwd=launch.ROOT,
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert 'DroneCAN airspeed node 127 on mcast:9' in result.stdout


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
    launcher.imu_names = ['icm42688_ext', 'icm20948_ext', 'icm20649']
    launcher.real_iomcu = True
    launcher.iomcu_force_update = True
    launcher.uds = True
    launcher.usb = True
    launcher.dfu = True
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
    assert [command[index + 1] for index, value in enumerate(command)
            if value == '--imu'] == launcher.imu_names
    assert command[command.index('--data-cache') + 1] == args.data_cache
    assert '--real-iomcu' in command
    assert '--iomcu-force-update' in command
    assert '--no-device-sidecars' in command
    assert '--uds' in command
    assert launcher.status_snapshot()['uds'] is True
    assert '--usb' in command
    assert '--dfu' in command
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


def test_build_command_skips_disabled_and_keeps_device_configuration(tmp_path):
    args = SimpleNamespace(
        monitor_port=12390, uart_port=5762, usbip_port=3240,
        renode=None, state_dir=None,
    )
    launcher = launch.Launcher(args)
    launcher.board = 'CubeOrangePlus'
    launcher.bootloader = 'none'
    launcher.attachments = [{
        'port': 'SERIAL2', 'device': 'ublox-gps', 'bus': 'uart',
        'enabled': False,
    }, {
        'port': 'I2C2', 'device': 'ist8310-compass', 'bus': 'i2c',
        'configuration': {'orientation': 4},
    }]

    command = launcher.build_command()
    devices = [json.loads(command[index + 1])
               for index, value in enumerate(command) if value == '--device']

    assert devices == [{
        'configuration': {'orientation': 4},
        'device': 'ist8310-compass',
        'port': 'I2C2',
    }]


def test_launch_settings_round_trip_and_strip_runtime_state(tmp_path):
    settings_path = tmp_path / 'launch-settings.json'
    attachment = {
        'port': 'I2C2', 'device': 'ist8310-compass', 'bus': 'i2c',
        'enabled': False, 'configuration': {'orientation': 12},
        'runtime_id': 'configHotDevice1', 'state': 'disabled',
    }
    settings = {
        'format': launch.SETTINGS_FORMAT,
        'target': {'board': 'CubeOrangePlus', 'uds': True},
        'config': {
            'attachments': [launch.saved_attachment(attachment)],
            'imus': ['icm42688_ext', 'icm20948_ext', 'icm20649'],
        },
        'physics': {'model': 'quad', 'rate_hz': 400},
    }

    launch.save_launch_settings(settings, settings_path)

    assert launch.load_launch_settings(settings_path) == settings
    saved = settings['config']['attachments'][0]
    assert saved == {
        'port': 'I2C2', 'device': 'ist8310-compass',
        'enabled': False, 'configuration': {'orientation': 12},
    }
    settings_path.write_text('{broken')
    assert launch.load_launch_settings(settings_path) == {}


def test_launch_settings_path_uses_invocation_directory(tmp_path):
    script = (
        'import sys; '
        'sys.path.insert(0, sys.argv[1]); '
        'import launch; '
        'launch.save_launch_settings({"format": launch.SETTINGS_FORMAT, '
        '"target": {"board": sys.argv[2]}})'
    )
    first = tmp_path / 'first'
    second = tmp_path / 'second'
    first.mkdir()
    second.mkdir()
    for directory, board in ((first, 'CubeBlack'),
                             (second, 'CubeOrangePlus')):
        subprocess.run(
            [sys.executable, '-c', script, str(MODULE_PATH.parent), board],
            cwd=directory, check=True)

    assert json.loads((first / 'launch-settings.json').read_text())[
        'target']['board'] == 'CubeBlack'
    assert json.loads((second / 'launch-settings.json').read_text())[
        'target']['board'] == 'CubeOrangePlus'


def test_dfu_requires_usb():
    args = SimpleNamespace(
        monitor_port=12390,
        uart_port=5762,
        usbip_port=3240,
        renode=None,
        state_dir=None,
    )
    launcher = launch.Launcher(args)
    launcher.board = 'CubeBlack'
    launcher.bootloader = 'none'
    launcher.dfu = True

    with pytest.raises(ValueError, match='DFU requires USB'):
        launcher.build_command()


def test_physics_connect_command_and_status():
    args = SimpleNamespace(
        monitor_port=12390,
        uart_port=5762,
        usbip_port=3240,
        physics_port=9102,
        physics_binary=None,
        renode=None,
        state_dir=None,
    )
    launcher = launch.Launcher(args)
    launcher.physics_model = 'quadplane'
    launcher.physics_latitude = -35.363261
    launcher.physics_longitude = 149.165230
    launcher.physics_altitude = 584.25
    launcher.physics_heading = 353.0
    launcher.physics_rate = 400

    assert launcher.physics_connect_command() == (
        'physics Connect 9102 "quadplane" -35.363261 149.16523 '
        '584.25 353 400')
    launcher.physics_longitude = 179.1234567
    assert ' 179.1234567 ' in launcher.physics_connect_command()
    physics = launcher.status_snapshot()['physics']
    assert physics['model'] == 'quadplane'
    assert physics['port'] == 9102
    assert physics['location']['altitude_m'] == 584.25


def test_explicit_physics_binary_is_absolute(tmp_path, monkeypatch):
    binary = tmp_path / 'renode-physics'
    binary.touch()
    monkeypatch.chdir(tmp_path)

    assert launch.find_physics_binary('renode-physics') == binary.resolve()


def test_physics_connect_command_rejects_invalid_values():
    args = SimpleNamespace(
        monitor_port=12390,
        uart_port=5762,
        usbip_port=3240,
        physics_port=9002,
        physics_binary=None,
        renode=None,
        state_dir=None,
    )
    launcher = launch.Launcher(args)
    launcher.physics_latitude = float('nan')
    with pytest.raises(ValueError, match='invalid physics'):
        launcher.physics_connect_command()
    launcher.physics_latitude = 0.0
    launcher.physics_rate = 0
    with pytest.raises(ValueError, match='rate'):
        launcher.physics_connect_command()


def test_launcher_starts_standalone_physics_sidecar():
    binary = launch.find_physics_binary()
    if binary is None:
        pytest.skip('build tool/renode-physics or install the sidecar')
    listener = socket.socket()
    listener.bind(('127.0.0.1', 0))
    physics_port = listener.getsockname()[1]
    listener.close()
    args = SimpleNamespace(
        monitor_port=12390,
        uart_port=5762,
        usbip_port=3240,
        physics_port=physics_port,
        physics_binary=str(binary),
        renode=None,
        state_dir=None,
    )
    launcher = launch.Launcher(args)
    try:
        started = time.monotonic()
        assert launcher.start_physics() is None
        assert time.monotonic() - started < 1
        assert launcher.physics_runner.running()
        assert launcher.physics_status == 'starting'
        deadline = time.monotonic() + 10
        while launcher.physics_status == 'starting':
            launcher.handle_event(launcher.log_q.get(
                timeout=max(0.1, deadline - time.monotonic())))
        assert launcher.physics_status == 'listening'
    finally:
        launcher.stop_physics()
    assert launcher.physics_status == 'off'


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
            {'port': 'SERIAL3', 'device': 'benewake-rangefinder',
             'physics_index': 3},
            {'port': 'I2C2', 'device': 'ist8310-compass'},
            {'port': 'I2C2', 'device': 'ms4525-airspeed'},
            {'port': 'CAN2', 'device': 'dronecan-airspeed'},
            {'port': 'CAN2', 'device': 'dronecan-airspeed'},
        ])

    repl = generated['repl'].read_text()
    resc = generated['resc'].read_text()
    assert 'physics: Miscellaneous.AP_Physics @ sysbus' in repl
    assert 'configDevice0: Sensors.AP_UBlox @ sysbus' in repl
    assert ('configDevice1: Sensors.AP_Benewake @ sysbus' in repl and
            '    RangefinderIndex: 3' in repl)
    assert 'configDevice2: Sensors.AP_IST8310 @ i2c4 0x0E' in repl
    assert 'configDevice3: Sensors.AP_Airspeed @ i2c4 0x28' in repl
    assert 'connector Connect sysbus.usart3Host configDevice0Hub' in resc
    assert 'connector Connect sysbus.configDevice0 configDevice0Hub' in resc
    assert generated['serial'] == 'USART2'
    assert [attachment['device']['sidecar']
            for attachment in generated['attachments'][4:]] == [
                'dronecan-airspeed', 'dronecan-airspeed']


def test_generate_compass_orientation(tmp_path):
    generated = gen_board.generate(
        launch.ROOT, 'CubeOrangePlus', tmp_path / 'generated',
        state_dir=tmp_path,
        attachments=[{
            'port': 'I2C2', 'device': 'ist8310-compass',
            'configuration': {'orientation': 6},
        }])

    assert ('configDevice0: Sensors.AP_IST8310 @ i2c4 0x0E\n'
            '    Rotation: 6') in generated['repl'].read_text()


def test_generate_unix_socket_terminal(tmp_path):
    generated = gen_board.generate(
        launch.ROOT, 'CubeBlack', tmp_path / 'generated',
        state_dir=tmp_path, uds=True)

    expected = (tmp_path / ('APM-UDS-serial%u' %
                            generated['serial_index'])).resolve()
    resc = generated['resc'].read_text()
    assert generated['uart_socket'] == expected
    assert ('include $repo/Tools/renode/peripherals/common/'
            'AP_UnixSocketTerminal.cs') in resc
    assert 'emulation CreateUnixSocketTerminal %s "serial"' % json.dumps(
        str(expected)) in resc
    assert 'CreateServerSocketTerminal' not in resc


def test_timer_actuators_follow_hwdef_and_iomcu_offset(tmp_path):
    generated = gen_board.generate(
        launch.ROOT, 'CubeBlack', tmp_path / 'generated',
        state_dir=tmp_path)

    repl = generated['repl'].read_text()
    assert 'timer1Actuators: Miscellaneous.AP_STM32_Timer_Actuators' in repl
    assert '    timer: timer1\n' in repl
    assert '    output1: 11\n' in repl
    assert '    output2: 10\n' in repl
    assert '    output3: 9\n' in repl
    assert '    output4: 8\n' in repl
    assert 'timer4Actuators: Miscellaneous.AP_STM32_Timer_Actuators' in repl
    assert '    output2: 12\n' in repl
    assert '    output3: 13\n' in repl


def test_timer_actuators_start_at_output_one_without_iomcu(tmp_path):
    generated = gen_board.generate(
        launch.ROOT, 'BlitzWingH743', tmp_path / 'generated',
        state_dir=tmp_path)

    repl = generated['repl'].read_text()
    assert 'iomcu: Miscellaneous.AP_IOMCU' not in repl
    assert 'timer3Actuators: Miscellaneous.AP_STM32_Timer_Actuators' in repl
    assert '    output3: 0\n' in repl
    assert '    output4: 1\n' in repl


def test_system_timer_compare_zero_is_kept_enabled(tmp_path):
    generated = gen_board.generate(
        launch.ROOT, 'KakuteF4', tmp_path / 'generated',
        state_dir=tmp_path)

    resc = generated['resc'].read_text()
    assert (
        'sysbus SetHookBeforePeripheralWrite sysbus.timer4 '
        '"if offset == 0x34 and value == 0: value = 1"' in resc
    )


def test_timer_actuators_mark_complementary_hwdef_channels(tmp_path):
    generated = gen_board.generate(
        launch.ROOT, 'MatekH743', tmp_path / 'generated',
        state_dir=tmp_path)

    repl = generated['repl'].read_text()
    assert 'timer8Actuators: Miscellaneous.AP_STM32_Timer_Actuators' in repl
    assert '    output2: 0\n    complementary2: true\n' in repl
    assert '    output3: 1\n    complementary3: true\n' in repl


def test_spi_sensor_transactions_follow_chip_select(tmp_path):
    generated = gen_board.generate(
        launch.ROOT, 'MatekH743', tmp_path / 'generated',
        state_dir=tmp_path)

    repl = generated['repl'].read_text()
    assert 'spi4Mux: Miscellaneous.AP_SPIMultiplexer @ spi4' in repl
    assert 'frameOnTransfer: true' not in repl


def test_named_imus_select_cubeorangeplus_sensor_set(tmp_path):
    generated = gen_board.generate(
        launch.ROOT, 'CubeOrangePlus', tmp_path / 'generated',
        state_dir=tmp_path,
        imu_names=['icm42688_ext', 'icm20948_ext', 'icm20649'])

    repl = generated['repl'].read_text()
    assert generated['num_imus'] == 3
    assert 'imu0: Sensors.AP_ICM42688 @ spi4Mux' in repl
    assert 'imu2: Sensors.AP_InvensenseV2 @ spi4Mux' in repl
    assert 'imu3: Sensors.AP_InvensenseV2 @ spi1Mux' in repl
    # AK09916 behind ICM20948 instance 0 with the hwdef COMPASS rotation
    imu2 = repl[repl.index('imu2: Sensors.AP_InvensenseV2'):]
    imu2 = imu2[:imu2.index('\n\n')]
    assert 'whoAmI: 0xEA' in imu2
    assert 'AuxiliaryCompassRotation: %u' % gen_board.ROTATIONS[
        'ROTATION_ROLL_180_YAW_90'] in imu2
    assert repl.count('AuxiliaryCompassRotation') == 1
    assert 'imu1:' not in repl
    assert 'imu4:' not in repl
    assert 'imu5:' not in repl
    assert 'imu6:' not in repl


def test_unknown_named_imu_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='unknown IMU device: absent'):
        gen_board.generate(
            launch.ROOT, 'CubeOrangePlus', tmp_path / 'generated',
            state_dir=tmp_path, imu_names=['absent'])


def test_duplicate_i2c_address_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='already has a device at 0x0E'):
        gen_board.generate(
            launch.ROOT, 'CubeOrangePlus', tmp_path / 'generated',
            state_dir=tmp_path,
            attachments=[
                {'port': 'I2C2', 'device': 'ist8310-compass'},
                {'port': 'I2C2', 'device': 'ist8310-compass'},
            ])


def test_compound_i2c_attachment_occupies_every_address(tmp_path):
    generated = gen_board.generate(
        launch.ROOT, 'CubeOrangePlus', tmp_path / 'generated',
        state_dir=tmp_path,
        attachments=[
            {'port': 'I2C2', 'device': 'oreoled-set'},
        ])

    repl = generated['repl'].read_text()
    assert 'configDevice0Part0: Sensors.AP_OreoLED0 @ i2c4 0x68' in repl
    assert 'configDevice0Part1: Sensors.AP_OreoLED1 @ i2c4 0x69' in repl
    assert 'configDevice0Part2: Sensors.AP_OreoLED2 @ i2c4 0x6A' in repl
    assert 'configDevice0Part3: Sensors.AP_OreoLED3 @ i2c4 0x6B' in repl

    with pytest.raises(ValueError, match='already has a device at 0x68'):
        gen_board.generate(
            launch.ROOT, 'CubeOrangePlus', tmp_path / 'duplicate',
            state_dir=tmp_path,
            attachments=[
                {'port': 'I2C2', 'device': 'oreoled-set'},
                {'port': 'I2C2', 'device': 'bmi160-i2c-imu'},
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


def test_runtime_uart_device_physics_channel():
    attachment = {
        'runtime_id': 'configHotDevice0',
        'runtime_address': 0x6FFF0000,
        'physics_index': 2,
    }
    port = {'bus': 'uart', 'target': 'usart3Host'}
    device = {
        'model': 'Sensors.AP_Benewake',
        'physics': {
            'source': 'rangefinder',
            'property': 'RangefinderIndex',
            'count': 10,
        },
    }

    assert launch.runtime_device_commands(attachment, port, device, True)[0] == (
        'machine LoadPlatformDescriptionFromString '
        '"configHotDevice0: Sensors.AP_Benewake @ sysbus 0x6FFF0000'
        '\\n    RangefinderIndex: 2"')


def test_launcher_allocates_and_reuses_physics_channels():
    launcher = launch.Launcher.__new__(launch.Launcher)
    launcher.attachments = []
    device = launch.ATTACHABLE_DEVICES['benewake-rangefinder']
    first = {'device': 'benewake-rangefinder'}
    second = {'device': 'lightware-rangefinder'}

    launcher.allocate_physics_channel(first, device)
    launcher.attachments.append(first)
    launcher.allocate_physics_channel(
        second, launch.ATTACHABLE_DEVICES['lightware-rangefinder'])
    launcher.attachments.append(second)

    assert first['physics_index'] == 0
    assert second['physics_index'] == 1
    launcher.attachments.remove(first)
    replacement = {'device': 'benewake-rangefinder'}
    launcher.allocate_physics_channel(replacement, device)
    assert replacement['physics_index'] == 0


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


def test_runtime_compass_orientation():
    attachment = {
        'runtime_id': 'configHotDevice1',
        'configuration': {'orientation': 24},
    }
    port = {'bus': 'i2c', 'peripheral': 'I2C4'}
    device = launch.ATTACHABLE_DEVICES['ist8310-compass']

    assert launch.runtime_device_commands(attachment, port, device, True) == [
        'machine LoadPlatformDescriptionFromString '
        '"configHotDevice1: Sensors.AP_IST8310 @ i2c4 0x0E'
        '\\n    Rotation: 24"',
    ]


def test_runtime_compound_i2c_device_commands():
    attachment = {'runtime_id': 'configHotDevice4'}
    port = {'bus': 'i2c', 'peripheral': 'I2C4'}
    device = launch.ATTACHABLE_DEVICES['oreoled-set']

    assert launch.runtime_device_commands(attachment, port, device, True) == [
        'machine LoadPlatformDescriptionFromString '
        '"configHotDevice4Part0: Sensors.AP_OreoLED0 @ i2c4 0x68"',
        'machine LoadPlatformDescriptionFromString '
        '"configHotDevice4Part1: Sensors.AP_OreoLED1 @ i2c4 0x69"',
        'machine LoadPlatformDescriptionFromString '
        '"configHotDevice4Part2: Sensors.AP_OreoLED2 @ i2c4 0x6A"',
        'machine LoadPlatformDescriptionFromString '
        '"configHotDevice4Part3: Sensors.AP_OreoLED3 @ i2c4 0x6B"',
    ]
    assert launch.runtime_device_commands(attachment, port, device, False) == [
        'machine APHotUnplug "sysbus.i2c4.configHotDevice4Part0" ""',
        'machine APHotUnplug "sysbus.i2c4.configHotDevice4Part1" ""',
        'machine APHotUnplug "sysbus.i2c4.configHotDevice4Part2" ""',
        'machine APHotUnplug "sysbus.i2c4.configHotDevice4Part3" ""',
    ]


class MonitorBatchClient:
    def __init__(self, errors=()):
        self.errors = set(errors)
        self.commands = []

    def command(self, command, timeout):
        assert timeout == 15
        self.commands.append(command)
        if command in self.errors:
            return 'Error E01: failed %s' % command
        return '(ardupilot) '


def test_monitor_command_batch_rolls_back_completed_compound_parts():
    client = MonitorBatchClient(errors=('attach2',))

    error, rollback_failed = launch.execute_monitor_command_batch(
        client,
        ('attach0', 'attach1', 'attach2', 'attach3'),
        ('detach0', 'detach1', 'detach2', 'detach3'))

    assert 'failed attach2' in error
    assert not rollback_failed
    assert client.commands == [
        'attach0', 'attach1', 'attach2', 'detach1', 'detach0']


def test_monitor_command_batch_reports_failed_rollback():
    client = MonitorBatchClient(errors=('attach2', 'detach0'))

    error, rollback_failed = launch.execute_monitor_command_batch(
        client,
        ('attach0', 'attach1', 'attach2', 'attach3'),
        ('detach0', 'detach1', 'detach2', 'detach3'))

    assert 'failed attach2' in error
    assert 'rollback failed' in error
    assert 'failed detach0' in error
    assert rollback_failed


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
    if sys.platform.startswith('linux'):
        assert launch.Launcher.bindable_tcp_port(port) != port


def test_bindable_tcp_port_avoids_bound_port():
    blocker = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    blocker.bind(('0.0.0.0', 0))
    blocked_port = blocker.getsockname()[1]
    try:
        selected = launch.Launcher.bindable_tcp_port(blocked_port)
    finally:
        blocker.close()

    assert selected != blocked_port
    assert launch.Launcher.bindable_tcp_port(selected) == selected


def test_monitor_port_fallback_does_not_change_preference():
    blocker = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    blocker.bind(('0.0.0.0', 0))
    configured_port = blocker.getsockname()[1]
    args = SimpleNamespace(
        monitor_port=configured_port, uart_port=5762, usbip_port=3240,
        renode=None, state_dir=None,
    )
    launcher = launch.Launcher(args)
    try:
        first = launcher.select_monitor_port()
    finally:
        blocker.close()

    assert first != configured_port
    assert launcher.select_monitor_port() == configured_port
    assert args.monitor_port == configured_port


@pytest.mark.parametrize('port', [-1, 0, 65536, True])
def test_monitor_port_range_is_validated(port):
    args = SimpleNamespace(
        monitor_port=port, uart_port=5762, usbip_port=3240,
        renode=None, state_dir=None,
    )
    launcher = launch.Launcher(args)

    with pytest.raises(ValueError, match='monitor port'):
        launcher.select_monitor_port()


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


@pytest.mark.parametrize('runtime', [
    '', '.', '..', '../outside', '/tmp/outside', 'a/../../outside',
    'a\\..\\outside', 'C:\\outside', None,
])
def test_reject_unsafe_runtime_identifier(runtime):
    latest = latest_metadata({
        'filename': 'renode.tar.gz', 'sha256': 'a' * 64, 'size': 123,
    })
    latest['artifacts'][0]['target']['runtime_identifier'] = runtime
    with pytest.raises(RuntimeError, match='invalid runtime identifier'):
        launch.select_renode_package(latest, system='linux', machine='amd64')


def test_cpu_pinning_without_host_affinity_support(tmp_path, monkeypatch):
    monkeypatch.delattr(os, 'sched_getaffinity', raising=False)
    monkeypatch.delattr(os, 'sched_setaffinity', raising=False)
    launcher = launch.Launcher(SimpleNamespace(
        monitor_port=12390, uart_port=5762, usbip_port=3240,
        renode='renode', data_cache=None, state_dir=str(tmp_path)))
    launcher.board = 'CubeOrangePlus'
    launcher.bootloader = 'none'
    launcher.build_command()
    launcher.cpu = 0
    with pytest.raises(ValueError, match='CPU-affinity support'):
        launcher.build_command()


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
