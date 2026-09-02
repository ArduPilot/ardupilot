#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Probe Renode peripherals with their production ChibiOS drivers."""

import argparse
import contextlib
import dataclasses
import json
import os
import re
import socket
import subprocess
import sys
import threading
import time

from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import gen_board  # noqa: E402
import physics_stub  # noqa: E402
import test_mission as common  # noqa: E402

from driver_catalog import ATTACHABLE_DEVICES  # noqa: E402
from driver_catalog import DRIVER_PROBE_PROFILES  # noqa: E402
from launch import MonitorClient  # noqa: E402

DEFAULT_PROFILE = 'matekh743-navigation'
IST8310_ADDRESS = 0x0E
IST8310_DEVTYPE = 0x0A
AIRSPEED_DEVICE_IDS = {
    'ms4525-airspeed': (0x28, 0x02),
    'asp5033-airspeed': (0x6C, 0x0A),
    'auav-airspeed': (0x26, 0x0B),
}
BMP280_ADDRESS = 0x76
BMP280_DEVTYPE = 0x03
PHYSICS_RATE_HZ = 100
BASELINE = {
    'latitude_deg': -35.363261,
    'longitude_deg': 149.165230,
    'altitude_m': 584.0,
    'airspeed_m_s': 0.0,
    'magnetic_field_body_mgauss': (201.0, 0.0, 450.0),
    'rangefinder_m': tuple(5.0 + index for index in range(10)),
    'pressure_pa': 101325.0,
    'temperature_k': 293.15,
}
STEPPED = {
    'latitude_deg': -35.362261,
    'longitude_deg': 149.164230,
    'altitude_m': 614.0,
    'airspeed_m_s': 20.0,
    'magnetic_field_body_mgauss': (350.0, -120.0, 80.0),
    'rangefinder_m': tuple(8.0 + index for index in range(10)),
    'pressure_pa': 90000.0,
    'temperature_k': 303.15,
}


class ControlledPhysics:
    """Serve stationary truth which the test can switch deterministically."""

    def __init__(self):
        self.server = socket.create_server(
            ('127.0.0.1', 0), family=socket.AF_INET, backlog=1)
        self.server.settimeout(0.5)
        self.port = self.server.getsockname()[1]
        self.stepped = threading.Event()
        self.stopping = threading.Event()
        self.connection = None
        self.error = None
        self.thread = threading.Thread(
            target=self._serve, name='Renode driver-probe physics', daemon=True)

    def start(self):
        self.thread.start()

    def _truth(self, configuration, step):
        truth = physics_stub._truth_for_step(configuration, step)
        values = STEPPED if self.stepped.is_set() else BASELINE
        return dataclasses.replace(truth, **values)

    def _serve(self):
        try:
            while not self.stopping.is_set():
                try:
                    self.connection, _address = self.server.accept()
                    break
                except TimeoutError:
                    continue
            if self.connection is not None:
                with self.connection:
                    physics_stub.serve_connection(
                        self.connection, truth_provider=self._truth)
        except (OSError, RuntimeError, ValueError) as error:
            if not self.stopping.is_set():
                self.error = error

    def check(self):
        if self.error is not None:
            raise RuntimeError('controlled physics failed: %s' % self.error)

    def stop(self):
        self.stopping.set()
        with contextlib.suppress(OSError):
            self.server.close()
        if self.connection is not None:
            with contextlib.suppress(OSError):
                self.connection.shutdown(socket.SHUT_RDWR)
        self.thread.join(timeout=5)
        if self.thread.is_alive():
            raise RuntimeError('controlled physics thread did not stop')
        self.check()


def build_firmware(root, profile):
    defaults = root / profile['defaults']
    print('building %s %s driver-probe firmware' %
          (profile['board'], profile['vehicle']), flush=True)
    subprocess.run(
        ['./waf', 'configure', '--board', profile['board'],
         '--default-parameters', str(defaults)],
        cwd=root,
        check=True,
    )
    subprocess.run(['./waf', profile['build_target']], cwd=root, check=True)


def parameter_id(message):
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode(errors='replace')
    return value.rstrip('\x00')


def read_parameter(connection, process, log_path, name, deadline):
    while time.monotonic() < deadline:
        connection.mav.param_request_read_send(
            connection.target_system,
            connection.target_component,
            name.encode('ascii'),
            -1,
        )
        retry_deadline = min(deadline, time.monotonic() + 2)
        while time.monotonic() < retry_deadline:
            message = common.recv_match(
                connection, process, log_path, 'PARAM_VALUE', retry_deadline)
            if parameter_id(message) == name:
                return int(round(message.param_value))
    raise RuntimeError('timed out reading %s' % name)


def decode_device_id(device_id):
    return {
        'bus_type': device_id & 0x07,
        'bus': (device_id >> 3) & 0x1F,
        'address': (device_id >> 8) & 0xFF,
        'devtype': (device_id >> 16) & 0xFF,
    }


def test_decode_device_id():
    device_id = 1 | (7 << 3) | (IST8310_ADDRESS << 8) | (IST8310_DEVTYPE << 16)
    assert decode_device_id(device_id) == {
        'bus_type': 1,
        'bus': 7,
        'address': IST8310_ADDRESS,
        'devtype': IST8310_DEVTYPE,
    }


def wait_for_ublox(connection, process, log_path, deadline):
    detected = False
    fix_type = 0
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        message = connection.recv_match(blocking=True, timeout=1)
        if message is None:
            continue
        message_type = message.get_type()
        if message_type == 'STATUSTEXT':
            print('vehicle: %s' % message.text, flush=True)
            detected |= 'GPS 1: detected u-blox' in message.text
        elif message_type == 'GPS_RAW_INT':
            fix_type = max(fix_type, message.fix_type)
        if detected and fix_type >= mavutil.mavlink.GPS_FIX_TYPE_3D_FIX:
            print('u-blox production driver detected with a 3D fix', flush=True)
            return
    raise RuntimeError(
        'u-blox probe failed: detection=%s best fix=%u' %
        (detected, fix_type))


def find_ist8310(connection, process, log_path, deadline):
    parameters = ('COMPASS_DEV_ID', 'COMPASS_DEV_ID2', 'COMPASS_DEV_ID3')
    device_ids = [
        read_parameter(connection, process, log_path, name, deadline)
        for name in parameters
    ]
    for device_id in device_ids:
        decoded = decode_device_id(device_id)
        if (decoded['bus_type'] == 1 and
                decoded['address'] == IST8310_ADDRESS and
                decoded['devtype'] == IST8310_DEVTYPE):
            print(
                'IST8310 production driver detected on I2C bus %u at 0x%02X' %
                (decoded['bus'], decoded['address']),
                flush=True,
            )
            return
    raise RuntimeError('IST8310 device ID not found in %s' % device_ids)


def find_airspeed_device(connection, process, log_path, device, instance,
                         deadline):
    address, devtype = AIRSPEED_DEVICE_IDS[device]
    parameter = 'ARSPD_DEVID' if instance == 1 else 'ARSPD%u_DEVID' % instance
    last_decoded = decode_device_id(0)
    while time.monotonic() < deadline:
        try:
            device_id = read_parameter(
                connection, process, log_path, parameter,
                min(deadline, time.monotonic() + 2))
        except RuntimeError:
            continue
        last_decoded = decode_device_id(device_id)
        if (last_decoded['bus_type'] == 1 and
                last_decoded['address'] == address and
                last_decoded['devtype'] == devtype):
            print('%s production driver detected on I2C bus %u at 0x%02X' %
                  (ATTACHABLE_DEVICES[device]['name'], last_decoded['bus'],
                   last_decoded['address']), flush=True)
            return
    raise RuntimeError('%s device ID is invalid: %s' %
                       (ATTACHABLE_DEVICES[device]['name'], last_decoded))


def find_bmp280(connection, process, log_path, deadline):
    last_devices = []
    while time.monotonic() < deadline:
        last_devices = []
        for instance in range(1, 4):
            try:
                device_id = read_parameter(
                    connection, process, log_path, 'BARO%u_DEVID' % instance,
                    min(deadline, time.monotonic() + 2))
            except RuntimeError:
                continue
            decoded = decode_device_id(device_id)
            last_devices.append(decoded)
            if (decoded['bus_type'] == 1 and
                    decoded['address'] == BMP280_ADDRESS and
                    decoded['devtype'] == BMP280_DEVTYPE):
                print('BMP280 production driver detected as barometer %u on '
                      'I2C bus %u at 0x%02X' %
                      (instance, decoded['bus'], decoded['address']),
                      flush=True)
                return instance
    raise RuntimeError('BMP280 device ID not found in %s' % last_devices)


def gps_matches(message, expected):
    return (
        abs(message.lat * 1.0e-7 - expected['latitude_deg']) <= 2.0e-6 and
        abs(message.lon * 1.0e-7 - expected['longitude_deg']) <= 2.0e-6 and
        abs(message.alt * 0.001 - expected['altitude_m']) <= 2.0
    )


def compass_matches(message, expected):
    measured = (message.xmag, message.ymag, message.zmag)
    target = expected['magnetic_field_body_mgauss']
    return all(abs(actual - wanted) <= 15 for actual, wanted in zip(measured, target))


def wait_for_values(connection, process, log_path, physics, expected, devices,
                    deadline, description):
    need_gps = 'ublox-gps' in devices
    need_compass = 'ist8310-compass' in devices
    gps = not need_gps
    compass = not need_compass
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(blocking=True, timeout=1)
        if message is None:
            continue
        message_type = message.get_type()
        if (need_gps and message_type == 'GPS_RAW_INT' and
                gps_matches(message, expected)):
            gps = message
        elif (need_compass and message_type == 'RAW_IMU' and
              compass_matches(message, expected)):
            compass = message
        if gps and compass:
            details = []
            if need_gps:
                details.append('GPS %.7f %.7f %.1fm' % (
                    gps.lat * 1.0e-7, gps.lon * 1.0e-7, gps.alt * 0.001))
            if need_compass:
                details.append('compass %d %d %d mG' %
                               (compass.xmag, compass.ymag, compass.zmag))
            print('%s values passed: %s' %
                  (description, ', '.join(details)), flush=True)
            return
    raise RuntimeError('%s GPS/compass values did not reach expected inputs' % description)


def wait_for_compass_health(connection, process, log_path, physics, healthy,
                            deadline):
    sensor = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
        if message is None:
            continue
        enabled = bool(message.onboard_control_sensors_enabled & sensor)
        reported_healthy = bool(message.onboard_control_sensors_health & sensor)
        if enabled and reported_healthy == healthy:
            print('production compass became %s' %
                  ('healthy' if healthy else 'unhealthy'), flush=True)
            return
    raise RuntimeError('production compass did not become %s' %
                       ('healthy' if healthy else 'unhealthy'))


def wait_for_gps_health(connection, process, log_path, physics, healthy,
                        deadline):
    sensor = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
        if message is None:
            continue
        present = bool(message.onboard_control_sensors_present & sensor)
        reported_healthy = bool(message.onboard_control_sensors_health & sensor)
        if ((healthy and present and reported_healthy) or
                (not healthy and not reported_healthy)):
            print('production GPS became %s' %
                  ('healthy' if healthy else 'unhealthy'), flush=True)
            return
    raise RuntimeError('production GPS did not become %s' %
                       ('healthy' if healthy else 'unhealthy'))


def wait_for_gps_fix(connection, process, log_path, physics, deadline):
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type='GPS_RAW_INT', blocking=True, timeout=1)
        if (message is not None and
                message.fix_type >= mavutil.mavlink.GPS_FIX_TYPE_3D_FIX):
            print('production GPS recovered a 3D fix', flush=True)
            return
    raise RuntimeError('production GPS did not recover a 3D fix')


def wait_for_airspeed_health(connection, process, log_path, physics, healthy,
                             deadline):
    sensor = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
        if message is None:
            continue
        present = bool(message.onboard_control_sensors_present & sensor)
        reported_healthy = bool(message.onboard_control_sensors_health & sensor)
        if present and reported_healthy == healthy:
            print('production airspeed became %s' %
                  ('healthy' if healthy else 'unhealthy'), flush=True)
            return
    raise RuntimeError('production airspeed did not become %s' %
                       ('healthy' if healthy else 'unhealthy'))


def wait_for_barometer_health(connection, process, log_path, physics, healthy,
                              deadline):
    sensor = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type='SYS_STATUS', blocking=True, timeout=1)
        if message is None:
            continue
        present = bool(message.onboard_control_sensors_present & sensor)
        reported_healthy = bool(message.onboard_control_sensors_health & sensor)
        if present and reported_healthy == healthy:
            print('production barometers became %s' %
                  ('healthy' if healthy else 'unhealthy'), flush=True)
            return
    raise RuntimeError('production barometers did not become %s' %
                       ('healthy' if healthy else 'unhealthy'))


def expected_airspeed(truth):
    temperature_k = max(1.0, 288.15 - 0.0065 * truth['altitude_m'])
    atmosphere_base = max(
        0.01, 1.0 - 2.25577e-5 * truth['altitude_m'])
    pressure_pa = 101325.0 * atmosphere_base ** 5.25588
    density = pressure_pa / (287.05 * temperature_k)
    return truth['airspeed_m_s'] * density ** 0.5


def expected_differential_pressure(truth):
    airspeed = expected_airspeed(truth)
    return 0.5 * airspeed * airspeed


def wait_for_airspeed(connection, process, log_path, physics, truth, deadline,
                      description):
    expected = expected_airspeed(truth)
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(type='VFR_HUD', blocking=True, timeout=1)
        if message is None or abs(message.airspeed - expected) > 1.0:
            continue
        print('%s MS4525 airspeed passed: %.2f m/s (expected %.2f)' %
              (description, message.airspeed, expected), flush=True)
        return
    raise RuntimeError('%s MS4525 airspeed did not reach %.2f m/s' %
                       (description, expected))


def wait_for_airspeed_pressures(connection, process, log_path, physics,
                                instances, truth, deadline, description):
    message_for_instance = {1: 'SCALED_PRESSURE', 2: 'SCALED_PRESSURE2'}
    expected_hpa = expected_differential_pressure(truth) * 0.01
    remaining = set(instances)
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type=list(message_for_instance.values()), blocking=True, timeout=1)
        if message is None:
            continue
        for instance in tuple(remaining):
            if (message.get_type() == message_for_instance[instance] and
                    abs(message.press_diff - expected_hpa) <= 0.1):
                remaining.remove(instance)
        if not remaining:
            print('%s airspeed pressures passed for instances %s: %.2f hPa' %
                  (description, sorted(instances), expected_hpa), flush=True)
            return
    raise RuntimeError('%s airspeed pressure did not reach %.2f hPa for %s' %
                       (description, expected_hpa, sorted(remaining)))


def wait_for_barometer(connection, process, log_path, physics, instance, truth,
                       deadline, description):
    message_type = {
        1: 'SCALED_PRESSURE',
        2: 'SCALED_PRESSURE2',
        3: 'SCALED_PRESSURE3',
    }[instance]
    expected_hpa = truth['pressure_pa'] * 0.01
    expected_temperature = round((truth['temperature_k'] - 273.15) * 100)
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type=message_type, blocking=True, timeout=1)
        if (message is not None and
                abs(message.press_abs - expected_hpa) <= 0.2 and
                abs(message.temperature - expected_temperature) <= 2):
            print('%s BMP280 passed: %.2f hPa, %.2f C' %
                  (description, message.press_abs,
                   message.temperature * 0.01), flush=True)
            return
    raise RuntimeError('%s BMP280 did not reach %.2f hPa, %.2f C' %
                       (description, expected_hpa,
                        expected_temperature * 0.01))


def wait_for_rangefinders(connection, process, log_path, physics, devices,
                          truth, deadline):
    sensor_ids = {
        'benewake-rangefinder': 0,
        'lightware-rangefinder': 1,
    }
    expected = {
        sensor_ids[device]: round(truth['rangefinder_m'][physics_index] * 100)
        for device, physics_index in devices.items()
    }
    wait_for_rangefinder_ids(
        connection, process, log_path, physics, expected, deadline)


def wait_for_rangefinder_ids(connection, process, log_path, physics,
                             expected, deadline):
    detected_ids = set()
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type='DISTANCE_SENSOR', blocking=True, timeout=1)
        if (message is None or message.id not in expected or
                message.current_distance != expected[message.id]):
            continue
        detected_ids.add(message.id)
        if set(expected).issubset(detected_ids):
            print('serial rangefinder production drivers reported %s cm' %
                  ', '.join('%u=%u' % item for item in sorted(expected.items())),
                  flush=True)
            return
    raise RuntimeError('serial rangefinders did not report expected values: %s' %
                       sorted(detected_ids))


def wait_for_rangefinder_silence(connection, process, log_path, physics,
                                 sensor_id, deadline):
    quiet_started = time.monotonic()
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type='DISTANCE_SENSOR', blocking=True, timeout=0.25)
        now = time.monotonic()
        if message is not None and message.id == sensor_id:
            quiet_started = now
        if now - quiet_started >= 2.0:
            print('serial rangefinder ID %u stopped after output suppression' %
                  sensor_id, flush=True)
            return
    raise RuntimeError('serial rangefinder ID %u did not stop reporting' %
                       sensor_id)


def monitor_command(monitor, command):
    response = monitor.command(command)
    if 'error' in response.lower():
        raise RuntimeError('Renode monitor rejected %s: %s' %
                           (command, response.strip()))


def monitor_property(monitor, model, property_name):
    response = monitor.command('%s %s' % (model, property_name))
    values = re.findall(r'(?m)^\s*(0x[0-9A-Fa-f]+|[0-9]+)\s*$', response)
    if not values:
        raise RuntimeError('Renode monitor did not return %s.%s: %s' %
                           (model, property_name, response.strip()))
    return int(values[-1], 0)


def monitor_bool_property(monitor, model, property_name):
    response = monitor.command('%s %s' % (model, property_name))
    values = re.findall(r'(?im)^\s*(true|false)\s*$', response)
    if not values:
        raise RuntimeError('Renode monitor did not return %s.%s: %s' %
                           (model, property_name, response.strip()))
    return values[-1].lower() == 'true'


def run_probe(args, root, output_dir):
    profile = DRIVER_PROBE_PROFILES[args.profile]
    assertions = {
        attachment['device']: set(attachment['assertions'])
        for attachment in profile['devices']
    }
    ports = {
        port['id']: port for port in gen_board.configuration_ports(
            root, profile['board'], output_dir / 'hwdef')
    }
    runtime_names = {}
    for index, attachment in enumerate(profile['devices']):
        model_name = 'configDevice%u' % index
        port = ports.get(attachment['port'])
        if port is None:
            raise RuntimeError('profile uses unknown port %s' %
                               attachment['port'])
        if port['bus'] == 'i2c':
            model_name = 'sysbus.%s.%s' % (
                port['peripheral'].lower(), model_name)
        elif port['bus'] == 'uart':
            model_name = 'sysbus.%s' % model_name
        runtime_names[attachment['device']] = model_name
    supported = {
        'asp5033-airspeed', 'auav-airspeed', 'benewake-rangefinder',
        'bmp280-barometer', 'ist8310-compass', 'lightware-rangefinder',
        'ms4525-airspeed', 'ublox-gps',
    }
    unsupported = set(assertions) - supported
    if unsupported:
        raise RuntimeError('no probe checker for %s' %
                           ', '.join(sorted(unsupported)))
    if not args.skip_build:
        build_firmware(root, profile)
    firmware = root / 'build' / profile['board'] / 'bin' / profile['vehicle']
    if not firmware.is_file():
        raise RuntimeError('required firmware does not exist: %s' % firmware)

    state_dir = output_dir / 'state'
    state_dir.mkdir()
    renode_log = output_dir / 'renode.log'
    uart_port = common.unused_tcp_port()
    monitor_port = common.unused_tcp_port()
    while monitor_port == uart_port:
        monitor_port = common.unused_tcp_port()
    physics = ControlledPhysics()
    physics.start()
    command = [
        sys.executable,
        str(root / 'Tools' / 'renode' / 'run.py'),
        profile['board'],
        '--vehicle', profile['vehicle'],
        '--firmware', str(firmware),
        '--state-dir', str(state_dir),
        '--uart-port', str(uart_port),
        '--port', str(monitor_port),
        '--unthrottled',
        '--exec',
        'sysbus.physics Connect %u "stationary" %.7f %.7f %.1f 0 %u' % (
            physics.port,
            BASELINE['latitude_deg'],
            BASELINE['longitude_deg'],
            BASELINE['altitude_m'],
            PHYSICS_RATE_HZ,
        ),
    ]
    for attachment in profile['devices']:
        if attachment['device'] not in ATTACHABLE_DEVICES:
            raise RuntimeError('profile uses unknown device %s' %
                               attachment['device'])
        specification = {
            'device': attachment['device'],
            'port': attachment['port'],
        }
        physics_metadata = ATTACHABLE_DEVICES[attachment['device']].get(
            'physics')
        if physics_metadata is not None:
            specification['physics_index'] = attachment['instance'] - 1
        command.extend([
            '--device',
            json.dumps(specification, sort_keys=True, separators=(',', ':')),
        ])
    if args.renode:
        command.extend(['--renode', args.renode])
    if args.data_cache:
        command.extend(['--data-cache', args.data_cache])

    environment = os.environ.copy()
    environment['XDG_CONFIG_HOME'] = str(state_dir)
    environment['TMPDIR'] = str(state_dir)
    connection = None
    monitor = None
    process = None
    try:
        with renode_log.open('w') as log:
            process = subprocess.Popen(
                command,
                cwd=root,
                env=environment,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        deadline = time.monotonic() + args.timeout
        connection = common.connect_mavlink(
            uart_port, process, renode_log, deadline)
        monitor = MonitorClient('127.0.0.1', monitor_port)
        monitor.connect()
        connection.mav.request_data_stream_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            5,
            1,
        )
        if 'detection' in assertions.get('ublox-gps', ()):
            wait_for_ublox(connection, process, renode_log, deadline)
        if 'device-id' in assertions.get('ist8310-compass', ()):
            find_ist8310(connection, process, renode_log, deadline)
        bmp280_instance = None
        if 'device-id' in assertions.get('bmp280-barometer', ()):
            bmp280_instance = find_bmp280(
                connection, process, renode_log, deadline)
        for attachment in profile['devices']:
            device = attachment['device']
            if ('device-id' in assertions.get(device, ()) and
                    device in AIRSPEED_DEVICE_IDS):
                find_airspeed_device(
                    connection, process, renode_log, device,
                    attachment['instance'], deadline)
        value_devices = {
            device_id for device_id, checks in assertions.items()
            if ('stable-values' in checks and
                device_id in ('ist8310-compass', 'ublox-gps'))
        }
        if value_devices:
            wait_for_values(
                connection, process, renode_log, physics,
                BASELINE, value_devices, deadline, 'baseline')
        rangefinder_devices = {
            attachment['device']: attachment['instance'] - 1
            for attachment in profile['devices']
            if ('stable-values' in assertions[attachment['device']] and
                attachment['device'] in ('benewake-rangefinder',
                                         'lightware-rangefinder'))
        }
        rangefinder_sensor_ids = {
            'benewake-rangefinder': 0,
            'lightware-rangefinder': 1,
        }
        if rangefinder_devices:
            wait_for_rangefinders(
                connection, process, renode_log, physics,
                rangefinder_devices, BASELINE, deadline)
        for device, sensor_id in rangefinder_sensor_ids.items():
            if ('output-suppression-recovery' not in
                    assertions.get(device, ())):
                continue
            model = runtime_names[device]
            monitor_command(monitor, '%s SuppressOutput true' % model)
            if not monitor_bool_property(monitor, model, 'SuppressOutput'):
                raise RuntimeError(
                    'Renode did not suppress %s output' % device)
            wait_for_rangefinder_silence(
                connection, process, renode_log, physics,
                sensor_id, deadline)
            monitor_command(monitor, '%s SuppressOutput false' % model)
            if monitor_bool_property(monitor, model, 'SuppressOutput'):
                raise RuntimeError(
                    'Renode did not restore %s output' % device)
            wait_for_rangefinder_ids(
                connection, process, renode_log, physics,
                {sensor_id: round(
                    BASELINE['rangefinder_m'][rangefinder_devices[device]] *
                    100)}, deadline)
        if 'stable-values' in assertions.get('ms4525-airspeed', ()):
            wait_for_airspeed_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_airspeed(
                connection, process, renode_log, physics,
                BASELINE, deadline, 'baseline')
        pressure_instances = {
            attachment['instance'] for attachment in profile['devices']
            if (attachment['device'] in ('asp5033-airspeed', 'auav-airspeed') and
                'stable-values' in assertions[attachment['device']])
        }
        if pressure_instances:
            wait_for_airspeed_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_airspeed_pressures(
                connection, process, renode_log, physics,
                pressure_instances, BASELINE, deadline, 'baseline')
        if ('stable-values' in assertions.get('bmp280-barometer', ()) and
                bmp280_instance is not None):
            wait_for_barometer_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_barometer(
                connection, process, renode_log, physics, bmp280_instance,
                BASELINE, deadline, 'baseline')
        stepped_devices = {
            device_id for device_id, checks in assertions.items()
            if 'stepped-values' in checks
        }
        if stepped_devices:
            physics.stepped.set()
        stepped_navigation_devices = stepped_devices.intersection(
            ('ist8310-compass', 'ublox-gps'))
        if stepped_navigation_devices:
            wait_for_values(
                connection, process, renode_log, physics,
                STEPPED, stepped_navigation_devices, deadline, 'stepped')
        if 'ms4525-airspeed' in stepped_devices:
            wait_for_airspeed(
                connection, process, renode_log, physics,
                STEPPED, deadline, 'stepped')
        stepped_pressure_instances = {
            attachment['instance'] for attachment in profile['devices']
            if attachment['device'] in stepped_devices and
            attachment['device'] in ('asp5033-airspeed', 'auav-airspeed')
        }
        if stepped_pressure_instances:
            wait_for_airspeed_pressures(
                connection, process, renode_log, physics,
                stepped_pressure_instances, STEPPED, deadline, 'stepped')
        if ('bmp280-barometer' in stepped_devices and
                bmp280_instance is not None):
            wait_for_barometer(
                connection, process, renode_log, physics, bmp280_instance,
                STEPPED, deadline, 'stepped')
        stepped_rangefinders = {
            device: physics_index
            for device, physics_index in rangefinder_devices.items()
            if device in stepped_devices
        }
        if stepped_rangefinders:
            wait_for_rangefinders(
                connection, process, renode_log, physics,
                stepped_rangefinders, STEPPED, deadline)
        if 'corrupt-read-recovery' in assertions.get('ist8310-compass', ()):
            model = runtime_names['ist8310-compass']
            monitor_command(monitor, '%s ReadXorMask 170' % model)
            if monitor_property(monitor, model, 'ReadXorMask') != 170:
                raise RuntimeError('Renode did not enable I2C read corruption')
            wait_for_compass_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s ReadXorMask 0' % model)
            if monitor_property(monitor, model, 'ReadXorMask') != 0:
                raise RuntimeError('Renode did not clear I2C read corruption')
            wait_for_compass_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_values(
                connection, process, renode_log, physics,
                STEPPED, {'ist8310-compass'}, deadline, 'recovered')
        if 'corrupt-read-recovery' in assertions.get('ms4525-airspeed', ()):
            model = runtime_names['ms4525-airspeed']
            monitor_command(monitor, '%s ReadXorMask 192' % model)
            if monitor_property(monitor, model, 'ReadXorMask') != 192:
                raise RuntimeError('Renode did not enable airspeed corruption')
            wait_for_airspeed_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s ReadXorMask 0' % model)
            if monitor_property(monitor, model, 'ReadXorMask') != 0:
                raise RuntimeError('Renode did not clear airspeed corruption')
            wait_for_airspeed_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_airspeed(
                connection, process, renode_log, physics,
                STEPPED, deadline, 'recovered')
        airspeed_fault_masks = {
            'asp5033-airspeed': 8,
            'auav-airspeed': 32,
        }
        for device, mask in airspeed_fault_masks.items():
            if 'corrupt-read-recovery' not in assertions.get(device, ()):
                continue
            model = runtime_names[device]
            monitor_command(monitor, '%s ReadXorMask %u' % (model, mask))
            if monitor_property(monitor, model, 'ReadXorMask') != mask:
                raise RuntimeError(
                    'Renode did not enable %s corruption' % device)
            wait_for_airspeed_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s ReadXorMask 0' % model)
            if monitor_property(monitor, model, 'ReadXorMask') != 0:
                raise RuntimeError('Renode did not clear %s corruption' % device)
            wait_for_airspeed_health(
                connection, process, renode_log, physics, True, deadline)
            instance = next(
                attachment['instance'] for attachment in profile['devices']
                if attachment['device'] == device)
            wait_for_airspeed_pressures(
                connection, process, renode_log, physics,
                {instance}, STEPPED, deadline, 'recovered %s' % device)
        if ('stuck-sample-recovery' in
                assertions.get('bmp280-barometer', ())):
            model = runtime_names['bmp280-barometer']
            monitor_command(monitor, '%s FreezeSample true' % model)
            if not monitor_bool_property(monitor, model, 'FreezeSample'):
                raise RuntimeError('Renode did not freeze BMP280 samples')
            wait_for_barometer_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s FreezeSample false' % model)
            if monitor_bool_property(monitor, model, 'FreezeSample'):
                raise RuntimeError('Renode did not resume BMP280 samples')
            wait_for_barometer_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_barometer(
                connection, process, renode_log, physics, bmp280_instance,
                STEPPED, deadline, 'recovered')
        if 'output-suppression-recovery' in assertions.get('ublox-gps', ()):
            model = runtime_names['ublox-gps']
            monitor_command(monitor, '%s SuppressOutput true' % model)
            if not monitor_bool_property(monitor, model, 'SuppressOutput'):
                raise RuntimeError('Renode did not suppress u-blox output')
            wait_for_gps_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s SuppressOutput false' % model)
            if monitor_bool_property(monitor, model, 'SuppressOutput'):
                raise RuntimeError('Renode did not restore u-blox output')
            wait_for_gps_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_gps_fix(
                connection, process, renode_log, physics, deadline)
            wait_for_values(
                connection, process, renode_log, physics,
                STEPPED, {'ublox-gps'}, deadline, 'recovered after silence')
        if 'output-corruption-recovery' in assertions.get('ublox-gps', ()):
            model = runtime_names['ublox-gps']
            monitor_command(monitor, '%s OutputXorMask 255' % model)
            if monitor_property(monitor, model, 'OutputXorMask') != 255:
                raise RuntimeError('Renode did not corrupt u-blox output')
            wait_for_gps_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s OutputXorMask 0' % model)
            if monitor_property(monitor, model, 'OutputXorMask') != 0:
                raise RuntimeError('Renode did not clear u-blox corruption')
            wait_for_gps_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_gps_fix(
                connection, process, renode_log, physics, deadline)
            wait_for_values(
                connection, process, renode_log, physics,
                STEPPED, {'ublox-gps'}, deadline,
                'recovered after corruption')
    finally:
        if monitor is not None:
            monitor.close()
        if connection is not None:
            with contextlib.suppress(OSError):
                connection.close()
        if process is not None:
            common.stop_process_group(process)
        physics.stop()


def main(argv=None):
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'profile', nargs='?', default=DEFAULT_PROFILE,
        choices=sorted(DRIVER_PROBE_PROFILES),
        help='catalog probe profile (default: %(default)s)')
    parser.add_argument('--renode', help='Renode executable passed to run.py')
    parser.add_argument('--data-cache', help='directory for downloaded Renode model data')
    parser.add_argument('--skip-build', action='store_true')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--timeout', type=common.positive_float, default=180)
    args = parser.parse_args(argv)
    default_renode = root / 'build' / 'renode' / 'renode'
    if args.renode is None and default_renode.is_file():
        args.renode = str(default_renode)
    local_data_cache = root / 'build' / 'renode-data-all-mcus-20260825'
    if args.data_cache is None and local_data_cache.is_dir():
        args.data_cache = str(local_data_cache)
    if args.output_dir is None:
        output_root = root / 'build' / 'renode-test'
        output_root.mkdir(parents=True, exist_ok=True)
        output_dir = output_root / ('driver-probe-' + time.strftime('%Y%m%d-%H%M%S'))
    else:
        output_dir = args.output_dir.resolve()
    if output_dir.exists():
        parser.error('--output-dir already exists: %s' % output_dir)
    output_dir.mkdir(parents=True)

    try:
        run_probe(args, root, output_dir)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print('Renode driver probe failed: %s' % error, file=sys.stderr)
        print('test artifacts: %s' % output_dir, file=sys.stderr)
        return 1
    print('Renode driver probe passed', flush=True)
    print('test artifacts: %s' % output_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
