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

from pymavlink import DFReader
from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import gen_board  # noqa: E402
import physics_stub  # noqa: E402
import test_mission as common  # noqa: E402

from driver_catalog import ATTACHABLE_DEVICES  # noqa: E402
from driver_catalog import DRIVER_PROBE_PROFILES  # noqa: E402
from driver_catalog import i2c_endpoints  # noqa: E402
from launch import MonitorClient  # noqa: E402

DEFAULT_PROFILE = 'matekh743-navigation'
IST8310_ADDRESS = 0x0E
IST8310_DEVTYPE = 0x0A
COMPASS_DEVICE_IDS = {
    'ak09916-compass': (0x0C, 0x09),
    'ak8963-compass': (0x0C, 0x04),
    'bmm150-compass': (0x10, 0x05),
    'bmm350-compass': (0x14, 0x17),
    'hmc5843-compass': (0x1E, 0x07),
    'iis2mdc-compass': (0x1E, 0x18),
    'ist8308-compass': (0x0E, 0x10),
    'ist8310-compass': (IST8310_ADDRESS, IST8310_DEVTYPE),
    'lis3mdl-compass': (0x1C, 0x08),
    'lsm303d-compass': (0x1D, 0x02),
    'lsm9ds1-compass': (0x1C, 0x06),
    'mag3110-compass': (0x0E, 0x0E),
    'mmc3416-compass': (0x30, 0x0C),
    'mmc5983-compass': (0x30, 0x13),
    'qmc5883l-compass': (0x0D, 0x0D),
    'qmc5883p-compass': (0x2C, 0x16),
    'rm3100-compass': (0x20, 0x11),
}
AIRSPEED_DEVICE_IDS = {
    'ms4525-airspeed': (0x28, 0x02),
    'asp5033-airspeed': (0x6C, 0x0A),
    'auav-airspeed': (0x26, 0x0B),
    'dlvr-airspeed': (0x28, 0x04),
    'ms5525-airspeed': (0x76, 0x03),
    'sdp3x-airspeed': (0x21, 0x06),
}
BAROMETER_DEVICE_IDS = {
    'auav-barometer': (0x27, 0x17),
    'bmp085-barometer': (0x77, 0x02),
    'bmp280-barometer': (0x76, 0x03),
    'bmp388-barometer': (0x76, 0x04),
    'bmp581-barometer': (0x46, 0x15),
    'dps280-barometer': (0x76, 0x05),
    'icm20789-package': (0x63, 0x08),
    'icp101xx-barometer': (0x63, 0x0F),
    'icp201xx-barometer': (0x63, 0x10),
    'keller-barometer': (0x40, 0x09),
    'lps2xh-barometer': (0x5D, 0x0A),
    'ms5611-barometer': (0x77, 0x0B),
    'spl06-barometer': (0x76, 0x0C),
}
TEMPERATURE_DEVICES = {
    'mcp9600-temperature',
    'mlx90614-temperature',
    'sht3x-temperature',
    'tmp119-temperature',
    'tsys01-temperature',
    'tsys03-temperature',
}
LED_DEVICES = {
    'is31fl3195-led',
    'lp5562-led',
    'ncp5623-led',
    'pca9685-led',
    'toshiba-led',
}
DISPLAY_DEVICES = {
    'sh1106-display',
    'ssd1306-display',
}
POWER_MONITOR_DEVICES = {
    'ad7091r5-power-monitor',
    'bq76952-bms',
    'ina226-power-monitor',
    'ina228-power-monitor',
    'ina238-power-monitor',
    'ina231-power-monitor',
    'ina260-power-monitor',
    'ina3221-power-monitor',
    'ltc2946-power-monitor',
    'smbus-generic-battery',
    'smbus-maxell-battery',
    'smbus-neodesign-battery',
    'smbus-rotoye-battery',
    'smbus-solo-battery',
    'smbus-sui3-battery',
    'smbus-sui6-battery',
}
CELL_VOLTAGE_POWER_DEVICES = {
    'bq76952-bms',
    'smbus-generic-battery',
    'smbus-maxell-battery',
    'smbus-neodesign-battery',
    'smbus-rotoye-battery',
    'smbus-solo-battery',
    'smbus-sui3-battery',
    'smbus-sui6-battery',
}
TEMPERATURE_POWER_DEVICES = {
    'bq76952-bms',
    'ina228-power-monitor',
    'ina238-power-monitor',
    'smbus-generic-battery',
    'smbus-maxell-battery',
    'smbus-neodesign-battery',
    'smbus-rotoye-battery',
    'smbus-solo-battery',
    'smbus-sui3-battery',
    'smbus-sui6-battery',
}
POWER_MONITOR_CELL_FRACTIONS = {
    'bq76952-bms': (1.0 / 6.0,) * 6,
    'smbus-solo-battery': (1.0 / 6.0,) * 6,
    'smbus-sui6-battery': (0.10, 0.12, 0.14, 0.16, 0.24, 0.24),
}
I2C_RANGEFINDER_DEVICES = {
    'lidarlite-i2c-rangefinder',
    'lightware-i2c-rangefinder',
    'lightware-grf-i2c-rangefinder',
    'maxsonar-i2c-rangefinder',
    'teraranger-i2c-rangefinder',
    'tfmini-plus-i2c-rangefinder',
    'tfs20l-i2c-rangefinder',
    'tofsensef-i2c-rangefinder',
    'vl53l0x-i2c-rangefinder',
    'vl53l1x-i2c-rangefinder',
}
RANGEFINDER_DEVICES = I2C_RANGEFINDER_DEVICES.union((
    'benewake-rangefinder',
    'lightware-rangefinder',
))
INVENSENSE_I2C_ADDRESS = 0x68
INVENSENSE_ACCEL_DEVTYPE = 0x13
INVENSENSE_GYRO_DEVTYPE = 0x21
BMI160_I2C_ADDRESS = 0x68
BMI160_DEVTYPE = 0x09
BMI270_I2C_ADDRESS = 0x68
BMI270_DEVTYPE = 0x38
ICM20948_I2C_ADDRESS = 0x68
ICM20948_DEVTYPE = 0x2C
ICM20789_I2C_ADDRESS = 0x68
ICM20789_DEVTYPE = 0x27
PHYSICS_RATE_HZ = 100
BASELINE = {
    'latitude_deg': -35.363261,
    'longitude_deg': 149.165230,
    'altitude_m': 584.0,
    'airspeed_m_s': 0.0,
    'magnetic_field_body_mgauss': (201.0, 0.0, 450.0),
    'rangefinder_m': tuple(2.0 + index * 0.1 for index in range(10)),
    'pressure_pa': 101325.0,
    'temperature_k': 293.15,
    'battery_voltage_v': 12.0,
    'battery_current_a': 3.0,
}
STEPPED = {
    'latitude_deg': -35.362261,
    'longitude_deg': 149.164230,
    'altitude_m': 614.0,
    'airspeed_m_s': 20.0,
    'magnetic_field_body_mgauss': (350.0, -120.0, 80.0),
    'rangefinder_m': tuple(3.0 + index * 0.1 for index in range(10)),
    'pressure_pa': 90000.0,
    'temperature_k': 303.15,
    'battery_voltage_v': 24.0,
    'battery_current_a': 12.0,
}
IMU_BASELINE = {
    'gyro_rad_s': (0.12, -0.23, 0.34),
    'specific_force_m_s2': (1.25, -2.50, -7.50),
}
IMU_STEPPED = {
    'gyro_rad_s': (-0.31, 0.17, -0.09),
    'specific_force_m_s2': (-3.00, 4.25, -6.50),
}
OPTICAL_FLOW_BASELINE = {
    'gyro_rad_s': (0.10, -0.20, 0.0),
    'velocity_ned_m_s': (2.0, -1.0, 0.0),
}
OPTICAL_FLOW_STEPPED = {
    'gyro_rad_s': (-0.15, 0.25, 0.0),
    'velocity_ned_m_s': (-3.0, 1.5, 0.0),
}


class ControlledPhysics:
    """Serve stationary truth which the test can switch deterministically."""

    def __init__(self, imu_motion=False, optical_flow_motion=False):
        self.server = socket.create_server(
            ('127.0.0.1', 0), family=socket.AF_INET, backlog=1)
        self.server.settimeout(0.5)
        self.port = self.server.getsockname()[1]
        self.stepped = threading.Event()
        self.stopping = threading.Event()
        self.connection = None
        self.error = None
        self.imu_motion = imu_motion
        self.optical_flow_motion = optical_flow_motion
        self.thread = threading.Thread(
            target=self._serve, name='Renode driver-probe physics', daemon=True)

    def start(self):
        self.thread.start()

    def _truth(self, configuration, step):
        truth = physics_stub._truth_for_step(configuration, step)
        values = STEPPED if self.stepped.is_set() else BASELINE
        if self.imu_motion:
            imu_values = IMU_STEPPED if self.stepped.is_set() else IMU_BASELINE
            values = dict(values, **imu_values)
        if self.optical_flow_motion:
            flow_values = (OPTICAL_FLOW_STEPPED if self.stepped.is_set()
                           else OPTICAL_FLOW_BASELINE)
            values = dict(values, **flow_values)
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
    configure = [
        './waf', 'configure', '--board', profile['board'],
        '--default-parameters', str(defaults),
    ]
    if 'extra_hwdef' in profile:
        configure.extend(['--extra-hwdef', str(root / profile['extra_hwdef'])])
    subprocess.run(
        configure,
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


def find_compass_device(connection, process, log_path, device, expected_bus,
                        deadline):
    address, devtype = COMPASS_DEVICE_IDS[device]
    parameters = ('COMPASS_DEV_ID', 'COMPASS_DEV_ID2', 'COMPASS_DEV_ID3')
    device_ids = []
    while time.monotonic() < deadline:
        device_ids = []
        for name in parameters:
            try:
                device_id = read_parameter(
                    connection, process, log_path, name,
                    min(deadline, time.monotonic() + 2))
            except RuntimeError:
                continue
            device_ids.append(device_id)
            decoded = decode_device_id(device_id)
            if (decoded['bus_type'] == 1 and
                    decoded['bus'] == expected_bus and
                    decoded['address'] == address and
                    decoded['devtype'] == devtype):
                print(
                    '%s production driver detected on I2C bus %u at 0x%02X' %
                    (ATTACHABLE_DEVICES[device]['name'], decoded['bus'],
                     decoded['address']),
                    flush=True,
                )
                return
    raise RuntimeError('%s device ID not found in %s' %
                       (ATTACHABLE_DEVICES[device]['name'], device_ids))


def find_airspeed_device(connection, process, log_path, device, instance,
                         expected_bus, deadline):
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
                last_decoded['bus'] == expected_bus and
                last_decoded['address'] == address and
                last_decoded['devtype'] == devtype):
            print('%s production driver detected on I2C bus %u at 0x%02X' %
                  (ATTACHABLE_DEVICES[device]['name'], last_decoded['bus'],
                   last_decoded['address']), flush=True)
            return
    raise RuntimeError('%s device ID is invalid: %s' %
                       (ATTACHABLE_DEVICES[device]['name'], last_decoded))


def find_barometer_device(connection, process, log_path, device, expected_bus,
                          deadline):
    address, devtype = BAROMETER_DEVICE_IDS[device]
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
                    decoded['bus'] == expected_bus and
                    decoded['address'] == address and
                    decoded['devtype'] == devtype):
                print('%s production driver detected as barometer %u on I2C '
                      'bus %u at 0x%02X' %
                      (ATTACHABLE_DEVICES[device]['name'], instance,
                       decoded['bus'], decoded['address']),
                      flush=True)
                return instance
    raise RuntimeError('%s device ID not found in %s' %
                       (ATTACHABLE_DEVICES[device]['name'], last_devices))


def find_invensense_i2c(connection, process, log_path, expected_bus, deadline,
                        address=INVENSENSE_I2C_ADDRESS,
                        accel_devtype=INVENSENSE_ACCEL_DEVTYPE,
                        gyro_devtype=INVENSENSE_GYRO_DEVTYPE,
                        name='Invensense MPU6000'):
    devices = (
        ('accelerometer', accel_devtype,
         ('INS_ACC_ID', 'INS_ACC2_ID', 'INS_ACC3_ID')),
        ('gyroscope', gyro_devtype,
         ('INS_GYR_ID', 'INS_GYR2_ID', 'INS_GYR3_ID')),
    )
    matched_instance = None
    for kind, devtype, parameters in devices:
        last_devices = []
        while time.monotonic() < deadline:
            last_devices = []
            for instance, parameter in enumerate(parameters):
                try:
                    device_id = read_parameter(
                        connection, process, log_path, parameter,
                        min(deadline, time.monotonic() + 2))
                except RuntimeError:
                    continue
                decoded = decode_device_id(device_id)
                last_devices.append(decoded)
                if (decoded['bus_type'] == 1 and
                        decoded['bus'] == expected_bus and
                        decoded['address'] == address and
                        decoded['devtype'] == devtype):
                    if matched_instance is not None and instance != matched_instance:
                        raise RuntimeError(
                            '%s I2C accel/gyro instance mismatch: %u/%u' %
                            (name, matched_instance, instance))
                    matched_instance = instance
                    break
            else:
                continue
            break
        else:
            raise RuntimeError('%s I2C %s ID not found in %s' %
                               (name, kind, last_devices))
    print('%s production driver detected on I2C bus %u at 0x%02X' %
          (name, expected_bus, address), flush=True)
    return matched_instance


def find_bmi160_i2c(connection, process, log_path, expected_bus, deadline):
    parameters = (
        ('INS_ACC_ID', 'INS_ACC2_ID', 'INS_ACC3_ID'),
        ('INS_GYR_ID', 'INS_GYR2_ID', 'INS_GYR3_ID'),
    )
    matched_instance = None
    for names in parameters:
        last_devices = []
        while time.monotonic() < deadline:
            last_devices = []
            for instance, name in enumerate(names):
                try:
                    device_id = read_parameter(
                        connection, process, log_path, name,
                        min(deadline, time.monotonic() + 2))
                except RuntimeError:
                    continue
                decoded = decode_device_id(device_id)
                last_devices.append(decoded)
                if (decoded['bus_type'] == 1 and
                        decoded['bus'] == expected_bus and
                        decoded['address'] == BMI160_I2C_ADDRESS and
                        decoded['devtype'] == BMI160_DEVTYPE):
                    if (matched_instance is not None and
                            instance != matched_instance):
                        raise RuntimeError(
                            'BMI160 I2C accel/gyro instance mismatch: %u/%u' %
                            (matched_instance, instance))
                    matched_instance = instance
                    break
            else:
                continue
            break
        else:
            raise RuntimeError('BMI160 I2C device ID not found in %s' %
                               last_devices)
    print('Bosch BMI160 production driver detected on I2C bus %u at 0x%02X' %
          (expected_bus, BMI160_I2C_ADDRESS), flush=True)
    return matched_instance


def find_bmi270_i2c(connection, process, log_path, expected_bus, deadline):
    parameters = (
        ('INS_ACC_ID', 'INS_ACC2_ID', 'INS_ACC3_ID'),
        ('INS_GYR_ID', 'INS_GYR2_ID', 'INS_GYR3_ID'),
    )
    matched_instance = None
    for names in parameters:
        last_devices = []
        while time.monotonic() < deadline:
            last_devices = []
            for instance, name in enumerate(names):
                try:
                    device_id = read_parameter(
                        connection, process, log_path, name,
                        min(deadline, time.monotonic() + 2))
                except RuntimeError:
                    continue
                decoded = decode_device_id(device_id)
                last_devices.append(decoded)
                if (decoded['bus_type'] == 1 and
                        decoded['bus'] == expected_bus and
                        decoded['address'] == BMI270_I2C_ADDRESS and
                        decoded['devtype'] == BMI270_DEVTYPE):
                    if (matched_instance is not None and
                            instance != matched_instance):
                        raise RuntimeError(
                            'BMI270 I2C accel/gyro instance mismatch: %u/%u' %
                            (matched_instance, instance))
                    matched_instance = instance
                    break
            else:
                continue
            break
        else:
            raise RuntimeError('BMI270 I2C device ID not found in %s' %
                               last_devices)
    print('Bosch BMI270 production driver detected on I2C bus %u at 0x%02X' %
          (expected_bus, BMI270_I2C_ADDRESS), flush=True)
    return matched_instance


def find_icm20948_i2c(connection, process, log_path, expected_bus, deadline):
    parameters = (
        ('INS_ACC_ID', 'INS_ACC2_ID', 'INS_ACC3_ID'),
        ('INS_GYR_ID', 'INS_GYR2_ID', 'INS_GYR3_ID'),
    )
    matched_instance = None
    for names in parameters:
        last_devices = []
        while time.monotonic() < deadline:
            last_devices = []
            for instance, name in enumerate(names):
                try:
                    device_id = read_parameter(
                        connection, process, log_path, name,
                        min(deadline, time.monotonic() + 2))
                except RuntimeError:
                    continue
                decoded = decode_device_id(device_id)
                last_devices.append(decoded)
                if (decoded['bus_type'] == 1 and
                        decoded['bus'] == expected_bus and
                        decoded['address'] == ICM20948_I2C_ADDRESS and
                        decoded['devtype'] == ICM20948_DEVTYPE):
                    if (matched_instance is not None and
                            instance != matched_instance):
                        raise RuntimeError(
                            'ICM20948 I2C accel/gyro instance mismatch: %u/%u' %
                            (matched_instance, instance))
                    matched_instance = instance
                    break
            else:
                continue
            break
        else:
            raise RuntimeError('ICM20948 I2C device ID not found in %s' %
                               last_devices)
    print('Invensense ICM20948 production driver detected on I2C bus %u at '
          '0x%02X' % (expected_bus, ICM20948_I2C_ADDRESS), flush=True)
    return matched_instance


def wait_for_invensense_values(connection, process, log_path, physics,
                               instance, truth, deadline, description,
                               name='Invensense MPU6000'):
    message_type = ('RAW_IMU', 'SCALED_IMU2', 'SCALED_IMU3')[instance]
    acceleration = truth['specific_force_m_s2']
    gyro = truth['gyro_rad_s']
    expected_accel = tuple(value * 1000.0 / 9.80665 for value in acceleration)
    expected_gyro = tuple(value * 1000.0 for value in gyro)
    expected_temperature = round((truth['temperature_k'] - 273.15) * 100.0)
    last = None
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type=message_type, blocking=True, timeout=1)
        if message is None:
            continue
        actual_accel = (message.xacc, message.yacc, message.zacc)
        actual_gyro = (message.xgyro, message.ygyro, message.zgyro)
        last = (actual_accel, actual_gyro, message.temperature)
        if (all(abs(actual - expected) <= 20
                for actual, expected in zip(actual_accel, expected_accel)) and
                all(abs(actual - expected) <= 10
                    for actual, expected in zip(actual_gyro, expected_gyro)) and
                abs(message.temperature - expected_temperature) <= 30):
            print('%s %s IMU%u values passed: accel %s, '
                  'gyro %s, %.2f C' %
                  (description, name, instance + 1, actual_accel, actual_gyro,
                   message.temperature * 0.01), flush=True)
            return
    raise RuntimeError(
        '%s %s IMU%u values did not reach accel %s, gyro %s, '
        'temperature %.2f C; last %s' %
        (description, name, instance + 1, expected_accel, expected_gyro,
         expected_temperature * 0.01, last))


def wait_for_bmi160_values(connection, process, log_path, physics, instance,
                           truth, deadline, description):
    message_type = ('RAW_IMU', 'SCALED_IMU2', 'SCALED_IMU3')[instance]
    acceleration = truth['specific_force_m_s2']
    gyro = truth['gyro_rad_s']
    expected_accel = tuple(value * 1000.0 / 9.80665 for value in acceleration)
    expected_gyro = tuple(value * 1000.0 for value in gyro)
    last = None
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type=message_type, blocking=True, timeout=1)
        if message is None:
            continue
        actual_accel = (message.xacc, message.yacc, message.zacc)
        actual_gyro = (message.xgyro, message.ygyro, message.zgyro)
        last = (actual_accel, actual_gyro)
        if (all(abs(actual - expected) <= 20
                for actual, expected in zip(actual_accel, expected_accel)) and
                all(abs(actual - expected) <= 10
                    for actual, expected in zip(actual_gyro, expected_gyro))):
            print('%s Bosch BMI160 IMU%u values passed: accel %s, gyro %s' %
                  (description, instance + 1, actual_accel, actual_gyro),
                  flush=True)
            return
    raise RuntimeError(
        '%s Bosch BMI160 IMU%u values did not reach accel %s, gyro %s; '
        'last %s' % (description, instance + 1, expected_accel,
                     expected_gyro, last))


def wait_for_bmi270_values(connection, process, log_path, physics, instance,
                           truth, deadline, description):
    message_type = ('RAW_IMU', 'SCALED_IMU2', 'SCALED_IMU3')[instance]
    acceleration = truth['specific_force_m_s2']
    gyro = truth['gyro_rad_s']
    expected_accel = tuple(value * 1000.0 / 9.80665 for value in acceleration)
    expected_gyro = tuple(value * 1000.0 for value in gyro)
    expected_temperature = round((truth['temperature_k'] - 273.15) * 100.0)
    last = None
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type=message_type, blocking=True, timeout=1)
        if message is None:
            continue
        actual_accel = (message.xacc, message.yacc, message.zacc)
        actual_gyro = (message.xgyro, message.ygyro, message.zgyro)
        last = (actual_accel, actual_gyro, message.temperature)
        if (all(abs(actual - expected) <= 20
                for actual, expected in zip(actual_accel, expected_accel)) and
                all(abs(actual - expected) <= 10
                    for actual, expected in zip(actual_gyro, expected_gyro)) and
                abs(message.temperature - expected_temperature) <= 30):
            print('%s Bosch BMI270 IMU%u values passed: accel %s, gyro %s, '
                  '%.2f C' %
                  (description, instance + 1, actual_accel, actual_gyro,
                   message.temperature * 0.01), flush=True)
            return
    raise RuntimeError(
        '%s Bosch BMI270 IMU%u values did not reach accel %s, gyro %s, '
        'temperature %.2f C; last %s' %
        (description, instance + 1, expected_accel, expected_gyro,
         expected_temperature * 0.01, last))


def wait_for_icm20948_values(connection, process, log_path, physics, instance,
                             truth, deadline, description):
    message_type = ('RAW_IMU', 'SCALED_IMU2', 'SCALED_IMU3')[instance]
    acceleration = truth['specific_force_m_s2']
    gyro = truth['gyro_rad_s']
    expected_accel = tuple(value * 1000.0 / 9.80665 for value in acceleration)
    expected_gyro = tuple(value * 1000.0 for value in gyro)
    expected_temperature = round((truth['temperature_k'] - 273.15) * 100.0)
    last = None
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type=message_type, blocking=True, timeout=1)
        if message is None:
            continue
        actual_accel = (message.xacc, message.yacc, message.zacc)
        actual_gyro = (message.xgyro, message.ygyro, message.zgyro)
        last = (actual_accel, actual_gyro, message.temperature)
        if (all(abs(actual - expected) <= 20
                for actual, expected in zip(actual_accel, expected_accel)) and
                all(abs(actual - expected) <= 10
                    for actual, expected in zip(actual_gyro, expected_gyro)) and
                abs(message.temperature - expected_temperature) <= 30):
            print('%s Invensense ICM20948 IMU%u values passed: accel %s, '
                  'gyro %s, %.2f C' %
                  (description, instance + 1, actual_accel, actual_gyro,
                   message.temperature * 0.01), flush=True)
            return
    raise RuntimeError(
        '%s Invensense ICM20948 IMU%u values did not reach accel %s, gyro %s, '
        'temperature %.2f C; last %s' %
        (description, instance + 1, expected_accel, expected_gyro,
         expected_temperature * 0.01, last))


def expected_led_values(device, values):
    if device == 'ncp5623-led':
        return tuple((value * 255 // 31) & 0x1F for value in values)
    if device == 'toshiba-led':
        return tuple(value >> 4 for value in values)
    return values


def set_and_wait_for_led(connection, process, log_path, physics, monitor,
                         model, device, values, deadline, description):
    payload = list(values) + [0] * 21
    expected = expected_led_values(device, values)
    connection.mav.led_control_send(
        connection.target_system,
        connection.target_component,
        0,
        mavutil.mavlink.LED_CONTROL_PATTERN_CUSTOM,
        3,
        payload,
    )
    last = None
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        if not monitor_bool_property(monitor, model, 'Initialized'):
            time.sleep(0.05)
            continue
        last = tuple(monitor_property(monitor, model, color)
                     for color in ('Red', 'Green', 'Blue'))
        if last == expected:
            print('%s %s output passed: RGB %s' %
                  (description, ATTACHABLE_DEVICES[device]['name'], last),
                  flush=True)
            return
        time.sleep(0.05)
    raise RuntimeError('%s %s output did not reach RGB %s; last %s' %
                       (description, ATTACHABLE_DEVICES[device]['name'],
                        expected, last))


def set_and_wait_for_oreoled(connection, process, log_path, physics, monitor,
                             models, values, deadline, description):
    custom = list(b'RGB0') + [2] + list(values)
    payload = custom + [0] * (24 - len(custom))
    connection.mav.led_control_send(
        connection.target_system,
        connection.target_component,
        0xFF,
        mavutil.mavlink.LED_CONTROL_PATTERN_CUSTOM,
        len(custom),
        payload,
    )
    last = None
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        states = []
        for model in models:
            initialized = monitor_bool_property(monitor, model, 'Initialized')
            pattern = monitor_property(monitor, model, 'Pattern')
            color = tuple(monitor_property(monitor, model, channel)
                          for channel in ('Red', 'Green', 'Blue'))
            checksum_errors = monitor_property(
                monitor, model, 'ChecksumErrors')
            states.append((initialized, pattern, color, checksum_errors))
        last = states
        if all(initialized and pattern == 2 and color == values and
               checksum_errors == 0
               for initialized, pattern, color, checksum_errors in states):
            print('%s Solo OreoLED set output passed: RGB %s on all four LEDs' %
                  (description, values), flush=True)
            return
        time.sleep(0.05)
    raise RuntimeError('%s Solo OreoLED set did not reach RGB %s: %s' %
                       (description, values, last))


def wait_for_display(process, log_path, physics, monitor, model, device,
                     deadline, previous_hash=None):
    last_hash = None
    stable_count = 0
    last = None
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        initialized = monitor_bool_property(monitor, model, 'Initialized')
        writes = monitor_property(monitor, model, 'DataWriteCount')
        lit_pixels = monitor_property(monitor, model, 'LitPixels')
        content_hash = monitor_property(monitor, model, 'ContentHash')
        last = (initialized, writes, lit_pixels, content_hash)
        valid = (initialized and writes >= 16 and lit_pixels > 0 and
                 (previous_hash is None or content_hash != previous_hash))
        if valid and content_hash == last_hash:
            stable_count += 1
        else:
            stable_count = 0
        last_hash = content_hash
        if stable_count >= 2:
            print('%s %s output passed: %u lit pixels, hash %u' % (
                'baseline' if previous_hash is None else 'stepped',
                ATTACHABLE_DEVICES[device]['name'], lit_pixels,
                content_hash), flush=True)
            return content_hash
        time.sleep(0.05)
    raise RuntimeError('%s display output did not stabilize; last %s' %
                       (ATTACHABLE_DEVICES[device]['name'], last))


def expected_px4flow(truth):
    height = truth['rangefinder_m'][0]
    velocity = truth['velocity_ned_m_s']
    gyro = truth['gyro_rad_s']
    return gyro[0] - velocity[1] / height, gyro[1] + velocity[0] / height


def wait_for_px4flow(connection, process, log_path, physics, truth, deadline,
                     description):
    expected = expected_px4flow(truth)
    last = None
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type='OPTICAL_FLOW', blocking=True, timeout=1)
        if message is None:
            continue
        last = (message.flow_rate_x, message.flow_rate_y, message.quality)
        if (abs(message.flow_rate_x - expected[0]) <= 0.002 and
                abs(message.flow_rate_y - expected[1]) <= 0.002 and
                message.quality == 200):
            print('%s PX4Flow values passed: %.3f, %.3f rad/s quality %u' %
                  (description, message.flow_rate_x, message.flow_rate_y,
                   message.quality), flush=True)
            return
    raise RuntimeError('%s PX4Flow did not reach %s; last %s' %
                       (description, expected, last))


def wait_for_irlock_health(connection, process, log_path, physics, deadline):
    sensor = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_VISION_POSITION
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type='SYS_STATUS', blocking=True, timeout=1)
        if message is None:
            continue
        present = bool(message.onboard_control_sensors_present & sensor)
        enabled = bool(message.onboard_control_sensors_enabled & sensor)
        healthy = bool(message.onboard_control_sensors_health & sensor)
        if present and enabled and healthy:
            print('production IR-LOCK became healthy', flush=True)
            return
    raise RuntimeError('production IR-LOCK did not become healthy')


def wait_for_irlock_frames(process, log_path, physics, monitor, model,
                           minimum, deadline):
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        count = monitor_property(monitor, model, 'FrameCount')
        if count >= minimum:
            return count
        time.sleep(0.05)
    raise RuntimeError('IR-LOCK produced fewer than %u frames' % minimum)


def wait_with_process_checks(connection, process, log_path, physics, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        connection.recv_match(blocking=True, timeout=0.2)


def pixel_to_plane(pixel_x, pixel_y):
    denominator = (
        4.43013552642296e-6 * (pixel_x - 160.0) ** 2 +
        4.79331390531725e-6 * (pixel_y - 100.0) ** 2 - 1.0)
    return (
        (-0.00293875727162397 * pixel_x + 0.470201163459835) /
        denominator,
        (-0.003056843086277 * pixel_y + 0.3056843086277) /
        denominator,
    )


def expected_irlock_measurement(pixel_x, pixel_y, size_x, size_y, height):
    first = pixel_to_plane(pixel_x - size_x // 2, pixel_y - size_y // 2)
    second = pixel_to_plane(pixel_x + size_x // 2, pixel_y + size_y // 2)
    position_x = 0.5 * (first[0] + second[0])
    position_y = 0.5 * (first[1] + second[1])
    return (-position_y * height, position_x * height, height)


def check_irlock_log(path):
    expected_baseline = expected_irlock_measurement(120, 70, 20, 10, 2.0)
    expected_stepped = expected_irlock_measurement(210, 130, 30, 20, 3.0)
    measurements = []
    reader = DFReader.DFReader_binary(str(path), zero_time_base=True)
    try:
        while True:
            message = reader.recv_msg()
            if message is None:
                break
            if message.get_type() != 'PL' or not message.Heal:
                continue
            time_ms = message.TimeUS * 0.001
            measurement = (message.mX, message.mY, message.mZ)
            measurements.append((time_ms, message.LastMeasMS, measurement))
    finally:
        reader.close()

    def matches(values, expected):
        return any(all(abs(actual - wanted) <= 0.15
                       for actual, wanted in zip(value, expected))
                   for value in values)

    values = [value for _time, _last_meas, value in measurements]
    if not matches(values, expected_baseline):
        raise RuntimeError(
            'IR-LOCK log lacks baseline measurement %s; last %s' %
            (expected_baseline, values[-1:] if values else 'none'))
    if not matches(values, expected_stepped):
        raise RuntimeError(
            'IR-LOCK log lacks stepped measurement %s; last %s' %
            (expected_stepped, values[-1:] if values else 'none'))
    longest_plateau = None
    start = 0
    for index in range(1, len(measurements) + 1):
        if (index < len(measurements) and
                measurements[index][1] == measurements[start][1]):
            continue
        duration = measurements[index - 1][0] - measurements[start][0]
        valid_candidate = (
            measurements[start][1] > 0 and
            matches([measurements[start][2]], expected_stepped))
        if (valid_candidate and
                (longest_plateau is None or duration > longest_plateau[0])):
            longest_plateau = (
                duration, measurements[start][1], index - 1)
        start = index
    if longest_plateau is None or longest_plateau[0] < 1000:
        raise RuntimeError(
            'IR-LOCK log lacks a checksum-rejection plateau; longest %s' %
            (longest_plateau,))
    plateau_value = longest_plateau[1]
    if not any(last_meas > plateau_value
               for _time, last_meas, _value
               in measurements[longest_plateau[2] + 1:]):
        raise RuntimeError(
            'IR-LOCK did not accept measurements after checksum recovery')
    print('IR-LOCK production values and checksum rejection passed',
          flush=True)


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
    need_compass = bool(set(devices).intersection(COMPASS_DEVICE_IDS))
    gps = not need_gps
    compass = not need_compass
    last_compass = None
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
        elif need_compass and message_type == 'RAW_IMU':
            last_compass = message
            if compass_matches(message, expected):
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
    details = ''
    if last_compass is not None:
        details = '; last compass was %d %d %d mG' % (
            last_compass.xmag, last_compass.ymag, last_compass.zmag)
    raise RuntimeError('%s GPS/compass values did not reach expected inputs%s' %
                       (description, details))


def wait_for_temperatures(connection, process, log_path, physics, devices,
                          truth, deadline, description):
    expected_c = truth['temperature_k'] - 273.15
    remaining = {
        instance - 1: device for device, instance in devices.items()
    }
    last = {}
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type='BATTERY_STATUS', blocking=True, timeout=1)
        if message is None or message.id not in remaining:
            continue
        last[message.id] = message.temperature
        measured_c = message.temperature * 0.01
        if abs(measured_c - expected_c) > 0.2:
            continue
        device = remaining.pop(message.id)
        print('%s %s passed: %.2f C' %
              (description, ATTACHABLE_DEVICES[device]['name'], measured_c),
              flush=True)
        if not remaining:
            return
    raise RuntimeError('%s temperatures did not reach %.2f C: %s' %
                       (description, expected_c, last))


def wait_for_power_monitors(connection, process, log_path, physics, devices,
                            truth, deadline, description):
    remaining = {
        instance - 1: device for instance, device in devices.items()
    }
    last = {}
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type='BATTERY_STATUS', blocking=True, timeout=1)
        if message is None or message.id not in remaining:
            continue
        device = remaining[message.id]
        if device in CELL_VOLTAGE_POWER_DEVICES:
            voltage = sum(value for value in message.voltages
                          if value not in (0, 0xFFFF)) * 0.001
        else:
            voltage = message.voltages[0] * 0.001
        current = message.current_battery * 0.01
        temperature = message.temperature * 0.01
        last[message.id] = (voltage, current, temperature)
        if (abs(voltage - truth['battery_voltage_v']) > 0.03 or
                abs(current - truth['battery_current_a']) > 0.05):
            continue
        if (device in TEMPERATURE_POWER_DEVICES and
                abs(temperature - (truth['temperature_k'] - 273.15)) > 0.2):
            continue
        expected_fractions = POWER_MONITOR_CELL_FRACTIONS.get(device)
        if expected_fractions is not None:
            cells = message.voltages[:len(expected_fractions)]
            if any(abs(cell * 0.001 -
                       truth['battery_voltage_v'] * fraction) > 0.003
                   for cell, fraction in zip(cells, expected_fractions)):
                continue
        remaining.pop(message.id)
        print('%s %s battery %u passed: %.3f V, %.2f A' %
              (description, ATTACHABLE_DEVICES[device]['name'],
               message.id + 1, voltage, current), flush=True)
        if not remaining:
            return
    raise RuntimeError('%s power monitors did not reach %.3f V, %.2f A: %s' %
                       (description, truth['battery_voltage_v'],
                        truth['battery_current_a'], last))


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
    density = truth['pressure_pa'] / (287.05 * truth['temperature_k'])
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
    last_pressures = {}
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type=list(message_for_instance.values()), blocking=True, timeout=1)
        if message is None:
            continue
        for instance in tuple(remaining):
            if message.get_type() == message_for_instance[instance]:
                last_pressures[instance] = message.press_diff
                if abs(message.press_diff - expected_hpa) <= 0.1:
                    remaining.remove(instance)
        if not remaining:
            print('%s airspeed pressures passed for instances %s: %.2f hPa' %
                  (description, sorted(instances), expected_hpa), flush=True)
            return
    raise RuntimeError(
        '%s airspeed pressure did not reach %.2f hPa for %s; last %s' %
        (description, expected_hpa, sorted(remaining), last_pressures))


def wait_for_barometer(connection, process, log_path, physics, device, instance,
                       truth, deadline, description):
    message_type = {
        1: 'SCALED_PRESSURE',
        2: 'SCALED_PRESSURE2',
        3: 'SCALED_PRESSURE3',
    }[instance]
    expected_hpa = truth['pressure_pa'] * 0.01
    expected_temperature = round((truth['temperature_k'] - 273.15) * 100)
    last = None
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type=message_type, blocking=True, timeout=1)
        if message is not None:
            last = (message.press_abs, message.temperature * 0.01)
        if (message is not None and
                abs(message.press_abs - expected_hpa) <= 0.2 and
                abs(message.temperature - expected_temperature) <= 2):
            print('%s %s passed: %.2f hPa, %.2f C' %
                  (description, ATTACHABLE_DEVICES[device]['name'],
                   message.press_abs,
                   message.temperature * 0.01), flush=True)
            return
    raise RuntimeError('%s %s did not reach %.2f hPa, %.2f C; last %s' %
                       (description, ATTACHABLE_DEVICES[device]['name'],
                        expected_hpa,
                        expected_temperature * 0.01, last))


def wait_for_rangefinders(connection, process, log_path, physics, devices,
                          truth, deadline):
    expected = {
        physics_index: round(truth['rangefinder_m'][physics_index] * 100)
        for device, physics_index in devices.items()
    }
    wait_for_rangefinder_ids(
        connection, process, log_path, physics, expected, deadline)


def wait_for_rangefinder_ids(connection, process, log_path, physics,
                             expected, deadline):
    detected_ids = set()
    last_distances = {}
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type='DISTANCE_SENSOR', blocking=True, timeout=1)
        if message is None or message.id not in expected:
            continue
        last_distances[message.id] = message.current_distance
        if abs(message.current_distance - expected[message.id]) > 1:
            continue
        detected_ids.add(message.id)
        if set(expected).issubset(detected_ids):
            print('rangefinder production drivers reported %s cm' %
                  ', '.join('%u=%u' % item for item in sorted(expected.items())),
                  flush=True)
            return
    raise RuntimeError('rangefinders did not report expected values: matched '
                       '%s; last %s; expected %s' %
                       (sorted(detected_ids), last_distances, expected))


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
            print('rangefinder ID %u stopped after data suppression' %
                  sensor_id, flush=True)
            return
    raise RuntimeError('rangefinder ID %u did not stop reporting' %
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
            endpoints = i2c_endpoints(
                ATTACHABLE_DEVICES[attachment['device']])
            model_names = []
            for endpoint_index, _endpoint in enumerate(endpoints):
                endpoint_name = model_name
                if len(endpoints) > 1:
                    endpoint_name += 'Part%u' % endpoint_index
                model_names.append('sysbus.%s.%s' % (
                    port['peripheral'].lower(), endpoint_name))
            model_name = (model_names[0] if len(model_names) == 1
                          else tuple(model_names))
        elif port['bus'] == 'uart':
            model_name = 'sysbus.%s' % model_name
        runtime_names[attachment['device']] = model_name
    supported = {
        'ak09916-compass', 'ak8963-compass', 'asp5033-airspeed',
        'auav-airspeed', 'auav-barometer', 'bmm150-compass',
        'bmm350-compass',
        'benewake-rangefinder', 'bmp085-barometer', 'bmp280-barometer',
        'bmp388-barometer', 'bmp581-barometer', 'dps280-barometer',
        'hmc5843-compass', 'icm20789-package', 'icp101xx-barometer',
        'icp201xx-barometer', 'iis2mdc-compass',
        'ist8308-compass', 'keller-barometer',
        'ist8310-compass', 'lightware-rangefinder', 'invensense-i2c-imu',
        'bmi160-i2c-imu', 'bmi270-i2c-imu', 'icm20948-i2c-imu',
        'oreoled-set',
        'lis3mdl-compass', 'lsm303d-compass', 'lsm9ds1-compass',
        'mag3110-compass',
        'mmc3416-compass', 'mmc5983-compass', 'qmc5883l-compass',
        'qmc5883p-compass', 'rm3100-compass',
        'lps2xh-barometer', 'ms4525-airspeed', 'ms5611-barometer',
        'spl06-barometer', 'ublox-gps',
        'px4flow-optical-flow', 'irlock-i2c',
    }
    supported.update(AIRSPEED_DEVICE_IDS)
    supported.update(I2C_RANGEFINDER_DEVICES)
    supported.update(POWER_MONITOR_DEVICES)
    supported.update(TEMPERATURE_DEVICES)
    supported.update(LED_DEVICES)
    supported.update(DISPLAY_DEVICES)
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
    physics = ControlledPhysics(
        imu_motion=bool(
            {'invensense-i2c-imu', 'icm20789-package', 'bmi160-i2c-imu',
             'bmi270-i2c-imu', 'icm20948-i2c-imu'}.intersection(assertions)),
        optical_flow_motion='px4flow-optical-flow' in assertions)
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
    check_irlock_dataflash = False
    irlock_log = output_dir / 'irlock.BIN'
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
        for device in COMPASS_DEVICE_IDS:
            if 'device-id' not in assertions.get(device, ()):
                continue
            attachment = next(
                attachment for attachment in profile['devices']
                if attachment['device'] == device)
            find_compass_device(
                connection, process, renode_log, device,
                ports[attachment['port']]['index'], deadline)
        invensense_instance = None
        if 'device-id' in assertions.get('invensense-i2c-imu', ()):
            attachment = next(
                attachment for attachment in profile['devices']
                if attachment['device'] == 'invensense-i2c-imu')
            invensense_instance = find_invensense_i2c(
                connection, process, renode_log,
                ports[attachment['port']]['index'], deadline)
        bmi160_instance = None
        if 'device-id' in assertions.get('bmi160-i2c-imu', ()):
            attachment = next(
                attachment for attachment in profile['devices']
                if attachment['device'] == 'bmi160-i2c-imu')
            bmi160_instance = find_bmi160_i2c(
                connection, process, renode_log,
                ports[attachment['port']]['index'], deadline)
        bmi270_instance = None
        if 'device-id' in assertions.get('bmi270-i2c-imu', ()):
            attachment = next(
                attachment for attachment in profile['devices']
                if attachment['device'] == 'bmi270-i2c-imu')
            bmi270_instance = find_bmi270_i2c(
                connection, process, renode_log,
                ports[attachment['port']]['index'], deadline)
        icm20948_instance = None
        if 'device-id' in assertions.get('icm20948-i2c-imu', ()):
            attachment = next(
                attachment for attachment in profile['devices']
                if attachment['device'] == 'icm20948-i2c-imu')
            icm20948_instance = find_icm20948_i2c(
                connection, process, renode_log,
                ports[attachment['port']]['index'], deadline)
        icm20789_instance = None
        if 'device-id' in assertions.get('icm20789-package', ()):
            attachment = next(
                attachment for attachment in profile['devices']
                if attachment['device'] == 'icm20789-package')
            icm20789_instance = find_invensense_i2c(
                connection, process, renode_log,
                ports[attachment['port']]['index'], deadline,
                address=ICM20789_I2C_ADDRESS,
                accel_devtype=ICM20789_DEVTYPE,
                gyro_devtype=ICM20789_DEVTYPE,
                name='Invensense ICM20789')
        barometer_instances = {}
        for device in BAROMETER_DEVICE_IDS:
            if 'device-id' in assertions.get(device, ()):
                attachment = next(
                    attachment for attachment in profile['devices']
                    if attachment['device'] == device)
                barometer_instances[device] = find_barometer_device(
                    connection, process, renode_log, device,
                    ports[attachment['port']]['index'], deadline)
        for attachment in profile['devices']:
            device = attachment['device']
            if ('device-id' in assertions.get(device, ()) and
                    device in AIRSPEED_DEVICE_IDS):
                find_airspeed_device(
                    connection, process, renode_log, device,
                    attachment['instance'],
                    ports[attachment['port']]['index'], deadline)
        value_devices = {
            device_id for device_id, checks in assertions.items()
            if ('stable-values' in checks and
                (device_id in COMPASS_DEVICE_IDS or device_id == 'ublox-gps'))
        }
        if value_devices:
            wait_for_values(
                connection, process, renode_log, physics,
                BASELINE, value_devices, deadline, 'baseline')
        if ('stable-values' in
                assertions.get('invensense-i2c-imu', ())):
            wait_for_invensense_values(
                connection, process, renode_log, physics,
                invensense_instance, dict(BASELINE, **IMU_BASELINE), deadline,
                'baseline')
        if 'stable-values' in assertions.get('icm20789-package', ()):
            wait_for_invensense_values(
                connection, process, renode_log, physics,
                icm20789_instance, dict(BASELINE, **IMU_BASELINE), deadline,
                'baseline', name='Invensense ICM20789')
        if 'stable-values' in assertions.get('bmi160-i2c-imu', ()):
            wait_for_bmi160_values(
                connection, process, renode_log, physics,
                bmi160_instance, dict(BASELINE, **IMU_BASELINE), deadline,
                'baseline')
        if 'stable-values' in assertions.get('bmi270-i2c-imu', ()):
            wait_for_bmi270_values(
                connection, process, renode_log, physics,
                bmi270_instance, dict(BASELINE, **IMU_BASELINE), deadline,
                'baseline')
        if 'stable-values' in assertions.get('icm20948-i2c-imu', ()):
            wait_for_icm20948_values(
                connection, process, renode_log, physics,
                icm20948_instance, dict(BASELINE, **IMU_BASELINE), deadline,
                'baseline')
        rangefinder_devices = {
            attachment['device']: attachment['instance'] - 1
            for attachment in profile['devices']
            if ('stable-values' in assertions[attachment['device']] and
                attachment['device'] in RANGEFINDER_DEVICES)
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
        for device in I2C_RANGEFINDER_DEVICES:
            if 'data-ready-recovery' not in assertions.get(device, ()):
                continue
            model = runtime_names[device]
            monitor_command(monitor, '%s SuppressData true' % model)
            if not monitor_bool_property(monitor, model, 'SuppressData'):
                raise RuntimeError('Renode did not suppress %s data' % device)
            sensor_id = next(
                attachment['instance'] - 1
                for attachment in profile['devices']
                if attachment['device'] == device)
            wait_for_rangefinder_silence(
                connection, process, renode_log, physics,
                sensor_id, deadline)
            monitor_command(monitor, '%s SuppressData false' % model)
            if monitor_bool_property(monitor, model, 'SuppressData'):
                raise RuntimeError('Renode did not restore %s data' % device)
            wait_for_rangefinder_ids(
                connection, process, renode_log, physics,
                {sensor_id: round(
                    BASELINE['rangefinder_m'][sensor_id] * 100)}, deadline)
        if 'stable-values' in assertions.get('ms4525-airspeed', ()):
            wait_for_airspeed_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_airspeed(
                connection, process, renode_log, physics,
                BASELINE, deadline, 'baseline')
        pressure_instances = {
            attachment['instance'] for attachment in profile['devices']
            if (attachment['device'] in AIRSPEED_DEVICE_IDS and
                attachment['device'] != 'ms4525-airspeed' and
                'stable-values' in assertions[attachment['device']])
        }
        if pressure_instances:
            wait_for_airspeed_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_airspeed_pressures(
                connection, process, renode_log, physics,
                pressure_instances, BASELINE, deadline, 'baseline')
        stable_barometers = {
            device for device in barometer_instances
            if 'stable-values' in assertions.get(device, ())
        }
        if stable_barometers:
            wait_for_barometer_health(
                connection, process, renode_log, physics, True, deadline)
            for device in stable_barometers:
                wait_for_barometer(
                    connection, process, renode_log, physics, device,
                    barometer_instances[device], BASELINE, deadline,
                    'baseline')
        temperature_devices = {
            attachment['device']: attachment['instance']
            for attachment in profile['devices']
            if (attachment['device'] in TEMPERATURE_DEVICES and
                'stable-values' in assertions[attachment['device']])
        }
        if temperature_devices:
            wait_for_temperatures(
                connection, process, renode_log, physics,
                temperature_devices, BASELINE, deadline, 'baseline')
        power_monitor_devices = {
            instance: attachment['device']
            for attachment in profile['devices']
            if (attachment['device'] in POWER_MONITOR_DEVICES and
                'stable-values' in assertions[attachment['device']])
            for instance in attachment.get(
                'battery_instances', (attachment['instance'],))
        }
        if power_monitor_devices:
            wait_for_power_monitors(
                connection, process, renode_log, physics,
                power_monitor_devices, BASELINE, deadline, 'baseline')
        for device in LED_DEVICES:
            if 'stable-values' not in assertions.get(device, ()):
                continue
            set_and_wait_for_led(
                connection, process, renode_log, physics, monitor,
                runtime_names[device], device, (31, 15, 7), deadline,
                'baseline')
        if 'stable-values' in assertions.get('oreoled-set', ()):
            set_and_wait_for_oreoled(
                connection, process, renode_log, physics, monitor,
                runtime_names['oreoled-set'], (31, 15, 7), deadline,
                'baseline')
        display_hashes = {}
        for device in DISPLAY_DEVICES:
            if 'stable-values' not in assertions.get(device, ()):
                continue
            display_hashes[device] = wait_for_display(
                process, renode_log, physics, monitor,
                runtime_names[device], device, deadline)
        if 'stable-values' in assertions.get('px4flow-optical-flow', ()):
            wait_for_px4flow(
                connection, process, renode_log, physics,
                dict(BASELINE, **OPTICAL_FLOW_BASELINE), deadline, 'baseline')
        if 'stable-values' in assertions.get('irlock-i2c', ()):
            wait_for_irlock_health(
                connection, process, renode_log, physics, deadline)
            model = runtime_names['irlock-i2c']
            wait_for_irlock_frames(
                process, renode_log, physics, monitor, model, 10, deadline)
        if 'capacity-scaler' in assertions.get('smbus-maxell-battery', ()):
            capacity = read_parameter(
                connection, process, renode_log, 'BATT_CAPACITY', deadline)
            if capacity != 10000:
                raise RuntimeError(
                    'Maxell capacity scaler produced %u, expected 10000' %
                    capacity)
            print('Maxell capacity scaler passed: %u mAh' % capacity,
                  flush=True)
        stepped_devices = {
            device_id for device_id, checks in assertions.items()
            if 'stepped-values' in checks
        }
        if stepped_devices:
            physics.stepped.set()
        stepped_displays = stepped_devices.intersection(DISPLAY_DEVICES)
        if stepped_displays:
            connection.mav.command_long_send(
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                2989,
                0, 0, 0, 0, 0,
            )
            for device in stepped_displays:
                wait_for_display(
                    process, renode_log, physics, monitor,
                    runtime_names[device], device, deadline,
                    previous_hash=display_hashes[device])
        if 'irlock-i2c' in stepped_devices:
            model = runtime_names['irlock-i2c']
            for property_name, value in (
                    ('PixelX', 210), ('PixelY', 130),
                    ('PixelSizeX', 30), ('PixelSizeY', 20)):
                monitor_command(
                    monitor, '%s %s %u' % (model, property_name, value))
                if monitor_property(monitor, model, property_name) != value:
                    raise RuntimeError(
                        'Renode did not set IR-LOCK %s' % property_name)
        stepped_navigation_devices = stepped_devices.intersection(
            set(COMPASS_DEVICE_IDS).union(('ublox-gps',)))
        if stepped_navigation_devices:
            wait_for_values(
                connection, process, renode_log, physics,
                STEPPED, stepped_navigation_devices, deadline, 'stepped')
        if 'invensense-i2c-imu' in stepped_devices:
            wait_for_invensense_values(
                connection, process, renode_log, physics,
                invensense_instance, dict(STEPPED, **IMU_STEPPED), deadline,
                'stepped')
        if 'icm20789-package' in stepped_devices:
            wait_for_invensense_values(
                connection, process, renode_log, physics,
                icm20789_instance, dict(STEPPED, **IMU_STEPPED), deadline,
                'stepped', name='Invensense ICM20789')
        if 'bmi160-i2c-imu' in stepped_devices:
            wait_for_bmi160_values(
                connection, process, renode_log, physics,
                bmi160_instance, dict(STEPPED, **IMU_STEPPED), deadline,
                'stepped')
        if 'bmi270-i2c-imu' in stepped_devices:
            wait_for_bmi270_values(
                connection, process, renode_log, physics,
                bmi270_instance, dict(STEPPED, **IMU_STEPPED), deadline,
                'stepped')
        if 'icm20948-i2c-imu' in stepped_devices:
            wait_for_icm20948_values(
                connection, process, renode_log, physics,
                icm20948_instance, dict(STEPPED, **IMU_STEPPED), deadline,
                'stepped')
        if 'ms4525-airspeed' in stepped_devices:
            wait_for_airspeed(
                connection, process, renode_log, physics,
                STEPPED, deadline, 'stepped')
        stepped_pressure_instances = {
            attachment['instance'] for attachment in profile['devices']
            if attachment['device'] in stepped_devices and
            attachment['device'] in AIRSPEED_DEVICE_IDS and
            attachment['device'] != 'ms4525-airspeed'
        }
        if stepped_pressure_instances:
            wait_for_airspeed_pressures(
                connection, process, renode_log, physics,
                stepped_pressure_instances, STEPPED, deadline, 'stepped')
        for device in stepped_devices.intersection(barometer_instances):
            wait_for_barometer(
                connection, process, renode_log, physics, device,
                barometer_instances[device], STEPPED, deadline, 'stepped')
        stepped_rangefinders = {
            device: physics_index
            for device, physics_index in rangefinder_devices.items()
            if device in stepped_devices
        }
        if stepped_rangefinders:
            wait_for_rangefinders(
                connection, process, renode_log, physics,
                stepped_rangefinders, STEPPED, deadline)
        stepped_temperatures = {
            device: instance for device, instance in temperature_devices.items()
            if device in stepped_devices
        }
        if stepped_temperatures:
            wait_for_temperatures(
                connection, process, renode_log, physics,
                stepped_temperatures, STEPPED, deadline, 'stepped')
        stepped_power_monitors = {
            instance: device for instance, device in power_monitor_devices.items()
            if device in stepped_devices
        }
        if stepped_power_monitors:
            wait_for_power_monitors(
                connection, process, renode_log, physics,
                stepped_power_monitors, STEPPED, deadline, 'stepped')
        for device in LED_DEVICES:
            if device not in stepped_devices:
                continue
            set_and_wait_for_led(
                connection, process, renode_log, physics, monitor,
                runtime_names[device], device, (8, 16, 24), deadline,
                'stepped')
        if 'oreoled-set' in stepped_devices:
            set_and_wait_for_oreoled(
                connection, process, renode_log, physics, monitor,
                runtime_names['oreoled-set'], (8, 16, 24), deadline,
                'stepped')
        if 'px4flow-optical-flow' in stepped_devices:
            wait_for_px4flow(
                connection, process, renode_log, physics,
                dict(STEPPED, **OPTICAL_FLOW_STEPPED), deadline, 'stepped')
        if 'checksum-recovery' in assertions.get('irlock-i2c', ()):
            model = runtime_names['irlock-i2c']
            wait_with_process_checks(
                connection, process, renode_log, physics, 1.0)
            monitor_command(monitor, '%s CorruptChecksum true' % model)
            if not monitor_bool_property(monitor, model, 'CorruptChecksum'):
                raise RuntimeError(
                    'Renode did not enable IR-LOCK checksum corruption')
            wait_with_process_checks(
                connection, process, renode_log, physics, 2.0)
            monitor_command(monitor, '%s CorruptChecksum false' % model)
            if monitor_bool_property(monitor, model, 'CorruptChecksum'):
                raise RuntimeError(
                    'Renode did not clear IR-LOCK checksum corruption')
            wait_with_process_checks(
                connection, process, renode_log, physics, 3.0)
            check_irlock_dataflash = True
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
        for device in COMPASS_DEVICE_IDS:
            if ('data-ready-recovery' not in assertions.get(device, ())):
                continue
            model = runtime_names[device]
            monitor_command(monitor, '%s SuppressData true' % model)
            if not monitor_bool_property(monitor, model, 'SuppressData'):
                raise RuntimeError('Renode did not suppress %s data' % device)
            wait_for_compass_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s SuppressData false' % model)
            if monitor_bool_property(monitor, model, 'SuppressData'):
                raise RuntimeError('Renode did not restore %s data' % device)
            wait_for_compass_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_values(
                connection, process, renode_log, physics,
                STEPPED, {device}, deadline, 'recovered')
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
        for device in ('dlvr-airspeed', 'ms5525-airspeed',
                       'sdp3x-airspeed'):
            if 'data-ready-recovery' not in assertions.get(device, ()):
                continue
            model = runtime_names[device]
            monitor_command(monitor, '%s SuppressData true' % model)
            if not monitor_bool_property(monitor, model, 'SuppressData'):
                raise RuntimeError('Renode did not suppress %s data' % device)
            wait_for_airspeed_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s SuppressData false' % model)
            if monitor_bool_property(monitor, model, 'SuppressData'):
                raise RuntimeError('Renode did not restore %s data' % device)
            wait_for_airspeed_health(
                connection, process, renode_log, physics, True, deadline)
            instance = next(
                attachment['instance'] for attachment in profile['devices']
                if attachment['device'] == device)
            wait_for_airspeed_pressures(
                connection, process, renode_log, physics,
                {instance}, STEPPED, deadline, 'recovered %s' % device)
        stuck_barometers = {
            device for device in barometer_instances
            if 'stuck-sample-recovery' in assertions.get(device, ())
        }
        for device in stuck_barometers:
            model = runtime_names[device]
            monitor_command(monitor, '%s FreezeSample true' % model)
            if not monitor_bool_property(monitor, model, 'FreezeSample'):
                raise RuntimeError('Renode did not freeze %s samples' % device)
            wait_for_barometer_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s FreezeSample false' % model)
            if monitor_bool_property(monitor, model, 'FreezeSample'):
                raise RuntimeError('Renode did not resume %s samples' % device)
            wait_for_barometer_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_barometer(
                connection, process, renode_log, physics, device,
                barometer_instances[device], STEPPED, deadline, 'recovered')
        for device in (
                'bmp388-barometer', 'dps280-barometer', 'lps2xh-barometer',
                'spl06-barometer'):
            if 'data-ready-recovery' not in assertions.get(device, ()):
                continue
            model = runtime_names[device]
            monitor_command(monitor, '%s SuppressReady true' % model)
            if not monitor_bool_property(monitor, model, 'SuppressReady'):
                raise RuntimeError('Renode did not suppress %s readiness' %
                                   ATTACHABLE_DEVICES[device]['name'])
            wait_for_barometer_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s SuppressReady false' % model)
            if monitor_bool_property(monitor, model, 'SuppressReady'):
                raise RuntimeError('Renode did not restore %s readiness' %
                                   ATTACHABLE_DEVICES[device]['name'])
            wait_for_barometer_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_barometer(
                connection, process, renode_log, physics, device,
                barometer_instances[device], STEPPED, deadline, 'recovered')
        for device in ('bmp085-barometer', 'ms5611-barometer'):
            if 'adc-recovery' not in assertions.get(device, ()):
                continue
            model = runtime_names[device]
            monitor_command(monitor, '%s SuppressAdc true' % model)
            if not monitor_bool_property(monitor, model, 'SuppressAdc'):
                raise RuntimeError('Renode did not suppress %s ADC data' %
                                   ATTACHABLE_DEVICES[device]['name'])
            wait_for_barometer_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s SuppressAdc false' % model)
            if monitor_bool_property(monitor, model, 'SuppressAdc'):
                raise RuntimeError('Renode did not restore %s ADC data' %
                                   ATTACHABLE_DEVICES[device]['name'])
            wait_for_barometer_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_barometer(
                connection, process, renode_log, physics, device,
                barometer_instances[device], STEPPED, deadline, 'recovered')
        for device in (
                'bmp581-barometer', 'icm20789-package',
                'icp101xx-barometer',
                'icp201xx-barometer',
                'keller-barometer', 'spl06-barometer'):
            if 'data-invalid-recovery' not in assertions.get(device, ()):
                continue
            model = runtime_names[device]
            if isinstance(model, tuple):
                model = model[0]
            monitor_command(monitor, '%s SuppressData true' % model)
            if not monitor_bool_property(monitor, model, 'SuppressData'):
                raise RuntimeError('Renode did not suppress %s data' %
                                   ATTACHABLE_DEVICES[device]['name'])
            wait_for_barometer_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s SuppressData false' % model)
            if monitor_bool_property(monitor, model, 'SuppressData'):
                raise RuntimeError('Renode did not restore %s data' %
                                   ATTACHABLE_DEVICES[device]['name'])
            wait_for_barometer_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_barometer(
                connection, process, renode_log, physics, device,
                barometer_instances[device], STEPPED, deadline, 'recovered')
        if 'corrupt-read-recovery' in assertions.get('auav-barometer', ()):
            device = 'auav-barometer'
            model = runtime_names[device]
            monitor_command(monitor, '%s ReadXorMask 8' % model)
            if monitor_property(monitor, model, 'ReadXorMask') != 8:
                raise RuntimeError('Renode did not corrupt AUAV barometer data')
            wait_for_barometer_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s ReadXorMask 0' % model)
            if monitor_property(monitor, model, 'ReadXorMask') != 0:
                raise RuntimeError('Renode did not restore AUAV barometer data')
            wait_for_barometer_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_barometer(
                connection, process, renode_log, physics, device,
                barometer_instances[device], STEPPED, deadline, 'recovered')
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
        if check_irlock_dataflash:
            common.download_log(
                connection, process, renode_log, irlock_log,
                timeout=max(30, deadline - time.monotonic()))
    finally:
        if monitor is not None:
            monitor.close()
        if connection is not None:
            with contextlib.suppress(OSError):
                connection.close()
        if process is not None:
            common.stop_process_group(process)
        physics.stop()
    if check_irlock_dataflash:
        check_irlock_log(irlock_log)


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
