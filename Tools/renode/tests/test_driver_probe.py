#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Probe Renode peripherals with their production ChibiOS drivers."""

import argparse
import contextlib
import dataclasses
import json
import math
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
SERIAL_AIRSPEED_DEVICES = {
    'nmea-airspeed',
}
SERIAL_WIND_DEVICES = {
    'nmea-wind-vane',
}
SERIAL_OPTICAL_FLOW_DEVICES = {
    'cxof-optical-flow',
    'upflow-optical-flow',
}
OPTICAL_FLOW_EXPECTATIONS = {
    'px4flow-optical-flow': (200, 0.002),
    'cxof-optical-flow': (200, 0.025),
    'upflow-optical-flow': (245, 0.003),
}
AIS_DEVICE = 'nmea-ais-receiver'
AIS_MMSI = 123456789
POZYX_BEACON_DEVICE = 'pozyx-beacon'
MARVELMIND_BEACON_DEVICE = 'marvelmind-beacon'
NOOPLOOP_BEACON_DEVICE = 'nooploop-beacon'
SERIAL_BEACON_DEVICES = {
    POZYX_BEACON_DEVICE, MARVELMIND_BEACON_DEVICE, NOOPLOOP_BEACON_DEVICE,
}
NMEA_OUTPUT_DEVICE = 'nmea-output'
LTM_OUTPUT_DEVICE = 'ltm-output'
DEVO_OUTPUT_DEVICE = 'devo-output'
FRSKY_D_OUTPUT_DEVICE = 'frsky-d-output'
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
    'ainstein-lrd1-rangefinder',
    'benewake-rangefinder',
    'benewake-tf03-rangefinder',
    'benewake-tfmini-rangefinder',
    'blping-rangefinder',
    'dts6012m-rangefinder',
    'gyus42v2-rangefinder',
    'jre-rangefinder',
    'lanbao-rangefinder',
    'leddarone-rangefinder',
    'leddarvu8-rangefinder',
    'lightware-grf-rangefinder',
    'lightware-rangefinder',
    'maxsonar-serial-rangefinder',
    'nmea-rangefinder',
    'nooploop-rangefinder',
    'rds02uf-rangefinder',
    'teraranger-serial-rangefinder',
    'usd1-rangefinder',
    'wasp-rangefinder',
))
PROXIMITY_DEVICES = {
    'cygbot-d1-proximity',
    'ld06-proximity',
    'lightware-sf40c-proximity',
    'lightware-sf45b-proximity',
    'rplidar-a2-proximity',
    'teraranger-tower-evo-proximity',
    'teraranger-tower-proximity',
}
PROXIMITY_SENSOR_IDS = {
    'cygbot-d1-proximity': {10, 11, 17},
}
GPS_DEVICES = {
    'erb-gps': 'ERB',
    'gsof-gps': 'GSOF',
    'nmea-gps': 'NMEA',
    'nova-gps': 'NOVA',
    'sbp-gps': 'SBP',
    'sbp2-gps': 'SBP2',
    'sbf-gps': 'SBF',
    'sirf-gps': 'SIRF',
    'ublox-gps': 'u-blox',
}
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
WIND_BASELINE = (45.0, 5.0)
WIND_STEPPED = (135.0, 12.0)
WIND_SUPPRESSED = (225.0, 18.0)
WIND_CORRUPT = (315.0, 25.0)
AIS_BASELINE_OFFSET = (0.01, -0.02)
AIS_CHECKSUM_OFFSET = (0.02, -0.03)
AIS_SUPPRESSED_OFFSET = (0.03, -0.04)
AIS_CORRUPT_OFFSET = (0.04, -0.05)
# Allow for AIVDM coordinate quantisation and the production decoder's scale.
AIS_POSITION_TOLERANCE_E7 = 100
BEACON_BASELINE_POSITION = (1.2, -2.3, -0.4)
BEACON_STEPPED_POSITION = (-3.4, 4.5, 1.2)
BEACON_ANCHORS = (
    (10.0, 0.0, 0.0),
    (0.0, 10.0, 0.0),
    (-10.0, 0.0, 0.0),
    (0.0, -10.0, 0.0),
)
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
GPS_BASELINE = {
    'velocity_ned_m_s': (4.0, -3.0, 0.5),
}
GPS_STEPPED = {
    'velocity_ned_m_s': (-6.0, 8.0, -1.0),
}
FRSKY_D_BASELINE_ALTITUDE_M = 605.0
FRSKY_D_STEPPED_ALTITUDE_M = 614.0


class ControlledPhysics:
    """Serve stationary truth which the test can switch deterministically."""

    def __init__(self, imu_motion=False, optical_flow_motion=False,
                 gps_motion=False, beacon_motion=False, frsky_d_output=False):
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
        self.gps_motion = gps_motion
        self.beacon_motion = beacon_motion
        self.frsky_d_output = frsky_d_output
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
        if self.gps_motion:
            gps_values = GPS_STEPPED if self.stepped.is_set() else GPS_BASELINE
            values = dict(values, **gps_values)
        if self.beacon_motion:
            position = (BEACON_STEPPED_POSITION if self.stepped.is_set()
                        else BEACON_BASELINE_POSITION)
            values = dict(values, position_ned_m=position)
        if self.frsky_d_output:
            altitude = (FRSKY_D_STEPPED_ALTITUDE_M if self.stepped.is_set()
                        else FRSKY_D_BASELINE_ALTITUDE_M)
            values = dict(values, altitude_m=altitude)
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


def test_signed_int32():
    assert signed_int32(0xEAEBFF1E) == -353632482
    assert signed_int32(1491652300) == 1491652300


def test_devo_dms_to_e7():
    assert devo_dms_to_e7(-352_179566) == -353632610
    assert devo_dms_to_e7(1_490_991_380) == 1491652299


def test_frsky_d_coordinate_e7():
    assert frsky_d_coordinate_e7(3521, 7956, ord('S')) == -353632600
    assert frsky_d_coordinate_e7(14909, 9138, ord('E')) == 1491652300


def wait_for_gps_backend(connection, process, log_path, device, deadline):
    backend = GPS_DEVICES[device]
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
            detected |= 'GPS 1: detected %s' % backend in message.text
        elif message_type == 'GPS_RAW_INT':
            fix_type = max(fix_type, message.fix_type)
        if detected and fix_type >= mavutil.mavlink.GPS_FIX_TYPE_3D_FIX:
            print('%s production driver detected with a 3D fix' % backend,
                  flush=True)
            return
    raise RuntimeError(
        '%s probe failed: detection=%s best fix=%u' %
        (backend, detected, fix_type))


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


def wait_for_optical_flow(connection, process, log_path, physics, device,
                          truth, deadline, description):
    expected = expected_px4flow(truth)
    expected_quality, tolerance = OPTICAL_FLOW_EXPECTATIONS[device]
    last = None
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type='OPTICAL_FLOW', blocking=True, timeout=1)
        if message is None:
            continue
        last = (message.flow_rate_x, message.flow_rate_y, message.quality)
        if (abs(message.flow_rate_x - expected[0]) <= tolerance and
                abs(message.flow_rate_y - expected[1]) <= tolerance and
                message.quality == expected_quality):
            print('%s %s values passed: %.3f, %.3f rad/s quality %u' %
                  (description, ATTACHABLE_DEVICES[device]['name'],
                   message.flow_rate_x, message.flow_rate_y, message.quality),
                  flush=True)
            return
    raise RuntimeError('%s %s did not reach %s; last %s' %
                       (description, ATTACHABLE_DEVICES[device]['name'],
                        expected, last))


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


def wait_for_beacon_frames(process, log_path, physics, monitor, model,
                           minimum, deadline):
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        count = monitor_property(monitor, model, 'FramesSent')
        if count >= minimum:
            return
        time.sleep(0.05)
    raise RuntimeError('serial beacon produced fewer than %u frames' % minimum)


def beacon_expected(position):
    distances = []
    for anchor in BEACON_ANCHORS:
        distances.append(math.sqrt(sum(
            (value - origin) ** 2
            for value, origin in zip(position, anchor))))
    return tuple(distances)


def check_beacon_log(path, device):
    records = []
    reader = DFReader.DFReader_binary(str(path), zero_time_base=True)
    try:
        while True:
            message = reader.recv_msg()
            if message is None:
                break
            if message.get_type() != 'BCN':
                continue
            records.append((
                bool(message.Health), message.Cnt,
                (message.PosX, message.PosY, message.PosZ),
                (message.D0, message.D1, message.D2, message.D3),
            ))
    finally:
        reader.close()

    def has_values(position):
        expected_distances = beacon_expected(position)
        return any(
            healthy and count == len(BEACON_ANCHORS) and
            all(abs(actual - expected) <= 0.02
                for actual, expected in zip(measured_position, position)) and
            all(abs(actual - expected) <= 0.03
                for actual, expected in zip(distances, expected_distances))
            for healthy, count, measured_position, distances in records)

    if not has_values(BEACON_BASELINE_POSITION):
        raise RuntimeError('%s log lacks baseline position and ranges' %
                           ATTACHABLE_DEVICES[device]['name'])
    if not has_values(BEACON_STEPPED_POSITION):
        raise RuntimeError('%s log lacks stepped position and ranges' %
                           ATTACHABLE_DEVICES[device]['name'])

    health_states = []
    for healthy, _count, _position, _distances in records:
        if not health_states or health_states[-1] != healthy:
            health_states.append(healthy)
    losses = sum(previous and not current
                 for previous, current in zip(health_states, health_states[1:]))
    recoveries = sum(not previous and current
                     for previous, current in zip(health_states, health_states[1:]))
    if losses < 3 or recoveries < 3:
        raise RuntimeError(
            '%s log lacks three health loss/recovery cycles: %s' %
            (ATTACHABLE_DEVICES[device]['name'], health_states))
    print('%s DataFlash log passed baseline, step and three fault recoveries' %
          ATTACHABLE_DEVICES[device]['name'], flush=True)


def gps_matches(message, expected):
    velocity = expected.get('velocity_ned_m_s')
    motion_matches = True
    if velocity is not None:
        expected_speed = round(
            (velocity[0] ** 2 + velocity[1] ** 2) ** 0.5 * 100)
        expected_course = round(
            (math.degrees(math.atan2(velocity[1], velocity[0])) % 360) *
            100)
        course_error = abs(message.cog - expected_course)
        course_error = min(course_error, 36000 - course_error)
        motion_matches = (
            abs(message.vel - expected_speed) <= 3 and
            course_error <= 3)
    return motion_matches and (
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
    need_gps = bool(set(devices).intersection(GPS_DEVICES))
    need_compass = bool(set(devices).intersection(COMPASS_DEVICE_IDS))
    gps = not need_gps
    compass = not need_compass
    last_gps = None
    last_compass = None
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(blocking=True, timeout=1)
        if message is None:
            continue
        message_type = message.get_type()
        if need_gps and message_type == 'GPS_RAW_INT':
            last_gps = message
            if gps_matches(message, expected):
                gps = message
        elif need_compass and message_type == 'RAW_IMU':
            last_compass = message
            if compass_matches(message, expected):
                compass = message
        if gps and compass:
            details = []
            if need_gps:
                details.append('GPS %.7f %.7f %.1fm %.2fm/s %.2fdeg' % (
                    gps.lat * 1.0e-7, gps.lon * 1.0e-7, gps.alt * 0.001,
                    gps.vel * 0.01, gps.cog * 0.01))
            if need_compass:
                details.append('compass %d %d %d mG' %
                               (compass.xmag, compass.ymag, compass.zmag))
            print('%s values passed: %s' %
                  (description, ', '.join(details)), flush=True)
            return
    details = ''
    if last_gps is not None:
        details += '; last GPS was %.7f %.7f %.1fm %.2fm/s %.2fdeg' % (
            last_gps.lat * 1.0e-7, last_gps.lon * 1.0e-7,
            last_gps.alt * 0.001, last_gps.vel * 0.01,
            last_gps.cog * 0.01)
    if last_compass is not None:
        details += '; last compass was %d %d %d mG' % (
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


def wait_for_optical_flow_health(connection, process, log_path, physics,
                                 healthy, deadline):
    sensor = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type='SYS_STATUS', blocking=True, timeout=1)
        if message is None:
            continue
        present = bool(message.onboard_control_sensors_present & sensor)
        enabled = bool(message.onboard_control_sensors_enabled & sensor)
        reported_healthy = bool(message.onboard_control_sensors_health & sensor)
        if present and enabled and reported_healthy == healthy:
            print('production optical flow became %s' %
                  ('healthy' if healthy else 'unhealthy'), flush=True)
            return
    raise RuntimeError('production optical flow did not become %s' %
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


def wait_for_nmea_airspeed(connection, process, log_path, physics, truth,
                           deadline, description):
    expected_speed = truth['airspeed_m_s']
    expected_temperature = round((truth['temperature_k'] - 273.15) * 100)
    speed = None
    temperature = None
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type=['VFR_HUD', 'SCALED_PRESSURE'], blocking=True, timeout=1)
        if message is None:
            continue
        if message.get_type() == 'VFR_HUD':
            speed = message.airspeed
        else:
            temperature = getattr(message, 'temperature_press_diff', None)
        if (speed is not None and temperature is not None and
                abs(speed - expected_speed) <= 0.2 and
                abs(temperature - expected_temperature) <= 2):
            print('%s NMEA water speed passed: %.2f m/s, %.2f C' %
                  (description, speed, temperature * 0.01), flush=True)
            return
    raise RuntimeError(
        '%s NMEA water speed did not reach %.2f m/s and %.2f C; last %s, %s' %
        (description, expected_speed, expected_temperature * 0.01,
         speed, None if temperature is None else temperature * 0.01))


def wind_direction_error(actual, expected):
    return abs((actual - expected + 180.0) % 360.0 - 180.0)


def wait_for_wind(connection, process, log_path, physics, expected, deadline,
                  description):
    last = None
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(type='WIND', blocking=True, timeout=1)
        if message is None:
            continue
        last = (message.direction, message.speed)
        if (wind_direction_error(message.direction, expected[0]) <= 1.0 and
                abs(message.speed - expected[1]) <= 0.2):
            print('%s NMEA wind passed: %.1f deg, %.2f m/s' %
                  (description, message.direction, message.speed), flush=True)
            return
    raise RuntimeError('%s NMEA wind did not reach %s; last %s' %
                       (description, expected, last))


def wait_for_stale_wind(connection, process, log_path, physics, expected,
                        rejected, deadline, description):
    matched = 0
    last = None
    while time.monotonic() < deadline and matched < 5:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(type='WIND', blocking=True, timeout=1)
        if message is None:
            continue
        last = (message.direction, message.speed)
        if (wind_direction_error(message.direction, rejected[0]) <= 1.0 and
                abs(message.speed - rejected[1]) <= 0.2):
            raise RuntimeError('%s NMEA wind accepted invalid input: %s' %
                               (description, last))
        if (wind_direction_error(message.direction, expected[0]) <= 1.0 and
                abs(message.speed - expected[1]) <= 0.2):
            matched += 1
    if matched == 5:
        print('NMEA wind held its previous value during %s' % description,
              flush=True)
        return
    raise RuntimeError('%s NMEA wind did not retain %s; last %s' %
                       (description, expected, last))


def ais_expected(truth, offsets):
    velocity = truth['velocity_ned_m_s']
    speed = (velocity[0] ** 2 + velocity[1] ** 2) ** 0.5
    course = math.degrees(math.atan2(velocity[1], velocity[0])) % 360.0
    sog = round(speed * 1.9438444924406 * 10.0)
    return {
        'lat': round((truth['latitude_deg'] + offsets[0]) * 1.0e7),
        'lon': round((truth['longitude_deg'] + offsets[1]) * 1.0e7),
        'velocity': int(sog * 0.1 / 1.9438444924406 * 100.0),
        'cog': round(course * 10.0) * 10,
        'heading': round(course) % 360 * 100,
    }


def mavlink_text(value):
    if isinstance(value, bytes):
        return value.split(b'\0', 1)[0].decode('ascii')
    return value.split('\0', 1)[0]


def ais_matches(message, expected, static_data):
    dynamic = (
        message.MMSI == AIS_MMSI and
        abs(message.lat - expected['lat']) <= AIS_POSITION_TOLERANCE_E7 and
        abs(message.lon - expected['lon']) <= AIS_POSITION_TOLERANCE_E7 and
        abs(message.velocity - expected['velocity']) <= 2 and
        abs(message.COG - expected['cog']) <= 10 and
        abs(message.heading - expected['heading']) <= 100)
    if not dynamic or not static_data:
        return dynamic
    required_flags = (
        mavutil.mavlink.AIS_FLAGS_POSITION_ACCURACY |
        mavutil.mavlink.AIS_FLAGS_VALID_COG |
        mavutil.mavlink.AIS_FLAGS_VALID_VELOCITY |
        mavutil.mavlink.AIS_FLAGS_VALID_DIMENSIONS |
        mavutil.mavlink.AIS_FLAGS_VALID_CALLSIGN |
        mavutil.mavlink.AIS_FLAGS_VALID_NAME)
    return (
        message.flags & required_flags == required_flags and
        message.dimension_bow == 40 and
        message.dimension_stern == 15 and
        message.dimension_port == 6 and
        message.dimension_starboard == 8 and
        mavlink_text(message.callsign) == 'RENODE' and
        mavlink_text(message.name) == 'RENODE VESSEL')


def wait_for_ais(connection, process, log_path, physics, truth, offsets,
                 deadline, description, static_data=False):
    expected = ais_expected(truth, offsets)
    last = None
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type='AIS_VESSEL', blocking=True, timeout=1)
        if message is None:
            continue
        last = message
        if ais_matches(message, expected, static_data):
            print('%s AIS vessel passed: MMSI %u, %.7f %.7f, %.2f m/s' %
                  (description, message.MMSI, message.lat * 1.0e-7,
                   message.lon * 1.0e-7, message.velocity * 0.01), flush=True)
            return
    raise RuntimeError('%s AIS vessel did not reach %s; last %s' %
                       (description, expected, last))


def reject_ais_value(connection, process, log_path, physics, truth, offsets,
                     duration, description):
    expected = ais_expected(truth, offsets)
    end = time.monotonic() + duration
    while time.monotonic() < end:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type='AIS_VESSEL', blocking=True,
            timeout=min(1.0, end - time.monotonic()))
        if message is not None and ais_matches(message, expected, False):
            raise RuntimeError('%s AIS accepted invalid input: %s' %
                               (description, message))
    print('AIS rejected %s input' % description, flush=True)


def set_ais_offsets(monitor, model, offsets):
    monitor_command(
        monitor, '%s LatitudeOffsetDegrees %.6f' % (model, offsets[0]))
    monitor_command(
        monitor, '%s LongitudeOffsetDegrees %.6f' % (model, offsets[1]))


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
                                 sensor_id, deadline, reason='data suppression'):
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
            print('rangefinder ID %u stopped after %s' %
                  (sensor_id, reason), flush=True)
            return
    raise RuntimeError('rangefinder ID %u did not stop reporting' %
                       sensor_id)


def wait_for_proximity(connection, process, log_path, physics, expected_cm,
                       expected_ids, deadline, description):
    matched = set()
    last_distances = {}
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type='DISTANCE_SENSOR', blocking=True, timeout=1)
        if message is None or message.id not in expected_ids:
            continue
        last_distances[message.id] = message.current_distance
        if abs(message.current_distance - expected_cm) <= 1:
            matched.add(message.id)
        if matched == expected_ids:
            print('%s proximity production driver reported %u cm in expected '
                  'directions' % (description, expected_cm), flush=True)
            return
    raise RuntimeError('%s proximity did not report %u cm in expected directions; '
                       'matched %s, last %s' %
                       (description, expected_cm, sorted(matched),
                        last_distances))


def wait_for_proximity_silence(connection, process, log_path, physics,
                               deadline, reason):
    quiet_started = time.monotonic()
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        message = connection.recv_match(
            type='DISTANCE_SENSOR', blocking=True, timeout=0.25)
        now = time.monotonic()
        if message is not None and 10 <= message.id < 18:
            quiet_started = now
        if now - quiet_started >= 2.0:
            print('proximity driver stopped after %s' % reason, flush=True)
            return
    raise RuntimeError('proximity driver did not stop after %s' % reason)


def monitor_command(monitor, command):
    response = monitor.command(command)
    if 'error' in response.lower():
        raise RuntimeError('Renode monitor rejected %s: %s' %
                           (command, response.strip()))


def monitor_property(monitor, model, property_name):
    response = monitor.command('%s %s' % (model, property_name))
    values = re.findall(r'(?m)^\s*(0x[0-9A-Fa-f]+|-?[0-9]+)\s*$', response)
    if not values:
        raise RuntimeError('Renode monitor did not return %s.%s: %s' %
                           (model, property_name, response.strip()))
    return int(values[-1], 0)


def signed_int32(value):
    if 0x80000000 <= value <= 0xFFFFFFFF:
        return value - 0x100000000
    return value


def monitor_bool_property(monitor, model, property_name):
    response = monitor.command('%s %s' % (model, property_name))
    values = re.findall(r'(?im)^\s*(true|false)\s*$', response)
    if not values:
        raise RuntimeError('Renode monitor did not return %s.%s: %s' %
                           (model, property_name, response.strip()))
    return values[-1].lower() == 'true'


def wait_for_nmea_output(process, log_path, physics, monitor, model,
                         expected, label, deadline):
    north, east, _down = expected['velocity_ned_m_s']
    expected_values = {
        'LastLatitudeE7': round(expected['latitude_deg'] * 1.0e7),
        'LastLongitudeE7': round(expected['longitude_deg'] * 1.0e7),
        'LastAltitudeCm': round(expected['altitude_m'] * 100.0),
        'LastSpeedCentiKnots': round(
            math.hypot(north, east) * 1.94384449 * 100.0),
        'LastCourseCentiDegrees': round(
            math.degrees(math.atan2(north, east)) % 360.0 * 100.0),
    }
    tolerances = {
        'LastLatitudeE7': 300,
        'LastLongitudeE7': 300,
        'LastAltitudeCm': 200,
        'LastSpeedCentiKnots': 5,
        'LastCourseCentiDegrees': 5,
    }
    observed = {}
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        observed = {
            name: signed_int32(monitor_property(monitor, model, name))
            for name in expected_values
        }
        sentence_counts = tuple(
            monitor_property(monitor, model, name)
            for name in ('GGASentences', 'RMCSentences', 'PASHRSentences'))
        valid = monitor_bool_property(monitor, model, 'LastRmcValid')
        invalid = monitor_property(monitor, model, 'InvalidSentences')
        if (all(sentence_counts) and valid and invalid == 0 and
                all(abs(observed[name] - value) <= tolerances[name]
                    for name, value in expected_values.items())):
            print('NMEA output %s passed: %s' % (label, observed), flush=True)
            return
        time.sleep(0.1)
    raise RuntimeError(
        'NMEA output %s mismatch: expected %s observed %s' %
        (label, expected_values, observed))


def wait_for_ltm_output(process, log_path, physics, monitor, model,
                        expected, relative_altitude_cm, label, deadline):
    north, east, _down = expected['velocity_ned_m_s']
    expected_values = {
        'LastLatitudeE7': round(expected['latitude_deg'] * 1.0e7),
        'LastLongitudeE7': round(expected['longitude_deg'] * 1.0e7),
        'LastGroundSpeedMS': round(math.hypot(north, east)),
    }
    tolerances = {
        'LastLatitudeE7': 500,
        'LastLongitudeE7': 500,
        'LastGroundSpeedMS': 1,
    }
    if relative_altitude_cm is not None:
        expected_values['LastRelativeAltitudeCm'] = relative_altitude_cm
        tolerances['LastRelativeAltitudeCm'] = 500
    observed = {}
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        observed = {
            name: signed_int32(monitor_property(monitor, model, name))
            for name in expected_values
        }
        frame_counts = tuple(
            monitor_property(monitor, model, name)
            for name in ('GFrames', 'AFrames', 'SFrames'))
        invalid = monitor_property(monitor, model, 'InvalidFrames')
        fix_type = monitor_property(monitor, model, 'LastFixType')
        satellites = monitor_property(monitor, model, 'LastSatellites')
        if (all(frame_counts) and invalid == 0 and fix_type == 3 and
                satellites > 0 and
                all(abs(observed[name] - value) <= tolerances[name]
                    for name, value in expected_values.items())):
            print('LTM output %s passed: %s' % (label, observed), flush=True)
            return
        time.sleep(0.1)
    raise RuntimeError(
        'LTM output %s mismatch: expected %s observed %s' %
        (label, expected_values, observed))


def devo_dms_to_e7(value):
    degrees_minutes = value * 1.0e-7
    degrees = math.trunc(degrees_minutes)
    return math.trunc(
        (degrees + (degrees_minutes - degrees) * 100.0 / 60.0) * 1.0e7)


def wait_for_devo_output(process, log_path, physics, monitor, model,
                         expected, relative_altitude_cm, label, deadline):
    north, east, _down = expected['velocity_ned_m_s']
    # AP_DEVO_Telem::gpsDdToDmsFormat returns uint32_t. The ChibiOS ARM
    # conversion therefore clamps a negative DMS latitude to zero.
    expected_latitude_e7 = (0 if expected['latitude_deg'] < 0 else
                            round(expected['latitude_deg'] * 1.0e7))
    expected_values = {
        'latitude_e7': expected_latitude_e7,
        'longitude_e7': round(expected['longitude_deg'] * 1.0e7),
        'speed': int(math.hypot(north, east) * 0.0194384 * 100.0),
    }
    observed = {}
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        latitude_dms = signed_int32(monitor_property(
            monitor, model, 'LastLatitudeDmsE7'))
        longitude_dms = signed_int32(monitor_property(
            monitor, model, 'LastLongitudeDmsE7'))
        observed = {
            'latitude_e7': devo_dms_to_e7(latitude_dms),
            'longitude_e7': devo_dms_to_e7(longitude_dms),
            'speed': signed_int32(monitor_property(
                monitor, model, 'LastSpeed')),
        }
        if relative_altitude_cm is not None:
            observed['relative_altitude_cm'] = signed_int32(monitor_property(
                monitor, model, 'LastRelativeAltitudeCm'))
            expected_values['relative_altitude_cm'] = relative_altitude_cm
        valid = monitor_property(monitor, model, 'ValidFrames')
        invalid = monitor_property(monitor, model, 'InvalidFrames')
        tolerances = {
            'latitude_e7': 500,
            'longitude_e7': 500,
            'speed': 1,
            'relative_altitude_cm': 500,
        }
        if (valid > 0 and invalid == 0 and
                all(abs(observed[name] - value) <= tolerances[name]
                    for name, value in expected_values.items())):
            print('Devo output %s passed: %s' % (label, observed), flush=True)
            return
        time.sleep(0.1)
    raise RuntimeError(
        'Devo output %s mismatch: expected %s observed %s' %
        (label, expected_values, observed))


def frsky_d_coordinate_e7(degree_minutes, minute_fraction, hemisphere):
    degrees = degree_minutes // 100
    minutes = degree_minutes % 100 + minute_fraction / 10000.0
    value = round((degrees + minutes / 60.0) * 1.0e7)
    return -value if hemisphere in (ord('S'), ord('W')) else value


def wait_for_frsky_d_output(process, log_path, physics, monitor, model,
                            expected, label, deadline):
    north, east, _down = expected['velocity_ned_m_s']
    expected_values = {
        'latitude_e7': round(expected['latitude_deg'] * 1.0e7),
        'longitude_e7': round(expected['longitude_deg'] * 1.0e7),
        'speed_m_s': math.hypot(north, east),
        'altitude_m': expected['altitude_m'],
    }
    observed = {}
    while time.monotonic() < deadline:
        common.check_process(process, log_path)
        physics.check()
        observed = {
            'latitude_e7': frsky_d_coordinate_e7(
                monitor_property(
                    monitor, model, 'LastLatitudeDegreeMinutes'),
                monitor_property(
                    monitor, model, 'LastLatitudeMinuteFraction'),
                monitor_property(monitor, model, 'LastLatitudeHemisphere')),
            'longitude_e7': frsky_d_coordinate_e7(
                monitor_property(
                    monitor, model, 'LastLongitudeDegreeMinutes'),
                monitor_property(
                    monitor, model, 'LastLongitudeMinuteFraction'),
                monitor_property(monitor, model, 'LastLongitudeHemisphere')),
            'speed_m_s': (
                monitor_property(monitor, model, 'LastSpeedMeters') +
                monitor_property(
                    monitor, model, 'LastSpeedCentimeters') / 100.0),
            'altitude_m': (
                monitor_property(monitor, model, 'LastGpsAltitudeMeters') +
                monitor_property(
                    monitor, model, 'LastGpsAltitudeCentimeters') / 100.0),
        }
        frame_counts = tuple(
            monitor_property(monitor, model, name)
            for name in ('StatusFrames', 'PositionFrames', 'SpeedFrames',
                         'AltitudeFrames'))
        status = monitor_property(monitor, model, 'LastGpsStatus')
        invalid = monitor_property(monitor, model, 'InvalidFrames')
        stuffed = monitor_property(monitor, model, 'StuffedBytes')
        if (all(frame_counts) and invalid == 0 and stuffed > 0 and
                status % 10 == 3 and
                status // 10 > 0 and
                abs(observed['latitude_e7'] -
                    expected_values['latitude_e7']) <= 500 and
                abs(observed['longitude_e7'] -
                    expected_values['longitude_e7']) <= 500 and
                abs(observed['speed_m_s'] -
                    expected_values['speed_m_s']) <= 0.05 and
                abs(observed['altitude_m'] -
                    expected_values['altitude_m']) <= 1.0):
            print('FrSky D output %s passed: %s' %
                  (label, observed), flush=True)
            return
        time.sleep(0.1)
    raise RuntimeError(
        'FrSky D output %s mismatch: expected %s observed %s' %
        (label, expected_values, observed))


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
        'bmm350-compass', 'bmp085-barometer', 'bmp280-barometer',
        'bmp388-barometer', 'bmp581-barometer', 'dps280-barometer',
        'hmc5843-compass', 'icm20789-package', 'icp101xx-barometer',
        'icp201xx-barometer', 'iis2mdc-compass',
        'ist8308-compass', 'keller-barometer',
        'ist8310-compass', 'invensense-i2c-imu',
        'bmi160-i2c-imu', 'bmi270-i2c-imu', 'icm20948-i2c-imu',
        'oreoled-set',
        'lis3mdl-compass', 'lsm303d-compass', 'lsm9ds1-compass',
        'mag3110-compass',
        'mmc3416-compass', 'mmc5983-compass', 'qmc5883l-compass',
        'qmc5883p-compass', 'rm3100-compass',
        'lps2xh-barometer', 'ms4525-airspeed', 'ms5611-barometer',
        'spl06-barometer',
        'px4flow-optical-flow', 'irlock-i2c',
    }
    supported.update(GPS_DEVICES)
    supported.update(AIRSPEED_DEVICE_IDS)
    supported.update(SERIAL_AIRSPEED_DEVICES)
    supported.update(SERIAL_WIND_DEVICES)
    supported.update(SERIAL_OPTICAL_FLOW_DEVICES)
    supported.add(AIS_DEVICE)
    supported.update(SERIAL_BEACON_DEVICES)
    supported.add(NMEA_OUTPUT_DEVICE)
    supported.add(LTM_OUTPUT_DEVICE)
    supported.add(DEVO_OUTPUT_DEVICE)
    supported.add(FRSKY_D_OUTPUT_DEVICE)
    supported.update(RANGEFINDER_DEVICES)
    supported.update(PROXIMITY_DEVICES)
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
        optical_flow_motion=bool(
            set(assertions).intersection(OPTICAL_FLOW_EXPECTATIONS)),
        gps_motion=(
            AIS_DEVICE in assertions or
            bool(set(assertions).intersection(GPS_DEVICES))),
        beacon_motion=bool(SERIAL_BEACON_DEVICES.intersection(assertions)),
        frsky_d_output=FRSKY_D_OUTPUT_DEVICE in assertions)
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
    dataflash_beacon_device = None
    beacon_log = output_dir / 'beacon.BIN'
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
        for device in GPS_DEVICES:
            if 'detection' in assertions.get(device, ()):
                wait_for_gps_backend(
                    connection, process, renode_log, device, deadline)
                if (device in ('gsof-gps', 'sbf-gps') and
                        not monitor_bool_property(
                        monitor, runtime_names[device], 'Configured')):
                    raise RuntimeError(
                        '%s production configuration did not complete' %
                        GPS_DEVICES[device])
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
                (device_id in COMPASS_DEVICE_IDS or
                 device_id in GPS_DEVICES))
        }
        if value_devices:
            expected = BASELINE
            if set(value_devices).intersection(GPS_DEVICES):
                expected = dict(BASELINE, **GPS_BASELINE)
                if FRSKY_D_OUTPUT_DEVICE in assertions:
                    expected['altitude_m'] = FRSKY_D_BASELINE_ALTITUDE_M
            wait_for_values(
                connection, process, renode_log, physics,
                expected, value_devices, deadline, 'baseline')
        if 'stable-values' in assertions.get(NMEA_OUTPUT_DEVICE, ()):
            wait_for_nmea_output(
                process, renode_log, physics, monitor,
                runtime_names[NMEA_OUTPUT_DEVICE],
                dict(BASELINE, **GPS_BASELINE), 'baseline', deadline)
        if 'stable-values' in assertions.get(LTM_OUTPUT_DEVICE, ()):
            wait_for_ltm_output(
                process, renode_log, physics, monitor,
                runtime_names[LTM_OUTPUT_DEVICE],
                dict(BASELINE, **GPS_BASELINE), 0, 'baseline', deadline)
        if 'stable-values' in assertions.get(DEVO_OUTPUT_DEVICE, ()):
            wait_for_devo_output(
                process, renode_log, physics, monitor,
                runtime_names[DEVO_OUTPUT_DEVICE],
                dict(BASELINE, **GPS_BASELINE), 0, 'baseline', deadline)
        if 'stable-values' in assertions.get(FRSKY_D_OUTPUT_DEVICE, ()):
            expected = dict(BASELINE, **GPS_BASELINE)
            expected['altitude_m'] = FRSKY_D_BASELINE_ALTITUDE_M
            wait_for_frsky_d_output(
                process, renode_log, physics, monitor,
                runtime_names[FRSKY_D_OUTPUT_DEVICE],
                expected, 'baseline', deadline)
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
        serial_rangefinder_sensor_ids = {
            attachment['device']: attachment['instance'] - 1
            for attachment in profile['devices']
            if (attachment['device'] in RANGEFINDER_DEVICES and
                attachment['device'] not in I2C_RANGEFINDER_DEVICES)
        }
        if rangefinder_devices:
            wait_for_rangefinders(
                connection, process, renode_log, physics,
                rangefinder_devices, BASELINE, deadline)
        proximity_devices = {
            attachment['device']: attachment['instance'] - 1
            for attachment in profile['devices']
            if (attachment['device'] in PROXIMITY_DEVICES and
                'stable-values' in assertions[attachment['device']])
        }
        for device, physics_index in proximity_devices.items():
            wait_for_proximity(
                connection, process, renode_log, physics,
                round(BASELINE['rangefinder_m'][physics_index] * 100),
                PROXIMITY_SENSOR_IDS.get(device, set(range(10, 18))),
                deadline, 'baseline %s' % device)
        for device, checks in assertions.items():
            if 'configuration' not in checks:
                continue
            if not monitor_bool_property(
                    monitor, runtime_names[device], 'Configured'):
                raise RuntimeError(
                    '%s production configuration did not complete' %
                    ATTACHABLE_DEVICES[device]['name'])
        for device, sensor_id in serial_rangefinder_sensor_ids.items():
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
        for device, physics_index in proximity_devices.items():
            model = runtime_names[device]
            if 'output-suppression-recovery' in assertions[device]:
                monitor_command(monitor, '%s SuppressOutput true' % model)
                wait_for_proximity_silence(
                    connection, process, renode_log, physics, deadline,
                    'data suppression')
                monitor_command(monitor, '%s SuppressOutput false' % model)
                wait_for_proximity(
                    connection, process, renode_log, physics,
                    round(BASELINE['rangefinder_m'][physics_index] * 100),
                    PROXIMITY_SENSOR_IDS.get(device, set(range(10, 18))),
                    deadline, 'suppression recovery %s' % device)
            if 'output-corruption-recovery' in assertions[device]:
                monitor_command(monitor, '%s OutputXorMask 255' % model)
                wait_for_proximity_silence(
                    connection, process, renode_log, physics, deadline,
                    'output corruption')
                monitor_command(monitor, '%s OutputXorMask 0' % model)
                wait_for_proximity(
                    connection, process, renode_log, physics,
                    round(BASELINE['rangefinder_m'][physics_index] * 100),
                    PROXIMITY_SENSOR_IDS.get(device, set(range(10, 18))),
                    deadline, 'corruption recovery %s' % device)
        for device, sensor_id in serial_rangefinder_sensor_ids.items():
            if ('output-corruption-recovery' not in
                    assertions.get(device, ())):
                continue
            model = runtime_names[device]
            monitor_command(monitor, '%s OutputXorMask 255' % model)
            if monitor_property(monitor, model, 'OutputXorMask') != 255:
                raise RuntimeError(
                    'Renode did not corrupt %s output' % device)
            wait_for_rangefinder_silence(
                connection, process, renode_log, physics,
                sensor_id, deadline, reason='output corruption')
            monitor_command(monitor, '%s OutputXorMask 0' % model)
            if monitor_property(monitor, model, 'OutputXorMask') != 0:
                raise RuntimeError(
                    'Renode did not clear %s corruption' % device)
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
        if 'stable-values' in assertions.get('nmea-airspeed', ()):
            wait_for_airspeed_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_nmea_airspeed(
                connection, process, renode_log, physics,
                BASELINE, deadline, 'baseline')
        if 'stable-values' in assertions.get('nmea-wind-vane', ()):
            wait_for_wind(
                connection, process, renode_log, physics,
                WIND_BASELINE, deadline, 'baseline')
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
        for device in OPTICAL_FLOW_EXPECTATIONS:
            if 'stable-values' not in assertions.get(device, ()):
                continue
            wait_for_optical_flow(
                connection, process, renode_log, physics, device,
                dict(BASELINE, **OPTICAL_FLOW_BASELINE), deadline, 'baseline')
        if 'stable-values' in assertions.get(AIS_DEVICE, ()):
            wait_for_ais(
                connection, process, renode_log, physics,
                dict(BASELINE, **GPS_BASELINE), AIS_BASELINE_OFFSET, deadline,
                'baseline',
                static_data='static-data' in assertions.get(AIS_DEVICE, ()))
        for device in SERIAL_BEACON_DEVICES:
            if 'stable-values' not in assertions.get(device, ()):
                continue
            wait_for_beacon_frames(
                process, renode_log, physics, monitor,
                runtime_names[device], 10, deadline)
            wait_with_process_checks(
                connection, process, renode_log, physics, 10.0)
            if 'settings-request' in assertions.get(device, ()):
                requests = monitor_property(
                    monitor, runtime_names[device], 'SettingRequests')
                if requests < 1:
                    raise RuntimeError(
                        '%s did not receive the firmware settings request' %
                        ATTACHABLE_DEVICES[device]['name'])
                wait_with_process_checks(
                    connection, process, renode_log, physics, 5.0)
            dataflash_beacon_device = device
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
        if 'nmea-wind-vane' in stepped_devices:
            model = runtime_names['nmea-wind-vane']
            monitor_command(
                monitor, '%s WindDirectionDegrees %.1f' %
                (model, WIND_STEPPED[0]))
            monitor_command(
                monitor, '%s WindSpeedMS %.1f' %
                (model, WIND_STEPPED[1]))
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
            set(COMPASS_DEVICE_IDS).union(GPS_DEVICES))
        if stepped_navigation_devices:
            expected = STEPPED
            if stepped_navigation_devices.intersection(GPS_DEVICES):
                expected = dict(STEPPED, **GPS_STEPPED)
                if FRSKY_D_OUTPUT_DEVICE in assertions:
                    expected['altitude_m'] = FRSKY_D_STEPPED_ALTITUDE_M
            wait_for_values(
                connection, process, renode_log, physics,
                expected, stepped_navigation_devices, deadline, 'stepped')
        if NMEA_OUTPUT_DEVICE in stepped_devices:
            wait_for_nmea_output(
                process, renode_log, physics, monitor,
                runtime_names[NMEA_OUTPUT_DEVICE],
                dict(STEPPED, **GPS_STEPPED), 'stepped', deadline)
        if LTM_OUTPUT_DEVICE in stepped_devices:
            wait_for_ltm_output(
                process, renode_log, physics, monitor,
                runtime_names[LTM_OUTPUT_DEVICE],
                dict(STEPPED, **GPS_STEPPED), None, 'stepped', deadline)
        if DEVO_OUTPUT_DEVICE in stepped_devices:
            wait_for_devo_output(
                process, renode_log, physics, monitor,
                runtime_names[DEVO_OUTPUT_DEVICE],
                dict(STEPPED, **GPS_STEPPED), None, 'stepped', deadline)
        if FRSKY_D_OUTPUT_DEVICE in stepped_devices:
            expected = dict(STEPPED, **GPS_STEPPED)
            expected['altitude_m'] = FRSKY_D_STEPPED_ALTITUDE_M
            wait_for_frsky_d_output(
                process, renode_log, physics, monitor,
                runtime_names[FRSKY_D_OUTPUT_DEVICE],
                expected, 'stepped', deadline)
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
        if 'nmea-airspeed' in stepped_devices:
            wait_for_nmea_airspeed(
                connection, process, renode_log, physics,
                STEPPED, deadline, 'stepped')
        if 'nmea-wind-vane' in stepped_devices:
            wait_for_wind(
                connection, process, renode_log, physics,
                WIND_STEPPED, deadline, 'stepped')
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
        for device, physics_index in proximity_devices.items():
            if device in stepped_devices:
                wait_for_proximity(
                    connection, process, renode_log, physics,
                    round(STEPPED['rangefinder_m'][physics_index] * 100),
                    PROXIMITY_SENSOR_IDS.get(device, set(range(10, 18))),
                    deadline, 'stepped %s' % device)
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
        for device in OPTICAL_FLOW_EXPECTATIONS:
            if device not in stepped_devices:
                continue
            wait_for_optical_flow(
                connection, process, renode_log, physics, device,
                dict(STEPPED, **OPTICAL_FLOW_STEPPED), deadline, 'stepped')
        if AIS_DEVICE in stepped_devices:
            wait_for_ais(
                connection, process, renode_log, physics,
                dict(STEPPED, **GPS_STEPPED), AIS_BASELINE_OFFSET, deadline,
                'stepped', static_data=True)
        if SERIAL_BEACON_DEVICES.intersection(stepped_devices):
            wait_with_process_checks(
                connection, process, renode_log, physics, 2.0)
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
        if ('output-suppression-recovery' in
                assertions.get('nmea-airspeed', ())):
            device = 'nmea-airspeed'
            model = runtime_names[device]
            monitor_command(monitor, '%s SuppressOutput true' % model)
            if not monitor_bool_property(monitor, model, 'SuppressOutput'):
                raise RuntimeError('Renode did not suppress NMEA airspeed')
            wait_for_airspeed_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s SuppressOutput false' % model)
            if monitor_bool_property(monitor, model, 'SuppressOutput'):
                raise RuntimeError('Renode did not restore NMEA airspeed')
            wait_for_airspeed_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_nmea_airspeed(
                connection, process, renode_log, physics,
                STEPPED, deadline, 'suppression recovery')
        if ('output-corruption-recovery' in
                assertions.get('nmea-airspeed', ())):
            device = 'nmea-airspeed'
            model = runtime_names[device]
            monitor_command(monitor, '%s OutputXorMask 255' % model)
            if monitor_property(monitor, model, 'OutputXorMask') != 255:
                raise RuntimeError('Renode did not corrupt NMEA airspeed')
            wait_for_airspeed_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s OutputXorMask 0' % model)
            if monitor_property(monitor, model, 'OutputXorMask') != 0:
                raise RuntimeError('Renode did not restore NMEA airspeed')
            wait_for_airspeed_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_nmea_airspeed(
                connection, process, renode_log, physics,
                STEPPED, deadline, 'corruption recovery')
        if ('output-suppression-recovery' in
                assertions.get('nmea-wind-vane', ())):
            device = 'nmea-wind-vane'
            model = runtime_names[device]
            monitor_command(monitor, '%s SuppressOutput true' % model)
            monitor_command(
                monitor, '%s WindDirectionDegrees %.1f' %
                (model, WIND_SUPPRESSED[0]))
            monitor_command(
                monitor, '%s WindSpeedMS %.1f' %
                (model, WIND_SUPPRESSED[1]))
            wait_for_stale_wind(
                connection, process, renode_log, physics,
                WIND_STEPPED, WIND_SUPPRESSED, deadline, 'output suppression')
            monitor_command(monitor, '%s SuppressOutput false' % model)
            wait_for_wind(
                connection, process, renode_log, physics,
                WIND_SUPPRESSED, deadline, 'suppression recovery')
        if ('output-corruption-recovery' in
                assertions.get('nmea-wind-vane', ())):
            device = 'nmea-wind-vane'
            model = runtime_names[device]
            monitor_command(monitor, '%s OutputXorMask 255' % model)
            monitor_command(
                monitor, '%s WindDirectionDegrees %.1f' %
                (model, WIND_CORRUPT[0]))
            monitor_command(
                monitor, '%s WindSpeedMS %.1f' %
                (model, WIND_CORRUPT[1]))
            wait_for_stale_wind(
                connection, process, renode_log, physics,
                WIND_SUPPRESSED, WIND_CORRUPT, deadline, 'output corruption')
            monitor_command(monitor, '%s OutputXorMask 0' % model)
            wait_for_wind(
                connection, process, renode_log, physics,
                WIND_CORRUPT, deadline, 'corruption recovery')
        for device in SERIAL_OPTICAL_FLOW_DEVICES:
            model = runtime_names.get(device)
            if 'checksum-recovery' in assertions.get(device, ()):
                monitor_command(monitor, '%s CorruptChecksum true' % model)
                if not monitor_bool_property(
                        monitor, model, 'CorruptChecksum'):
                    raise RuntimeError(
                        'Renode did not corrupt %s checksum' % device)
                wait_for_optical_flow_health(
                    connection, process, renode_log, physics, False, deadline)
                monitor_command(monitor, '%s CorruptChecksum false' % model)
                if monitor_bool_property(monitor, model, 'CorruptChecksum'):
                    raise RuntimeError(
                        'Renode did not restore %s checksum' % device)
                wait_for_optical_flow_health(
                    connection, process, renode_log, physics, True, deadline)
                wait_for_optical_flow(
                    connection, process, renode_log, physics, device,
                    dict(STEPPED, **OPTICAL_FLOW_STEPPED), deadline,
                    'checksum recovery')
            if ('output-suppression-recovery' in
                    assertions.get(device, ())):
                monitor_command(monitor, '%s SuppressOutput true' % model)
                if not monitor_bool_property(
                        monitor, model, 'SuppressOutput'):
                    raise RuntimeError(
                        'Renode did not suppress %s output' % device)
                wait_for_optical_flow_health(
                    connection, process, renode_log, physics, False, deadline)
                monitor_command(monitor, '%s SuppressOutput false' % model)
                if monitor_bool_property(monitor, model, 'SuppressOutput'):
                    raise RuntimeError(
                        'Renode did not restore %s output' % device)
                wait_for_optical_flow_health(
                    connection, process, renode_log, physics, True, deadline)
                wait_for_optical_flow(
                    connection, process, renode_log, physics, device,
                    dict(STEPPED, **OPTICAL_FLOW_STEPPED), deadline,
                    'suppression recovery')
            if ('output-corruption-recovery' in
                    assertions.get(device, ())):
                monitor_command(monitor, '%s OutputXorMask 255' % model)
                if monitor_property(monitor, model, 'OutputXorMask') != 255:
                    raise RuntimeError(
                        'Renode did not corrupt %s output' % device)
                wait_for_optical_flow_health(
                    connection, process, renode_log, physics, False, deadline)
                monitor_command(monitor, '%s OutputXorMask 0' % model)
                if monitor_property(monitor, model, 'OutputXorMask') != 0:
                    raise RuntimeError(
                        'Renode did not restore %s output' % device)
                wait_for_optical_flow_health(
                    connection, process, renode_log, physics, True, deadline)
                wait_for_optical_flow(
                    connection, process, renode_log, physics, device,
                    dict(STEPPED, **OPTICAL_FLOW_STEPPED), deadline,
                    'corruption recovery')
        ais_checks = assertions.get(AIS_DEVICE, ())
        if ais_checks:
            model = runtime_names[AIS_DEVICE]
            truth = dict(STEPPED, **GPS_STEPPED)
            if 'checksum-recovery' in ais_checks:
                monitor_command(monitor, '%s CorruptChecksum true' % model)
                set_ais_offsets(monitor, model, AIS_CHECKSUM_OFFSET)
                reject_ais_value(
                    connection, process, renode_log, physics, truth,
                    AIS_CHECKSUM_OFFSET, 2.0, 'checksum-corrupt')
                monitor_command(monitor, '%s CorruptChecksum false' % model)
                wait_for_ais(
                    connection, process, renode_log, physics, truth,
                    AIS_CHECKSUM_OFFSET, deadline, 'checksum recovery', True)
            if 'output-suppression-recovery' in ais_checks:
                monitor_command(monitor, '%s SuppressOutput true' % model)
                set_ais_offsets(monitor, model, AIS_SUPPRESSED_OFFSET)
                reject_ais_value(
                    connection, process, renode_log, physics, truth,
                    AIS_SUPPRESSED_OFFSET, 2.0, 'suppressed')
                monitor_command(monitor, '%s SuppressOutput false' % model)
                wait_for_ais(
                    connection, process, renode_log, physics, truth,
                    AIS_SUPPRESSED_OFFSET, deadline, 'suppression recovery',
                    True)
            if 'output-corruption-recovery' in ais_checks:
                monitor_command(monitor, '%s OutputXorMask 255' % model)
                set_ais_offsets(monitor, model, AIS_CORRUPT_OFFSET)
                reject_ais_value(
                    connection, process, renode_log, physics, truth,
                    AIS_CORRUPT_OFFSET, 2.0, 'byte-corrupt')
                monitor_command(monitor, '%s OutputXorMask 0' % model)
                wait_for_ais(
                    connection, process, renode_log, physics, truth,
                    AIS_CORRUPT_OFFSET, deadline, 'corruption recovery', True)
        for device in SERIAL_BEACON_DEVICES:
            beacon_checks = assertions.get(device, ())
            if not beacon_checks:
                continue
            name = ATTACHABLE_DEVICES[device]['name']
            model = runtime_names[device]
            fault_dwell = 2.0 if 'settings-request' in beacon_checks else 1.0
            if 'checksum-recovery' in beacon_checks:
                monitor_command(monitor, '%s CorruptChecksum true' % model)
                if not monitor_bool_property(
                        monitor, model, 'CorruptChecksum'):
                    raise RuntimeError('Renode did not corrupt %s checksum' %
                                       name)
                wait_with_process_checks(
                    connection, process, renode_log, physics, fault_dwell)
                monitor_command(monitor, '%s CorruptChecksum false' % model)
                wait_with_process_checks(
                    connection, process, renode_log, physics, fault_dwell)
            if 'output-suppression-recovery' in beacon_checks:
                monitor_command(monitor, '%s SuppressOutput true' % model)
                if not monitor_bool_property(monitor, model, 'SuppressOutput'):
                    raise RuntimeError('Renode did not suppress %s output' %
                                       name)
                wait_with_process_checks(
                    connection, process, renode_log, physics, fault_dwell)
                monitor_command(monitor, '%s SuppressOutput false' % model)
                wait_with_process_checks(
                    connection, process, renode_log, physics, fault_dwell)
            if 'output-corruption-recovery' in beacon_checks:
                monitor_command(monitor, '%s OutputXorMask 255' % model)
                if monitor_property(monitor, model, 'OutputXorMask') != 255:
                    raise RuntimeError('Renode did not corrupt %s output' %
                                       name)
                wait_with_process_checks(
                    connection, process, renode_log, physics, fault_dwell)
                monitor_command(monitor, '%s OutputXorMask 0' % model)
                wait_with_process_checks(
                    connection, process, renode_log, physics, fault_dwell)
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
        for device in GPS_DEVICES:
            if ('output-suppression-recovery' not in
                    assertions.get(device, ())):
                continue
            model = runtime_names[device]
            monitor_command(monitor, '%s SuppressOutput true' % model)
            if not monitor_bool_property(monitor, model, 'SuppressOutput'):
                raise RuntimeError('Renode did not suppress %s output' %
                                   GPS_DEVICES[device])
            wait_for_gps_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s SuppressOutput false' % model)
            if monitor_bool_property(monitor, model, 'SuppressOutput'):
                raise RuntimeError('Renode did not restore %s output' %
                                   GPS_DEVICES[device])
            wait_for_gps_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_gps_fix(
                connection, process, renode_log, physics, deadline)
            wait_for_values(
                connection, process, renode_log, physics,
                dict(STEPPED, **GPS_STEPPED), {device}, deadline,
                'recovered after silence')
        for device in GPS_DEVICES:
            if ('output-corruption-recovery' not in
                    assertions.get(device, ())):
                continue
            model = runtime_names[device]
            monitor_command(monitor, '%s OutputXorMask 255' % model)
            if monitor_property(monitor, model, 'OutputXorMask') != 255:
                raise RuntimeError('Renode did not corrupt %s output' %
                                   GPS_DEVICES[device])
            wait_for_gps_health(
                connection, process, renode_log, physics, False, deadline)
            monitor_command(monitor, '%s OutputXorMask 0' % model)
            if monitor_property(monitor, model, 'OutputXorMask') != 0:
                raise RuntimeError(
                    'Renode did not clear %s corruption' % GPS_DEVICES[device])
            wait_for_gps_health(
                connection, process, renode_log, physics, True, deadline)
            wait_for_gps_fix(
                connection, process, renode_log, physics, deadline)
            wait_for_values(
                connection, process, renode_log, physics,
                dict(STEPPED, **GPS_STEPPED), {device}, deadline,
                'recovered after corruption')
        if check_irlock_dataflash:
            common.download_log(
                connection, process, renode_log, irlock_log,
                timeout=max(30, deadline - time.monotonic()))
        if dataflash_beacon_device is not None:
            common.download_log(
                connection, process, renode_log, beacon_log,
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
    if dataflash_beacon_device is not None:
        check_beacon_log(beacon_log, dataflash_beacon_device)


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
