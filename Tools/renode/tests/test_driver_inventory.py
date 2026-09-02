#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Tests for the Renode peripheral-driver inventory."""

import importlib.util
import json
import sys

from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / 'driver_inventory.py'
sys.path.insert(0, str(MODULE_PATH.parent))
import driver_catalog  # noqa: E402

SPEC = importlib.util.spec_from_file_location('driver_inventory', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
driver_inventory = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(driver_inventory)

ROOT = Path(__file__).resolve().parents[3]


def test_live_catalog_is_consistent():
    assert driver_inventory.validate_catalog(ROOT) == []
    assert driver_inventory.validate_classifications(ROOT) == []
    assert driver_inventory.validate_probe_profiles(ROOT) == []
    assert driver_inventory.ATTACHABLE_DEVICES['ublox-gps']['parameters'] == (
        ('SERIAL{serial}_PROTOCOL', 5),
        ('SERIAL{serial}_BAUD', 230),
        ('GPS{instance}_TYPE', 2),
    )
    assert driver_inventory.ATTACHABLE_DEVICES[
        'benewake-rangefinder']['physics'] == {
            'source': 'rangefinder',
            'property': 'RangefinderIndex',
            'count': 10,
        }


def test_parameter_recipes_resolve_for_hwdef_ports():
    uart = {
        'id': 'SERIAL4',
        'name': 'SERIAL4 (UART7)',
        'bus': 'uart',
        'index': 4,
    }
    i2c = {
        'id': 'I2C2',
        'name': 'I2C2 (I2C1)',
        'bus': 'i2c',
        'index': 1,
    }

    assert driver_catalog.resolve_parameter_recipe(
        'ublox-gps', uart, instance=2) == (
            ('SERIAL4_PROTOCOL', 5),
            ('SERIAL4_BAUD', 230),
            ('GPS2_TYPE', 2),
        )
    assert driver_catalog.resolve_parameter_recipe(
        'ms4525-airspeed', i2c) == (
            ('ARSPD_TYPE', 1),
            ('ARSPD_BUS', '1'),
        )
    assert driver_catalog.resolve_parameter_recipe(
        'asp5033-airspeed', i2c, instance=2) == (
            ('ARSPD2_TYPE', 15),
            ('ARSPD2_BUS', '1'),
        )
    assert driver_catalog.resolve_parameter_recipe(
        'ist8310-compass', i2c) == ()


def test_parameter_recipe_rejects_wrong_bus():
    with pytest.raises(ValueError, match='cannot attach'):
        driver_catalog.resolve_parameter_recipe(
            'ublox-gps', {'id': 'I2C1', 'name': 'I2C1',
                          'bus': 'i2c', 'index': 0})


def test_live_inventory_finds_driver_families():
    result = driver_inventory.inventory(ROOT)

    assert 'GPS' in result['serial_protocols']
    assert 'Rangefinder' in result['serial_protocols']
    assert 'BMM350' in result['hwdef_i2c_probes']['compass']
    assert 'BMP581' in result['hwdef_i2c_probes']['barometer']
    assert 'BMI088' in result['hwdef_i2c_probes']['imu']
    assert 'AP_UBlox' in result['renode_models']['uart']
    assert 'AP_IST8310' in result['renode_models']['i2c']
    assert 'AP_BMP085' in result['renode_models']['i2c']
    assert 'AP_I2CRegisterDevice' not in result['renode_models']['i2c']
    assert result['catalog']['ublox-gps']['coverage'] == 'dynamic'
    assert result['catalog']['ublox-gps']['parameters'][0] == (
        'SERIAL{serial}_PROTOCOL', 5)
    assert result['probe_profiles']['matekh743-navigation']['devices'][0][
        'device'] == 'ublox-gps'
    assert result['probe_profiles']['matekh743-rangefinders']['devices'][1][
        'device'] == 'lightware-rangefinder'
    assert result['probe_profiles']['matekh743-airspeed']['devices'][0][
        'device'] == 'ms4525-airspeed'
    assert result['probe_profiles']['matekh743-airspeeds']['devices'][1][
        'instance'] == 2
    assert result['serial_protocol_classifications']['GPS'] == {
        'kind': 'device',
        'status': 'partial',
    }
    assert result['i2c_source_classifications'][
        'AP_Airspeed/AP_Airspeed_MS4525.cpp'] == 'device'
    assert result['i2c_source_classifications'][
        'GCS_MAVLink/GCS_DeviceOp.cpp'] == 'service'
    assert result['serial_source_classifications'][
        'AP_GPS/AP_GPS_UBLOX.cpp'] == {
            'role': 'device',
            'family': 'gnss-ublox',
        }
    assert result['serial_source_classifications'][
        'AP_GPS/AP_GPS.cpp']['role'] == 'frontend'
    assert result['serial_source_classifications'][
        'AP_RCTelemetry/AP_CRSF_Telem.cpp']['family'] == 'rc-crsf'


def test_inventory_json_cli(capsys):
    assert driver_inventory.main(['--root', str(ROOT), '--json']) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['catalog_errors'] == []
    assert result['classification_errors'] == []
    assert len(result['serial_protocols']) > 40


def test_catalog_check_cli(capsys):
    assert driver_inventory.main(['--root', str(ROOT), '--check']) == 0
    assert 'peripheral-driver inventory' in capsys.readouterr().out
