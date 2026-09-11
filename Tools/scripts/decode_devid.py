#!/usr/bin/env python3

"""
Decode a device ID such as used for COMPASS_DEV_ID, INS_ACC_ID etc

To understand the devtype you should look at the backend headers for
the sensor library, such as libraries/AP_Compass/AP_Compass_Backend.h

AP_FLAKE8_CLEAN

SPDX-FileCopyrightText: 2017-2026 ArduPilot developers

SPDX-License-Identifier: GPL-3.0-or-later

"""

import argparse
import sys

from typing import Dict
from typing import Literal
from typing import Optional
from typing import Tuple
from typing import TypedDict

# Device type lookup tables
BUSTYPES: Dict[int, str] = {
    1: "I2C",
    2: "SPI",
    3: "DRONECAN",
    4: "SITL",
    5: "MSP",
    6: "SERIAL",
    7: "WSPI",
}

COMPASS_TYPES: Dict[int, str] = {
    0x01: "DEVTYPE_HMC5883_OLD",
    0x07: "DEVTYPE_HMC5883",
    0x02: "DEVTYPE_LSM303D",
    0x04: "DEVTYPE_AK8963 ",
    0x05: "DEVTYPE_BMM150 ",
    0x06: "DEVTYPE_LSM9DS1",
    0x08: "DEVTYPE_LIS3MDL",
    0x09: "DEVTYPE_AK0991x",
    0x0A: "DEVTYPE_IST8310",
    0x0B: "DEVTYPE_ICM20948",
    0x0C: "DEVTYPE_MMC3416",
    0x0D: "DEVTYPE_QMC5883L",
    0x0E: "DEVTYPE_MAG3110",
    0x0F: "DEVTYPE_SITL",
    0x10: "DEVTYPE_IST8308",
    0x11: "DEVTYPE_RM3100_OLD",
    0x12: "DEVTYPE_RM3100",
    0x13: "DEVTYPE_MMC5883",
    0x14: "DEVTYPE_AK09918",
    0x15: "DEVTYPE_AK09915",
    0x16: "DEVTYPE_QMC5883P",
    0x17: "DEVTYPE_BMM350",
    0x18: "DEVTYPE_IIS2MDC",
    0x19: "DEVTYPE_LIS2MDL",  # unused except on pre-release firmware
    0x1A: "DEVTYPE_AF9838",
}

IMU_TYPES: Dict[int, str] = {
    0x09: "DEVTYPE_BMI160",
    0x10: "DEVTYPE_L3G4200D",
    0x11: "DEVTYPE_ACC_LSM303D",
    0x12: "DEVTYPE_ACC_BMA180",
    0x13: "DEVTYPE_ACC_MPU6000",
    0x16: "DEVTYPE_ACC_MPU9250",
    0x17: "DEVTYPE_ACC_IIS328DQ",
    0x21: "DEVTYPE_GYR_MPU6000",
    0x22: "DEVTYPE_GYR_L3GD20",
    0x24: "DEVTYPE_GYR_MPU9250",
    0x25: "DEVTYPE_GYR_I3G4250D",
    0x26: "DEVTYPE_GYR_LSM9DS1",
    0x27: "DEVTYPE_INS_ICM20789",
    0x28: "DEVTYPE_INS_ICM20689",
    0x29: "DEVTYPE_INS_BMI055",
    0x2A: "DEVTYPE_SITL",
    0x2B: "DEVTYPE_INS_BMI088",
    0x2C: "DEVTYPE_INS_ICM20948",
    0x2D: "DEVTYPE_INS_ICM20648",
    0x2E: "DEVTYPE_INS_ICM20649",
    0x2F: "DEVTYPE_INS_ICM20602",
    0x30: "DEVTYPE_INS_ICM20601",
    0x31: "DEVTYPE_INS_ADIS1647x",
    0x32: "DEVTYPE_INS_SERIAL",
    0x33: "DEVTYPE_INS_ICM40609",
    0x34: "DEVTYPE_INS_ICM42688",
    0x35: "DEVTYPE_INS_ICM42605",
    0x36: "DEVTYPE_INS_ICM40605",
    0x37: "DEVTYPE_INS_IIM42652",
    0x38: "DEVTYPE_INS_BMI270",
    0x39: "DEVTYPE_INS_BMI085",
    0x3A: "DEVTYPE_INS_ICM42670",
    0x3B: "DEVTYPE_INS_ICM45686",
    0x3C: "DEVTYPE_INS_SCHA63T",
    0x3D: "DEVTYPE_INS_IIM42653",
    0x3E: "DEVTYPE_INS_LSM6DSV16X",
    0x3F: "DEVTYPE_INS_ASM330",
    0x40: "DEVTYPE_INS_ADIS16607",
    0x42: "DEVTYPE_INS_LSM6DSV32X",
    0x43: "DEVTYPE_INS_LSM6DSK320X",
}

BARO_TYPES: Dict[int, str] = {
    0x01: "DEVTYPE_BARO_SITL",
    0x02: "DEVTYPE_BARO_BMP085",
    0x03: "DEVTYPE_BARO_BMP280",
    0x04: "DEVTYPE_BARO_BMP388",
    0x05: "DEVTYPE_BARO_DPS280",
    0x06: "DEVTYPE_BARO_DPS310",
    0x07: "DEVTYPE_BARO_FBM320",
    0x08: "DEVTYPE_BARO_ICM20789",
    0x09: "DEVTYPE_BARO_KELLERLD",
    0x0A: "DEVTYPE_BARO_LPS2XH",
    0x0B: "DEVTYPE_BARO_MS5611",
    0x0C: "DEVTYPE_BARO_SPL06",
    0x0D: "DEVTYPE_BARO_DRONECAN",
    0x0E: "DEVTYPE_BARO_MSP",
    0x0F: "DEVTYPE_BARO_ICP101XX",
    0x10: "DEVTYPE_BARO_ICP201XX",
    0x11: "DEVTYPE_BARO_MS5607",
    0x12: "DEVTYPE_BARO_MS5837_30BA",
    0x13: "DEVTYPE_BARO_MS5637",
    0x14: "DEVTYPE_BARO_BMP390",
    0x15: "DEVTYPE_BARO_BMP581",
    0x16: "DEVTYPE_BARO_SPA06",
    0x17: "DEVTYPE_BARO_AUAV",
    0x18: "DEVTYPE_BARO_MS5837_02BA",
}

AIRSPEED_TYPES: Dict[int, str] = {
    0x01: "DEVTYPE_AIRSPEED_SITL",
    0x02: "DEVTYPE_AIRSPEED_MS4525",
    0x03: "DEVTYPE_AIRSPEED_MS5525",
    0x04: "DEVTYPE_AIRSPEED_DLVR",
    0x05: "DEVTYPE_AIRSPEED_MSP",
    0x06: "DEVTYPE_AIRSPEED_SDP3X",
    0x07: "DEVTYPE_AIRSPEED_DRONECAN",
    0x08: "DEVTYPE_AIRSPEED_ANALOG",
    0x09: "DEVTYPE_AIRSPEED_NMEA",
    0x0A: "DEVTYPE_AIRSPEED_ASP5033",
    0x0B: "DEVTYPE_AIRSPEED_AUAV",
    0x0C: "DEVTYPE_AIRSPEED_SCRIPTING",
}

MAVLINK_TYPES: Dict[int, str] = {
    0x01: "DEVTYPE_MAVLINK_UART",
    0x02: "DEVTYPE_MAVLINK_NETWORKING",
    0x03: "DEVTYPE_MAVLINK_CAN",
    0x04: "DEVTYPE_MAVLINK_SCRIPTING",
}


class DeviceInfo(TypedDict):
    bus_type_value: int
    bus_type_name: str
    bus: int
    address: int
    devtype: int
    is_dronecan: bool


DeviceCategory = Literal["compass", "imu", "baro", "airspeed", "mavlink"]


def parse_device_id(device_id_str: str) -> Tuple[Optional[int], Optional[str]]:
    """Parse a device ID and return a diagnostic message, if applicable."""
    # backwards compatibility:
    #  - assume all digit strings are decimal,
    #  - assume strings without "0x" prefix but with characters A-F are hexadecimal
    try:
        device_id = int(device_id_str)
    except ValueError:
        try:
            device_id = int(device_id_str, 16)
        except ValueError:
            return None, f"Invalid device ID '{device_id_str}'"

    if device_id < 0:
        return None, "device ID must not be negative"

    # Warn that ambiguous digits-only input is interpreted as decimal.
    if device_id_str.isdigit():
        return (
            device_id,
            f"Warning: device ID '{device_id_str}' is interpreted as decimal; prefix hexadecimal IDs with 0x.",
        )

    return device_id, None


def decode_device_id(device_id: int) -> Tuple[Optional[DeviceInfo], Optional[str]]:
    """
    Decode an ArduPilot device ID into its components.

    Args:
        device_id: Device ID as integer

    Returns:
        A tuple containing the decoded dictionary and an error message. If
        device_id does not fit in the 24-bit device ID format, the decoded
        dictionary is None. Otherwise, it contains these keys:
            - bus_type_value: Bus type numeric value
            - bus_type_name: Bus type name (I2C, SPI, etc.)
            - bus: Bus number
            - address: Device address
            - devtype: Device type numeric value
            - is_dronecan: Whether this is DRONECAN bus type

    """
    if device_id < 0 or device_id > 0x00FFFFFF:
        return None, f"Error: device ID 0x{device_id:x} is outside the 24-bit range."

    bus_type = device_id & 0x07
    bus = (device_id >> 3) & 0x1F
    address = (device_id >> 8) & 0xFF
    devtype = (device_id >> 16) & 0xFF

    decoded: DeviceInfo = {
        "bus_type_value": bus_type,
        "bus_type_name": BUSTYPES.get(bus_type, "UNKNOWN"),
        "bus": bus,
        "address": address,
        "devtype": devtype,
        "is_dronecan": bus_type == 3,
    }
    return decoded, None


def get_device_type_name(devtype: int, device_category: DeviceCategory) -> str:
    """
    Look up device type name based on device category.

    Args:
        devtype: Device type numeric value
        device_category: One of 'compass', 'imu', 'baro', 'airspeed', 'mavlink'

    Returns:
        Device type name string, or 'UNKNOWN' if not found

    """
    category_map = {
        "compass": COMPASS_TYPES,
        "imu": IMU_TYPES,
        "baro": BARO_TYPES,
        "airspeed": AIRSPEED_TYPES,
        "mavlink": MAVLINK_TYPES,
    }

    try:
        type_dict = category_map[device_category.lower()]
    except KeyError:
        raise ValueError(f"Unknown device category: {device_category}") from None

    return type_dict.get(devtype, "UNKNOWN")


def format_device_info(decode_info: DeviceInfo, device_type_name: str = "") -> str:
    """
    Format decoded device info as a readable string.

    Args:
        decode_info: Dictionary returned by decode_device_id()
        device_type_name: Device type name (optional; not applicable to DroneCAN IDs)

    Returns:
        Formatted string with device information

    """
    bus_type = decode_info["bus_type_value"]
    bus_type_name = decode_info["bus_type_name"]
    bus = decode_info["bus"]
    address = decode_info["address"]
    devtype = decode_info["devtype"]
    is_dronecan = decode_info["is_dronecan"]

    if is_dronecan:
        # Compass DroneCAN IDs store sensor_id + 1; other DroneCAN IDs use 0.
        sensor_id = devtype - 1 if devtype > 0 else None
        sensor_id_info = (
            f"sensor_id:{sensor_id}(0x{sensor_id:x})"
            if sensor_id is not None
            else "sensor_id:not encoded"
        )
        return (
            f"bus_type:{bus_type_name}({bus_type})  bus:{bus} "
            f"address:{address}(0x{address:x}) {sensor_id_info}"
        )
    device_type_suffix = f" {device_type_name}" if device_type_name else ""
    return (
        f"bus_type:{bus_type_name}({bus_type})  bus:{bus} "
        f"address:{address}(0x{address:x}) devtype:{devtype}(0x{devtype:x})"
        f"{device_type_suffix}"
    )


def main() -> None:
    """Command-line interface."""
    parser = argparse.ArgumentParser(description="DEVICE_ID must be decimal or hexadecimal with a 0x prefix")
    category_group = parser.add_mutually_exclusive_group()
    category_group.add_argument("-C", "--compass", action="store_true", help="decode compass IDs")
    category_group.add_argument("-I", "--imu", action="store_true", help="decode IMU IDs")
    category_group.add_argument("-B", "--baro", action="store_true", help="decode barometer IDs")
    category_group.add_argument("-A", "--airspeed", action="store_true", help="decode airspeed IDs")
    category_group.add_argument("-M", "--mavlink", action="store_true", help="decode MAVLink channel IDs")
    parser.add_argument("device_id", help="decimal or hexadecimal device ID")

    opts = parser.parse_args()

    device_id, err_msg = parse_device_id(opts.device_id)
    if err_msg is not None:
        print(err_msg, file=sys.stderr)
    if device_id is None:
        sys.exit(1)

    decoded, err_msg = decode_device_id(device_id)
    if err_msg is not None:
        print(err_msg, file=sys.stderr)
        sys.exit(1)

    if decoded is None:
        sys.exit(1)

    device_type_name = ""
    if not decoded["is_dronecan"]:
        if opts.compass:
            device_type_name = get_device_type_name(decoded["devtype"], "compass")
        elif opts.imu:
            device_type_name = get_device_type_name(decoded["devtype"], "imu")
        elif opts.baro:
            device_type_name = get_device_type_name(decoded["devtype"], "baro")
        elif opts.airspeed:
            device_type_name = get_device_type_name(decoded["devtype"], "airspeed")
        elif opts.mavlink:
            device_type_name = get_device_type_name(decoded["devtype"], "mavlink")

    print(format_device_info(decoded, device_type_name))


if __name__ == "__main__":
    main()
