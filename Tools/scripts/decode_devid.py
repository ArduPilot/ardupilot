#!/usr/bin/env python
'''
decode a device ID, such as used for COMPASS_DEV_ID, INS_ACC_ID etc

To understand the devtype you should look at the backend headers for
the sensor library, such as libraries/AP_Compass/AP_Compass_Backend.h
'''

import sys
import optparse

parser = optparse.OptionParser("decode_devid.py")
parser.add_option("-C", "--compass", action='store_true', help='decode compass IDs')
parser.add_option("-I", "--imu", action='store_true', help='decode IMU IDs')

opts, args = parser.parse_args()

if len(args) == 0:
    print("Please supply a device ID")
    sys.exit(1)

devid=int(args[0])

bus_type=devid & 0x07
bus=(devid>>3) & 0x1F
address=(devid>>8)&0xFF
devtype=(devid>>16)

bustypes = {
    1: "I2C",
    2: "SPI",
    3: "UAVCAN",
    4: "SITL"
}

compass_types = {
    0x01 : "DEVTYPE_HMC5883_OLD",
    0x07 : "DEVTYPE_HMC5883",
    0x02 : "DEVTYPE_LSM303D",
    0x04 : "DEVTYPE_AK8963 ",
    0x05 : "DEVTYPE_BMM150 ",
    0x06 : "DEVTYPE_LSM9DS1",
    0x08 : "DEVTYPE_LIS3MDL",
    0x09 : "DEVTYPE_AK09916",
    0x0A : "DEVTYPE_IST8310",
    0x0B : "DEVTYPE_ICM20948",
    0x0C : "DEVTYPE_MMC3416",
    0x0D : "DEVTYPE_QMC5883L",
    0x0E : "DEVTYPE_MAG3110",
    0x0F : "DEVTYPE_SITL"
}

imu_types = {
    0x09 : "DEVTYPE_BMI160",
    0x10 : "DEVTYPE_L3G4200D",
    0x11 : "DEVTYPE_ACC_LSM303D",
    0x12 : "DEVTYPE_ACC_BMA180",
    0x13 : "DEVTYPE_ACC_MPU6000",
    0x16 : "DEVTYPE_ACC_MPU9250",
    0x17 : "DEVTYPE_ACC_IIS328DQ",
    0x21 : "DEVTYPE_GYR_MPU6000",
    0x22 : "DEVTYPE_GYR_L3GD20",
    0x24 : "DEVTYPE_GYR_MPU9250",
    0x25 : "DEVTYPE_GYR_I3G4250D",
    0x26 : "DEVTYPE_GYR_LSM9DS1",
    0x27 : "DEVTYPE_INS_ICM20789",
    0x28 : "DEVTYPE_INS_ICM20689",
    0x29 : "DEVTYPE_INS_BMI055",
    0x2A : "DEVTYPE_SITL",
}

decoded_devname = ""

if opts.compass:
    decoded_devname = compass_types.get(devtype, "UNKNOWN")

if opts.imu:
    decoded_devname = imu_types.get(devtype, "UNKNOWN")

print("bus_type:%s(%u)  bus:%u address:%u(0x%x) devtype:%u(0x%x) %s" % (
    bustypes.get(bus_type,"UNKNOWN"), bus_type,
    bus, address, address, devtype, devtype, decoded_devname))
