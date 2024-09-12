#!/usr/bin/env python
'''
decode a device ID, such as used for COMPASS_DEV_ID, INS_ACC_ID etc

To understand the devtype you should look at the backend headers for
the sensor library, such as libraries/AP_Compass/AP_Compass_Backend.h
'''

import sys
import optparse

def num(s):
    try:
        return int(s)
    except ValueError:
        return int(s, 16)


parser = optparse.OptionParser("decode_devid.py")
parser.add_option("-C", "--compass", action='store_true', help='decode compass IDs')
parser.add_option("-I", "--imu", action='store_true', help='decode IMU IDs')
parser.add_option("-B", "--baro", action='store_true', help='decode barometer IDs')
parser.add_option("-A", "--airspeed", action='store_true', help='decode airspeed IDs')

opts, args = parser.parse_args()

if len(args) == 0:
    print("Please supply a device ID")
    sys.exit(1)

devid=num(args[0])

bus_type=devid & 0x07
bus=(devid>>3) & 0x1F
address=(devid>>8)&0xFF
devtype=(devid>>16)

bustypes = {
    1: "I2C",
    2: "SPI",
    3: "DRONECAN",
    4: "SITL",
    5: "MSP",
    6: "SERIAL",
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
    0x0F : "DEVTYPE_SITL",
    0x10 : "DEVTYPE_IST8308",
    0x11 : "DEVTYPE_RM3100_OLD",
    0x12 : "DEVTYPE_RM3100",
    0x13 : "DEVTYPE_MMC5883",
    0x14 : "DEVTYPE_AK09918",
    0x15 : "DEVTYPE_AK09915",
    0x16 : "DEVTYPE_QMC5883P",
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
    0x2B : "DEVTYPE_INS_BMI088",
    0x2C : "DEVTYPE_INS_ICM20948",
    0x2D : "DEVTYPE_INS_ICM20648",
    0x2E : "DEVTYPE_INS_ICM20649",
    0x2F : "DEVTYPE_INS_ICM20602",
    0x30 : "DEVTYPE_INS_ICM20601",
    0x31 : "DEVTYPE_INS_ADIS1647x",
    0x32 : "DEVTYPE_INS_SERIAL",
    0x33 : "DEVTYPE_INS_ICM40609",
    0x34 : "DEVTYPE_INS_ICM42688",
    0x35 : "DEVTYPE_INS_ICM42605",
    0x36 : "DEVTYPE_INS_ICM40605",
    0x37 : "DEVTYPE_INS_IIM42652",
    0x38 : "DEVTYPE_INS_BMI270",
    0x39 : "DEVTYPE_INS_BMI085",
    0x3A : "DEVTYPE_INS_ICM42670",
    0x3B : "DEVTYPE_INS_ICM45686",
    0x3C : "DEVTYPE_INS_SCHA63T",
}

baro_types = {
    0x01 : "DEVTYPE_BARO_SITL",
    0x02 : "DEVTYPE_BARO_BMP085",
    0x03 : "DEVTYPE_BARO_BMP280",
    0x04 : "DEVTYPE_BARO_BMP388",
    0x05 : "DEVTYPE_BARO_DPS280",
    0x06 : "DEVTYPE_BARO_DPS310",
    0x07 : "DEVTYPE_BARO_FBM320",
    0x08 : "DEVTYPE_BARO_ICM20789",
    0x09 : "DEVTYPE_BARO_KELLERLD",
    0x0A : "DEVTYPE_BARO_LPS2XH",
    0x0B : "DEVTYPE_BARO_MS5611",
    0x0C : "DEVTYPE_BARO_SPL06",
    0x0D : "DEVTYPE_BARO_DRONECAN",
    0x0E : "DEVTYPE_BARO_MSP",
    0x0F : "DEVTYPE_BARO_ICP101XX",
    0x10 : "DEVTYPE_BARO_ICP201XX",
    0x11 : "DEVTYPE_BARO_MS5607",
    0x12 : "DEVTYPE_BARO_MS5837",
    0x13 : "DEVTYPE_BARO_MS5637",
    0x14 : "DEVTYPE_BARO_BMP390",
    0x15 : "DEVTYPE_BARO_BMP581",
}

airspeed_types = {
    0x01 : "DEVTYPE_AIRSPEED_SITL",
    0x02 : "DEVTYPE_AIRSPEED_MS4525",
    0x03 : "DEVTYPE_AIRSPEED_MS5525",
    0x04 : "DEVTYPE_AIRSPEED_DLVR",
    0x05 : "DEVTYPE_AIRSPEED_MSP",
    0x06 : "DEVTYPE_AIRSPEED_SDP3X",
    0x07 : "DEVTYPE_AIRSPEED_DRONECAN",
    0x08 : "DEVTYPE_AIRSPEED_ANALOG",
    0x09 : "DEVTYPE_AIRSPEED_NMEA",
    0x0A : "DEVTYPE_AIRSPEED_ASP5033",
}
    
decoded_devname = ""

if opts.compass:
    decoded_devname = compass_types.get(devtype, "UNKNOWN")

if opts.imu:
    decoded_devname = imu_types.get(devtype, "UNKNOWN")

if opts.baro:
    decoded_devname = baro_types.get(devtype, "UNKNOWN")

if opts.airspeed:
    decoded_devname = airspeed_types.get(devtype, "UNKNOWN")
    
if bus_type == 3:
    #dronecan devtype represents sensor_id
    print("bus_type:%s(%u)  bus:%u address:%u(0x%x) sensor_id:%u(0x%x) %s" % (
        bustypes.get(bus_type,"UNKNOWN"), bus_type,
        bus, address, address, devtype-1, devtype-1, decoded_devname))
else:
    print("bus_type:%s(%u)  bus:%u address:%u(0x%x) devtype:%u(0x%x) %s" % (
        bustypes.get(bus_type,"UNKNOWN"), bus_type,
        bus, address, address, devtype, devtype, decoded_devname))
