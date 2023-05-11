/*
    Translated from Tools/scripts/decode_devid.py
*/

const DEVICE_TYPE_COMPASS = 0
const DEVICE_TYPE_IMU = 1
const DEVICE_TYPE_BARO = 2
const DEVICE_TYPE_AIRSPEED = 3

function decode_devid(ID, type) {

const bus_type = ID & 0x07
const bus =     (ID>>3) & 0x1F
const address = (ID>>8) & 0xFF
const devtype = (ID>>16)

bustypes = {
    1: "I2C",
    2: "SPI",
    3: "DRONECAN",
    4: "SITL",
    5: "MSP",
    6: "SERIAL",
}

compass_types = {
    0x01 : "HMC5883_OLD",
    0x07 : "HMC5883",
    0x02 : "LSM303D",
    0x04 : "AK8963 ",
    0x05 : "BMM150 ",
    0x06 : "LSM9DS1",
    0x08 : "LIS3MDL",
    0x09 : "AK09916",
    0x0A : "IST8310",
    0x0B : "ICM20948",
    0x0C : "MMC3416",
    0x0D : "QMC5883L",
    0x0E : "MAG3110",
    0x0F : "SITL",
    0x10 : "IST8308",
    0x11 : "RM3100_OLD",
    0x12 : "RM3100",
    0x13 : "MMC5883",
    0x14 : "AK09918",
}

imu_types = {
    0x09 : "BMI160",
    0x10 : "L3G4200D",
    0x11 : "ACC_LSM303D",
    0x12 : "ACC_BMA180",
    0x13 : "ACC_MPU6000",
    0x16 : "ACC_MPU9250",
    0x17 : "ACC_IIS328DQ",
    0x21 : "GYR_MPU6000",
    0x22 : "GYR_L3GD20",
    0x24 : "GYR_MPU9250",
    0x25 : "GYR_I3G4250D",
    0x26 : "GYR_LSM9DS1",
    0x27 : "ICM20789",
    0x28 : "ICM20689",
    0x29 : "BMI055",
    0x2A : "SITL",
    0x2B : "BMI088",
    0x2C : "ICM20948",
    0x2D : "ICM20648",
    0x2E : "ICM20649",
    0x2F : "ICM20602",
    0x30 : "ICM20601",
    0x31 : "ADIS1647x",
    0x32 : "SERIAL",
    0x33 : "ICM40609",
    0x34 : "ICM42688",
    0x35 : "ICM42605",
    0x36 : "ICM40605",
    0x37 : "IIM42652",
    0x38 : "BMI270",
    0x39 : "BMI085",
    0x3A : "ICM42670",
}

baro_types = {
    0x01 : "SITL",
    0x02 : "BMP085",
    0x03 : "BMP280",
    0x04 : "BMP388",
    0x05 : "DPS280",
    0x06 : "DPS310",
    0x07 : "FBM320",
    0x08 : "ICM20789",
    0x09 : "KELLERLD",
    0x0A : "LPS2XH",
    0x0B : "MS5611",
    0x0C : "SPL06",
    0x0D : "DRONECAN",
    0x0E : "MSP",
    0x0F : "ICP101XX",
    0x10 : "ICP201XX",
    0x11 : "MS5607",
    0x12 : "MS5837",
    0x13 : "MS5637",
    0x14 : "BMP390",
}

airspeed_types = {
    0x01 : "SITL",
    0x02 : "MS4525",
    0x03 : "MS5525",
    0x04 : "DLVR",
    0x05 : "MSP",
    0x06 : "SDP3X",
    0x07 : "DRONECAN",
    0x08 : "ANALOG",
    0x09 : "NMEA",
    0x0A : "ASP5033",
}

function get(lookup, index) {
    if (lookup[index] != null) {
        return lookup[index]
    }
    return "Unknown"
}

var name
switch (type) {
    case DEVICE_TYPE_COMPASS:
        name = "Compass: " + get(compass_types, devtype)
        break
    case DEVICE_TYPE_IMU:
        name = "IMU: " + get(imu_types, devtype)
        break
    case DEVICE_TYPE_BARO:
        name = "Baro: " + get(baro_types, devtype)
        break
    case DEVICE_TYPE_AIRSPEED:
        name = "Airspeed: " + get(airspeed_types, devtype)
        break
    default:
        console.error("Unknown type");
        return
}

const bus_type_string = get(bustypes, bus_type)

if (bus_type == 3) {
    // dronecan devtype represents sensor_id
    return { bus_type: bus_type_string, bus: bus, address: address, sensor_id: devtype-1, name: name }
}

return { bus_type: bus_type_string, bus: bus, address: address, devtype: devtype, name: name }

}
