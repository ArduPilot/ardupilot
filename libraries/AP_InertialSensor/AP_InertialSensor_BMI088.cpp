#include "AP_InertialSensor_BMI088_Custom.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;


// ==========================
// PROBE
// ==========================
AP_InertialSensor_Backend* AP_InertialSensor_BMI088_Custom::probe(AP_InertialSensor &imu)
{
    AP_InertialSensor_BMI088_Custom *sensor = new AP_InertialSensor_BMI088_Custom();

    sensor->_dev = hal.spi->get_device("BMI088");

    if (!sensor->_dev) {
        return nullptr;
    }

    if (!sensor->init()) {
        return nullptr;
    }

    return sensor;
}


// ==========================
// INIT
// ==========================
bool AP_InertialSensor_BMI088_Custom::init()
{
    write_reg(0x7E, 0xB6); // reset
    hal.scheduler->delay(10);

    write_reg(0x7D, 0x04); // enable accel
    hal.scheduler->delay(50);

    return true;
}


// ==========================
// UPDATE
// ==========================
void AP_InertialSensor_BMI088_Custom::update()
{
    read_accel();
    read_gyro();
}


// ==========================
// READ REG
// ==========================
uint8_t AP_InertialSensor_BMI088_Custom::read_reg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 };
    uint8_t rx[2];

    _dev->transfer(tx, rx, 2);
    return rx[1];
}


// ==========================
// WRITE REG
// ==========================
void AP_InertialSensor_BMI088_Custom::write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    uint8_t rx[2];

    _dev->transfer(tx, rx, 2);
}


// ==========================
// READ ACCEL
// ==========================
void AP_InertialSensor_BMI088_Custom::read_accel()
{
    _ax = (read_reg(0x13) << 8) | read_reg(0x12);
    _ay = (read_reg(0x15) << 8) | read_reg(0x14);
    _az = (read_reg(0x17) << 8) | read_reg(0x16);
}


// ==========================
// READ GYRO
// ==========================
void AP_InertialSensor_BMI088_Custom::read_gyro()
{
    _gx = (read_reg(0x03) << 8) | read_reg(0x02);
    _gy = (read_reg(0x05) << 8) | read_reg(0x04);
    _gz = (read_reg(0x07) << 8) | read_reg(0x06);
}
