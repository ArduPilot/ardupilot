/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_Compass_AK8963.cpp 
 *       Code by Georgii Staroselskii. Emlid.com
 *
 *       Sensor is connected to SPI port
 *
 */

#include <AP_Math.h>
#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "AP_Compass_AK8963.h"
#include "../AP_InertialSensor/AP_InertialSensor_MPU9250.h"

#define READ_FLAG                   0x80
#define MPUREG_I2C_SLV0_ADDR        0x25
#define MPUREG_I2C_SLV0_REG         0x26
#define MPUREG_I2C_SLV0_CTRL        0x27
#define MPUREG_EXT_SENS_DATA_00     0x49
#define MPUREG_I2C_SLV0_DO          0x63

#define MPUREG_PWR_MGMT_1                               0x6B
#       define BIT_PWR_MGMT_1_CLK_INTERNAL              0x00            // clock set to internal 8Mhz oscillator
#       define BIT_PWR_MGMT_1_CLK_XGYRO                 0x01            // PLL with X axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_YGYRO                 0x02            // PLL with Y axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_ZGYRO                 0x03            // PLL with Z axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_EXT32KHZ              0x04            // PLL with external 32.768kHz reference
#       define BIT_PWR_MGMT_1_CLK_EXT19MHZ              0x05            // PLL with external 19.2MHz reference
#       define BIT_PWR_MGMT_1_CLK_STOP                  0x07            // Stops the clock and keeps the timing generator in reset
#       define BIT_PWR_MGMT_1_TEMP_DIS                  0x08            // disable temperature sensor
#       define BIT_PWR_MGMT_1_CYCLE                     0x20            // put sensor into cycle mode.  cycles between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL
#       define BIT_PWR_MGMT_1_SLEEP                     0x40            // put sensor into low power sleep mode
#       define BIT_PWR_MGMT_1_DEVICE_RESET              0x80            // reset entire device

/* bit definitions for MPUREG_USER_CTRL */
#define MPUREG_USER_CTRL                                0x6A
#       define BIT_USER_CTRL_I2C_MST_EN                 0x20            /* Enable MPU to act as the I2C Master to external slave sensors */
#       define BIT_USER_CTRL_I2C_IF_DIS                 0x10

/* bit definitions for MPUREG_MST_CTRL */
#define MPUREG_I2C_MST_CTRL                             0x24
#        define I2C_SLV0_EN                             0x80
#        define I2C_MST_CLOCK_400KHZ                    0x0D
#        define I2C_MST_CLOCK_258KHZ                    0x08

#define AK8963_I2C_ADDR                                 0x0c

#define AK8963_WIA                                      0x00
#        define AK8963_Device_ID                        0x48

#define AK8963_INFO                                     0x01

#define AK8963_ST1                                      0x02
#        define AK8963_DRDY                             0x01
#        define AK8963_DOR                              0x02

#define AK8963_HXL                                      0x03

/* bit definitions for AK8963 CNTL1 */
#define AK8963_CNTL1                                    0x0A
#        define    AK8963_CONTINUOUS_MODE1              0x2
#        define    AK8963_CONTINUOUS_MODE2              0x6
#        define    AK8963_SELFTEST_MODE                 0x8
#        define    AK8963_POWERDOWN_MODE                0x0
#        define    AK8963_FUSE_MODE                     0xf
#        define    AK8963_16BIT_ADC                     0x10
#        define    AK8963_14BIT_ADC                     0x00

#define AK8963_CNTL2                                    0x0B
#        define AK8963_RESET                            0x01

#define AK8963_ASTC                                     0x0C
#        define AK8983_SELFTEST_MAGNETIC_FIELD_ON       0x40

#define AK8963_ASAX                                     0x10

#define AK8963_DEBUG 0
#if AK8963_DEBUG
#include <stdio.h>
#define error(...) do { fprintf(stderr, __VA_ARGS__); } while (0)
#define ASSERT(x) assert(x)
#else
#define error(...) do { } while (0)
#ifndef ASSERT
#define ASSERT(x)
#endif
#endif

#if !defined(HAL_COMPASS_AK8963_I2C_ADDR)
#define HAL_COMPASS_AK8963_I2C_ADDR 0xC
#endif

extern const AP_HAL::HAL& hal;

AP_Compass_AK8963::AP_Compass_AK8963(Compass &compass, AP_AK8963_SerialBus *bus) :
    AP_Compass_Backend(compass),
    _state(STATE_UNKNOWN),
    _initialized(false),
    _last_update_timestamp(0),
    _last_accum_time(0),
    _bus(bus)
{
    _mag_x_accum =_mag_y_accum = _mag_z_accum = 0;
    _mag_x =_mag_y = _mag_z = 0;
    _accum_count = 0;
    _magnetometer_adc_resolution = AK8963_16BIT_ADC;
}

AP_Compass_Backend *AP_Compass_AK8963::detect_mpu9250(Compass &compass)
{
    AP_Compass_AK8963 *sensor = new AP_Compass_AK8963(compass,
                                                  new AP_AK8963_SerialBus_MPU9250());

    if (sensor == nullptr) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_Backend *AP_Compass_AK8963::detect_i2c1(Compass &compass)
{
    AP_Compass_AK8963 *sensor = new AP_Compass_AK8963(compass,
                                                  new AP_AK8963_SerialBus_I2C(
                                                  hal.i2c1, HAL_COMPASS_AK8963_I2C_ADDR));

    if (sensor == nullptr) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

/* stub to satisfy Compass API*/
void AP_Compass_AK8963::accumulate(void)
{
}

bool AP_Compass_AK8963::init()
{
    _bus_sem = _bus->get_semaphore();

    hal.scheduler->suspend_timer_procs();

    if (!_bus_sem->take(100)) {
        hal.console->printf("AK8963: Unable to get bus semaphore");
        goto fail_sem;
    }

    if (!_bus->configure()) {
        hal.console->printf_P(PSTR("AK8963: Bus not configured for AK8963\n"));
        goto fail;
    }

    if (!_configure()) {
        hal.console->printf_P(PSTR("AK8963: not configured\n"));
        goto fail;
    }

    if (!_check_id()) {
        hal.console->printf_P(PSTR("AK8963: wrong id\n"));
        goto fail;
    }

    if (!_calibrate()) {
        hal.console->printf_P(PSTR("AK8963: not calibrated\n"));
        goto fail;
    }

    if (!_bus->start_conversion()) {
        hal.console->printf_P(PSTR("AK8963: conversion not started\n"));
        goto fail;
    }

    _state = STATE_SAMPLE;
    _initialized = true;

    /* register the compass instance in the frontend */
    _compass_instance = register_compass();

    set_dev_id(_compass_instance, _bus->get_dev_id());

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Compass_AK8963::_update, void));

    _bus_sem->give();

    hal.scheduler->resume_timer_procs();

    return true;

fail:
    _bus_sem->give();
fail_sem:
    hal.scheduler->resume_timer_procs();

    return false;
}

void AP_Compass_AK8963::read()
{
    if (!_initialized) {
        return;
    }

    if (_accum_count == 0) {
        /* We're not ready to publish*/
        return;
    }

    /* Update */
    Vector3f field(_mag_x_accum * _magnetometer_ASA[0],
                   _mag_y_accum * _magnetometer_ASA[1],
                   _mag_z_accum * _magnetometer_ASA[2]);

    field /= _accum_count;
    _mag_x_accum = _mag_y_accum = _mag_z_accum = 0;
    _accum_count = 0;

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
    field.rotate(ROTATION_YAW_90);
#endif
    publish_field(field, _compass_instance);
}

void AP_Compass_AK8963::_update()
{
    if (hal.scheduler->micros() - _last_update_timestamp < 10000) {
        return;
    }

    if (!_sem_take_nonblocking()) {
        return;
    }

    switch (_state)
    {
        case STATE_SAMPLE:
            if (!_collect_samples()) {
                _state = STATE_ERROR;
            }
            break;
        case STATE_ERROR:
            if (_bus->start_conversion()) {
                _state = STATE_SAMPLE;
            }
            break;
        default:
            break;
    }

    _last_update_timestamp = hal.scheduler->micros();
    _sem_give();
}

bool AP_Compass_AK8963::_check_id()
{
    for (int i = 0; i < 5; i++) {
        uint8_t deviceid;
        _bus->register_read(AK8963_WIA, &deviceid, 0x01); /* Read AK8963's id */

        if (deviceid == AK8963_Device_ID) {
            return true;
        }
    }

    return false;
}

bool AP_Compass_AK8963::_configure() {
    _bus->register_write(AK8963_CNTL1, AK8963_CONTINUOUS_MODE2 | _magnetometer_adc_resolution);
    return true;
}

bool AP_Compass_AK8963::_reset()
{
    _bus->register_write(AK8963_CNTL2, AK8963_RESET);
    return true;
}


bool AP_Compass_AK8963::_calibrate()
{
    uint8_t cntl1 = _bus->register_read(AK8963_CNTL1);

    _bus->register_write(AK8963_CNTL1, AK8963_FUSE_MODE | _magnetometer_adc_resolution); /* Enable FUSE-mode in order to be able to read calibreation data */

    uint8_t response[3];
    _bus->register_read(AK8963_ASAX, response, 3);

    for (int i = 0; i < 3; i++) {
        float data = response[i];
        _magnetometer_ASA[i] = ((data - 128) / 256 + 1);
        error("%d: %lf\n", i, _magnetometer_ASA[i]);
    }

    _bus->register_write(AK8963_CNTL1, cntl1);

    return true;
}

bool AP_Compass_AK8963::_collect_samples()
{
    if (!_initialized) {
        return false;
    }

    if (!_bus->read_raw(_mag_x, _mag_y, _mag_z)) {
        return false;
    } else {
        _mag_x_accum += _mag_x;
        _mag_y_accum += _mag_y;
        _mag_z_accum += _mag_z;
        _accum_count++;
        if (_accum_count == 10) {
             _mag_x_accum /= 2;
             _mag_y_accum /= 2;
             _mag_z_accum /= 2;
             _accum_count = 5;
        }
    }

    return true;
}

bool AP_Compass_AK8963::_sem_take_blocking()
{
    return _bus_sem->take(10);
}

bool AP_Compass_AK8963::_sem_give()
{
    return _bus_sem->give();
}

bool AP_Compass_AK8963::_sem_take_nonblocking()
{
    static int _sem_failure_count = 0;

    bool got = _bus_sem->take_nonblocking();

    if (!got) {
        if (!hal.scheduler->system_initializing()) {
            _sem_failure_count++;
            if (_sem_failure_count > 100) {
                hal.scheduler->panic(PSTR("PANIC: failed to take _bus->sem "
                                          "100 times in a row, in "
                                          "AP_Compass_AK8963"));
            }
        }
        return false; /* never reached */
    } else {
        _sem_failure_count = 0;
    }
    return got;
}

void AP_Compass_AK8963::_dump_registers()
{
#if AK8963_DEBUG
    error("MPU9250 registers\n");
    static uint8_t regs[0x7e];

    _bus_read(0x0, regs, 0x7e);

    for (uint8_t reg=0x00; reg<=0x7E; reg++) {
        uint8_t v = regs[reg];
        error(("%d:%02x "), (unsigned)reg, (unsigned)v);
        if (reg  % 16 == 0) {
            error("\n");
        }
    }
    error("\n");
#endif
}

/* MPU9250 implementation of the AK8963 */
AP_AK8963_SerialBus_MPU9250::AP_AK8963_SerialBus_MPU9250()
{
    _spi = hal.spi->device(AP_HAL::SPIDevice_MPU9250);

    if (_spi == NULL) {
        hal.console->println_P(PSTR("Cannot get SPIDevice_MPU9250"));
        return;
    }
}

void AP_AK8963_SerialBus_MPU9250::register_write(uint8_t address, uint8_t value)
{
    _write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR);  /* Set the I2C slave addres of AK8963 and set for register_write. */
    _write(MPUREG_I2C_SLV0_REG, address); /* I2C slave 0 register address from where to begin data transfer */
    _write(MPUREG_I2C_SLV0_DO, value); /* Register value to continuous measurement in 16-bit */
    _write(MPUREG_I2C_SLV0_CTRL, I2C_SLV0_EN | 0x01); /* Enable I2C and set 1 byte */
}

void AP_AK8963_SerialBus_MPU9250::register_read(uint8_t address, uint8_t *value, uint8_t count)
{
    _write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG);  /* Set the I2C slave addres of AK8963 and set for read. */
    _write(MPUREG_I2C_SLV0_REG, address); /* I2C slave 0 register address from where to begin data transfer */
    _write(MPUREG_I2C_SLV0_CTRL, I2C_SLV0_EN | count); /* Enable I2C and set @count byte */

    hal.scheduler->delay(10);
    _read(MPUREG_EXT_SENS_DATA_00, value, count);
}

void AP_AK8963_SerialBus_MPU9250::_read(uint8_t address, uint8_t *buf, uint32_t count)
{
    ASSERT(count < 150);
    uint8_t tx[150];
    uint8_t rx[150];

    tx[0] = address | READ_FLAG;
    tx[1] = 0;
    _spi->transaction(tx, rx, count + 1);

    memcpy(buf, rx + 1, count);
}

void AP_AK8963_SerialBus_MPU9250::_write(uint8_t address, const uint8_t *buf, uint32_t count)
{
    ASSERT(count < 2);
    uint8_t tx[2];

    tx[0] = address;
    memcpy(tx+1, buf, count);

    _spi->transaction(tx, NULL, count + 1);
}

bool AP_AK8963_SerialBus_MPU9250::configure()
{
    if (!AP_InertialSensor_MPU9250::initialize_driver_state())
        return false;

    uint8_t user_ctrl;
    register_read(MPUREG_USER_CTRL, &user_ctrl, 1);
    _write(MPUREG_USER_CTRL, user_ctrl | BIT_USER_CTRL_I2C_MST_EN);
    _write(MPUREG_I2C_MST_CTRL, I2C_MST_CLOCK_400KHZ);

    return true;
}

bool AP_AK8963_SerialBus_MPU9250::read_raw(float &mag_x, float &mag_y, float &mag_z)
{
    uint8_t rx[14] = {0};

    const uint8_t count = 9;
    _read(MPUREG_EXT_SENS_DATA_00, rx, count);

    uint8_t st2 = rx[8]; /* End data read by reading ST2 register */

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx + 1] << 8) | v[2*idx]))

    if(!(st2 & 0x08)) {
        mag_x = (float) int16_val(rx, 1);
        mag_y = (float) int16_val(rx, 2);
        mag_z = (float) int16_val(rx, 3);

        if (is_zero(mag_x) && is_zero(mag_y) && is_zero(mag_z)) {
            return false;
        }

        return true;
    } else {
        return false;
    }
}

AP_HAL::Semaphore * AP_AK8963_SerialBus_MPU9250::get_semaphore()
{
    return _spi->get_semaphore();
}

bool AP_AK8963_SerialBus_MPU9250::start_conversion()
{
    static const uint8_t address = AK8963_INFO;
    /* Read registers from INFO through ST2 */
    static const uint8_t count = 0x09;

    configure();
    _write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG);  /* Set the I2C slave addres of AK8963 and set for read. */
    _write(MPUREG_I2C_SLV0_REG, address); /* I2C slave 0 register address from where to begin data transfer */
    _write(MPUREG_I2C_SLV0_CTRL, I2C_SLV0_EN | count); /* Enable I2C and set @count byte */

    return true;
}

uint32_t AP_AK8963_SerialBus_MPU9250::get_dev_id()
{
    return AP_COMPASS_TYPE_AK8963_MPU9250;
}

/* I2C implementation of the AK8963 */
AP_AK8963_SerialBus_I2C::AP_AK8963_SerialBus_I2C(AP_HAL::I2CDriver *i2c, uint8_t addr) :
    _i2c(i2c),
    _addr(addr)
{
}

void AP_AK8963_SerialBus_I2C::register_write(uint8_t address, uint8_t value)
{
    _i2c->writeRegister(_addr, address, value);
}

void AP_AK8963_SerialBus_I2C::register_read(uint8_t address, uint8_t *value, uint8_t count)
{
    _i2c->readRegisters(_addr, address, count, value);
}

bool AP_AK8963_SerialBus_I2C::read_raw(float &mag_x, float &mag_y, float &mag_z)
{
    uint8_t rx[9] = {0};

    const uint8_t count = 9;
    _i2c->readRegisters(_addr, AK8963_INFO, count, rx);

    uint8_t st2 = rx[8]; /* End data read by reading ST2 register */

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx + 1] << 8) | v[2*idx]))

    if(!(st2 & 0x08)) {
        mag_x = (float) int16_val(rx, 1);
        mag_y = (float) int16_val(rx, 2);
        mag_z = (float) int16_val(rx, 3);

        if (is_zero(mag_x) && is_zero(mag_y) && is_zero(mag_z)) {
            return false;
        }

        return true;
    } else {
        return false;
    }
}

AP_HAL::Semaphore * AP_AK8963_SerialBus_I2C::get_semaphore()
{
    return _i2c->get_semaphore();
}

uint32_t AP_AK8963_SerialBus_I2C::get_dev_id()
{
    return AP_COMPASS_TYPE_AK8963_I2C;
}

#endif // CONFIG_HAL_BOARD
