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

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "AP_Compass_AK8963.h"
#include <AP_InertialSensor/AP_InertialSensor_MPU9250.h>

#define AK8963_I2C_ADDR                                 0x0c

#define AK8963_WIA                                      0x00
#        define AK8963_Device_ID                        0x48

#define AK8963_HXL                                      0x03

/* bit definitions for AK8963 CNTL1 */
#define AK8963_CNTL1                                    0x0A
#        define    AK8963_CONTINUOUS_MODE1              0x02
#        define    AK8963_CONTINUOUS_MODE2              0x06
#        define    AK8963_SELFTEST_MODE                 0x08
#        define    AK8963_POWERDOWN_MODE                0x00
#        define    AK8963_FUSE_MODE                     0x0f
#        define    AK8963_16BIT_ADC                     0x10
#        define    AK8963_14BIT_ADC                     0x00

#define AK8963_CNTL2                                    0x0B
#        define AK8963_RESET                            0x01

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

#define AK8963_MILLIGAUSS_SCALE 10.0f

extern const AP_HAL::HAL& hal;

AP_Compass_AK8963::AP_Compass_AK8963(Compass &compass, AP_AK8963_SerialBus *bus) :
    AP_Compass_Backend(compass),
    _initialized(false),
    _last_update_timestamp(0),
    _last_accum_time(0),
    _bus(bus)
{
    _reset_filter();
}

AP_Compass_Backend *AP_Compass_AK8963::detect_mpu9250(Compass &compass, uint8_t mpu9250_instance)
{
    AP_InertialSensor &ins = *AP_InertialSensor::get_instance();
    AP_AK8963_SerialBus *bus = new AP_AK8963_SerialBus_MPU9250(ins, AK8963_I2C_ADDR, mpu9250_instance);
    if (!bus)
        return nullptr;
    return _detect(compass, bus);
}

AP_Compass_Backend *AP_Compass_AK8963::detect_i2c(Compass &compass,
                                                  AP_HAL::I2CDriver *i2c,
                                                  uint8_t addr)
{
    AP_AK8963_SerialBus *bus = new AP_AK8963_SerialBus_I2C(i2c, addr);
    if (!bus)
        return nullptr;
    return _detect(compass, bus);
}

AP_Compass_Backend *AP_Compass_AK8963::detect_mpu9250_i2c(Compass &compass,
                                                          AP_HAL::I2CDriver *i2c,
                                                          uint8_t addr)
{
    AP_InertialSensor &ins = *AP_InertialSensor::get_instance();
    ins.detect_backends();
    return detect_i2c(compass, i2c, addr);
}

AP_Compass_Backend *AP_Compass_AK8963::_detect(Compass &compass,
                                               AP_AK8963_SerialBus *bus)
{
    AP_Compass_AK8963 *sensor = new AP_Compass_AK8963(compass, bus);
    if (sensor == nullptr) {
        delete bus;
        return nullptr;
    }
    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_AK8963::~AP_Compass_AK8963()
{
    delete _bus;
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

    if (!_check_id()) {
        hal.console->printf("AK8963: Wrong id\n");
        goto fail;
    }

    if (!_calibrate()) {
        hal.console->printf("AK8963: Could not read calibration data\n");
        goto fail;
    }

    if (!_setup_mode()) {
        hal.console->printf("AK8963: Could not setup mode\n");
        goto fail;
    }

    if (!_bus->start_measurements()) {
        hal.console->printf("AK8963: Could not start measurements\n");
        goto fail;
    }

    _initialized = true;

    /* register the compass instance in the frontend */
    _compass_instance = register_compass();
    set_dev_id(_compass_instance, _bus->get_dev_id());
    /* timer needs to be called every 10ms so set the freq_div to 10 */
    _timesliced = hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Compass_AK8963::_update, void), 10);

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

    hal.scheduler->suspend_timer_procs();
    auto field = _get_filtered_field();

    _reset_filter();
    hal.scheduler->resume_timer_procs();
    publish_filtered_field(field, _compass_instance);
}

Vector3f AP_Compass_AK8963::_get_filtered_field() const
{
    Vector3f field(_mag_x_accum, _mag_y_accum, _mag_z_accum);
    field /= _accum_count;

    return field;
}

void AP_Compass_AK8963::_reset_filter()
{
    _mag_x_accum = _mag_y_accum = _mag_z_accum = 0;
    _accum_count = 0;
}

void AP_Compass_AK8963::_make_adc_sensitivity_adjustment(Vector3f& field) const
{
    static const float ADC_16BIT_RESOLUTION = 0.15f;

    field *= ADC_16BIT_RESOLUTION;
}

void AP_Compass_AK8963::_make_factory_sensitivity_adjustment(Vector3f& field) const
{
    field.x *= _magnetometer_ASA[0];
    field.y *= _magnetometer_ASA[1];
    field.z *= _magnetometer_ASA[2];
}

void AP_Compass_AK8963::_update()
{
    struct AP_AK8963_SerialBus::raw_value rv;
    float mag_x, mag_y, mag_z;
    // get raw_field - sensor frame, uncorrected
    Vector3f raw_field;
    uint32_t time_us = AP_HAL::micros();

    if (!_timesliced &&
        AP_HAL::micros() - _last_update_timestamp < 10000) {
        goto end;
    }

    if (!_bus_sem->take_nonblocking()) {
        goto end;
    }

    _bus->read_raw(&rv);

    /* Check for overflow. See AK8963's datasheet, section
     * 6.4.3.6 - Magnetic Sensor Overflow. */
    if ((rv.st2 & 0x08)) {
        goto fail;
    }

    mag_x = (float) rv.val[0];
    mag_y = (float) rv.val[1];
    mag_z = (float) rv.val[2];

    if (is_zero(mag_x) && is_zero(mag_y) && is_zero(mag_z)) {
        goto fail;
    }

    raw_field = Vector3f(mag_x, mag_y, mag_z);

    _make_factory_sensitivity_adjustment(raw_field);
    _make_adc_sensitivity_adjustment(raw_field);
    raw_field *= AK8963_MILLIGAUSS_SCALE;

    // rotate raw_field from sensor frame to body frame
    rotate_field(raw_field, _compass_instance);

    // publish raw_field (uncorrected point sample) for calibration use
    publish_raw_field(raw_field, time_us, _compass_instance);

    // correct raw_field for known errors
    correct_field(raw_field, _compass_instance);

    // publish raw_field (corrected point sample) for EKF use
    publish_unfiltered_field(raw_field, time_us, _compass_instance);

    _mag_x_accum += raw_field.x;
    _mag_y_accum += raw_field.y;
    _mag_z_accum += raw_field.z;
    _accum_count++;
    if (_accum_count == 10) {
        _mag_x_accum /= 2;
        _mag_y_accum /= 2;
        _mag_z_accum /= 2;
        _accum_count = 5;
    }

    _last_update_timestamp = AP_HAL::micros();
fail:
    _bus_sem->give();
end:
    return;
}

bool AP_Compass_AK8963::_check_id()
{
    for (int i = 0; i < 5; i++) {
        uint8_t deviceid = 0;
        _bus->register_read(AK8963_WIA, &deviceid, 0x01); /* Read AK8963's id */

        if (deviceid == AK8963_Device_ID) {
            return true;
        }
    }

    return false;
}

bool AP_Compass_AK8963::_setup_mode() {
    _bus->register_write(AK8963_CNTL1, AK8963_CONTINUOUS_MODE2 | AK8963_16BIT_ADC);
    return true;
}

bool AP_Compass_AK8963::_reset()
{
    _bus->register_write(AK8963_CNTL2, AK8963_RESET);
    return true;
}


bool AP_Compass_AK8963::_calibrate()
{
    /* Enable FUSE-mode in order to be able to read calibration data */
    _bus->register_write(AK8963_CNTL1, AK8963_FUSE_MODE | AK8963_16BIT_ADC);

    uint8_t response[3];
    _bus->register_read(AK8963_ASAX, response, 3);

    for (int i = 0; i < 3; i++) {
        float data = response[i];
        _magnetometer_ASA[i] = ((data - 128) / 256 + 1);
        error("%d: %lf\n", i, _magnetometer_ASA[i]);
    }

    return true;
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
AP_AK8963_SerialBus_MPU9250::AP_AK8963_SerialBus_MPU9250(AP_InertialSensor &ins,
                                                         uint8_t addr,
                                                         uint8_t mpu9250_instance)
{
    // Only initialize members. Fails are handled by configure or while
    // getting the semaphore
    _bus = ins.get_auxiliary_bus(HAL_INS_MPU9250_SPI, mpu9250_instance);
    if (!_bus)
        AP_HAL::panic("Cannot get MPU9250 auxiliary bus");

    _slave = _bus->request_next_slave(addr);
}

AP_AK8963_SerialBus_MPU9250::~AP_AK8963_SerialBus_MPU9250()
{
    /* After started it's owned by AuxiliaryBus */
    if (!_started)
        delete _slave;
}

void AP_AK8963_SerialBus_MPU9250::register_write(uint8_t reg, uint8_t value)
{
    _slave->passthrough_write(reg, value);
}

void AP_AK8963_SerialBus_MPU9250::register_read(uint8_t reg, uint8_t *value, uint8_t count)
{
    _slave->passthrough_read(reg, value, count);
}

void AP_AK8963_SerialBus_MPU9250::read_raw(struct raw_value *rv)
{
    if (_started) {
        _slave->read((uint8_t*)rv);
        return;
    }

    _slave->passthrough_read(0x03, (uint8_t*)rv, sizeof(*rv));
}

AP_HAL::Semaphore * AP_AK8963_SerialBus_MPU9250::get_semaphore()
{
    return _bus ? _bus->get_semaphore() : nullptr;
}

bool AP_AK8963_SerialBus_MPU9250::start_measurements()
{
    if (_bus->register_periodic_read(_slave, AK8963_HXL, sizeof(struct raw_value)) < 0)
        return false;

    _started = true;

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

void AP_AK8963_SerialBus_I2C::register_write(uint8_t reg, uint8_t value)
{
    _i2c->writeRegister(_addr, reg, value);
}

void AP_AK8963_SerialBus_I2C::register_read(uint8_t reg, uint8_t *value, uint8_t count)
{
    _i2c->readRegisters(_addr, reg, count, value);
}

void AP_AK8963_SerialBus_I2C::read_raw(struct raw_value *rv)
{
    _i2c->readRegisters(_addr, AK8963_HXL, sizeof(*rv), (uint8_t *) rv);
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
