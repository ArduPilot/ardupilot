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

#include "AP_Compass_LSM303D.h"

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/GPIO.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
#define LSM303D_DRDY_M_PIN RPI_GPIO_27
#endif
#endif

#ifndef LSM303D_DRDY_M_PIN
#define LSM303D_DRDY_M_PIN -1
#endif

/* SPI protocol address bits */
#define DIR_READ                (1<<7)
#define DIR_WRITE               (0<<7)
#define ADDR_INCREMENT          (1<<6)

/* register addresses: A: accel, M: mag, T: temp */
#define ADDR_WHO_AM_I           0x0F
#define WHO_I_AM                     0x49

#define ADDR_OUT_TEMP_L         0x05
#define ADDR_OUT_TEMP_H         0x06
#define ADDR_STATUS_M           0x07
#define ADDR_OUT_X_L_M              0x08
#define ADDR_OUT_X_H_M              0x09
#define ADDR_OUT_Y_L_M              0x0A
#define ADDR_OUT_Y_H_M          0x0B
#define ADDR_OUT_Z_L_M          0x0C
#define ADDR_OUT_Z_H_M          0x0D

#define ADDR_INT_CTRL_M         0x12
#define ADDR_INT_SRC_M          0x13
#define ADDR_REFERENCE_X        0x1c
#define ADDR_REFERENCE_Y        0x1d
#define ADDR_REFERENCE_Z        0x1e

#define ADDR_STATUS_A           0x27
#define ADDR_OUT_X_L_A          0x28
#define ADDR_OUT_X_H_A          0x29
#define ADDR_OUT_Y_L_A          0x2A
#define ADDR_OUT_Y_H_A          0x2B
#define ADDR_OUT_Z_L_A          0x2C
#define ADDR_OUT_Z_H_A          0x2D

#define ADDR_CTRL_REG0          0x1F
#define ADDR_CTRL_REG1          0x20
#define ADDR_CTRL_REG2          0x21
#define ADDR_CTRL_REG3          0x22
#define ADDR_CTRL_REG4          0x23
#define ADDR_CTRL_REG5          0x24
#define ADDR_CTRL_REG6          0x25
#define ADDR_CTRL_REG7          0x26

#define ADDR_FIFO_CTRL          0x2e
#define ADDR_FIFO_SRC           0x2f

#define ADDR_IG_CFG1            0x30
#define ADDR_IG_SRC1            0x31
#define ADDR_IG_THS1            0x32
#define ADDR_IG_DUR1            0x33
#define ADDR_IG_CFG2            0x34
#define ADDR_IG_SRC2            0x35
#define ADDR_IG_THS2            0x36
#define ADDR_IG_DUR2            0x37
#define ADDR_CLICK_CFG          0x38
#define ADDR_CLICK_SRC          0x39
#define ADDR_CLICK_THS          0x3a
#define ADDR_TIME_LIMIT         0x3b
#define ADDR_TIME_LATENCY       0x3c
#define ADDR_TIME_WINDOW        0x3d
#define ADDR_ACT_THS            0x3e
#define ADDR_ACT_DUR            0x3f

#define REG1_RATE_BITS_A        ((1<<7) | (1<<6) | (1<<5) | (1<<4))
#define REG1_POWERDOWN_A        ((0<<7) | (0<<6) | (0<<5) | (0<<4))
#define REG1_RATE_3_125HZ_A     ((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define REG1_RATE_6_25HZ_A      ((0<<7) | (0<<6) | (1<<5) | (0<<4))
#define REG1_RATE_12_5HZ_A      ((0<<7) | (0<<6) | (1<<5) | (1<<4))
#define REG1_RATE_25HZ_A        ((0<<7) | (1<<6) | (0<<5) | (0<<4))
#define REG1_RATE_50HZ_A        ((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define REG1_RATE_100HZ_A       ((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define REG1_RATE_200HZ_A       ((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define REG1_RATE_400HZ_A       ((1<<7) | (0<<6) | (0<<5) | (0<<4))
#define REG1_RATE_800HZ_A       ((1<<7) | (0<<6) | (0<<5) | (1<<4))
#define REG1_RATE_1600HZ_A      ((1<<7) | (0<<6) | (1<<5) | (0<<4))

#define REG1_BDU_UPDATE         (1<<3)
#define REG1_Z_ENABLE_A         (1<<2)
#define REG1_Y_ENABLE_A         (1<<1)
#define REG1_X_ENABLE_A         (1<<0)

#define REG2_ANTIALIAS_FILTER_BW_BITS_A ((1<<7) | (1<<6))
#define REG2_AA_FILTER_BW_773HZ_A       ((0<<7) | (0<<6))
#define REG2_AA_FILTER_BW_194HZ_A       ((0<<7) | (1<<6))
#define REG2_AA_FILTER_BW_362HZ_A       ((1<<7) | (0<<6))
#define REG2_AA_FILTER_BW_50HZ_A        ((1<<7) | (1<<6))

#define REG2_FULL_SCALE_BITS_A  ((1<<5) | (1<<4) | (1<<3))
#define REG2_FULL_SCALE_2G_A    ((0<<5) | (0<<4) | (0<<3))
#define REG2_FULL_SCALE_4G_A    ((0<<5) | (0<<4) | (1<<3))
#define REG2_FULL_SCALE_6G_A    ((0<<5) | (1<<4) | (0<<3))
#define REG2_FULL_SCALE_8G_A    ((0<<5) | (1<<4) | (1<<3))
#define REG2_FULL_SCALE_16G_A   ((1<<5) | (0<<4) | (0<<3))

#define REG5_ENABLE_T           (1<<7)

#define REG5_RES_HIGH_M         ((1<<6) | (1<<5))
#define REG5_RES_LOW_M          ((0<<6) | (0<<5))

#define REG5_RATE_BITS_M        ((1<<4) | (1<<3) | (1<<2))
#define REG5_RATE_3_125HZ_M     ((0<<4) | (0<<3) | (0<<2))
#define REG5_RATE_6_25HZ_M      ((0<<4) | (0<<3) | (1<<2))
#define REG5_RATE_12_5HZ_M      ((0<<4) | (1<<3) | (0<<2))
#define REG5_RATE_25HZ_M        ((0<<4) | (1<<3) | (1<<2))
#define REG5_RATE_50HZ_M        ((1<<4) | (0<<3) | (0<<2))
#define REG5_RATE_100HZ_M       ((1<<4) | (0<<3) | (1<<2))
#define REG5_RATE_DO_NOT_USE_M  ((1<<4) | (1<<3) | (0<<2))

#define REG6_FULL_SCALE_BITS_M  ((1<<6) | (1<<5))
#define REG6_FULL_SCALE_2GA_M   ((0<<6) | (0<<5))
#define REG6_FULL_SCALE_4GA_M   ((0<<6) | (1<<5))
#define REG6_FULL_SCALE_8GA_M   ((1<<6) | (0<<5))
#define REG6_FULL_SCALE_12GA_M  ((1<<6) | (1<<5))

#define REG7_CONT_MODE_M        ((0<<1) | (0<<0))


#define INT_CTRL_M              0x12
#define INT_SRC_M               0x13

/* default values for this device */
#define LSM303D_ACCEL_DEFAULT_RANGE_G           8
#define LSM303D_ACCEL_DEFAULT_RATE          800
#define LSM303D_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ    50
#define LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ    30

#define LSM303D_MAG_DEFAULT_RANGE_GA            2
#define LSM303D_MAG_DEFAULT_RATE            100

#define LSM303D_DEBUG 0
#if LSM303D_DEBUG
#include <stdio.h>
#define error(...) fprintf(stderr, __VA_ARGS__)
#define debug(...) hal.console->printf(__VA_ARGS__)
#define ASSERT(x) assert(x)
#else
#define error(...)
#define debug(...)
#define ASSERT(x)
#endif

// constructor
AP_Compass_LSM303D::AP_Compass_LSM303D(Compass &compass):
    AP_Compass_Backend(compass)
{}

// detect the sensor
AP_Compass_Backend *AP_Compass_LSM303D::detect_spi(Compass &compass)
{
    AP_Compass_LSM303D *sensor = new AP_Compass_LSM303D(compass);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->init()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

uint8_t AP_Compass_LSM303D::_register_read(uint8_t reg)
{
    uint8_t addr = reg | 0x80; // Set most significant bit

    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = addr;
    tx[1] = 0;
    _spi->transaction(tx, rx, 2);

    return rx[1];
}

void AP_Compass_LSM303D::_register_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg;
    tx[1] = val;
    _spi->transaction(tx, rx, 2);
}

void AP_Compass_LSM303D::_register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits)
{
    uint8_t val;

    val = _register_read(reg);
    val &= ~clearbits;
    val |= setbits;
    _register_write(reg, val);
}

/**
 * Return true if the LSM303D has new data available for both the mag and
 * the accels.
 */
bool AP_Compass_LSM303D::_data_ready()
{
    return (_drdy_pin_m->read()) != 0;
}


// Read Sensor data
bool AP_Compass_LSM303D::_read_raw()
{
    if (_register_read(ADDR_CTRL_REG7) != _reg7_expected) {
        hal.console->println(
                               "LSM303D _read_data_transaction_accel: _reg7_expected unexpected");
        // reset();
        return false;
    }

    if (!_data_ready()) {
        return false;
    }

    struct PACKED {
        uint8_t     cmd;
        uint8_t     status;
        int16_t     x;
        int16_t     y;
        int16_t     z;
    } raw_mag_report_tx;

    struct PACKED {
        uint8_t     cmd;
        uint8_t     status;
        int16_t     x;
        int16_t     y;
        int16_t     z;
    } raw_mag_report_rx;

    /* fetch data from the sensor */
    memset(&raw_mag_report_tx, 0, sizeof(raw_mag_report_tx));
    memset(&raw_mag_report_rx, 0, sizeof(raw_mag_report_rx));
    raw_mag_report_tx.cmd = ADDR_STATUS_M | DIR_READ | ADDR_INCREMENT;
    _spi->transaction((uint8_t *)&raw_mag_report_tx, (uint8_t *)&raw_mag_report_rx, sizeof(raw_mag_report_tx));

    _mag_x = raw_mag_report_rx.x;
    _mag_y = raw_mag_report_rx.y;
    _mag_z = raw_mag_report_rx.z;

    if (is_zero(_mag_x) && is_zero(_mag_y) && is_zero(_mag_z)) {
        return false;
    }

    return true;
}

// Public Methods //////////////////////////////////////////////////////////////
bool
AP_Compass_LSM303D::init()
{
    // TODO: support users without data ready pin
    if (LSM303D_DRDY_M_PIN < 0)
        return false;

    hal.scheduler->suspend_timer_procs();

    _spi = hal.spi->device(AP_HAL::SPIDevice_LSM303D);
    _spi_sem = _spi->get_semaphore();

    _drdy_pin_m = hal.gpio->channel(LSM303D_DRDY_M_PIN);
    _drdy_pin_m->mode(HAL_GPIO_INPUT);

    // Test WHOAMI
    uint8_t whoami = _register_read(ADDR_WHO_AM_I);
    if (whoami != WHO_I_AM) {
        hal.console->printf("LSM303D: unexpected WHOAMI 0x%x\n", (unsigned)whoami);
        AP_HAL::panic("LSM303D: bad WHOAMI");
    }

    uint8_t tries = 0;
    do {
        // TODO: don't try to init 25 times
        bool success = _hardware_init();
        if (success) {
            hal.scheduler->delay(5+2);
            if (!_spi_sem->take(100)) {
                AP_HAL::panic("LSM303D: Unable to get semaphore");
            }
            if (_data_ready()) {
                _spi_sem->give();
                break;
            } else {
                hal.console->println(
                                       "LSM303D startup failed: no data ready");
            }
            _spi_sem->give();
        }
        if (tries++ > 5) {
            AP_HAL::panic("PANIC: failed to boot LSM303D 5 times");
        }
    } while (1);

    _scaling[0] = 1.0;
    _scaling[1] = 1.0;
    _scaling[2] = 1.0;

    /* register the compass instance in the frontend */
    _compass_instance = register_compass();
    set_dev_id(_compass_instance, get_dev_id());
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
    set_external(_compass_instance, false);
#endif

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Compass_LSM303D::_update, void));

    _spi_sem->give();
    hal.scheduler->resume_timer_procs();
    _initialised = true;

    return _initialised;
}

uint32_t AP_Compass_LSM303D::get_dev_id()
{
    return AP_COMPASS_TYPE_LSM303D;
}

bool AP_Compass_LSM303D::_hardware_init(void)
{
    if (!_spi_sem->take(100)) {
        AP_HAL::panic("LSM303D: Unable to get semaphore");
    }

    // initially run the bus at low speed
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);

    // ensure the chip doesn't interpret any other bus traffic as I2C
    _disable_i2c();

    /* enable mag */
    _reg7_expected = REG7_CONT_MODE_M;
    _register_write(ADDR_CTRL_REG7, _reg7_expected);
    _register_write(ADDR_CTRL_REG5, REG5_RES_HIGH_M);
    _register_write(ADDR_CTRL_REG4, 0x04); // DRDY on MAG on INT2

    _mag_set_range(LSM303D_MAG_DEFAULT_RANGE_GA);
    _mag_set_samplerate(LSM303D_MAG_DEFAULT_RATE);

    // TODO: Software filtering

    // now that we have initialised, we set the SPI bus speed to high
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);
    _spi_sem->give();

    return true;
}

void AP_Compass_LSM303D::_update()
{
    if (AP_HAL::micros() - _last_update_timestamp < 10000) {
        return;
    }

    if (!_spi_sem->take_nonblocking()) {
        return;
    }

    _collect_samples();

    _last_update_timestamp = AP_HAL::micros();
    _spi_sem->give();
}

void AP_Compass_LSM303D::_collect_samples()
{
    if (!_initialised) {
        return;
    }

    if (!_read_raw()) {
        error("_read_raw() failed\n");
    } else {
        Vector3f raw_field = Vector3f(_mag_x, _mag_y, _mag_z) * _mag_range_scale;
        uint32_t time_us = AP_HAL::micros();

        // rotate raw_field from sensor frame to body frame
        rotate_field(raw_field, _compass_instance);

        // publish raw_field (uncorrected point sample) for _scaling use
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
    }
}

// Read Sensor data
void AP_Compass_LSM303D::read()
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }

    if (_accum_count == 0) {
        /* We're not ready to publish*/
        return;
    }

    hal.scheduler->suspend_timer_procs();
    Vector3f field(_mag_x_accum * _scaling[0],
                   _mag_y_accum * _scaling[1],
                   _mag_z_accum * _scaling[2]);
    field /= _accum_count;

    _accum_count = 0;
    _mag_x_accum = _mag_y_accum = _mag_z_accum = 0;
    hal.scheduler->resume_timer_procs();

    publish_filtered_field(field, _compass_instance);
}

void AP_Compass_LSM303D::_disable_i2c(void)
{
    // TODO: use the register names
    uint8_t a = _register_read(0x02);
    _register_write(0x02, (0x10 | a));
    a = _register_read(0x02);
    _register_write(0x02, (0xF7 & a));
    a = _register_read(0x15);
    _register_write(0x15, (0x80 | a));
    a = _register_read(0x02);
    _register_write(0x02, (0xE7 & a));
}

uint8_t AP_Compass_LSM303D::_mag_set_range(uint8_t max_ga)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG6_FULL_SCALE_BITS_M;
    float new_scale_ga_digit = 0.0f;

    if (max_ga == 0)
        max_ga = 12;

    if (max_ga <= 2) {
        _mag_range_ga = 2;
        setbits |= REG6_FULL_SCALE_2GA_M;
        new_scale_ga_digit = 0.080f;

    } else if (max_ga <= 4) {
        _mag_range_ga = 4;
        setbits |= REG6_FULL_SCALE_4GA_M;
        new_scale_ga_digit = 0.160f;

    } else if (max_ga <= 8) {
        _mag_range_ga = 8;
        setbits |= REG6_FULL_SCALE_8GA_M;
        new_scale_ga_digit = 0.320f;

    } else if (max_ga <= 12) {
        _mag_range_ga = 12;
        setbits |= REG6_FULL_SCALE_12GA_M;
        new_scale_ga_digit = 0.479f;

    } else {
        return -1;
    }

    _mag_range_scale = new_scale_ga_digit;
    _register_modify(ADDR_CTRL_REG6, clearbits, setbits);
    return 0;
}

uint8_t AP_Compass_LSM303D::_mag_set_samplerate(uint16_t frequency)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG5_RATE_BITS_M;

    if (frequency == 0)
        frequency = 100;

    if (frequency <= 25) {
        setbits |= REG5_RATE_25HZ_M;
        _mag_samplerate = 25;

    } else if (frequency <= 50) {
        setbits |= REG5_RATE_50HZ_M;
        _mag_samplerate = 50;

    } else if (frequency <= 100) {
        setbits |= REG5_RATE_100HZ_M;
        _mag_samplerate = 100;

    } else {
        return -1;
    }

    _register_modify(ADDR_CTRL_REG5, clearbits, setbits);
    return 0;
}
