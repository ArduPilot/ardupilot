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

#include "AP_Compass_LSM303D.h"

#if AP_COMPASS_LSM303D_ENABLED

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/GPIO.h>
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

#define REG5_RES_HIGH_M         ((1<<6) | (1<<5) | (1<<7))
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

#define LSM303D_MAG_DEFAULT_RANGE_GA          2
#define LSM303D_MAG_DEFAULT_RATE            100

AP_Compass_LSM303D::AP_Compass_LSM303D(AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : _dev(std::move(dev))
{
}

AP_Compass_Backend *AP_Compass_LSM303D::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_LSM303D *sensor = new AP_Compass_LSM303D(std::move(dev));
    if (!sensor || !sensor->init(rotation)) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

uint8_t AP_Compass_LSM303D::_register_read(uint8_t reg)
{
    uint8_t val = 0;

    reg |= DIR_READ;
    _dev->read_registers(reg, &val, 1);

    return val;
}

bool AP_Compass_LSM303D::_block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    reg |= DIR_READ | ADDR_INCREMENT;
    return _dev->read_registers(reg, buf, size);
}

void AP_Compass_LSM303D::_register_write(uint8_t reg, uint8_t val)
{
    _dev->write_register(reg, val);
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
    return _drdy_pin_m == nullptr || (_drdy_pin_m->read() != 0);
}


// Read Sensor data
bool AP_Compass_LSM303D::_read_sample()
{
    struct PACKED {
        uint8_t status;
        int16_t x;
        int16_t y;
        int16_t z;
    } rx;

    if (_register_read(ADDR_CTRL_REG7) != _reg7_expected) {
        DEV_PRINTF("LSM303D _read_data_transaction_accel: _reg7_expected unexpected\n");
        return false;
    }

    if (!_data_ready()) {
        return false;
    }

    if (!_block_read(ADDR_STATUS_M, (uint8_t *) &rx, sizeof(rx))) {
        return false;
    }

    /* check for overrun */
    if ((rx.status & 0x70) != 0) {
        return false;
    }

    if (rx.x == 0 && rx.y == 0 && rx.z == 0) {
        return false;
    }

    _mag_x = rx.x;
    _mag_y = rx.y;
    _mag_z = rx.z;

    return true;
}

bool AP_Compass_LSM303D::init(enum Rotation rotation)
{
    if (LSM303D_DRDY_M_PIN >= 0) {
        _drdy_pin_m = hal.gpio->channel(LSM303D_DRDY_M_PIN);
        _drdy_pin_m->mode(HAL_GPIO_INPUT);
    }

    bool success = _hardware_init();

    if (!success) {
        return false;
    }

    _initialised = true;

    /* register the compass instance in the frontend */
    _dev->set_device_type(DEVTYPE_LSM303D);
    if (!register_compass(_dev->get_bus_id(), _compass_instance)) {
        return false;
    }
    set_dev_id(_compass_instance, _dev->get_bus_id());

    set_rotation(_compass_instance, rotation);

    // read at 91Hz. We don't run at 100Hz as fetching data too fast can cause some very
    // odd periodic changes in the output data
    _dev->register_periodic_callback(11000, FUNCTOR_BIND_MEMBER(&AP_Compass_LSM303D::_update, void));

    return true;
}

bool AP_Compass_LSM303D::_hardware_init()
{
    _dev->get_semaphore()->take_blocking();

    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // Test WHOAMI
    uint8_t whoami = _register_read(ADDR_WHO_AM_I);
    if (whoami != WHO_I_AM) {
        goto fail_whoami;
    }

    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {
        // ensure the chip doesn't interpret any other bus traffic as I2C
        _disable_i2c();

        /* enable mag */
        _reg7_expected = REG7_CONT_MODE_M;
        _register_write(ADDR_CTRL_REG7, _reg7_expected);
        _register_write(ADDR_CTRL_REG5, REG5_RES_HIGH_M);

        // DRDY on MAG on INT2
        _register_write(ADDR_CTRL_REG4, 0x04);

        _mag_set_range(LSM303D_MAG_DEFAULT_RANGE_GA);
        _mag_set_samplerate(LSM303D_MAG_DEFAULT_RATE);

        hal.scheduler->delay(10);
        if (_data_ready()) {
            break;
        }
    }
    if (tries == 5) {
        DEV_PRINTF("Failed to boot LSM303D 5 times\n");
        goto fail_tries;
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    _dev->get_semaphore()->give();

    return true;

fail_tries:
fail_whoami:
    _dev->get_semaphore()->give();
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    return false;
}

void AP_Compass_LSM303D::_update()
{
    if (!_read_sample()) {
        return;
    }

    Vector3f raw_field = Vector3f(_mag_x, _mag_y, _mag_z) * _mag_range_scale;

    accumulate_sample(raw_field, _compass_instance, 10);
}

// Read Sensor data
void AP_Compass_LSM303D::read()
{
    if (!_initialised) {
        return;
    }

    drain_accumulated_samples(_compass_instance);
}

void AP_Compass_LSM303D::_disable_i2c()
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

bool AP_Compass_LSM303D::_mag_set_range(uint8_t max_ga)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG6_FULL_SCALE_BITS_M;
    float new_scale_ga_digit = 0.0f;

    if (max_ga == 0) {
        max_ga = 12;
    }

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
        return false;
    }

    _mag_range_scale = new_scale_ga_digit;
    _register_modify(ADDR_CTRL_REG6, clearbits, setbits);

    return true;
}

bool AP_Compass_LSM303D::_mag_set_samplerate(uint16_t frequency)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG5_RATE_BITS_M;

    if (frequency == 0) {
        frequency = 100;
    }

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
        return false;
    }

    _register_modify(ADDR_CTRL_REG5, clearbits, setbits);

    return true;
}

#endif  // AP_COMPASS_LSM303D_ENABLED
