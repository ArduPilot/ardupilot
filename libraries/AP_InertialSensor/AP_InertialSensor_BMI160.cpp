/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <utility>

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL_Linux/GPIO.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_BMI160.h"

/* Registers and bits definitions. The indented ones are the bits for the upper
 * register. */
#define BMI160_REG_CHIPID 0x00
#define     BMI160_CHIPID 0xD1
#define BMI160_REG_ERR_REG 0x02
#define BMI160_REG_FIFO_LENGTH 0x22
#define BMI160_REG_FIFO_DATA 0x24
#define BMI160_REG_ACC_CONF 0x40
#define BMI160_REG_ACC_RANGE 0x41
            /* For convenience, use log2(range) - 1 instead of bits defined in
             * the datasheet. See _configure_accel(). */
#define     BMI160_ACC_RANGE_16G 3
#define BMI160_REG_GYR_CONF 0x42
#define BMI160_REG_GYR_RANGE 0x43
#define     BMI160_GYR_RANGE_2000DPS 0x00
#define BMI160_REG_FIFO_CONFIG_0 0x46
#define BMI160_REG_FIFO_CONFIG_1 0x47
#define     BMI160_FIFO_ACC_EN 0x40
#define     BMI160_FIFO_GYR_EN 0x80
#define BMI160_REG_INT_EN_1 0x51
#define     BMI160_INT_FWM_EN 0x40
#define BMI160_REG_INT_OUT_CTRL 0x53
#define     BMI160_INT1_LVL 0x02
#define     BMI160_INT1_OUTPUT_EN 0x08
#define BMI160_REG_INT_MAP_1 0x56
#define     BMI160_INT_MAP_INT1_FWM 0x40
#define BMI160_REG_CMD 0x7E
#define     BMI160_CMD_ACCEL_NORMAL_POWER_MODE 0x11
#define     BMI160_CMD_GYRO_NORMAL_POWER_MODE 0x15
#define     BMI160_CMD_FIFO_FLUSH 0xB0
#define     BMI160_CMD_SOFTRESET 0xB6

#define     BMI160_OSR_NORMAL 0x20
#define     BMI160_ODR_1600HZ 0x0C

/* Datasheet says that the device powers up in less than 10ms, so waiting for
 * 10 ms before initialization is enough. */
#define BMI160_POWERUP_DELAY_MSEC 10
/* TODO: Investigate this. The delay below is way too high and with that
 * there's still at least 1% of failures on initialization. Lower values
 * increase that percentage. */
#define BMI160_SOFTRESET_DELAY_MSEC 100
/* Define a little bit more than the maximum value in the datasheet's timing
 * table. The datasheet recommends adding 300 us to the time for startup
 * occasions. */
#define BMI160_ACCEL_NORMAL_POWER_MODE_DELAY_MSEC 4
#define BMI160_GYRO_NORMAL_POWER_MODE_DELAY_MSEC 81

#define BMI160_OSR BMI160_OSR_NORMAL
#define BMI160_ODR BMI160_ODR_1600HZ
#define BMI160_ACC_RANGE BMI160_ACC_RANGE_16G
#define BMI160_GYR_RANGE BMI160_GYR_RANGE_2000DPS

/* By looking at the datasheet, the accel range i (as defined by the macros
 * BMI160_ACC_RANGE_*G) maps to the range bits by the function f defined:
 *     f(0) = 3; f(i) = f(i - 1) + i + 1
 * Which can be written as the closed formula:
 *     f(i) = (i * (i + 3)) / 2 + 3 */
#define BMI160_ACC_RANGE_BITS \
    (BMI160_ACC_RANGE * (BMI160_ACC_RANGE + 3) / 2 + 3)

/* The rate in Hz based on the ODR bits can be calculated with
 * 100 / (2 ^ (8 - odr) */
#define BMI160_ODR_TO_HZ(odr_) \
    (uint16_t)(odr_ > 8 ? 100 << (odr_ - 8) : 100 >> (8 - odr_))

/* This number of samples should provide only one read burst operation on the
 * FIFO most of the time (99.99%). */
#define BMI160_MAX_FIFO_SAMPLES 8

#define BMI160_READ_FLAG 0x80
#define BMI160_HARDWARE_INIT_MAX_TRIES 5

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_AERO
#    define BMI160_INT1_GPIO AERO_GPIO_BMI160_INT1
#else
#    define BMI160_INT1_GPIO -1
#endif

extern const AP_HAL::HAL& hal;

struct PACKED RawData {
    struct {
        le16_t x;
        le16_t y;
        le16_t z;
    } gyro;
    struct {
        le16_t x;
        le16_t y;
        le16_t z;
    } accel;
};

AP_InertialSensor_BMI160::AP_InertialSensor_BMI160(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_BMI160::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_BMI160(imu, std::move(dev));

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_BMI160::start()
{
    bool r;

    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return;
    }

    r = _configure_accel();
    if (!r) {
        AP_HAL::panic("BMI160: Unable to configure accelerometer");
    }

    r = _configure_gyro();
    if (!r) {
        AP_HAL::panic("BMI160: Unable to configure gyroscope");
    }

    r = _configure_fifo();
    if (!r) {
        AP_HAL::panic("BMI160: Unable to configure FIFO");
    }

    if (BMI160_INT1_GPIO >= 0) {
        r = _configure_int1_pin();
        if (!r) {
            AP_HAL::panic("BMI160: unable to configure INT1 pin");
        }
    }

    _dev->get_semaphore()->give();

    _accel_instance = _imu.register_accel(BMI160_ODR_TO_HZ(BMI160_ODR), _dev->get_bus_id_devtype(DEVTYPE_BMI160));
    _gyro_instance = _imu.register_gyro(BMI160_ODR_TO_HZ(BMI160_ODR),   _dev->get_bus_id_devtype(DEVTYPE_BMI160));

    /* Call _poll_data() at 1kHz */
    _dev->register_periodic_callback(1000,
        FUNCTOR_BIND_MEMBER(&AP_InertialSensor_BMI160::_poll_data, void));
}

bool AP_InertialSensor_BMI160::update()
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);
    return true;
}

void AP_InertialSensor_BMI160::_check_err_reg()
{
#ifdef BMI160_DEBUG
    uint8_t v;
    bool r;

    r = _dev->read_registers(BMI160_REG_ERR_REG, &v, 1);
    if (!r) {
        AP_HAL::panic("BMI160: couldn't read ERR_REG\n");
    }
    if (v) {
        AP_HAL::panic("BMI160: error detected on ERR_REG\n");
    }
#endif
}

bool AP_InertialSensor_BMI160::_configure_accel()
{
    bool r;

    r = _dev->write_register(BMI160_REG_ACC_CONF, BMI160_OSR | BMI160_ODR);
    if (!r) {
        return false;
    }
    hal.scheduler->delay(1);

    _check_err_reg();

    r = _dev->write_register(BMI160_REG_ACC_RANGE, BMI160_ACC_RANGE_BITS);
    if (!r) {
        return false;
    }
    hal.scheduler->delay(1);

    /* The sensitivity in LSb/g for an accel range i (as defined by the macros
     * BMI160_ACC_RANGE_*G) can be calculated with:
     *     2 ^ 16 / (2 * 2 ^ (i + 1)) = 2 ^(14 - i)
     * That matches the typical values in the datasheet. */
    _accel_scale = GRAVITY_MSS / (1 << (14 - BMI160_ACC_RANGE));

    return true;
}

bool AP_InertialSensor_BMI160::_configure_gyro()
{
    bool r;

    r = _dev->write_register(BMI160_REG_GYR_CONF, BMI160_OSR | BMI160_ODR);
    if (!r) {
        return false;
    }
    hal.scheduler->delay(1);

    _check_err_reg();

    r = _dev->write_register(BMI160_REG_GYR_RANGE, BMI160_GYR_RANGE);
    if (!r) {
        return false;
    }
    hal.scheduler->delay(1);

    /* The sensitivity in LSb/degrees/s a gyro range i can be calculated with:
     *     2 ^ 16 / (2 * 2000 / 2 ^ i) = 2 ^ (14 + i) / 1000
     * The scale is the inverse of that. */
    _gyro_scale = radians(1000.f / (1 << (14 + BMI160_GYR_RANGE)));

    return true;
}

bool AP_InertialSensor_BMI160::_configure_int1_pin()
{
    bool r;

    r = _dev->write_register(BMI160_REG_INT_EN_1, BMI160_INT_FWM_EN);
    if (!r) {
        hal.console->printf("BMI160: Unable to enable FIFO watermark interrupt engine\n");
        return false;
    }
    hal.scheduler->delay(1);

    r = _dev->write_register(BMI160_REG_INT_MAP_1, BMI160_INT_MAP_INT1_FWM);
    if (!r) {
        hal.console->printf("BMI160: Unable to configure interrupt mapping\n");
        return false;
    }
    hal.scheduler->delay(1);

    r = _dev->write_register(BMI160_REG_INT_OUT_CTRL,
                             BMI160_INT1_OUTPUT_EN | BMI160_INT1_LVL);
    if (!r) {
        hal.console->printf("BMI160: Unable to configure interrupt output\n");
        return false;
    }
    hal.scheduler->delay(1);

    _int1_pin = hal.gpio->channel(BMI160_INT1_GPIO);
    if (_int1_pin == nullptr) {
        hal.console->printf("BMI160: Couldn't request data ready GPIO channel\n");
        return false;
    }
    _int1_pin->mode(HAL_GPIO_INPUT);

    return true;
}

bool AP_InertialSensor_BMI160::_configure_fifo()
{
    bool r;

    /* The unit for the FIFO watermark is 4 bytes. */
    r = _dev->write_register(BMI160_REG_FIFO_CONFIG_0,
                             sizeof(struct RawData) / 4);
    if (!r) {
        hal.console->printf("BMI160: Unable to configure FIFO watermark level\n");
        return false;
    }
    hal.scheduler->delay(1);

    r = _dev->write_register(BMI160_REG_FIFO_CONFIG_1,
                             BMI160_FIFO_ACC_EN | BMI160_FIFO_GYR_EN);
    if (!r) {
        hal.console->printf("BMI160: Unable to enable FIFO\n");
        return false;
    }
    hal.scheduler->delay(1);

    _check_err_reg();

    r = _dev->write_register(BMI160_REG_CMD, BMI160_CMD_FIFO_FLUSH);
    if (!r) {
        hal.console->printf("BMI160: Unable to flush FIFO\n");
        return false;
    }

    return true;
}

void AP_InertialSensor_BMI160::_read_fifo()
{
    struct RawData raw_data[BMI160_MAX_FIFO_SAMPLES];
    uint16_t num_bytes;
    uint16_t excess;
    uint8_t num_samples = 0;
    bool r = true;

    static_assert(sizeof(raw_data) <= 100, "Too big to keep on stack");

    /* If FIFO watermark not surpassed. */
    if (_int1_pin && _int1_pin->read() == 0) {
        goto read_fifo_end;
    }

    r = _dev->read_registers(BMI160_REG_FIFO_LENGTH,
                             (uint8_t *)&num_bytes,
                             sizeof(num_bytes));
    if (!r) {
        goto read_fifo_end;
    }

    num_bytes = le16toh(num_bytes);
    if (!num_bytes) {
        goto read_fifo_end;
    }

read_fifo_read_data:
    if (num_bytes > sizeof(raw_data)) {
        excess = num_bytes - sizeof(raw_data);
        num_bytes = sizeof(raw_data);
    } else {
        excess = 0;
    }

    r = _dev->read_registers(BMI160_REG_FIFO_DATA,
                             (uint8_t *)raw_data,
                             num_bytes);
    if (!r) {
        goto read_fifo_end;
    }

    /* Read again just once */
    if (excess && num_samples) {
        hal.console->printf("BMI160: dropping %u samples from fifo\n",
                            (uint8_t)(excess / sizeof(struct RawData)));
        _dev->write_register(BMI160_REG_CMD, BMI160_CMD_FIFO_FLUSH);
        excess = 0;
    }

    num_samples = num_bytes / sizeof(struct RawData);
    for (uint8_t i = 0; i < num_samples; i++) {
        Vector3f accel{(float)(int16_t)le16toh(raw_data[i].accel.x),
                       (float)(int16_t)le16toh(raw_data[i].accel.y),
                       (float)(int16_t)le16toh(raw_data[i].accel.z)};
        Vector3f gyro{(float)(int16_t)le16toh(raw_data[i].gyro.x),
                      (float)(int16_t)le16toh(raw_data[i].gyro.y),
                      (float)(int16_t)le16toh(raw_data[i].gyro.z)};

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_AERO
        accel.rotate(ROTATION_ROLL_180);
        gyro.rotate(ROTATION_ROLL_180);
#endif

        accel *= _accel_scale;
        gyro *= _gyro_scale;

        _rotate_and_correct_accel(_accel_instance, accel);
        _rotate_and_correct_gyro(_gyro_instance, gyro);

        _notify_new_accel_raw_sample(_accel_instance, accel);
        _notify_new_gyro_raw_sample(_gyro_instance, gyro);
    }

    if (excess) {
        num_bytes = excess;
        goto read_fifo_read_data;
    }

read_fifo_end:
    if (!r) {
        hal.console->printf("BMI160: error on reading FIFO\n");
    }
}

void AP_InertialSensor_BMI160::_poll_data()
{
    _read_fifo();
}

bool AP_InertialSensor_BMI160::_hardware_init()
{
    bool ret = false;

    hal.scheduler->delay(BMI160_POWERUP_DELAY_MSEC);

    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    for (unsigned i = 0; i < BMI160_HARDWARE_INIT_MAX_TRIES; i++) {
        uint8_t v;
        ret = _dev->write_register(BMI160_REG_CMD,
                                   BMI160_CMD_SOFTRESET);

        if (!ret) {
            continue;
        }
        hal.scheduler->delay(BMI160_SOFTRESET_DELAY_MSEC);

        /* The datasheet recommends doing a read operation on the register 0x7F
         * in order to guarantee the sensor works using the SPI protocol. This
         * shouldn't have side effects for I2C. */
        ret = _dev->read_registers(0x7F, &v, 1);
        if (!ret) {
            continue;
        }

        ret = _dev->read_registers(BMI160_REG_CHIPID, &v, 1);
        if (!ret) {
            continue;
        }
        if (v != BMI160_CHIPID) {
            ret = false;
            continue;
        }

        ret = _dev->write_register(BMI160_REG_CMD,
                                   BMI160_CMD_ACCEL_NORMAL_POWER_MODE);
        if (!ret) {
            continue;
        }
        hal.scheduler->delay(BMI160_ACCEL_NORMAL_POWER_MODE_DELAY_MSEC);

        ret = _dev->write_register(BMI160_REG_CMD,
                                   BMI160_CMD_GYRO_NORMAL_POWER_MODE);
        if (!ret) {
            continue;
        }
        hal.scheduler->delay(BMI160_GYRO_NORMAL_POWER_MODE_DELAY_MSEC);

        break;
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    _dev->get_semaphore()->give();
    return ret;
}

bool AP_InertialSensor_BMI160::_init()
{
    bool ret = false;
    _dev->set_read_flag(BMI160_READ_FLAG);

    ret = _hardware_init();
    if (!ret) {
        hal.console->printf("BMI160: failed to init\n");
    }

    return ret;
}

#endif
