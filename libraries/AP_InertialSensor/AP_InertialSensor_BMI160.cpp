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
#include <AP_Math/AP_Math.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/GPIO.h>
#endif

#include "AP_InertialSensor_BMI160.h"

/* Registers and bits definitions. The indented ones are the bits for the upper
 * register. */
#define BMI160_REG_CHIPID 0x00
#define     BMI160_CHIPID 0xD1
#define BMI160_REG_ERR_REG 0x02
#define BMI160_REG_DATA_GYR_X_L 0x0C
#define BMI160_REG_FIFO_LENGTH 0x22
#define BMI160_REG_STATUS 0x1B
#define     BMI160_STATUS_DRDY_GYR 0x40
#define     BMI160_STATUS_DRDY_ACC 0x80
#define BMI160_REG_FIFO_DATA 0x24
#define BMI160_REG_ACC_CONF 0x40
#define BMI160_REG_ACC_RANGE 0x41
#define BMI160_REG_GYR_CONF 0x42
#define BMI160_REG_GYR_RANGE 0x43
#define BMI160_REG_FIFO_CONFIG_0 0x46
#define BMI160_REG_FIFO_CONFIG_1 0x47
#define     BMI160_FIFO_ACC_EN 0x40
#define     BMI160_FIFO_GYR_EN 0x80
#define BMI160_REG_INT_EN_1 0x51
#define     BMI160_INT_DRDY_EN 0x10
#define     BMI160_INT_FWM_EN 0x40
#define BMI160_REG_INT_OUT_CTRL 0x53
#define     BMI160_INT1_LVL 0x02
#define     BMI160_INT1_OUTPUT_EN 0x08
#define BMI160_REG_INT_MAP_1 0x56
#define     BMI160_INT_MAP_INT1_FWM 0x40
#define     BMI160_INT_MAP_INT1_DRDY 0x80
#define BMI160_REG_CMD 0x7E
#define     BMI160_CMD_ACCEL_NORMAL_POWER_MODE 0x11
#define     BMI160_CMD_GYRO_NORMAL_POWER_MODE 0x15
#define     BMI160_CMD_FIFO_FLUSH 0xB0
#define     BMI160_CMD_SOFTRESET 0xB6

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

/* The rate in Hz based on the ODR bits can be calculated with
 * 100 / (2 ^ (8 - odr) */
#define BMI160_ODR_TO_HZ(odr_) \
    (uint16_t)(odr_ > 8 ? 100 << (odr_ - 8) : 100 >> (8 - odr_))
#define BMI160_MAX_ODR ODR::ODR_1600Hz

/* Consider number maximum of samples as four times of the expected number of
 * samples in the FIFO for the maximum ODR considering the frequency the timer
 * thread routing is called (1kHz). Round it up.*/
#define BMI160_MAX_FIFO_SAMPLES \
    ((4 * BMI160_ODR_TO_HZ(BMI160_MAX_ODR) + 999) / 1000)

#define BMI160_READ_FLAG 0x80
#define BMI160_HARDWARE_INIT_MAX_TRIES 5

#ifdef BMI160_DEBUG
#    define BMI160_CHECK_ERR_REG() _check_err_reg()
#else
#    define BMI160_CHECK_ERR_REG()
#endif

#define BMI160_INT1_GPIO -1

extern const AP_HAL::HAL& hal;

AP_InertialSensor_BMI160::AP_InertialSensor_BMI160(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                   bool use_fifo)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _use_fifo(use_fifo)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_BMI160::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev)
{
    auto sensor = new AP_InertialSensor_BMI160(imu,
                                               std::move(dev),
                                               true);

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
    AccelConfig accel_cfg = {
        .osr = NORMAL,
        .odr = ODR_1600Hz,
        .range = RANGE_16G,
    };
    GyroConfig gyro_cfg = {
        .osr = NORMAL,
        .odr = ODR_1600Hz,
        .range = RANGE_2000DPS,
    };

    if (!_use_fifo) {
        accel_cfg.odr = ODR_800Hz;
        gyro_cfg.odr = ODR_800Hz;
    }

    hal.scheduler->suspend_timer_procs();
    if (!_dev->get_semaphore()->take(100)) {
        AP_HAL::panic("BMI160: Unable to get semaphore");
    }

    r = _configure_accel(accel_cfg);
    if (!r) {
        AP_HAL::panic("BMI160: Unable to configure accelerometer");
    }

    r = _configure_gyro(gyro_cfg);
    if (!r) {
        AP_HAL::panic("BMI160: Unable to configure gyroscope");
    }

    if (_use_fifo) {
        r = _configure_fifo();
        if (!r) {
            AP_HAL::panic("BMI160: Unable to configure FIFO");
        }
    }

    if (BMI160_INT1_GPIO >= 0) {
        r = _configure_int1_pin();
        if (!r) {
            AP_HAL::panic("BMI160: unable to configure INT1 pin");
        }
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    _dev->get_semaphore()->give();

    _accel_instance = _imu.register_accel(BMI160_ODR_TO_HZ(accel_cfg.odr));
    _gyro_instance = _imu.register_gyro(BMI160_ODR_TO_HZ(gyro_cfg.odr));

    hal.scheduler->resume_timer_procs();

    hal.scheduler->register_timer_process(
        FUNCTOR_BIND_MEMBER(&AP_InertialSensor_BMI160::_poll_data, void));
}

bool AP_InertialSensor_BMI160::update()
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);
    return true;
}

#ifdef BMI160_DEBUG
void AP_InertialSensor_BMI160::_check_err_reg()
{
    uint8_t v;
    bool r;

    r = _dev->read_registers(BMI160_REG_ERR_REG, &v, 1);
    if (!r) {
        AP_HAL::panic("BMI160: couldn't read ERR_REG\n");
    }
    if (v) {
        AP_HAL::panic("BMI160: error detected on ERR_REG\n");
    }
}
#endif


bool AP_InertialSensor_BMI160::_configure_accel(AccelConfig &cfg)
{
    bool r;
    uint8_t range_bits;

    r = _dev->write_register(BMI160_REG_ACC_CONF, cfg.osr << 4 | cfg.odr);
    if (!r) {
        return false;
    }
    hal.scheduler->delay(1);

    BMI160_CHECK_ERR_REG();

    /* By looking at the datasheet, an AccelRange i maps to the range bits by
     * the function f defined:
     *     f(0) = 3; f(i) = f(i - 1) + i + 1
     * Which can be written as the closed formula:
     *     f(i) = (i * (i + 3)) / 2 + 3 */
    range_bits = cfg.range * (cfg.range + 3) / 2 + 3;
    r = _dev->write_register(BMI160_REG_ACC_RANGE, range_bits);
    if (!r) {
        return false;
    }
    hal.scheduler->delay(1);

    /* The sensitivity in LSb/g for an AccelRange i can be calculated with:
     *     2 ^ 16 / (2 * 2 ^ (i + 1)) = 2 ^(14 - i)
     * That matches the typical values in the datasheet. */
    _accel_scale = GRAVITY_MSS / (1 << (14 - cfg.range));

    return true;
}

bool AP_InertialSensor_BMI160::_configure_gyro(GyroConfig &cfg)
{
    bool r;

    r = _dev->write_register(BMI160_REG_GYR_CONF, cfg.osr << 4 | cfg.odr);
    if (!r) {
        return false;
    }
    hal.scheduler->delay(1);

    BMI160_CHECK_ERR_REG();

    r = _dev->write_register(BMI160_REG_GYR_RANGE, cfg.range);
    if (!r) {
        return false;
    }
    hal.scheduler->delay(1);

    /* The sensitivity in LSb/degrees/s a GyroRange i can be calculated with:
     *     2 ^ 16 / (2 * 2000 / 2 ^ i) = 2 ^ (14 + i) / 1000
     * The scale is the inverse of that. */
    _gyro_scale = radians(1000.f / (1 << (14 + cfg.range)));

    return true;
}

bool AP_InertialSensor_BMI160::_configure_int1_pin()
{
    bool r;
    uint8_t mapping;

    if (_use_fifo) {
        r = _dev->write_register(BMI160_REG_INT_EN_1, BMI160_INT_FWM_EN);
        if (!r) {
            hal.console->printf("BMI160: Unable to enable FIFO watermark interrupt engine\n");
            return false;
        }
        hal.scheduler->delay(1);

        mapping = BMI160_INT_MAP_INT1_FWM;
    } else {
        r = _dev->write_register(BMI160_REG_INT_EN_1, BMI160_INT_DRDY_EN);
        if (!r) {
            hal.console->printf("BMI160: Unable to enable data ready interrupt engine\n");
            return false;
        }
        hal.scheduler->delay(1);

        mapping = BMI160_INT_MAP_INT1_DRDY;
    }

    r = _dev->write_register(BMI160_REG_INT_MAP_1, mapping);
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
                             sizeof(SensorRawData) / 4);
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

    BMI160_CHECK_ERR_REG();

    r = _dev->write_register(BMI160_REG_CMD, BMI160_CMD_FIFO_FLUSH);
    if (!r) {
        hal.console->printf("BMI160: Unable to flush FIFO\n");
        return false;
    }

    return true;
}

void AP_InertialSensor_BMI160::_read_sample()
{
    bool r = true;
    bool data_ready;
    struct SensorRawData raw_data;

    if (_int1_pin) {
        data_ready = _int1_pin->read() != 0;
    } else {
        uint8_t v;
        r = _dev->read_registers(BMI160_REG_STATUS, &v, 1);
        if (!r) {
            goto read_sample_end;
        }
        data_ready = v & (BMI160_STATUS_DRDY_ACC | BMI160_STATUS_DRDY_GYR);
    }

    if (!data_ready) {
        goto read_sample_end;
    }

    r = _dev->read_registers(BMI160_REG_DATA_GYR_X_L,
                             (uint8_t *)&raw_data,
                             sizeof(raw_data));
    if (!r) {
        goto read_sample_end;
    }

    _accumulate(&raw_data, 1);

read_sample_end:
    if (!r) {
        hal.console->printf("BMI160: error on reading sample\n");
    }
}

void AP_InertialSensor_BMI160::_read_fifo()
{
    /* NOTE: *Maybe* we could improve this function so that it does only one
     * SPI transaction by guessing the number of bytes in the FIFO in order to
     * read FIFO length and data altogether. If the prediction is wrong we can
     * prune the data retrieved case there's less than predicted. This approach
     * needs some investigation. */
    struct SensorRawData raw_data[BMI160_MAX_FIFO_SAMPLES];
    uint16_t num_bytes;
    uint16_t excess = 0;
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
    if (num_bytes > sizeof(raw_data)) {
        excess = num_bytes - sizeof(raw_data);
        num_bytes = sizeof(raw_data);
    }

    r = _dev->read_registers(BMI160_REG_FIFO_DATA,
                             (uint8_t *)raw_data,
                             num_bytes);
    if (!r) {
        goto read_fifo_end;
    }

    if (excess) {
        hal.console->printf("BMI160: dropping %u samples from fifo\n",
                            (uint8_t)(excess / sizeof(raw_data[0])));
        _dev->write_register(BMI160_REG_CMD, BMI160_CMD_FIFO_FLUSH);
    }

    _accumulate(raw_data, num_bytes / sizeof(raw_data[0]));

read_fifo_end:
    if (!r) {
        hal.console->printf("BMI160: error on reading FIFO\n");
    }
}

void AP_InertialSensor_BMI160::_poll_data()
{
    if (!_dev->get_semaphore()->take_nonblocking()) {
        return;
    }

    if (_use_fifo) {
        _read_fifo();
    } else {
        _read_sample();
    }

    _dev->get_semaphore()->give();
}

void AP_InertialSensor_BMI160::_accumulate(SensorRawData *data, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        Vector3f accel{(float) (int16_t)le16toh(data[i].accel.y),
                       (float) (int16_t)le16toh(data[i].accel.x),
                       (float)-(int16_t)le16toh(data[i].accel.z)};
        Vector3f gyro{(float) (int16_t)le16toh(data[i].gyro.y),
                      (float) (int16_t)le16toh(data[i].gyro.x),
                      (float)-(int16_t)le16toh(data[i].gyro.z)};

        accel *= _accel_scale;
        gyro *= _gyro_scale;

        _rotate_and_correct_accel(_accel_instance, accel);
        _rotate_and_correct_gyro(_gyro_instance, gyro);

        _notify_new_accel_raw_sample(_accel_instance, accel);
        _notify_new_gyro_raw_sample(_gyro_instance, gyro);
    }
}

bool AP_InertialSensor_BMI160::_hardware_init()
{
    bool ret = false;

    hal.scheduler->delay(BMI160_POWERUP_DELAY_MSEC);

    if (!_dev->get_semaphore()->take(100)) {
        AP_HAL::panic("BMI160: Unable to get semaphore");
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

    hal.scheduler->suspend_timer_procs();
    ret = _hardware_init();
    hal.scheduler->resume_timer_procs();

    if (!ret) {
        hal.console->printf("BMI160: failed to init\n");
    }

    return ret;
}

#endif
