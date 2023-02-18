/*
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

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_BMI270.h"

//#define BMI270_DEBUG

// BMI270 registers (not the complete list)
enum BMI270Register : uint8_t {
    BMI270_REG_CHIP_ID = 0x00,
    BMI270_REG_ERR_REG = 0x02,
    BMI270_REG_STATUS = 0x03,
    BMI270_REG_ACC_DATA_X_LSB = 0x0C,
    BMI270_REG_GYR_DATA_X_LSB = 0x12,
    BMI270_REG_SENSORTIME_0 = 0x18,
    BMI270_REG_SENSORTIME_1 = 0x19,
    BMI270_REG_SENSORTIME_2 = 0x1A,
    BMI270_REG_EVENT = 0x1B,
    BMI270_REG_INT_STATUS_0 = 0x1C,
    BMI270_REG_INT_STATUS_1 = 0x1D,
    BMI270_REG_INTERNAL_STATUS = 0x21,
    BMI270_REG_TEMPERATURE_LSB = 0x22,
    BMI270_REG_TEMPERATURE_MSB = 0x23,
    BMI270_REG_FIFO_LENGTH_LSB = 0x24,
    BMI270_REG_FIFO_LENGTH_MSB = 0x25,
    BMI270_REG_FIFO_DATA = 0x26,
    BMI270_REG_ACC_CONF = 0x40,
    BMI270_REG_ACC_RANGE = 0x41,
    BMI270_REG_GYRO_CONF = 0x42,
    BMI270_REG_GYRO_RANGE = 0x43,
    BMI270_REG_AUX_CONF = 0x44,
    BMI270_REG_FIFO_DOWNS = 0x45,
    BMI270_REG_FIFO_WTM_0 = 0x46,
    BMI270_REG_FIFO_WTM_1 = 0x47,
    BMI270_REG_FIFO_CONFIG_0 = 0x48,
    BMI270_REG_FIFO_CONFIG_1 = 0x49,
    BMI270_REG_SATURATION = 0x4A,
    BMI270_REG_INT1_IO_CTRL = 0x53,
    BMI270_REG_INT2_IO_CTRL = 0x54,
    BMI270_REG_INT_LATCH = 0x55,
    BMI270_REG_INT1_MAP_FEAT = 0x56,
    BMI270_REG_INT2_MAP_FEAT = 0x57,
    BMI270_REG_INT_MAP_DATA = 0x58,
    BMI270_REG_INIT_CTRL = 0x59,
    BMI270_REG_INIT_DATA = 0x5E,
    BMI270_REG_ACC_SELF_TEST = 0x6D,
    BMI270_REG_GYR_SELF_TEST_AXES = 0x6E,
    BMI270_REG_PWR_CONF = 0x7C,
    BMI270_REG_PWR_CTRL = 0x7D,
    BMI270_REG_CMD = 0x7E,
};

/**
* The following device config microcode has the following copyright:
*
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
const uint8_t AP_InertialSensor_BMI270::maximum_fifo_config_file[] = { BMI270_REG_INIT_DATA,
    0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
    0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5,
    0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00, 0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22,
    0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f, 0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
    0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00,
    0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50, 0x00, 0x30, 0x12, 0x24, 0xeb,
    0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f, 0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5,
    0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80, 0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41,
    0x33, 0x98, 0x2e, 0xc2, 0xc4, 0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24,
    0x00, 0xfc, 0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe, 0x94,
    0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2, 0xfe, 0x82, 0x05, 0x2f,
    0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20,
    0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a, 0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08,
    0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42, 0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0,
    0x6f, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
    0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30, 0x23, 0x2e, 0x61,
    0xf5, 0xeb, 0x2c, 0xe1, 0x6f
};
/*
 * End of Bosch microcode copyright
 */

#define BMI270_CHIP_ID 0x24
#define BMI270_CMD_SOFTRESET 0xB6
#define BMI270_CMD_FIFOFLUSH 0xB0
#define BMI270_FIFO_ACC_EN 0x40
#define BMI270_FIFO_GYR_EN 0x80
#define BMI270_BACKEND_SAMPLE_RATE 1600

/* Datasheet says that the device powers up and soft resets in 2s, so waiting for
 * 5ms before initialization is enough. */
#define BMI270_POWERUP_DELAY_MSEC 5

/* This number of samples should provide only one read burst operation on the
 * FIFO most of the time (99.99%). */
#define BMI270_MAX_FIFO_SAMPLES 8

#define BMI270_HARDWARE_INIT_MAX_TRIES 5

const uint32_t BACKEND_PERIOD_US = 1000000UL / BMI270_BACKEND_SAMPLE_RATE;

extern const AP_HAL::HAL& hal;

AP_InertialSensor_BMI270::AP_InertialSensor_BMI270(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                   enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _rotation(rotation)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_BMI270::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_BMI270(imu, std::move(dev), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_InertialSensor_Backend *
AP_InertialSensor_BMI270::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_BMI270(imu, std::move(dev), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_BMI270::start()
{
    _dev->get_semaphore()->take_blocking();

    configure_accel();

    configure_gyro();

    configure_fifo();

    _dev->get_semaphore()->give();

    if (!_imu.register_accel(_accel_instance, BMI270_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(DEVTYPE_BMI270)) ||
        !_imu.register_gyro(_gyro_instance, BMI270_BACKEND_SAMPLE_RATE, _dev->get_bus_id_devtype(DEVTYPE_BMI270))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(_gyro_instance, _rotation);
    set_accel_orientation(_accel_instance, _rotation);

    /* Call read_fifo() at 1600Hz */
    periodic_handle = _dev->register_periodic_callback(BACKEND_PERIOD_US, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_BMI270::read_fifo, void));
}

bool AP_InertialSensor_BMI270::update()
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);
    return true;
}

/*
  read from registers, special SPI handling needed
*/
bool AP_InertialSensor_BMI270::read_registers(uint8_t reg, uint8_t *data, uint8_t len)
{
    // for SPI we need to discard the first returned byte. See
    // datasheet for explanation
    uint8_t b[len+2];
    b[0] = reg | 0x80;
    memset(&b[1], 0, len+1);
    if (!_dev->transfer(b, len+2, b, len+2)) {
        return false;
    }
    memcpy(data, &b[2], len);
    return true;
}

/*
  write registers with retries. The SPI sensor may take
  several tries to correctly write a register
*/
bool AP_InertialSensor_BMI270::write_register(uint8_t reg, uint8_t v)
{
    for (uint8_t i=0; i<8; i++) {
        _dev->write_register(reg, v);
        uint8_t v2 = 0;
        if (read_registers(reg, &v2, 1) && v2 == v) {
            return true;
        }
    }
    return false;
}

void AP_InertialSensor_BMI270::check_err_reg()
{
#ifdef BMI270_DEBUG
    uint8_t err = 0;
    read_registers(BMI270_REG_ERR_REG, &err, 1);

    if (err) {
        if ((err & 1) == 1) {
            uint8_t status =  0;
            read_registers(BMI270_REG_INTERNAL_STATUS, &status, 1);
            switch (status & 0xF) {
            case 0:
                AP_HAL::panic("BMI270: not_init");
                break;
            case 2:
                AP_HAL::panic("BMI270: init_err");
                break;
            case 3:
                AP_HAL::panic("BMI270: drv_err");
                break;
            case 4:
                AP_HAL::panic("BMI270: sns_stop");
                break;
            case 5:
                AP_HAL::panic("BMI270: nvm_error");
                break;
            case 6:
                AP_HAL::panic("BMI270: start_up_error");
                break;
            case 7:
                AP_HAL::panic("BMI270: compat_error");
                break;
            case 1: // init ok
                if ((status>>5 & 1) == 1) {
                    AP_HAL::panic("BMI270: axes_remap_error");
                } else if ((status>>6 & 1) == 1) {
                    AP_HAL::panic("BMI270: odr_50hz_error");
                }
                break;
            }
        } else if ((err>>6 & 1) == 1) {
            AP_HAL::panic("BMI270: fifo_err");
        } else if ((err>>7 & 1) == 1) {
            AP_HAL::panic("BMI270: aux_err");
        } else {
            AP_HAL::panic("BMI270: internal error detected %d", err>>1 & 0xF);
        }
    }
#endif
}

void AP_InertialSensor_BMI270::configure_accel()
{
    // set acc in high performance mode with OSR4 filtering (751Hz/4) at 1600Hz
    // see https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BMI270-OSR-mode-behaviour/td-p/52020
    // OSR4 is a 188Hz filter cutoff, acc_bwp == 0, equivalent to other driver filters
    write_register(BMI270_REG_ACC_CONF, 1U<<7 | 0x0C);
    // set acc to 16G full scale
    write_register(BMI270_REG_ACC_RANGE, 0x03);

    check_err_reg();
}

void AP_InertialSensor_BMI270::configure_gyro()
{
    // set gyro in high performance filter mode, high performance noise mode, normal filtering at 3.2KHz
    // filter cutoff 751hz
    write_register(BMI270_REG_GYRO_CONF, 1U<<7 | 1<<6 | 2<<4 | 0x0D);
    // set gyro to 2000dps full scale
    // for some reason you have to enable the ois_range bit (bit 3) for 2000dps as well
    // or else the gyro scale will be 250dps when in prefiltered FIFO mode (not documented in datasheet!)
    write_register(BMI270_REG_GYRO_RANGE, 0x08);

    check_err_reg();
}

void AP_InertialSensor_BMI270::configure_fifo()
{
    // stop when full, disable sensortime frame
    write_register(BMI270_REG_FIFO_CONFIG_0, 0x01);
    // accel + gyro data in FIFO together with headers
    write_register(BMI270_REG_FIFO_CONFIG_1, 1U<<7 | 1U<<6 | 1U<<4);
    // filtered data downsampled by 2**1 to 1600Hz
    write_register(BMI270_REG_FIFO_DOWNS, 1U<<7 | 1U<<3 | 0x01);
    // disable advanced power save, enable FIFO self-wake
    write_register(BMI270_REG_PWR_CONF, 0x02);
    // Enable the gyro, accelerometer and temperature sensor - disable aux interface
    write_register(BMI270_REG_PWR_CTRL, 0x0E);

    fifo_reset();

    check_err_reg();
}

void AP_InertialSensor_BMI270::fifo_reset()
{
    // flush and reset FIFO
    write_register(BMI270_REG_CMD, BMI270_CMD_FIFOFLUSH);

    notify_accel_fifo_reset(_accel_instance);
    notify_gyro_fifo_reset(_gyro_instance);
}

/*
  read fifo
 */
void AP_InertialSensor_BMI270::read_fifo(void)
{
    // check for FIFO errors/overflow
    uint8_t err = 0;
    read_registers(BMI270_REG_ERR_REG, &err, 1);
    if ((err>>6 & 1) == 1) {
        fifo_reset();
        return;
    }

    uint8_t len[2];
    if (!read_registers(BMI270_REG_FIFO_LENGTH_LSB, len, 2)) {
        _inc_accel_error_count(_accel_instance);
        _inc_gyro_error_count(_gyro_instance);
        return;
    }
    uint16_t fifo_length = len[0] + (len[1]<<8);
    if (fifo_length & 0x8000) {
        // empty
        return;
    }

    // don't read more than 8 frames at a time
    if (fifo_length > BMI270_MAX_FIFO_SAMPLES*13) {
        fifo_length = BMI270_MAX_FIFO_SAMPLES*13;
    }
    if (fifo_length == 0) {
        return;
    }

    uint8_t data[fifo_length];
    if (!read_registers(BMI270_REG_FIFO_DATA, data, fifo_length)) {
        _inc_accel_error_count(_accel_instance);
        _inc_gyro_error_count(_gyro_instance);
        return;
    }

    // adjust the periodic callback to be synchronous with the incoming data
    // this means that we rarely run read_fifo() without updating the sensor data
    _dev->adjust_periodic_callback(periodic_handle, BACKEND_PERIOD_US);

    const uint8_t *p = &data[0];
    while (fifo_length >= 12) {
        /*
          the fifo frames are variable length, with the frame type in the first byte
         */
        uint8_t frame_len = 2;
        switch (p[0] & 0xFC) {
        case 0x84: // accel
            frame_len = 7;
            parse_accel_frame(p+1);
            break;
        case 0x88: // gyro
            frame_len = 7;
            parse_gyro_frame(p+1);
            break;
        case 0x8C: // accel + gyro
            frame_len = 13;
            parse_gyro_frame(p+1);
            parse_accel_frame(p+7);
            break;
        case 0x40:
            // skip frame
            frame_len = 2;
            break;
        case 0x44:
            // sensortime frame
            frame_len = 4;
            break;
        case 0x48:
            // fifo config frame
            frame_len = 5;
            break;
        case 0x50:
            // sample drop frame
            frame_len = 2;
            break;
        case 0x80:
            // invalid frame
            fifo_reset();
            return;
        }
        p += frame_len;
        fifo_length -= frame_len;
    }

    // temperature sensor updated every 10ms
    if (temperature_counter++ == 100) {
        temperature_counter = 0;
        uint8_t tbuf[2];
        if (!read_registers(BMI270_REG_TEMPERATURE_LSB, tbuf, 2)) {
            _inc_accel_error_count(_accel_instance);
            _inc_gyro_error_count(_gyro_instance);
        } else {
            uint16_t tval = tbuf[0] | (tbuf[1] << 8);
            if (tval != 0x8000) {   // 0x8000 is invalid
                int16_t klsb = static_cast<int16_t>(tval);
                float temp_degc = klsb * 0.002f + 23.0f;
                _publish_temperature(_accel_instance, temp_degc);
            }
        }
    }
}

void AP_InertialSensor_BMI270::parse_accel_frame(const uint8_t* d)
{
    // assume configured for 16g range
    const float scale = (1.0/32768.0) * GRAVITY_MSS * 16.0;
    int16_t xyz[3] {
        int16_t(uint16_t(d[0] | (d[1]<<8))),
        int16_t(uint16_t(d[2] | (d[3]<<8))),
        int16_t(uint16_t(d[4] | (d[5]<<8)))};
    Vector3f accel(xyz[0], xyz[1], xyz[2]);

    accel *= scale;

    _rotate_and_correct_accel(_accel_instance, accel);
    _notify_new_accel_raw_sample(_accel_instance, accel);
}

void AP_InertialSensor_BMI270::parse_gyro_frame(const uint8_t* d)
{
    // data is 16 bits with 2000dps range
    const float scale = radians(2000.0f) / 32767.0f;
    int16_t xyz[3] {
        int16_t(uint16_t(d[0] | d[1]<<8)),
        int16_t(uint16_t(d[2] | d[3]<<8)),
        int16_t(uint16_t(d[4] | d[5]<<8)) };
    Vector3f gyro(xyz[0], xyz[1], xyz[2]);
    gyro *= scale;

    _rotate_and_correct_gyro(_gyro_instance, gyro);
    _notify_new_gyro_raw_sample(_gyro_instance, gyro);
}

bool AP_InertialSensor_BMI270::hardware_init()
{
    bool init = false;
    bool read_chip_id = false;

    hal.scheduler->delay(BMI270_POWERUP_DELAY_MSEC);

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    for (unsigned i = 0; i < BMI270_HARDWARE_INIT_MAX_TRIES; i++) {

        uint8_t chip_id = 0;
        /* If CSB sees a rising edge after power-up, the device interface switches to SPI
         * after 200 μs until a reset or the next power-up occurs therefore it is recommended
         * to perform a SPI single read of register CHIP_ID (the obtained value will be invalid)
         * before the actual communication start, in order to use the SPI interface */
        read_registers(BMI270_REG_CHIP_ID, &chip_id, 1);
        hal.scheduler->delay(1);

        write_register(BMI270_REG_CMD, BMI270_CMD_SOFTRESET);
        hal.scheduler->delay(BMI270_POWERUP_DELAY_MSEC);    // power on and soft reset time is 2ms

        // switch to SPI mode again
        read_registers(BMI270_REG_CHIP_ID, &chip_id, 1);
        hal.scheduler->delay(1);

        read_registers(BMI270_REG_CHIP_ID, &chip_id, 1);
        if (chip_id != BMI270_CHIP_ID) {
            continue;
        }

        // successfully identified the chip, proceed with initialisation
        read_chip_id = true;

        // disable power save
        write_register(BMI270_REG_PWR_CONF, 0x00);
        hal.scheduler->delay(1); // needs to be at least 450us

        // upload config
        write_register(BMI270_REG_INIT_CTRL, 0x00);

        // Transfer the config file, data packet includes INIT_DATA
        _dev->transfer(maximum_fifo_config_file, sizeof(maximum_fifo_config_file), nullptr, 0);

        // config is done
        write_register(BMI270_REG_INIT_CTRL, 1);

        // check the results
        hal.scheduler->delay(20);

        uint8_t status = 0;
        read_registers(BMI270_REG_INTERNAL_STATUS, &status, 1);

        if ((status & 1) == 1) {
            init = true;
            DEV_PRINTF("BMI270 initialized after %d retries\n", i+1);
            break;
        }
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    if (read_chip_id && !init) {
        DEV_PRINTF("BMI270: failed to init\n");
    }

    return init;
}

bool AP_InertialSensor_BMI270::init()
{
    _dev->set_read_flag(0x80);

    return hardware_init();
}