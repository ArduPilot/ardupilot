/*
 * Copyright (C) 2018  Lucas De Marchi. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or
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
#include "AP_Compass_IST8308.h"

#include <stdio.h>
#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

#define WAI_REG 0x0
#define DEVICE_ID 0x08

#define STAT1_REG 0x10
#define STAT1_VAL_DRDY 0x1
#define STAT1_VAL_DOR 0x2

#define DATAX_L_REG 0x11
#define DATAX_H_REG 0x12
#define DATAY_L_REG 0x13
#define DATAY_H_REG 0x14
#define DATAZ_L_REG 0x15
#define DATAZ_H_REG 0x16

#define CNTL1_REG 0x30

#define CNTL2_REG 0x31
#define CNTL2_VAL_STANDBY_MODE      0x0
#define CNTL2_VAL_SINGLE_MODE       0x1
#define CNTL2_VAL_CONT_ODR10_MODE   0x2
#define CNTL2_VAL_CONT_ODR20_MODE   0x4
#define CNTL2_VAL_CONT_ODR50_MODE   0x6
#define CNTL2_VAL_CONT_ODR100_MODE  0x8
#define CNTL2_VAL_CONT_ODR200_MODE  0xA
#define CNTL2_VAL_CONT_ODR8_MODE    0xB
#define CNTL2_VAL_CONT_ODR1_MODE    0xC
#define CNTL2_VAL_CONT_ODR0P5_MODE  0xD
#define CNTL2_VAL_SINGLE_TEST_MODE  0x10

#define CNTL3_REG 0x32
#define CNTL3_VAL_SRST 1
#define CNTL3_VAL_DRDY_POLARITY_HIGH (1 << 2)
#define CNTL3_VAL_DRDY_EN (1 << 3)

#define CNTL4_REG 0x34
#define CNTL4_VAL_DYNAMIC_RANGE_500 0
#define CNTL4_VAL_DYNAMIC_RANGE_200 0x1

#define OSRCNTL_REG 0x41
#define OSRCNTL_VAL_XZ_1  (0)
#define OSRCNTL_VAL_XZ_2  (1)
#define OSRCNTL_VAL_XZ_4  (2)
#define OSRCNTL_VAL_XZ_8  (3)
#define OSRCNTL_VAL_XZ_16 (4)
#define OSRCNTL_VAL_XZ_32 (4)
#define OSRCNTL_VAL_Y_1  (0 << 3)
#define OSRCNTL_VAL_Y_2  (1 << 3)
#define OSRCNTL_VAL_Y_4  (2 << 3)
#define OSRCNTL_VAL_Y_8  (3 << 3)
#define OSRCNTL_VAL_Y_16 (4 << 3)
#define OSRCNTL_VAL_Y_32 (5 << 3)

#define SAMPLING_PERIOD_USEC (10 * AP_USEC_PER_MSEC)

extern const AP_HAL::HAL &hal;

AP_Compass_Backend *AP_Compass_IST8308::probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_Compass_IST8308 *sensor = new AP_Compass_IST8308(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_IST8308::AP_Compass_IST8308(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                       bool force_external,
                                       enum Rotation rotation)
    : _dev(std::move(dev))
    , _rotation(rotation)
    , _force_external(force_external)
{
}

bool AP_Compass_IST8308::init()
{
    uint8_t reset_count = 0;

    _dev->get_semaphore()->take_blocking();

    // high retries for init
    _dev->set_retries(10);

    uint8_t whoami;
    if (!_dev->read_registers(WAI_REG, &whoami, 1) ||
        whoami != DEVICE_ID) {
        // not an IST8308
        goto fail;
    }

    for (; reset_count < 5; reset_count++) {
        if (!_dev->write_register(CNTL3_REG, CNTL3_VAL_SRST)) {
            hal.scheduler->delay(10);
            continue;
        }

        hal.scheduler->delay(20);

        uint8_t cntl3 = 0xFF;
        if (_dev->read_registers(CNTL3_REG, &cntl3, 1) &&
            (cntl3 & 0x01) == 0) {
            break;
        }
    }

    if (reset_count == 5) {
        printf("IST8308: failed to reset device\n");
        goto fail;
    }

    // DRDY enabled
    // Dynamic Range=±500 uT, Sensitivity=6.6 LSB/uT
    // OSR 16 (max 100Hz)
    // Start continuous mode at 100Hz
    if (!_dev->write_register(CNTL3_REG, CNTL3_VAL_DRDY_EN) ||
        !_dev->write_register(CNTL4_REG, CNTL4_VAL_DYNAMIC_RANGE_500) ||
        !_dev->write_register(OSRCNTL_REG, OSRCNTL_VAL_Y_16 | OSRCNTL_VAL_XZ_16) ||
        !_dev->write_register(CNTL2_REG, CNTL2_VAL_CONT_ODR100_MODE)) {
        printf("IST8308: found device but could not set it up\n");
        goto fail;
    }

    // lower retries for run
    _dev->set_retries(3);

    _dev->get_semaphore()->give();

    _instance = register_compass();

    printf("%s found on bus %u id %u address 0x%02x\n", name,
           _dev->bus_num(), _dev->get_bus_id(), _dev->get_bus_address());

    set_rotation(_instance, _rotation);

    _dev->set_device_type(DEVTYPE_IST8308);
    set_dev_id(_instance, _dev->get_bus_id());

    if (_force_external) {
        set_external(_instance, true);
    }

    _dev->register_periodic_callback(SAMPLING_PERIOD_USEC,
                                     FUNCTOR_BIND_MEMBER(&AP_Compass_IST8308::timer, void));

    _perf_xfer_err = hal.util->perf_alloc(AP_HAL::Util::PC_COUNT, "IST8308_xfer_err");

    return true;

fail:
    _dev->get_semaphore()->give();
    return false;
}

void AP_Compass_IST8308::timer()
{
    struct PACKED {
        le16_t rx;
        le16_t ry;
        le16_t rz;
    } buffer;
    uint8_t stat;

    if (!_dev->read_registers(STAT1_REG, &stat, 1) ||
        !(stat & STAT1_VAL_DRDY)) {
        hal.util->perf_count(_perf_xfer_err);
        return;
    }

    if (stat & STAT1_VAL_DOR) {
        printf("IST8308: data overrun\n");
    }

    if (!_dev->read_registers(DATAX_L_REG, (uint8_t *) &buffer,
                              sizeof(buffer))) {
        hal.util->perf_count(_perf_xfer_err);
        return;
    }

    auto x = static_cast<int16_t>(le16toh(buffer.rx));
    auto y = static_cast<int16_t>(le16toh(buffer.ry));
    auto z = static_cast<int16_t>(le16toh(buffer.rz));

    // flip Z to conform to right-hand rule convention
    z = -z;

    /* Resolution: 0.1515 µT/LSB - already convert to milligauss */
    Vector3f field = Vector3f{x * 1.515f, y * 1.515f, z * 1.515f};

    accumulate_sample(field, _instance);
}

void AP_Compass_IST8308::read()
{
    drain_accumulated_samples(_instance);
}
