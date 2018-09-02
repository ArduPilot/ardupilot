/*
 * Copyright (C) 2016  Emlid Ltd. All rights reserved.
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
 *
 * Driver by Georgii Staroselskii, Sep 2016
 */
#include "AP_Compass_IST8310.h"

#include <stdio.h>
#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

#define WAI_REG 0x0
#define DEVICE_ID 0x10

#define OUTPUT_X_L_REG 0x3
#define OUTPUT_X_H_REG 0x4
#define OUTPUT_Y_L_REG 0x5
#define OUTPUT_Y_H_REG 0x6
#define OUTPUT_Z_L_REG 0x7
#define OUTPUT_Z_H_REG 0x8

#define CNTL1_REG 0xA
#define CNTL1_VAL_SINGLE_MEASUREMENT_MODE 0x1

#define CNTL2_REG 0xB
#define CNTL2_VAL_SRST 1

#define AVGCNTL_REG 0x41
#define AVGCNTL_VAL_XZ_0  (0)
#define AVGCNTL_VAL_XZ_2  (1)
#define AVGCNTL_VAL_XZ_4  (2)
#define AVGCNTL_VAL_XZ_8  (3)
#define AVGCNTL_VAL_XZ_16 (4)
#define AVGCNTL_VAL_Y_0  (0 << 3)
#define AVGCNTL_VAL_Y_2  (1 << 3)
#define AVGCNTL_VAL_Y_4  (2 << 3)
#define AVGCNTL_VAL_Y_8  (3 << 3)
#define AVGCNTL_VAL_Y_16 (4 << 3)

#define PDCNTL_REG 0x42
#define PDCNTL_VAL_PULSE_DURATION_NORMAL 0xC0

#define SAMPLING_PERIOD_USEC (10 * AP_USEC_PER_MSEC)

/*
 * FSR:
 *   x, y: +- 1600 µT
 *   z:    +- 2500 µT
 *
 * Resolution according to datasheet is 0.3µT/LSB
 */
#define IST8310_RESOLUTION 0.3

static const int16_t IST8310_MAX_VAL_XY = (1600 / IST8310_RESOLUTION) + 1;
static const int16_t IST8310_MIN_VAL_XY = -IST8310_MAX_VAL_XY;
static const int16_t IST8310_MAX_VAL_Z  = (2500 / IST8310_RESOLUTION) + 1;
static const int16_t IST8310_MIN_VAL_Z  = -IST8310_MAX_VAL_Z;


extern const AP_HAL::HAL &hal;

AP_Compass_Backend *AP_Compass_IST8310::probe(Compass &compass,
                                              AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_Compass_IST8310 *sensor = new AP_Compass_IST8310(compass, std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_IST8310::AP_Compass_IST8310(Compass &compass,
                                       AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                       bool force_external,
                                       enum Rotation rotation)
    : AP_Compass_Backend(compass)
    , _dev(std::move(dev))
    , _rotation(rotation)
    , _force_external(force_external)
{
}

bool AP_Compass_IST8310::init()
{
    uint8_t reset_count = 0;

    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // high retries for init
    _dev->set_retries(10);

    uint8_t whoami;
    if (!_dev->read_registers(WAI_REG, &whoami, 1) ||
        whoami != DEVICE_ID) {
        // not an IST8310
        goto fail;
    }

    for (; reset_count < 5; reset_count++) {
        if (!_dev->write_register(CNTL2_REG, CNTL2_VAL_SRST)) {
            hal.scheduler->delay(10);
            continue;
        }

        hal.scheduler->delay(10);

        uint8_t cntl2 = 0xFF;
        if (_dev->read_registers(CNTL2_REG, &cntl2, 1) &&
            (cntl2 & 0x01) == 0) {
            break;
        }
    }

    if (reset_count == 5) {
        printf("IST8310: failed to reset device\n");
        goto fail;
    }

    if (!_dev->write_register(AVGCNTL_REG, AVGCNTL_VAL_Y_16 | AVGCNTL_VAL_XZ_16) ||
        !_dev->write_register(PDCNTL_REG, PDCNTL_VAL_PULSE_DURATION_NORMAL)) {
        printf("IST8310: found device but could not set it up\n");
        goto fail;
    }

    // lower retries for run
    _dev->set_retries(3);

    // start state machine: request a sample
    start_conversion();

    _dev->get_semaphore()->give();

    _instance = register_compass();

    printf("%s found on bus %u id %u address 0x%02x\n", name,
           _dev->bus_num(), _dev->get_bus_id(), _dev->get_bus_address());

    set_rotation(_instance, _rotation);

    _dev->set_device_type(DEVTYPE_IST8310);
    set_dev_id(_instance, _dev->get_bus_id());

    if (_force_external) {
        set_external(_instance, true);
    }
    
    _periodic_handle = _dev->register_periodic_callback(SAMPLING_PERIOD_USEC,
        FUNCTOR_BIND_MEMBER(&AP_Compass_IST8310::timer, void));

    _perf_xfer_err = hal.util->perf_alloc(AP_HAL::Util::PC_COUNT, "IST8310_xfer_err");
    _perf_bad_data = hal.util->perf_alloc(AP_HAL::Util::PC_COUNT, "IST8310_bad_data");

    return true;

fail:
    _dev->get_semaphore()->give();
    return false;
}

void AP_Compass_IST8310::start_conversion()
{
    if (!_dev->write_register(CNTL1_REG, CNTL1_VAL_SINGLE_MEASUREMENT_MODE)) {
        hal.util->perf_count(_perf_xfer_err);
        _ignore_next_sample = true;
    }
}

void AP_Compass_IST8310::timer()
{
    if (_ignore_next_sample) {
        _ignore_next_sample = false;
        start_conversion();
        return;
    }

    struct PACKED {
        le16_t rx;
        le16_t ry;
        le16_t rz;
    } buffer;

    bool ret = _dev->read_registers(OUTPUT_X_L_REG, (uint8_t *) &buffer, sizeof(buffer));
    if (!ret) {
        hal.util->perf_count(_perf_xfer_err);
        return;
    }

    start_conversion();

    /* same period, but start counting from now */
    _dev->adjust_periodic_callback(_periodic_handle, SAMPLING_PERIOD_USEC);

    auto x = static_cast<int16_t>(le16toh(buffer.rx));
    auto y = static_cast<int16_t>(le16toh(buffer.ry));
    auto z = static_cast<int16_t>(le16toh(buffer.rz));

    /*
     * Check if value makes sense according to the FSR and Resolution of
     * this sensor, discarding outliers
     */
    if (x > IST8310_MAX_VAL_XY || x < IST8310_MIN_VAL_XY ||
        y > IST8310_MAX_VAL_XY || y < IST8310_MIN_VAL_XY ||
        z > IST8310_MAX_VAL_Z  || z < IST8310_MIN_VAL_Z) {
        hal.util->perf_count(_perf_bad_data);
        return;
    }

    // flip Z to conform to right-hand rule convention
    z = -z;

    /* Resolution: 0.3 µT/LSB - already convert to milligauss */
    Vector3f field = Vector3f{x * 3.0f, y * 3.0f, z * 3.0f};

    /* rotate raw_field from sensor frame to body frame */
    rotate_field(field, _instance);

    /* publish raw_field (uncorrected point sample) for calibration use */
    publish_raw_field(field, _instance);

    /* correct raw_field for known errors */
    correct_field(field, _instance);

    if (_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _accum += field;
        _accum_count++;
        _sem->give();
    }
}

void AP_Compass_IST8310::read()
{
    if (!_sem->take_nonblocking()) {
        return;
    }

    if (_accum_count == 0) {
        _sem->give();
        return;
    }

    Vector3f field(_accum);
    field /= _accum_count;

    publish_filtered_field(field, _instance);

    _accum.zero();
    _accum_count = 0;

    _sem->give();
}
