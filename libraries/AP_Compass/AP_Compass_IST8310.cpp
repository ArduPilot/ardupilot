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

#include <AP_HAL/AP_HAL.h>

#include <utility>
#include <stdio.h>

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

#define WAI_REG 0x0
#define DEVICE_ID 0x10

#define CNTL1_REG 0xA
#define SINGLE_MEASUREMENT_MODE 0x1
#define ODR_100HZ 0x6

#define STAT1_REG 0x2
#define DATA_RDY 0x1

#define AVGCNTL_REG 0x41
#define AVERAGING_Y_BY_2 0x20
#define AVERAGING_XZ_BY_4 0x04

#define PDCNTL_REG 0x42
#define NORMAL_PULSE_DURATION 0xC0

extern const AP_HAL::HAL &hal;

AP_Compass_Backend *AP_Compass_IST8310::probe(Compass &compass,
                                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                             enum Rotation rotation
                                             )
{
    AP_Compass_IST8310 *sensor = new AP_Compass_IST8310(compass, std::move(dev), rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_IST8310::AP_Compass_IST8310(Compass &compass,
                                     AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     enum Rotation _rotation)
    : AP_Compass_Backend(compass)
    , _dev(std::move(dev))
    , rotation(_rotation)
{
}

bool AP_Compass_IST8310::init()
{
    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.console->printf("IST8310: Unable to get bus semaphore\n");
        goto fail_sem;
    }

    if (!check_id()) {
        hal.console->printf("IST8310: Not a IST device\n");
        goto fail;
    }

    if (!setup_sampling()) {
        goto fail;
    }

    start_conversion();

    _dev->get_semaphore()->give();

    printf("%s found on bus %u address 0x%02x\n", name, _dev->bus_num(), _dev->get_bus_address());

    compass_instance = register_compass();

    hal.console->printf("Found a IST8310 on 0x%x as compass %u\n", _dev->get_bus_id(), compass_instance);

    set_rotation(compass_instance, rotation);

    /* register the compass instance in the frontend */
    _dev->set_device_type(DEVTYPE_IST8310);
    set_dev_id(compass_instance, _dev->get_bus_id());

    _dev->register_periodic_callback(10 * USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Compass_IST8310::timer, bool));

    return true;

fail:
    _dev->get_semaphore()->give();
fail_sem:
    return false;
}

bool AP_Compass_IST8310::check_id()
{
    uint8_t val = 0;

    if (!_dev->read_registers(WAI_REG, &val, 1)) {
        return false;
    }

    if (val != DEVICE_ID) {
        return false;
    }

    return true;
}

bool AP_Compass_IST8310::setup_sampling()
{
    bool ret = _dev->write_register(AVGCNTL_REG, AVERAGING_Y_BY_2 | AVERAGING_XZ_BY_4);

    return ret && _dev->write_register(PDCNTL_REG, NORMAL_PULSE_DURATION);
}

void AP_Compass_IST8310::start_conversion()
{
    _dev->write_register(CNTL1_REG,
            SINGLE_MEASUREMENT_MODE);
}

bool AP_Compass_IST8310::timer()
{

    bool ret = false;

    struct PACKED {
        uint8_t status;
        le16_t rx;
        le16_t ry;
        le16_t rz;
    } buffer;


    ret = _dev->read_registers(STAT1_REG, (uint8_t *) &buffer, sizeof(buffer));

    if (!ret) {
        return true; /* We're going to be back on the next iteration either way */
    }

    auto status = buffer.status;

    if (!(status & 0x01)) {
        /* We're not ready yet */
       return true;
    }

    auto x = static_cast<int16_t>(le16toh(buffer.rx));
    auto y = static_cast<int16_t>(le16toh(buffer.ry));
    auto z = static_cast<int16_t>(le16toh(buffer.rz));

    /* convert uT to milligauss */
    Vector3f field = Vector3f{x * 3.0f, y * 3.0f, z * 3.0f};

    /* rotate raw_field from sensor frame to body frame */
    rotate_field(field, compass_instance);

    const auto now = AP_HAL::micros();

    /* publish raw_field (uncorrected point sample) for calibration use */
    publish_raw_field(field, now, compass_instance);

    /* correct raw_field for known errors */
    correct_field(field, compass_instance);

    if (_sem->take(0)) {
        mag_accum += field;
        accum_count++;
        _sem->give();
    }


    start_conversion();

    return true;
}

void AP_Compass_IST8310::read()
{
    if (!_sem->take_nonblocking()) {
        return;
    }

    if (accum_count == 0) {
        _sem->give();
        return;
    }

    Vector3f field(mag_accum);
    field /= accum_count;
    mag_accum.zero();
    accum_count = 0;

    _sem->give();

    publish_filtered_field(field, compass_instance);

}
