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
/*
  Driver by Andrew Tridgell, Nov 2016
 */
#include "AP_Compass_MMC3416.h"

#include <AP_HAL/AP_HAL.h>
#include <utility>
#include <AP_Math/AP_Math.h>
#include <stdio.h>

extern const AP_HAL::HAL &hal;

#define REG_PRODUCT_ID      0x20
#define REG_XOUT_L          0x00
#define REG_STATUS          0x06
#define REG_CONTROL0        0x07
#define REG_CONTROL1        0x08

AP_Compass_Backend *AP_Compass_MMC3416::probe(Compass &compass,
                                              AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_MMC3416 *sensor = new AP_Compass_MMC3416(compass, std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_MMC3416::AP_Compass_MMC3416(Compass &compass,
                                       AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                       bool _force_external,
                                       enum Rotation _rotation)
    : AP_Compass_Backend(compass)
    , dev(std::move(_dev))
    , force_external(_force_external)
    , rotation(_rotation)
{
}

bool AP_Compass_MMC3416::init()
{
    if (!dev->get_semaphore()->take(0)) {
        return false;
    }

    dev->set_retries(10);
    
    uint8_t whoami;
    if (!dev->read_registers(REG_PRODUCT_ID, &whoami, 1) ||
        whoami != 0x06) {
        // not a MMC3416
        goto fail;
    }

    dev->setup_checked_registers(2);

    // reset sensor
    dev->write_register(REG_CONTROL1, 0x80);
    hal.scheduler->delay(10);
    
    dev->write_register(REG_CONTROL0, 0x0E, true); // continuous 50Hz
    dev->write_register(REG_CONTROL1, 0x00, true); // 16 bit, 7.92ms
    
    dev->get_semaphore()->give();

    /* register the compass instance in the frontend */
    compass_instance = register_compass();

    printf("Found a MMC3416 on 0x%x as compass %u\n", dev->get_bus_id(), compass_instance);
    
    set_rotation(compass_instance, rotation);

    if (force_external) {
        set_external(compass_instance, true);
    }
    
    dev->set_device_type(DEVTYPE_MMC3416);
    set_dev_id(compass_instance, dev->get_bus_id());

    dev->set_retries(1);
    
    // call timer() at 50Hz
    dev->register_periodic_callback(20000,
                                    FUNCTOR_BIND_MEMBER(&AP_Compass_MMC3416::timer, bool));

    return true;

fail:
    dev->get_semaphore()->give();
    return false;
}

bool AP_Compass_MMC3416::timer()
{
    struct PACKED {
        uint16_t magx;
        uint16_t magy;
        uint16_t magz;
    } data;
    const uint16_t zero_offset = 32768; // 16 bit mode
    const uint16_t sensitivity = 2048; // counts per Gauss, 16 bit mode
    const float counts_to_milliGauss = 1.0e3f / sensitivity;
    Vector3f field;

    if (!dev->read_registers(REG_XOUT_L, (uint8_t *)&data, sizeof(data))) {
        goto check_registers;
    }

    // this assumes 16 bit output, which gives zero of 32768
    field(float(data.magx) - zero_offset, float(data.magy) - zero_offset, float(data.magz) - zero_offset);
    field *= counts_to_milliGauss;

    /* rotate raw_field from sensor frame to body frame */
    rotate_field(field, compass_instance);

    /* publish raw_field (uncorrected point sample) for calibration use */
    publish_raw_field(field, AP_HAL::micros(), compass_instance);

    /* correct raw_field for known errors */
    correct_field(field, compass_instance);

    if (_sem->take(0)) {
        accum += field;
        accum_count++;
        _sem->give();
    }

check_registers:
    dev->check_next_register();
    return true;
}

void AP_Compass_MMC3416::read()
{
    if (!_sem->take_nonblocking()) {
        return;
    }
    if (accum_count == 0) {
        _sem->give();
        return;
    }

    accum /= accum_count;

    publish_filtered_field(accum, compass_instance);

    accum.zero();
    accum_count = 0;
    
    _sem->give();
}
