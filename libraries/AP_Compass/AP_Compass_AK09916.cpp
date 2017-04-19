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
#include "AP_Compass_AK09916.h"

#include <AP_HAL/AP_HAL.h>
#include <utility>
#include <AP_Math/AP_Math.h>
#include <stdio.h>

#define REG_COMPANY_ID      0x00
#define REG_DEVICE_ID       0x01
#define REG_ST1             0x10
#define REG_HXL             0x11
#define REG_HXH             0x12
#define REG_HYL             0x13
#define REG_HYH             0x14
#define REG_HZL             0x15
#define REG_HZH             0x16
#define REG_TMPS            0x17
#define REG_ST2             0x18
#define REG_CNTL1           0x30
#define REG_CNTL2           0x31
#define REG_CNTL3           0x32

extern const AP_HAL::HAL &hal;

AP_Compass_Backend *AP_Compass_AK09916::probe(Compass &compass,
                                              AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_AK09916 *sensor = new AP_Compass_AK09916(compass, std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_AK09916::AP_Compass_AK09916(Compass &compass,
                                       AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                       bool _force_external,
                                       enum Rotation _rotation)
    : AP_Compass_Backend(compass)
    , dev(std::move(_dev))
    , force_external(_force_external)
    , rotation(_rotation)
{
}

bool AP_Compass_AK09916::init()
{
    if (!dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    uint16_t whoami;
    if (!dev->read_registers(REG_COMPANY_ID, (uint8_t *)&whoami, 2) ||
        whoami != 0x0948) {
        // not a AK09916
        goto fail;
    }

    dev->setup_checked_registers(2);

    dev->write_register(REG_CNTL2, 0x08, true); // continuous 100Hz
    dev->write_register(REG_CNTL3, 0x00, true);
    
    dev->get_semaphore()->give();

    /* register the compass instance in the frontend */
    compass_instance = register_compass();

    printf("Found a AK09916 on 0x%x as compass %u\n", dev->get_bus_id(), compass_instance);
    
    set_rotation(compass_instance, rotation);

    if (force_external) {
        set_external(compass_instance, true);
    }
    
    dev->set_device_type(DEVTYPE_AK09916);
    set_dev_id(compass_instance, dev->get_bus_id());

    // call timer() at 100Hz
    dev->register_periodic_callback(10000,
                                    FUNCTOR_BIND_MEMBER(&AP_Compass_AK09916::timer, void));

    return true;

fail:
    dev->get_semaphore()->give();
    return false;
}

void AP_Compass_AK09916::timer()
{
    struct PACKED {
        int16_t magx;
        int16_t magy;
        int16_t magz;
        uint8_t tmps;
        uint8_t status2;
    } data;
    const float to_utesla = 4912.0f / 32752.0f;
    const float utesla_to_milliGauss = 10;
    const float range_scale = to_utesla * utesla_to_milliGauss;
    Vector3f field;

    // check data ready
    uint8_t st1;
    if (!dev->read_registers(REG_ST1, &st1, 1) || !(st1 & 1)) {
        goto check_registers;
    }

    if (!dev->read_registers(REG_HXL, (uint8_t *)&data, sizeof(data))) {
        goto check_registers;
    }

    field(data.magx * range_scale, data.magy * range_scale, data.magz * range_scale);

    /* rotate raw_field from sensor frame to body frame */
    rotate_field(field, compass_instance);

    /* publish raw_field (uncorrected point sample) for calibration use */
    publish_raw_field(field, AP_HAL::micros(), compass_instance);

    /* correct raw_field for known errors */
    correct_field(field, compass_instance);

    if (_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        accum += field;
        accum_count++;
        _sem->give();
    }

check_registers:
    dev->check_next_register();
}

void AP_Compass_AK09916::read()
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
