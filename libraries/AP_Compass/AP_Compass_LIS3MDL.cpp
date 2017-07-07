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

  thanks to Robert Dickenson and the PX4 team for register definitions
 */
#include "AP_Compass_LIS3MDL.h"

#include <AP_HAL/AP_HAL.h>
#include <utility>
#include <AP_Math/AP_Math.h>
#include <stdio.h>

#define ADDR_CTRL_REG1      0x20
#define ADDR_CTRL_REG2      0x21
#define ADDR_CTRL_REG3      0x22
#define ADDR_CTRL_REG4      0x23
#define ADDR_CTRL_REG5      0x24

#define ADDR_STATUS_REG     0x27
#define ADDR_OUT_X_L        0x28
#define ADDR_OUT_X_H        0x29
#define ADDR_OUT_Y_L        0x2a
#define ADDR_OUT_Y_H        0x2b
#define ADDR_OUT_Z_L        0x2c
#define ADDR_OUT_Z_H        0x2d
#define ADDR_OUT_T_L        0x2e
#define ADDR_OUT_T_H        0x2f

#define MODE_REG_CONTINOUS_MODE     (0 << 0)
#define MODE_REG_SINGLE_MODE        (1 << 0)

#define ADDR_WHO_AM_I       0x0f
#define ID_WHO_AM_I         0x3d

extern const AP_HAL::HAL &hal;

AP_Compass_Backend *AP_Compass_LIS3MDL::probe(Compass &compass,
                                              AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_LIS3MDL *sensor = new AP_Compass_LIS3MDL(compass, std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_LIS3MDL::AP_Compass_LIS3MDL(Compass &compass,
                                       AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                       bool _force_external,
                                       enum Rotation _rotation)
    : AP_Compass_Backend(compass)
    , dev(std::move(_dev))
    , force_external(_force_external)
    , rotation(_rotation)
{
}

bool AP_Compass_LIS3MDL::init()
{
    if (!dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    if (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        dev->set_read_flag(0xC0);
    }

    // high retries for init
    dev->set_retries(10);
    
    uint8_t whoami;
    if (!dev->read_registers(ADDR_WHO_AM_I, &whoami, 1) ||
        whoami != ID_WHO_AM_I) {
        // not a 3MDL
        goto fail;
    }

    dev->setup_checked_registers(5);

    dev->write_register(ADDR_CTRL_REG1, 0x62, true); // 155Hz, UHP
    dev->write_register(ADDR_CTRL_REG2, 0, true); // 4Ga range
    dev->write_register(ADDR_CTRL_REG3, 0, true); // continuous
    dev->write_register(ADDR_CTRL_REG4, 0x0C, true); // z-axis ultra high perf
    dev->write_register(ADDR_CTRL_REG5, 0x40, true); // block-data-update

    // lower retries for run
    dev->set_retries(3);
    
    dev->get_semaphore()->give();

    /* register the compass instance in the frontend */
    compass_instance = register_compass();

    printf("Found a LIS3MDL on 0x%x as compass %u\n", dev->get_bus_id(), compass_instance);
    
    set_rotation(compass_instance, rotation);

    if (force_external) {
        set_external(compass_instance, true);
    }
    
    dev->set_device_type(DEVTYPE_LIS3MDL);
    set_dev_id(compass_instance, dev->get_bus_id());

    // call timer() at 155Hz
    dev->register_periodic_callback(1000000U/155U,
                                    FUNCTOR_BIND_MEMBER(&AP_Compass_LIS3MDL::timer, void));

    return true;

fail:
    dev->get_semaphore()->give();
    return false;
}

void AP_Compass_LIS3MDL::timer()
{
    struct PACKED {
        int16_t magx;
        int16_t magy;
        int16_t magz;
    } data;
    const float range_scale = 1000.0f / 6842.0f;
    Vector3f field;

    // check data ready
    uint8_t status;
    if (!dev->read_registers(ADDR_STATUS_REG, (uint8_t *)&status, 1)) {
        goto check_registers;
    }
    if (!(status & 0x08)) {
        // data not available yet
        goto check_registers;
    }

    if (!dev->read_registers(ADDR_OUT_X_L, (uint8_t *)&data, sizeof(data))) {
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

void AP_Compass_LIS3MDL::read()
{
    if (!_sem->take_nonblocking()) {
        return;
    }
    if (accum_count == 0) {
        _sem->give();
        return;
    }

#if 0
    // debugging code for sample rate
    static uint32_t lastt;
    static uint32_t total;
    total += accum_count;
    uint32_t now = AP_HAL::micros();
    float dt = (now - lastt) * 1.0e-6;
    if (dt > 1) {
        printf("%u samples\n", total);
        lastt = now;
        total = 0;
    }
#endif
    
    accum /= accum_count;

    publish_filtered_field(accum, compass_instance);

    accum.zero();
    accum_count = 0;
    
    _sem->give();
}
