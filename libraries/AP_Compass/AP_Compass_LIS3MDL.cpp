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

#if AP_COMPASS_LIS3MDL_ENABLED

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

AP_Compass_Backend *AP_Compass_LIS3MDL::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_LIS3MDL *sensor = NEW_NOTHROW AP_Compass_LIS3MDL(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_LIS3MDL::AP_Compass_LIS3MDL(AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                       bool _force_external,
                                       enum Rotation _rotation)
    : dev(std::move(_dev))
    , force_external(_force_external)
    , rotation(_rotation)
{
}

bool AP_Compass_LIS3MDL::init()
{
    dev->get_semaphore()->take_blocking();

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

    dev->write_register(ADDR_CTRL_REG1, 0xFC, true); // 80Hz, UHP
    dev->write_register(ADDR_CTRL_REG2, 0, true); // 4Ga range
    dev->write_register(ADDR_CTRL_REG3, 0, true); // continuous
    dev->write_register(ADDR_CTRL_REG4, 0x0C, true); // z-axis ultra high perf
    dev->write_register(ADDR_CTRL_REG5, 0x40, true); // block-data-update

    // lower retries for run
    dev->set_retries(3);
    
    dev->get_semaphore()->give();

    /* register the compass instance in the frontend */
    dev->set_device_type(DEVTYPE_LIS3MDL);
    if (!register_compass(dev->get_bus_id())) {
        return false;
    }

    printf("Found a LIS3MDL on 0x%x as compass %u\n", unsigned(dev->get_bus_id()), instance);

    set_rotation(rotation);

    if (force_external) {
        set_external(true);
    }
    
    // call timer() at 80Hz
    dev->register_periodic_callback(1000000U/80U,
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

    {
        Vector3f field{
            data.magx * range_scale,
            data.magy * range_scale,
            data.magz * range_scale,
        };

        accumulate_sample(field);
    }

check_registers:
    dev->check_next_register();
}

void AP_Compass_LIS3MDL::read()
{
    drain_accumulated_samples();
}

#endif  // AP_COMPASS_LIS3MDL_ENABLED
