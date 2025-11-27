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
  Driver for the LIS2MDL magnetometer.
 */
#include "AP_Compass_config.h"

#if AP_COMPASS_LIS2MDL_ENABLED

#include "AP_Compass_LIS2MDL.h"

#include <AP_HAL/AP_HAL.h>
#include <utility>
#include <AP_Math/AP_Math.h>
#include <stdio.h>

// Register addresses
#define ADDR_WHO_AM_I       0x4F
#define ADDR_CFG_REG_A      0x60
#define ADDR_CFG_REG_B      0x61
#define ADDR_CFG_REG_C      0x62
#define ADDR_STATUS_REG     0x67
#define ADDR_OUT_X_L        0x68

// WHO_AM_I device ID
#define ID_WHO_AM_I         0x40

AP_Compass_Backend *AP_Compass_LIS2MDL::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_LIS2MDL *sensor = NEW_NOTHROW AP_Compass_LIS2MDL(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_LIS2MDL::AP_Compass_LIS2MDL(AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                       bool _force_external,
                                       enum Rotation _rotation)
    : dev(std::move(_dev))
    , force_external(_force_external)
    , rotation(_rotation)
{
}

// @brief Initialize the sensor
bool AP_Compass_LIS2MDL::init()
{
    WITH_SEMAPHORE(dev->get_semaphore());

    if (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        // LIS2MDL SPI reads are MSb=1, autoincrement.
        dev->set_read_flag(0xC0);
    }

    // high retries for init
    dev->set_retries(10);

    uint8_t whoami;
    if (!dev->read_registers(ADDR_WHO_AM_I, &whoami, 1) ||
        whoami != ID_WHO_AM_I) {
        // not a LIS2MDL
        return false;
    }

    dev->setup_checked_registers(3);

    // Configure for 100Hz continuous mode, with temperature compensation.
    dev->write_register(ADDR_CFG_REG_A, 0b10001100, true); // ODR=100Hz, continuous mode, temp comp on

    // Default settings for CFG_REG_B are fine.
    dev->write_register(ADDR_CFG_REG_B, 0x00, true);

    // Enable Block Data Update (BDU)
    dev->write_register(ADDR_CFG_REG_C, 0b00010000, true); 

    // lower retries for run
    dev->set_retries(3);

    /* register the compass instance in the frontend */
    dev->set_device_type(DEVTYPE_LIS2MDL);
    if (!register_compass(dev->get_bus_id())) {
        return false;
    }

    printf("Found a LIS2MDL on 0x%x as compass %u\n", unsigned(dev->get_bus_id()), instance);

    set_rotation(rotation);

    if (force_external) {
        set_external(force_external);
    }

    // call timer() at 100Hz
    dev->register_periodic_callback(1000000U/100U,
                                    FUNCTOR_BIND_MEMBER(&AP_Compass_LIS2MDL::timer, void));

    return true;
}

// @brief Read data from the sensor and accumulate it
void AP_Compass_LIS2MDL::timer()
{
    struct PACKED {
        int16_t magx;
        int16_t magy;
        int16_t magz;
    } data;

    // Sensitivity is 1.5 mgauss/LSB
    const float range_scale = 1.5f;

    // check data ready
    uint8_t status;
    if (!dev->read_registers(ADDR_STATUS_REG, &status, 1)) {
        goto check_registers;
    }
    if (!(status & 0x08)) { // ZYXDA bit
        // data not available yet
        goto check_registers;
    }

    if (!dev->read_registers(ADDR_OUT_X_L, (uint8_t *)&data, sizeof(data))) {
        goto check_registers;
    }

    {
        Vector3f field{
            (float)data.magx * range_scale,
            (float)data.magy * range_scale,
            (float)-data.magz * range_scale,    // for unknown reasons the Z axis appears to be reversed, without this it is impossible to orient correctly
        };

        accumulate_sample(field, instance);
    }

check_registers:
    dev->check_next_register();
}

// @brief Publish accumulated data to the frontend
void AP_Compass_LIS2MDL::read()
{
    drain_accumulated_samples();
}

#endif  // AP_COMPASS_LIS2MDL_ENABLED
