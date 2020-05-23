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
  Driver by Thomas Schumacher, Jan 2019
  Structure based on LIS3MDL driver

 */
#include "AP_Compass_RM3100.h"

#include <AP_HAL/AP_HAL.h>
#include <utility>
#include <AP_Math/AP_Math.h>
#include <stdio.h>

#define RM3100_POLL_REG        0x00

#define RM3100_CMM_REG         0x01

#define RM3100_CCX1_REG        0x04
#define RM3100_CCX0_REG        0x05
#define RM3100_CCY1_REG        0x06
#define RM3100_CCY0_REG        0x07
#define RM3100_CCZ1_REG        0x08
#define RM3100_CCZ0_REG        0x09

#define RM3100_TMRC_REG        0x0B

#define RM3100_MX2_REG      0x24
#define RM3100_MX1_REG      0x25
#define RM3100_MX0_REG      0x26
#define RM3100_MY2_REG      0x27
#define RM3100_MY1_REG      0x28
#define RM3100_MY0_REG      0x29
#define RM3100_MZ2_REG      0x2A
#define RM3100_MZ1_REG      0x2B
#define RM3100_MZ0_REG      0x2C

#define RM3100_BIST_REG       0x33
#define RM3100_STATUS_REG     0x34
#define RM3100_HSHAKE_REG     0x34
#define RM3100_REVID_REG      0x36

#define CCP0    0xC8      // Cycle Count values
#define CCP1    0x00
#define CCP0_DEFAULT 0xC8 // Default Cycle Count values (used as a whoami check)
#define CCP1_DEFAULT 0x00
#define GAIN_CC50 20.0f   // LSB/uT
#define GAIN_CC100 38.0f
#define GAIN_CC200 75.0f
#define UTESLA_TO_MGAUSS   10.0f // uT to mGauss conversion

#define TMRC    0x94    // Update rate 150Hz
#define CMM     0x71    // read 3 axes and set data ready if 3 axes are ready

extern const AP_HAL::HAL &hal;

AP_Compass_Backend *AP_Compass_RM3100::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_RM3100 *sensor = new AP_Compass_RM3100(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_RM3100::AP_Compass_RM3100(AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                       bool _force_external,
                                       enum Rotation _rotation)
    : dev(std::move(_dev))
    , force_external(_force_external)
    , rotation(_rotation)
{
}

bool AP_Compass_RM3100::init()
{
    dev->get_semaphore()->take_blocking();

    if (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        // read has high bit set for SPI
        dev->set_read_flag(0x80);
    }

    // high retries for init
    dev->set_retries(10);

    // use default cycle count values as a whoami test
    uint8_t ccx0;
    uint8_t ccx1;
    uint8_t ccy0;
    uint8_t ccy1;
    uint8_t ccz0;
    uint8_t ccz1;
    if (!dev->read_registers(RM3100_CCX1_REG, &ccx1, 1) ||
        !dev->read_registers(RM3100_CCX0_REG, &ccx0, 1) ||
        !dev->read_registers(RM3100_CCY1_REG, &ccy1, 1) ||
        !dev->read_registers(RM3100_CCY0_REG, &ccy0, 1) ||
        !dev->read_registers(RM3100_CCZ1_REG, &ccz1, 1) ||
        !dev->read_registers(RM3100_CCZ0_REG, &ccz0, 1) ||
        ccx1 != CCP1_DEFAULT || ccx0 != CCP0_DEFAULT ||
        ccy1 != CCP1_DEFAULT || ccy0 != CCP0_DEFAULT ||
        ccz1 != CCP1_DEFAULT || ccz0 != CCP0_DEFAULT) {
        // couldn't read one of the cycle count registers or didn't recognize the default cycle count values
        dev->get_semaphore()->give();
        return false;
    }

    dev->setup_checked_registers(8);

    dev->write_register(RM3100_TMRC_REG, TMRC, true); // CMM data rate
    dev->write_register(RM3100_CMM_REG, CMM, true); // CMM configuration
    dev->write_register(RM3100_CCX1_REG, CCP1, true); // cycle count x
    dev->write_register(RM3100_CCX0_REG, CCP0, true); // cycle count x
    dev->write_register(RM3100_CCY1_REG, CCP1, true); // cycle count y
    dev->write_register(RM3100_CCY0_REG, CCP0, true); // cycle count y
    dev->write_register(RM3100_CCZ1_REG, CCP1, true); // cycle count z
    dev->write_register(RM3100_CCZ0_REG, CCP0, true); // cycle count z

    _scaler = (1 / GAIN_CC200) * UTESLA_TO_MGAUSS; // has to be changed if using a different cycle count

    // lower retries for run
    dev->set_retries(3);

    dev->get_semaphore()->give();

    /* register the compass instance in the frontend */
    dev->set_device_type(DEVTYPE_RM3100);
    if (!register_compass(dev->get_bus_id(), compass_instance)) {
        return false;
    }
    set_dev_id(compass_instance, dev->get_bus_id());

    hal.console->printf("RM3100: Found at address 0x%x as compass %u\n", dev->get_bus_address(), compass_instance);
    
    set_rotation(compass_instance, rotation);

    if (force_external) {
        set_external(compass_instance, true);
    }
    
    // call timer() at 80Hz
    dev->register_periodic_callback(1000000U/80U,
                                    FUNCTOR_BIND_MEMBER(&AP_Compass_RM3100::timer, void));

    return true;
}

void AP_Compass_RM3100::timer()
{
    struct PACKED {
        uint8_t magx_2;
        uint8_t magx_1;
        uint8_t magx_0;
        uint8_t magy_2;
        uint8_t magy_1;
        uint8_t magy_0;
        uint8_t magz_2;
        uint8_t magz_1;
        uint8_t magz_0;
    } data;
    Vector3f field;

    int32_t magx = 0;
    int32_t magy = 0;
    int32_t magz = 0;

    // check data ready on 3 axis
    uint8_t status;
    if (!dev->read_registers(RM3100_STATUS_REG, (uint8_t *)&status, 1)) {
        goto check_registers;
    }

    if (!(status & 0x80)) {
        // data not available yet
        goto check_registers;
    }

    if (!dev->read_registers(RM3100_MX2_REG, (uint8_t *)&data, sizeof(data))) {
        goto check_registers;
    }

    // the 24 bits of data for each axis are in 2s complement representation
    // each byte is shifted to its position in a 24-bit unsigned integer and from 8 more bits to be left-aligned in a 32-bit integer
    magx = ((uint32_t)data.magx_2 << 24) | ((uint32_t)data.magx_1 << 16) | ((uint32_t)data.magx_0 << 8);
    magy = ((uint32_t)data.magy_2 << 24) | ((uint32_t)data.magy_1 << 16) | ((uint32_t)data.magy_0 << 8);
    magz = ((uint32_t)data.magz_2 << 24) | ((uint32_t)data.magz_1 << 16) | ((uint32_t)data.magz_0 << 8);

    // right-shift signed integer back to get correct measurement value
    magx >>= 8;
    magy >>= 8;
    magz >>= 8;

    // apply scaler and store in field vector
    field(magx * _scaler, magy * _scaler, magz * _scaler);

    accumulate_sample(field, compass_instance);

check_registers:
    dev->check_next_register();
}

void AP_Compass_RM3100::read()
{
	drain_accumulated_samples(compass_instance);
}
