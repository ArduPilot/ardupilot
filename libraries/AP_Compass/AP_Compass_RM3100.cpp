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

#define CCP0    0xC8      // Default Cycle Count
#define CCP1    0x00
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
    if (!dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // high retries for init
    dev->set_retries(10);

    // maybe add a test for revision id value here and goto fail if not correct
    // it is not clear if revision id is the same for all compasses, in the same manner as a whoami

    dev->setup_checked_registers(8);

    dev->write_register(RM3100_TMRC_REG, TMRC, true); // cycle count z
    dev->write_register(RM3100_CMM_REG, CMM, false); // CMM configuration
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
    compass_instance = register_compass();

    printf("Found a RM3100 at address 0x%x as compass %u\n", dev->get_bus_address(), compass_instance);
    
    set_rotation(compass_instance, rotation);

    if (force_external) {
        set_external(compass_instance, true);
    }
    
    dev->set_device_type(DEVTYPE_RM3100);
    set_dev_id(compass_instance, dev->get_bus_id());

    // call timer() at 80Hz
    dev->register_periodic_callback(1000000U/80U,
                                    FUNCTOR_BIND_MEMBER(&AP_Compass_RM3100::timer, void));

    return true;

//fail:
//    dev->get_semaphore()->give();
//    return false;
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

    uint32_t magx = 0;
    uint32_t magy = 0;
    uint32_t magz = 0;

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



    magx = ((uint32_t)data.magx_2 << 16) | ((uint32_t)data.magx_1 << 8) | (uint32_t)data.magx_0;
    magy = ((uint32_t)data.magy_2 << 16) | ((uint32_t)data.magy_1 << 8) | (uint32_t)data.magy_0;
    magz = ((uint32_t)data.magz_2 << 16) | ((uint32_t)data.magz_1 << 8) | (uint32_t)data.magz_0;

    if (magx & 0x800000) {
    	magx |= 0xFF000000;
    }
    if (magy & 0x800000) {
        magy |= 0xFF000000;
    }
    if (magz & 0x800000) {
        magz |= 0xFF000000;
	}

    field((int32_t)magx * _scaler, (int32_t)magy * _scaler, (int32_t)magz * _scaler);

    // rotate raw_field from sensor frame to body frame
    rotate_field(field, compass_instance);

    // publish raw_field (uncorrected point sample) for calibration use
    publish_raw_field(field, compass_instance);

    // correct raw_field for known errors
    correct_field(field, compass_instance);

    if (_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        accum += field;
        accum_count++;
        _sem->give();
    }

check_registers:
    dev->check_next_register();
}

void AP_Compass_RM3100::read()
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
    	printf("%u %f samples RM3100 \n", total, dt);
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
