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

    // reset sensor
    dev->write_register(REG_CONTROL1, 0x80);
    hal.scheduler->delay(10);
    
    dev->write_register(REG_CONTROL0, 0x00); // single shot
    dev->write_register(REG_CONTROL1, 0x00); // 16 bit, 7.92ms
    
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
    
    // call timer() at 100Hz
    dev->register_periodic_callback(10000,
                                    FUNCTOR_BIND_MEMBER(&AP_Compass_MMC3416::timer, bool));

    return true;

fail:
    dev->get_semaphore()->give();
    return false;
}

bool AP_Compass_MMC3416::timer()
{
    uint32_t now = AP_HAL::millis();
    
    switch (state) {
    case STATE_REFILL1:
        if (dev->write_register(REG_CONTROL0, 0x80)) { // REFILL
            state = STATE_REFILL1_WAIT;
        }
        break;

    case STATE_REFILL1_WAIT:
        if (now - last_state_ms >= 50) {
            if (!dev->write_register(REG_CONTROL0, 0x20) || // SET
                !dev->write_register(REG_CONTROL0, 0x01)) { // Take Measurement
                state = STATE_REFILL1;
            } else {
                state = STATE_MEASURE_WAIT1;
            }
        }
        break;
        
    case STATE_MEASURE_WAIT1: {
        uint8_t status;
        if (dev->read_registers(REG_STATUS, &status, 1) && (status & 1)) {
            if (!dev->read_registers(REG_XOUT_L, (uint8_t *)&data0[0], 6)) {
                state = STATE_REFILL1;
                break;
            }
            if (!dev->write_register(REG_CONTROL0, 0x80)) { // REFILL
                state = STATE_REFILL1;
            } else {
                state = STATE_REFILL2_WAIT;
            }
        }
        break;
    }

    case STATE_REFILL2_WAIT:
        if (now - last_state_ms >= 50) {
            if (!dev->write_register(REG_CONTROL0, 0x40) || // RESET
                !dev->write_register(REG_CONTROL0, 0x01)) { // Take Measurement
                state = STATE_REFILL1;
            } else {
                state = STATE_MEASURE_WAIT2;
            }
        }
        break;

    case STATE_MEASURE_WAIT2: {
        uint8_t status;
        if (!dev->read_registers(REG_STATUS, &status, 1) || !(status & 1)) {
            break;
        }
        uint16_t data1[3];
        if (!dev->read_registers(REG_XOUT_L, (uint8_t *)&data1[0], 6)) {
            state = STATE_REFILL1;
            break;
        }
        const uint16_t zero_offset = 32768; // 16 bit mode
        const uint16_t sensitivity = 2048; // counts per Gauss, 16 bit mode
        const float counts_to_milliGauss = 1.0e3f / sensitivity;
        Vector3f field;
    
        Vector3f f1(float(data0[0]) - zero_offset,
                    float(data0[1]) - zero_offset,
                    float(data0[2]) - zero_offset);
        Vector3f f2(float(data1[0]) - zero_offset,
                    float(data1[1]) - zero_offset,
                    float(data1[2]) - zero_offset);
        field = (f1 - f2) / 2;
        field * counts_to_milliGauss;
        
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
        state = STATE_REFILL1;
        break;
    }
    }
        
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
