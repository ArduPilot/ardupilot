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
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

#define REG_PRODUCT_ID      0x20
#define REG_XOUT_L          0x00
#define REG_STATUS          0x06
#define REG_CONTROL0        0x07
#define REG_CONTROL1        0x08

// bits in REG_CONTROL0
#define REG_CONTROL0_REFILL 0x80
#define REG_CONTROL0_RESET  0x40
#define REG_CONTROL0_SET    0x20
#define REG_CONTROL0_NB     0x10
#define REG_CONTROL0_TM     0x01

// datasheet says 50ms min for refill
#define MIN_DELAY_SET_RESET 50

AP_Compass_Backend *AP_Compass_MMC3416::probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_MMC3416 *sensor = new AP_Compass_MMC3416(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_MMC3416::AP_Compass_MMC3416(AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                       bool _force_external,
                                       enum Rotation _rotation)
    : dev(std::move(_dev))
    , force_external(_force_external)
    , rotation(_rotation)
{
}

bool AP_Compass_MMC3416::init()
{
    dev->get_semaphore()->take_blocking();

    dev->set_retries(10);
    
    uint8_t whoami;
    if (!dev->read_registers(REG_PRODUCT_ID, &whoami, 1) ||
        whoami != 0x06) {
        // not a MMC3416
        dev->get_semaphore()->give();
        return false;
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
                                    FUNCTOR_BIND_MEMBER(&AP_Compass_MMC3416::timer, void));

    // wait 250ms for the compass to make it's initial readings
    hal.scheduler->delay(250);
    
    return true;
}

void AP_Compass_MMC3416::timer()
{
    const uint16_t measure_count_limit = 50;
    const uint16_t zero_offset = 32768; // 16 bit mode
    const uint16_t sensitivity = 2048; // counts per Gauss, 16 bit mode
    const float counts_to_milliGauss = 1.0e3f / sensitivity;

    uint32_t now = AP_HAL::millis();
    if (now - last_sample_ms > 500) {
        // seems to be stuck or on first sample, reset state machine
        state = STATE_REFILL1;
        last_sample_ms = now;
    }
    
    /*
      we use the SET/RESET method to remove bridge offset every
      measure_count_limit measurements. This involves a fairly complex
      state machine, but means we are much less sensitive to
      temperature changes
     */
    switch (state) {
    case STATE_REFILL1:
        if (dev->write_register(REG_CONTROL0, REG_CONTROL0_REFILL)) {
            state = STATE_REFILL1_WAIT;
            refill_start_ms = AP_HAL::millis();
        }
        break;

    case STATE_REFILL1_WAIT: {
        uint8_t status;
        if (AP_HAL::millis() - refill_start_ms > MIN_DELAY_SET_RESET &&
            dev->read_registers(REG_STATUS, &status, 1) &&
            (status & 0x02) == 0) {
            if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_SET) ||
                !dev->write_register(REG_CONTROL0, REG_CONTROL0_TM)) { // Take Measurement
                state = STATE_REFILL1;
            } else {
                state = STATE_MEASURE_WAIT1;
            }
        }
        break;
    }
        
    case STATE_MEASURE_WAIT1: {
        uint8_t status;
        if (dev->read_registers(REG_STATUS, &status, 1) && (status & 1)) {
            if (!dev->read_registers(REG_XOUT_L, (uint8_t *)&data0[0], 6)) {
                state = STATE_REFILL1;
                break;
            }
            if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_REFILL)) {
                state = STATE_REFILL1;
            } else {
                state = STATE_REFILL2_WAIT;
                refill_start_ms = AP_HAL::millis();
            }
        }
        break;
    }

    case STATE_REFILL2_WAIT: {
        uint8_t status;
        if (AP_HAL::millis() - refill_start_ms > MIN_DELAY_SET_RESET &&
            dev->read_registers(REG_STATUS, &status, 1) &&
            (status & 0x02) == 0) {
            if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_RESET) ||
                !dev->write_register(REG_CONTROL0, REG_CONTROL0_TM)) { // Take Measurement
                state = STATE_REFILL1;
            } else {
                state = STATE_MEASURE_WAIT2;
            }
        }
        break;
    }

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
        Vector3f field;

        /*
          calculate field and offset
         */
        Vector3f f1(float(data0[0]) - zero_offset,
                    float(data0[1]) - zero_offset,
                    float(data0[2]) - zero_offset);
        Vector3f f2(float(data1[0]) - zero_offset,
                    float(data1[1]) - zero_offset,
                    float(data1[2]) - zero_offset);
        field = (f1 - f2) * (counts_to_milliGauss / 2);
        Vector3f new_offset = (f1 + f2) * (counts_to_milliGauss / 2);
        if (!have_initial_offset) {
            offset = new_offset;
            have_initial_offset = true;
        } else {
            // low pass changes to the offset
            offset = offset * 0.95f + new_offset * 0.05f;
        }

#if 0
        AP::logger().Write("MMO", "TimeUS,Nx,Ny,Nz,Ox,Oy,Oz", "Qffffff",
                                               AP_HAL::micros64(),
                                               (double)new_offset.x,
                                               (double)new_offset.y,
                                               (double)new_offset.z,
                                               (double)offset.x,
                                               (double)offset.y,
                                               (double)offset.z);
        printf("F(%.1f %.1f %.1f) O(%.1f %.1f %.1f)\n",
               field.x, field.y, field.z,
               offset.x, offset.y, offset.z);
#endif

        last_sample_ms = AP_HAL::millis();
        accumulate_sample(field, compass_instance);

        if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_TM)) {
            state = STATE_REFILL1;
        } else {
            state = STATE_MEASURE_WAIT3;
        }
        break;
    }

    case STATE_MEASURE_WAIT3: {
        uint8_t status;
        if (!dev->read_registers(REG_STATUS, &status, 1) || !(status & 1)) {
            break;
        }
        uint16_t data1[3];
        if (!dev->read_registers(REG_XOUT_L, (uint8_t *)&data1[0], 6)) {
            state = STATE_REFILL1;
            break;
        }
        Vector3f field(float(data1[0]) - zero_offset,
                       float(data1[1]) - zero_offset,
                       float(data1[2]) - zero_offset);
        field *= -counts_to_milliGauss;
        field += offset;

        last_sample_ms = AP_HAL::millis();
        accumulate_sample(field, compass_instance);

        // we stay in STATE_MEASURE_WAIT3 for measure_count_limit cycles
        if (measure_count++ >= measure_count_limit) {
            measure_count = 0;
            state = STATE_REFILL1;
        } else {
            if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_TM)) { // Take Measurement
                state = STATE_REFILL1;
            }
        }
        break;
    }
    }
}

void AP_Compass_MMC3416::read()
{
    drain_accumulated_samples(compass_instance);
}
