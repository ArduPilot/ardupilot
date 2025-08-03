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

#include "AP_Compass_MMC5603.h"

#if AP_COMPASS_MMC5603_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <utility>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

#define REG_XOUT0          0x00
#define REG_STATUS          0x18
#define REG_CONTROL0        0x1B
#define REG_CONTROL1        0x1C
#define REG_CONTROL2        0x1D
#define REG_PRODUCT_ID      0x39

// bits in REG_CONTROL0
#define REG_CONTROL0_RESET  (1 << 4)
#define REG_CONTROL0_SET    (1 << 3)
#define REG_CONTROL0_TMM     (1 << 0)
#define REG_CONTROL1_SWRST  (1 << 7)

#define REG_STATUS_MM_DONE (1 << 6)

// datasheet says 50ms min for refill
#define MIN_DELAY_SET_RESET 50
#define MMC5603_PRODUCT_ID (1 << 4)

AP_Compass_Backend *AP_Compass_MMC5603::probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_MMC5603 *sensor = new AP_Compass_MMC5603(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_MMC5603::AP_Compass_MMC5603(AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                       bool _force_external,
                                       enum Rotation _rotation)
    : dev(std::move(_dev))
    , force_external(_force_external)
    , rotation(_rotation)
{
}

static Vector3f field_vector(uint8_t magdata[]){
    const uint32_t zero_offset = 524288; // 20 bit mode
    /* Vector3f ret = {float((magdata[0] << 8) + magdata[1]) - zero_offset, */
    /*     float((magdata[2] << 8) + magdata[3]) - zero_offset, */
    /*     float((magdata[4] << 8) + magdata[5]) - zero_offset}; */
    Vector3f ret = {
        float((((uint32_t) magdata[0]) << 12) + (((uint32_t) magdata[1]) << 4) + magdata[6]) - zero_offset,
        float((((uint32_t) magdata[2]) << 12) + (((uint32_t) magdata[3]) << 4) + magdata[7]) - zero_offset,
        float((((uint32_t) magdata[4]) << 12) + (((uint32_t) magdata[5]) << 4) + magdata[8]) - zero_offset
    };
    return ret;
}

bool AP_Compass_MMC5603::get_measurement(Vector3f &ret){
    uint8_t data[9];
    uint8_t status;
    if (!dev->read_registers(REG_STATUS, &status, 1)) {
        state = MMCState::STATE_SET;
        return false;
    }

    // check if measurement is ready
    if (!(status & REG_STATUS_MM_DONE)) {
        return false;
    }

    // read measurement
    if (!dev->read_registers(REG_XOUT0, data, sizeof(data))) {
        state = MMCState::STATE_SET;
        return false;
    }
    ret = field_vector(data);
    return true;
}

bool AP_Compass_MMC5603::init()
{
    dev->get_semaphore()->take_blocking();

    dev->set_retries(10);
    
    uint8_t whoami;
    if (!dev->read_registers(REG_PRODUCT_ID, &whoami, 1) ||
        whoami != MMC5603_PRODUCT_ID) {
        // not a MMC5603
        dev->get_semaphore()->give();
        return false;
    }

    // reset sensor
    dev->write_register(REG_CONTROL1, REG_CONTROL1_SWRST);
    hal.scheduler->delay(50);
    
    dev->write_register(REG_CONTROL0, 0x00);
    dev->write_register(REG_CONTROL1, 0x00);
    
    dev->get_semaphore()->give();

    /* register the compass instance in the frontend */
    dev->set_device_type(DEVTYPE_MMC5603);
    if (!register_compass(dev->get_bus_id(), compass_instance)) {
        return false;
    }
    
    set_dev_id(compass_instance, dev->get_bus_id());

    printf("Found a MMC5603 on 0x%x as compass %u\n", unsigned(dev->get_bus_id()), compass_instance);
    
    set_rotation(compass_instance, rotation);

    if (force_external) {
        set_external(compass_instance, true);
    }
    
    dev->set_retries(1);
    
    // call timer() at 100Hz
    dev->register_periodic_callback(10000,
                                    FUNCTOR_BIND_MEMBER(&AP_Compass_MMC5603::timer, void));

    // wait 250ms for the compass to make it's initial readings
    hal.scheduler->delay(250);
    
    return true;
}

void AP_Compass_MMC5603::timer()
{
    
    // recalculate the offset with set/reset operation every measure_count_limit measurements
    // sensor is read at about 100Hz, so about every 10 seconds
    /* const uint16_t measure_count_limit = 1000U; */
    const uint16_t measure_count_limit = 50U;
    const uint16_t sensitivity = 16384U; // counts per Gauss, 16 bit mode
    constexpr float counts_to_milliGauss = -1.0e3f / sensitivity;

    /*
      we use the SET/RESET method to remove bridge offset every
      measure_count_limit measurements. This involves a fairly complex
      state machine, but means we are much less sensitive to
      temperature changes
     */
    switch (state) {

    // perform a set operation
    case MMCState::STATE_SET: {
        if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_SET)) {
            break;
        }
        state = MMCState::STATE_SET_MEASURE;
        break;
    }

    // request a measurement for field and offset calculation after set operation
    case MMCState::STATE_SET_MEASURE: {
        if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_TMM)) {
            break;
        }
        state = MMCState::STATE_SET_WAIT;
        break;
    }

    // wait for measurement to be ready after set operation, then read the
    // measurement data and request a reset operation
    case MMCState::STATE_SET_WAIT: {
        if(!get_measurement(measurement0)){
            break;
        }
        // request set operation
        if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_RESET)) {
            break;
        }
        // minimum time to wait after set/reset before take measurement request is 1ms
        state = MMCState::STATE_RESET_MEASURE;
        break;
    }

    // request a measurement for field and offset calculation after reset operation
    case MMCState::STATE_RESET_MEASURE: {
        // take measurement request
        if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_TMM)) {
            state = MMCState::STATE_SET;
            break;
        }

        state = MMCState::STATE_RESET_WAIT;
        break;
    }

    // wait for measurement to be ready after reset operation,
    // then read the measurement data, calculate the field and offset,
    // and begin requesting field measurements
    case MMCState::STATE_RESET_WAIT: {
        Vector3f measurement1;
        if(!get_measurement(measurement1)){
            break;
        }
        Vector3f field = (measurement1 - measurement0) * counts_to_milliGauss * 0.5f;
        Vector3f new_offset = (measurement0 + measurement1) * counts_to_milliGauss * 0.5f;

        if (!have_initial_offset) {
            offset = new_offset;
            have_initial_offset = true;
        } else {
            offset = new_offset;
        }

        accumulate_sample(field, compass_instance);
        

        if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_TMM)) {
            printf("failed to initiate measurement\n");
            state = MMCState::STATE_SET;
        } else {
            state = MMCState::STATE_MEASURE;
        }

        break;
    }

    // take repeated field measurements, set/reset is performed again after
    // measure_count_limit measurements
    case MMCState::STATE_MEASURE: {
        Vector3f field;
        if(!get_measurement(field)){
            break;
        }
        field *= counts_to_milliGauss;
        field -= offset;
        accumulate_sample(field, compass_instance);

        // we stay in STATE_MEASURE for measure_count_limit cycles
        if (measure_count++ >= measure_count_limit) {
            measure_count = 0;
            state = MMCState::STATE_SET;
        } else {
            if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_TMM)) { // Take Measurement
                state = MMCState::STATE_SET;
            }
        }
        break;
    }
    }
}

void AP_Compass_MMC5603::read()
{
    drain_accumulated_samples(compass_instance);
}

#endif  // AP_COMPASS_MMC5603_ENABLED

