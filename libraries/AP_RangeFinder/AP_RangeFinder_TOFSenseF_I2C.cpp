/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_TOFSenseF_I2C.h"

#if AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED

#define TOFSENSEP_I2C_COMMAND_TAKE_RANGE_READING 0x24
#define TOFSENSEP_I2C_COMMAND_SIGNAL_STATUS 0x28

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

// initialise sensor
bool AP_RangeFinder_TOFSenseF_I2C::init(void)
{
    WITH_SEMAPHORE(dev.get_semaphore());

    if (!start_reading()) {
        return false;
    }

    // give time for the sensor to process the request
    hal.scheduler->delay(100);

    uint32_t reading_mm;
    uint16_t status;
    uint16_t signal_strength;

    if (!get_reading(reading_mm, signal_strength, status)) {
        return false;
    }

    dev.register_periodic_callback(100000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_TOFSenseF_I2C::timer, void));

    return true;
}

// start_reading() - ask sensor to make a range reading
bool AP_RangeFinder_TOFSenseF_I2C::start_reading()
{
    const uint8_t cmd[] = {TOFSENSEP_I2C_COMMAND_TAKE_RANGE_READING, TOFSENSEP_I2C_COMMAND_SIGNAL_STATUS};

    // send command to take reading
    return dev.transfer(cmd, sizeof(cmd), nullptr, 0);
}

// read - return last value measured by sensor
bool AP_RangeFinder_TOFSenseF_I2C::get_reading(uint32_t &reading_mm, uint16_t &signal_strength, uint16_t &status)
{

    struct PACKED {
        uint32_t distance_mm;
        uint32_t signal_strength_and_status;
    } packet;

    // take range reading and read back results
    const bool ret = dev.transfer(nullptr, 0, (uint8_t *) &packet, sizeof(packet));

    if (ret) {
        // combine results into distance
        reading_mm = packet.distance_mm;
        signal_strength = (uint16_t)(packet.signal_strength_and_status >> 16);
        status = (uint16_t)(packet.signal_strength_and_status);
    }

    // trigger a new reading
    start_reading();

    return ret;
}

//  timer called at 10Hz
void AP_RangeFinder_TOFSenseF_I2C::timer(void)
{
    uint32_t dist_mm;
    uint16_t status;
    uint16_t signal_strength;

    if (get_reading(dist_mm, signal_strength, status)) {
        WITH_SEMAPHORE(_sem);
        if (status == 1) {
            // healthy data
            distance_mm = dist_mm;
            new_distance = true;
            state.last_reading_ms = AP_HAL::millis();
        }
    }
}

// update the state of the sensor
void AP_RangeFinder_TOFSenseF_I2C::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (new_distance) {
        state.distance_m = distance_mm * 0.001f;
        new_distance = false;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 300) {
        // if no updates for 0.3 seconds set no-data
        set_status(RangeFinder::Status::NoData);
    }
}

#endif  // AP_RANGEFINDER_TOFSENSEF_I2C_ENABLED
