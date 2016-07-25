// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
#include "AP_RangeFinder_LightWareI2C.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initializes the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_LightWareI2C::AP_RangeFinder_LightWareI2C(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_RangeFinder_Backend(_ranger, instance, _state)
    , _dev(std::move(dev))
{
}

/*
   detect if a Lightware rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_LightWareI2C::detect(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    AP_RangeFinder_LightWareI2C *sensor
        = new AP_RangeFinder_LightWareI2C(_ranger, instance, _state, std::move(dev));

    uint16_t reading_cm;

    if (!sensor || !sensor->get_reading(reading_cm)) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

// read - return last value measured by sensor
bool AP_RangeFinder_LightWareI2C::get_reading(uint16_t &reading_cm)
{
    be16_t val;

    if (ranger._address[state.instance] == 0) {
        return false;
    }


    // exit immediately if we can't take the semaphore
    if (!_dev || !_dev->get_semaphore()->take(1)) {
        return false;
    }

    // read the high and low byte distance registers
    bool ret = _dev->read((uint8_t *) &val, sizeof(val));
    if (ret) {
        // combine results into distance
        reading_cm = be16toh(val);
    }

    _dev->get_semaphore()->give();

    return ret;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_LightWareI2C::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
