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
AP_RangeFinder_LightWareI2C::AP_RangeFinder_LightWareI2C(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_RangeFinder_Backend(_state, _params)
    , _dev(std::move(dev)) {}

/*
   detect if a Lightware rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_LightWareI2C::detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_LightWareI2C *sensor
        = new AP_RangeFinder_LightWareI2C(_state, _params, std::move(dev));

    if (!sensor) {
        delete sensor;
        return nullptr;
    }

    if (sensor->_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        uint16_t reading_cm;
        if (!sensor->get_reading(reading_cm)) {
            sensor->_dev->get_semaphore()->give();
            delete sensor;
            return nullptr;
        }
        sensor->_dev->get_semaphore()->give();
    }

    sensor->init();

    return sensor;
}

void AP_RangeFinder_LightWareI2C::init()
{
    // call timer() at 20Hz
    _dev->register_periodic_callback(50000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_LightWareI2C::timer, void));
}

// read - return last value measured by sensor
bool AP_RangeFinder_LightWareI2C::get_reading(uint16_t &reading_cm)
{
    be16_t val;

    if (params.address == 0) {
        return false;
    }

    // read the high and low byte distance registers
    bool ret = _dev->read((uint8_t *) &val, sizeof(val));
    if (ret) {
        // combine results into distance
        reading_cm = be16toh(val);
    }

    return ret;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_LightWareI2C::update(void)
{
    // nothing to do - its all done in the timer()
}

void AP_RangeFinder_LightWareI2C::timer(void)
{
    if (get_reading(state.distance_cm)) {
        state.last_reading_ms = AP_HAL::millis();
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
