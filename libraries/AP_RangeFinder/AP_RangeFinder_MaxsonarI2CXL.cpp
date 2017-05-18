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

/*
 *       AP_RangeFinder_MaxsonarI2CXL.cpp - Arduino Library for MaxBotix I2C XL sonar
 *       Code by Randy Mackay. DIYDrones.com
 *
 *       datasheet: http://www.maxbotix.com/documents/I2CXL-MaxSonar-EZ_Datasheet.pdf
 *
 *       Sensor should be connected to the I2C port
 */
#include "AP_RangeFinder_MaxsonarI2CXL.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initializes the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_MaxsonarI2CXL::AP_RangeFinder_MaxsonarI2CXL(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state)
    : AP_RangeFinder_Backend(_ranger, instance, _state)
    , _dev(hal.i2c_mgr->get_device(1, AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR))
{
}

/*
   detect if a Maxbotix rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_MaxsonarI2CXL::detect(RangeFinder &_ranger, uint8_t instance,
                                                             RangeFinder::RangeFinder_State &_state)
{
    AP_RangeFinder_MaxsonarI2CXL *sensor
        = new AP_RangeFinder_MaxsonarI2CXL(_ranger, instance, _state);
    if (!sensor) {
        return nullptr;
    }

    if (!sensor->_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

/*
  initialise sensor
 */
bool AP_RangeFinder_MaxsonarI2CXL::_init(void)
{
    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }
    

    if (!start_reading()) {
        _dev->get_semaphore()->give();
        return false;
    }

    // give time for the sensor to process the request
    hal.scheduler->delay(50);

    uint16_t reading_cm;
    if (!get_reading(reading_cm)) {
        _dev->get_semaphore()->give();
        return false;
    }

    _dev->get_semaphore()->give();
    
    _dev->register_periodic_callback(50000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_MaxsonarI2CXL::_timer, void));
    
    return true;
}

// start_reading() - ask sensor to make a range reading
bool AP_RangeFinder_MaxsonarI2CXL::start_reading()
{
    uint8_t cmd = AP_RANGE_FINDER_MAXSONARI2CXL_COMMAND_TAKE_RANGE_READING;

    // send command to take reading
    return _dev->transfer(&cmd, sizeof(cmd), nullptr, 0);
}

// read - return last value measured by sensor
bool AP_RangeFinder_MaxsonarI2CXL::get_reading(uint16_t &reading_cm)
{
    be16_t val;

    // take range reading and read back results
    bool ret = _dev->transfer(nullptr, 0, (uint8_t *) &val, sizeof(val));

    if (ret) {
        // combine results into distance
        reading_cm = be16toh(val);

        // trigger a new reading
        start_reading();
    }

    return ret;
}

/*
  timer called at 20Hz
*/
void AP_RangeFinder_MaxsonarI2CXL::_timer(void)
{
    uint16_t d;
    if (get_reading(d)) {
        if (_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            distance = d;
            new_distance = true;
            _sem->give();
        }
    }
}


/*
   update the state of the sensor
*/
void AP_RangeFinder_MaxsonarI2CXL::update(void)
{
    if (_sem->take_nonblocking()) {
        if (new_distance) {
            state.distance_cm = distance;
            new_distance = false;
            update_status();
        } else {
            set_status(RangeFinder::RangeFinder_NoData);
        }
         _sem->give();
    }
}
