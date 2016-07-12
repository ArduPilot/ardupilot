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
    if (!sensor || !sensor->start_reading()) {
        delete sensor;
        return nullptr;
    }

    // give time for the sensor to process the request
    hal.scheduler->delay(50);

    uint16_t reading_cm;
    if (!sensor->get_reading(reading_cm)) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

// start_reading() - ask sensor to make a range reading
bool AP_RangeFinder_MaxsonarI2CXL::start_reading()
{
    if (!_dev || !_dev->get_semaphore()->take(1)) {
        return false;
    }

    uint8_t cmd = AP_RANGE_FINDER_MAXSONARI2CXL_COMMAND_TAKE_RANGE_READING;

    // send command to take reading
    bool ret = _dev->transfer(&cmd, sizeof(cmd), nullptr, 0);

    _dev->get_semaphore()->give();

    return ret;
}

// read - return last value measured by sensor
bool AP_RangeFinder_MaxsonarI2CXL::get_reading(uint16_t &reading_cm)
{
    be16_t val;

    // exit immediately if we can't take the semaphore
    if (!_dev->get_semaphore()->take(1)) {
        return false;
    }

    // take range reading and read back results
    bool ret = _dev->transfer(nullptr, 0, (uint8_t *) &val, sizeof(val));
    _dev->get_semaphore()->give();

    if (ret) {
        // combine results into distance
        reading_cm = be16toh(val);

        // trigger a new reading
        start_reading();
    }

    return ret;
}


/*
   update the state of the sensor
*/
void AP_RangeFinder_MaxsonarI2CXL::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
