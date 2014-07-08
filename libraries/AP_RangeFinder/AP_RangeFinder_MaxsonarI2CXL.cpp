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
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_MaxsonarI2CXL::AP_RangeFinder_MaxsonarI2CXL(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
}

/* 
   detect if a Maxbotix rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_MaxsonarI2CXL::detect(RangeFinder &_ranger, uint8_t instance)
{
    if (!start_reading()) {
        return false;
    }
    // give time for the sensor to process the request
    hal.scheduler->delay(50);
    uint16_t reading_cm;
    return get_reading(reading_cm);
}

// start_reading() - ask sensor to make a range reading
bool AP_RangeFinder_MaxsonarI2CXL::start_reading()
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(1)) {
        return false;
    }

    uint8_t tosend[1] = 
        { AP_RANGE_FINDER_MAXSONARI2CXL_COMMAND_TAKE_RANGE_READING };

    // send command to take reading
    if (hal.i2c->write(AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR,
                       1, tosend) != 0) {
        i2c_sem->give();
        return false;
    }

    // return semaphore
    i2c_sem->give();

    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_MaxsonarI2CXL::get_reading(uint16_t &reading_cm)
{
    uint8_t buff[2];

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(1)) {
        return false;
    }

    // take range reading and read back results
    if (hal.i2c->read(AP_RANGE_FINDER_MAXSONARI2CXL_DEFAULT_ADDR, 2, buff) != 0) {
        i2c_sem->give();
        return false;
    }
    i2c_sem->give();

    // combine results into distance
    reading_cm = ((uint16_t)buff[0]) << 8 | buff[1];

    // trigger a new reading
    start_reading();

    return true;
}


/* 
   update the state of the sensor
*/
void AP_RangeFinder_MaxsonarI2CXL::update(void)
{
    state.healthy = get_reading(state.distance_cm);
}
