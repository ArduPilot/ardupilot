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
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_LightWareI2C::AP_RangeFinder_LightWareI2C(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
}

/* 
   detect if a Lightware rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_LightWareI2C::detect(RangeFinder &_ranger, uint8_t instance)
{
    uint8_t buff[2];
    if (_ranger._address[instance] == 0) {
        return false;
    }
    return hal.i2c->read(_ranger._address[instance], 2, &buff[0]) == 0;
}

// read - return last value measured by sensor
bool AP_RangeFinder_LightWareI2C::get_reading(uint16_t &reading_cm)
{
    uint8_t buff[2];

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(1)) {
        return false;
    }

    // read the high and low byte distance registers
    if (hal.i2c->read(ranger._address[state.instance], 2, &buff[0]) != 0) {
        i2c_sem->give();
        return false;
    }

    // combine results into distance
    reading_cm = ((uint16_t)buff[0]) << 8 | buff[1];

    // return semaphore
    i2c_sem->give();

    return true;
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
