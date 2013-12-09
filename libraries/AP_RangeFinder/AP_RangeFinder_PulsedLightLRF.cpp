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
 *       AP_RangeFinder_PulsedLightLRF.cpp - Arduino Library for Pulsed Light's Laser Range Finder
 *       Code by Randy Mackay. DIYDrones.com
 *
 *       Sensor should be connected to the I2C port
 *
 *       Variables:
 *               bool healthy : indicates whether last communication with sensor was successful
 *
 *       Methods:
 *               take_reading(): ask the sonar to take a new distance measurement
 *               read() : read last distance measured (in cm)
 *
 */

#include "AP_RangeFinder_PulsedLightLRF.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Constructor //////////////////////////////////////////////////////////////

AP_RangeFinder_PulsedLightLRF::AP_RangeFinder_PulsedLightLRF(FilterInt16 *filter) :
    RangeFinder(NULL, filter),
    healthy(true),
    _addr(AP_RANGEFINDER_PULSEDLIGHTLRF_ADDR)
{
    min_distance = AP_RANGEFINDER_PULSEDLIGHTLRF_MIN_DISTANCE;
    max_distance = AP_RANGEFINDER_PULSEDLIGHTLRF_MAX_DISTANCE;
}

// Public Methods //////////////////////////////////////////////////////////////

// init - simply sets the i2c address
void AP_RangeFinder_PulsedLightLRF::init(uint8_t address)
{
    // set sensor i2c address
    _addr = address;
}

// take_reading - ask sensor to make a range reading
bool AP_RangeFinder_PulsedLightLRF::take_reading()
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(5)) {
        healthy = false;
        return healthy;
    }

    // send command to take reading
    if (hal.i2c->writeRegister(_addr, AP_RANGEFINDER_PULSEDLIGHTLRF_COMMAND_REG, AP_RANGEFINDER_PULSEDLIGHTLRF_CMDREG_ACQUISITION) != 0) {
        healthy = false;
    }else{
        healthy = true;
    }

    // return semaphore
    i2c_sem->give();

    return healthy;
}

// read - return last value measured by sensor
int AP_RangeFinder_PulsedLightLRF::read()
{
    uint8_t buff[2];
    int16_t ret_value = 0;

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(5)) {
        healthy = false;
        return healthy;
    }

    // assume the worst
    healthy = false;

    // read the high byte
    if (hal.i2c->readRegisters(_addr, AP_RANGEFINDER_PULSEDLIGHTLRF_DISTHIGH_REG, 1, &buff[0]) == 0) {
        // read the low byte
        if (hal.i2c->readRegisters(_addr, AP_RANGEFINDER_PULSEDLIGHTLRF_DISTLOW_REG, 1, &buff[1]) == 0) {
            healthy = true;
            // combine results into distance
            ret_value = buff[0] << 8 | buff[1];
        }
    }

    // ensure distance is within min and max
    ret_value = constrain_int16(ret_value, min_distance, max_distance);
    ret_value = _mode_filter->apply(ret_value);

    // return semaphore
    i2c_sem->give();

    // kick off another reading for next time
    // To-Do: replace this with continuous mode
    take_reading();

    // to-do: do we really want to return 0 if reading the distance fails?
    return ret_value;
}
