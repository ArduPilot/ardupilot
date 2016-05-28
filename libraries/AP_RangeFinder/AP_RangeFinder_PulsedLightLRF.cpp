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

#include "AP_RangeFinder_PulsedLightLRF.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_PulsedLightLRF::AP_RangeFinder_PulsedLightLRF(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
}

/* 
   detect if a PulsedLight rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_PulsedLightLRF::detect(RangeFinder &_ranger, uint8_t instance)
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
bool AP_RangeFinder_PulsedLightLRF::start_reading()
{
    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(1)) {
        return false;
    }

    // send command to take reading
    if (hal.i2c->writeRegister(AP_RANGEFINDER_PULSEDLIGHTLRF_ADDR, 
                               AP_RANGEFINDER_PULSEDLIGHTLRF_MEASURE_REG, 
                               AP_RANGEFINDER_PULSEDLIGHTLRF_MSRREG_ACQUIRE) != 0) {
        i2c_sem->give();
        return false;
    }

    // return semaphore
    i2c_sem->give();

    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_PulsedLightLRF::get_reading(uint16_t &reading_cm)
{
    uint8_t buff[2];

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(1)) {
        return false;
    }

    // read the high and low byte distance registers
    if (hal.i2c->readRegisters(AP_RANGEFINDER_PULSEDLIGHTLRF_ADDR, 
                               AP_RANGEFINDER_PULSEDLIGHTLRF_DISTHIGH_REG, 2, &buff[0]) != 0) {
        i2c_sem->give();
        return false;
    }

    // combine results into distance
    reading_cm = ((uint16_t)buff[0]) << 8 | buff[1];

    // return semaphore
    i2c_sem->give();

    // kick off another reading for next time
    // To-Do: replace this with continuous mode
    hal.scheduler->delay_microseconds(200);
    start_reading();

    return true;
}

/* 
   update the state of the sensor
*/
void AP_RangeFinder_PulsedLightLRF::update(void)
{
    state.healthy = get_reading(state.distance_cm);
}
