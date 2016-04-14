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
#include <AP_HAL/AP_HAL.h>
#include <utility>

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_PulsedLightLRF::AP_RangeFinder_PulsedLightLRF(RangeFinder &_ranger, uint8_t instance,
                                                             RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state),
    _dev(hal.i2c_mgr->get_device(0, AP_RANGEFINDER_PULSEDLIGHTLRF_ADDR))
{
}

/* 
   detect if a PulsedLight rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_PulsedLightLRF::detect(RangeFinder &_ranger, uint8_t instance,
                                                              RangeFinder::RangeFinder_State &_state)
{
    AP_RangeFinder_PulsedLightLRF *sensor
        = new AP_RangeFinder_PulsedLightLRF(_ranger, instance, _state);

    if (!sensor->start_reading()) {
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
bool AP_RangeFinder_PulsedLightLRF::start_reading()
{
    if (!_dev->get_semaphore()->take(1)) {
        return false;
    }

    // send command to take reading
    if (!_dev->write_register(AP_RANGEFINDER_PULSEDLIGHTLRF_MEASURE_REG,
                              AP_RANGEFINDER_PULSEDLIGHTLRF_MSRREG_ACQUIRE)) {
        _dev->get_semaphore()->give();
        return false;
    }

    // return semaphore
    _dev->get_semaphore()->give();

    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_PulsedLightLRF::get_reading(uint16_t &reading_cm)
{
    if (_dev->get_semaphore()->take(1)) {
        return false;
    }

    uint8_t buff[2];
    // read the high and low byte distance registers
    if (!_dev->read_registers(AP_RANGEFINDER_PULSEDLIGHTLRF_DISTHIGH_REG, buff, sizeof(buff))) {
        _dev->get_semaphore()->give();
        return false;
    }

    // combine results into distance
    reading_cm = ((uint16_t)buff[0]) << 8 | buff[1];

    // return semaphore
    _dev->get_semaphore()->give();

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
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        update_status();
    } else {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
