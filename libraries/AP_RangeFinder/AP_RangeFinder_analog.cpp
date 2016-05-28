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
 *   AP_RangeFinder_analog.cpp - rangefinder for analog source
 *
 */

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include "RangeFinder.h"
#include "AP_RangeFinder_analog.h"

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_analog::AP_RangeFinder_analog(RangeFinder &_ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    source = hal.analogin->channel(ranger._pin[instance]);
    if (source == NULL) {
        // failed to allocate a ADC channel? This shouldn't happen
        state.healthy = false;
        return;
    }
    source->set_stop_pin((uint8_t)ranger._stop_pin[instance]);
    source->set_settle_time((uint16_t)ranger._settle_time_ms[instance]);
}

/* 
   detect if an analog rangefinder is connected. The only thing we
   can do is check if the pin number is valid. If it is, then assume
   that the device is connected
*/
bool AP_RangeFinder_analog::detect(RangeFinder &_ranger, uint8_t instance)
{
    if (_ranger._pin[instance] != -1) {
        return true;
    }
    return false;
}


/*
  update raw voltage state
 */
void AP_RangeFinder_analog::update_voltage(void)
{
   if (source == NULL) {
       state.voltage_mv = 0;
       return;
   }
   // cope with changed settings
   source->set_pin(ranger._pin[state.instance]);
   source->set_stop_pin((uint8_t)ranger._stop_pin[state.instance]);
   source->set_settle_time((uint16_t)ranger._settle_time_ms[state.instance]);
   if (ranger._ratiometric[state.instance]) {
       state.voltage_mv = source->voltage_average_ratiometric() * 1000U;
   } else {
       state.voltage_mv = source->voltage_average() * 1000U;
   }
}

/*
  update distance_cm 
 */
void AP_RangeFinder_analog::update(void)
{
    update_voltage();
    float v = state.voltage_mv * 0.001f;
    float dist_m = 0;
    float scaling = ranger._scaling[state.instance];
    float offset  = ranger._offset[state.instance];
    RangeFinder::RangeFinder_Function function = (RangeFinder::RangeFinder_Function)ranger._function[state.instance].get();
    int16_t max_distance_cm = ranger._max_distance_cm[state.instance];

    switch (function) {
    case RangeFinder::FUNCTION_LINEAR:
        dist_m = (v - offset) * scaling;
        break;
	  
    case RangeFinder::FUNCTION_INVERTED:
        dist_m = (offset - v) * scaling;
        break;

    case RangeFinder::FUNCTION_HYPERBOLA:
        if (v <= offset) {
            dist_m = 0;
        }
        dist_m = scaling / (v - offset);
        if (isinf(dist_m) || dist_m > max_distance_cm) {
            dist_m = max_distance_cm * 0.01;
        }
        break;
    }
    if (dist_m < 0) {
        dist_m = 0;
    }
    state.distance_cm = dist_m * 100.0f;  

    // we can't actually tell if an analog rangefinder is healthy, so
    // always set as healthy
    state.healthy = true;
}

