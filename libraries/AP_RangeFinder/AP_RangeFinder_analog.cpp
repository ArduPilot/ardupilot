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

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Params.h"
#include "AP_RangeFinder_analog.h"

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_analog::AP_RangeFinder_analog(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params)
{
    source = hal.analogin->channel(_params.pin);
    if (source == nullptr) {
        // failed to allocate a ADC channel? This shouldn't happen
        set_status(RangeFinder::Status::NotConnected);
        return;
    }
    set_status(RangeFinder::Status::NoData);
}

/* 
   detect if an analog rangefinder is connected. The only thing we
   can do is check if the pin number is valid. If it is, then assume
   that the device is connected
*/
bool AP_RangeFinder_analog::detect(AP_RangeFinder_Params &_params)
{
    if (_params.pin != -1) {
        return true;
    }
    return false;
}


/*
  update raw voltage state
 */
void AP_RangeFinder_analog::update_voltage(void)
{
   if (source == nullptr) {
       state.voltage_mv = 0;
       return;
   }
   // cope with changed settings
   source->set_pin(params.pin);
   if (params.ratiometric) {
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
    float scaling = params.scaling;
    float offset  = params.offset;
    RangeFinder::Function function = (RangeFinder::Function)params.function.get();
    int16_t _max_distance_cm = params.max_distance_cm;

    switch (function) {
    case RangeFinder::Function::LINEAR:
        dist_m = (v - offset) * scaling;
        break;
	  
    case RangeFinder::Function::INVERTED:
        dist_m = (offset - v) * scaling;
        break;

    case RangeFinder::Function::HYPERBOLA:
        if (v <= offset) {
            dist_m = 0;
        } else {
            dist_m = scaling / (v - offset);
        }
        if (dist_m > _max_distance_cm * 0.01f) {
            dist_m = _max_distance_cm * 0.01f;
        }
        break;
    }
    if (dist_m < 0) {
        dist_m = 0;
    }
    state.distance_cm = dist_m * 100.0f;
    state.last_reading_ms = AP_HAL::millis();

    // update range_valid state based on distance measured
    update_status();
}

