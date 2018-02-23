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
 *   Adapted from APM HAL AP_RangeFinder_analog.cpp
 *   rangefinder for analog source.
 *   Hiroshi Takey, November, 2017.
 *
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

class AnalogSensor
{
public:

    typedef struct __state_t {
        uint8_t instance;    // the instance number of this RangeFinder
        uint16_t distance_cm; // distance: in cm
        uint16_t voltage_mv;  // voltage in millivolts,
        uint8_t pin;
        uint8_t ratiometric = 1;
        uint8_t stop_pin = -1;
        uint16_t settle_time_ms = 0;
        float scaling = 4.0;
        float offset = 0;
        uint8_t function = 0;
        uint16_t min_distance_cm;
        uint16_t max_distance_cm;
        uint8_t ground_clearance_cm;
    } state_t;

    enum AnalogSensor_Function {
        FUNCTION_LINEAR    = 0,
        FUNCTION_INVERTED  = 1,
        FUNCTION_HYPERBOLA = 2
    };

    AnalogSensor(state_t *_state);

    void update(void);

private:

    void update_voltage(void);
    AP_HAL::AnalogSource *source;
    AnalogSensor::state_t &state;
};
