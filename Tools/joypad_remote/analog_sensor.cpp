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

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

#include "analog_sensor.h"

extern const AP_HAL::HAL& hal;

AnalogSensor::AnalogSensor(state_t *_state):
    state(*_state)
{
    source = hal.analogin->channel(_state->pin);
    if (source == nullptr) {
        // failed to allocate a ADC channel? This shouldn't happen
        return;
    }

    source->set_stop_pin((uint8_t)_state->stop_pin);
    source->set_settle_time((uint16_t)_state->settle_time_ms);
}

void AnalogSensor::update_voltage(void)
{
   if (source == nullptr) {
       state.voltage_mv = 0;
       return;
   }
   // cope with changed settings
   source->set_pin(state.pin);
   source->set_stop_pin((uint8_t)state.stop_pin);
   source->set_settle_time((uint16_t)state.settle_time_ms);
   if (state.ratiometric) {
       state.voltage_mv = source->voltage_average_ratiometric() * 1000U;
   } else {
       state.voltage_mv = source->voltage_average() * 1000U;
   }
}

/*
  update distance_cm
 */
void AnalogSensor::update(void)
{
    update_voltage();

    float volt = state.voltage_mv * 0.001f;
    float dist_m = 0;
    float scaling = state.scaling;
    float offset  = state.offset;
    AnalogSensor::AnalogSensor_Function function = (AnalogSensor::AnalogSensor_Function)state.function;
    int16_t _max_distance_cm = state.max_distance_cm;

    switch (function) {
    case AnalogSensor::FUNCTION_LINEAR:
        dist_m = (volt - offset) * scaling;
        break;
    case AnalogSensor::FUNCTION_INVERTED:
        dist_m = (offset - volt) * scaling;
        break;
    case AnalogSensor::FUNCTION_HYPERBOLA:
        if (volt <= offset) {
            dist_m = 0;
        }
        dist_m = scaling / (volt - offset);
        if (isinf(dist_m) || dist_m > _max_distance_cm * 0.01f) {
            dist_m = _max_distance_cm * 0.01f;
        }
        break;
    }

    if (dist_m < 0) {
        dist_m = 0;
    }

    state.distance_cm = dist_m * 100.0f;
}
