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

#include "AP_RangeFinder_PWM.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder.
*/
AP_RangeFinder_PWM::AP_RangeFinder_PWM(RangeFinder::RangeFinder_State &_state,
                                       AP_RangeFinder_Params &_params,
                                       float &_estimated_terrain_height) :
    AP_RangeFinder_Backend(_state, _params),
    estimated_terrain_height(_estimated_terrain_height)
{
    // this gives one mm per us
    params.scaling.set_default(1.0);
}

/*
   There's no sensible way of detecting a PWM rangefinder as the pins are configurable
*/
bool AP_RangeFinder_PWM::detect()
{
    return true;
}

// read - return last value measured by sensor
bool AP_RangeFinder_PWM::get_reading(float &reading_m)
{
    const uint32_t value_us = pwm_source.get_pwm_avg_us();
    if (value_us == 0) {
        return false;
    }

    // LidarLite uses one mm per us
    reading_m = value_us * 0.001 * params.scaling;
    return true;
}

bool AP_RangeFinder_PWM::check_pin()
{
    if (!pwm_source.set_pin(params.pin, "RangeFinder_PWM")) {
        return false;
    }
    return true;
}

void AP_RangeFinder_PWM::check_stop_pin()
{
    if (params.stop_pin == last_stop_pin) {
        return;
    }

    hal.gpio->pinMode(params.stop_pin, HAL_GPIO_OUTPUT);

    last_stop_pin = params.stop_pin;
}

bool AP_RangeFinder_PWM::check_pins()
{
    check_stop_pin();
    return check_pin();
}


/*
   update the state of the sensor
*/
void AP_RangeFinder_PWM::update(void)
{
    // check if pin has changed and configure interrupt handlers if required:
    if (!check_pins()) {
        return;
    }

    if (params.stop_pin != -1) {
        const bool oor = out_of_range();
        if (oor) {
            if (!was_out_of_range) {
                // we are above the power saving range. Disable the sensor
                hal.gpio->write(params.stop_pin, false);
                set_status(RangeFinder::Status::NoData);
                state.distance_m = 0.0f;
                state.voltage_mv = 0;
                was_out_of_range = oor;
            }
            return;
        }
        // re-enable the sensor:
        if (!oor && was_out_of_range) {
            hal.gpio->write(params.stop_pin, true);
            was_out_of_range = oor;
        }
    }

    if (!get_reading(state.distance_m)) {
        // failure; consider changing our state
        if (AP_HAL::millis() - state.last_reading_ms > 200) {
            set_status(RangeFinder::Status::NoData);
        }
        return;
    }
    // add offset
    state.distance_m += params.offset * 0.01f;

    // update range_valid state based on distance measured
    state.last_reading_ms = AP_HAL::millis();
    update_status();
}


// return true if we are beyond the power saving range
bool AP_RangeFinder_PWM::out_of_range(void) const {
    return params.powersave_range > 0 && estimated_terrain_height > params.powersave_range;
}
