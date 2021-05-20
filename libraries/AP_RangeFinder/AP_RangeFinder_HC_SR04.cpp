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
 *   AP_RangeFinder_HC_SR04.cpp - rangefinder for HC_SR04 source
 *
 *   https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
 *
 *   There are two pins involved - one we attach an interrupt handler
 *   to and use for measuring the supplied interval which is
 *   proportional to distance.
 *
 *   The second pin we use for triggering the ultransonic pulse
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Params.h"
#include "AP_RangeFinder_HC_SR04.h"

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_RangeFinder_HC_SR04::AP_RangeFinder_HC_SR04(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params)
{
    set_status(RangeFinder::Status::NoData);
}

bool AP_RangeFinder_HC_SR04::check_pins()
{
    check_trigger_pin();
    return check_echo_pin() && trigger_pin > 0;
}

void AP_RangeFinder_HC_SR04::check_trigger_pin()
{
    if (params.stop_pin == trigger_pin) {
        // no change
        return;
    }
    trigger_pin = params.stop_pin;
}

bool AP_RangeFinder_HC_SR04::check_echo_pin()
{
    return pwm_source.set_pin(params.pin, "HC_SR04");
}



/*
   detect if an HC_SR04 rangefinder is connected. The only thing we
   can do is check if the pin number is valid. If it is, then assume
   that the device is connected
*/
bool AP_RangeFinder_HC_SR04::detect(AP_RangeFinder_Params &_params)
{
    if (_params.pin == -1) {
        return false;
    }
    if (_params.stop_pin == -1) {
        return false;
    }
    return true;
}


/*
  update distance_cm
 */
void AP_RangeFinder_HC_SR04::update(void)
{
    // check if pin has changed and configure interrupt handlers if required:
    if (!check_pins()) {
        // disabled (either by configuration or failure to attach interrupt)
        state.distance_cm = 0.0f;
        return;
    }

    const uint32_t value_us = pwm_source.get_pwm_us();

    const uint32_t now = AP_HAL::millis();
    if (value_us == 0) {
        // no reading; check for timeout:
        if (now - last_reading_ms > 1000) {
            // no reading for a second - something is broken
            state.distance_cm = 0.0f;
        }
    } else {
        // gcs().send_text(MAV_SEVERITY_WARNING, "Pong!");
        // a new reading - convert time to distance
        state.distance_cm = value_us * (1.0/58.0f);  // 58 is from datasheet, mult for performance

        // glitch remover: measurement is greater than .5m from last.
        // the SR-04 seeems to suffer from single-measurement glitches
        // which can be removed by a simple filter.
        if (labs(int32_t(uint32_t(state.distance_cm) - last_distance_cm)) > 50) {
            // if greater for 5 readings then pass it as new height,
            // otherwise use last reading
            if (glitch_count++ > 4) {
                 last_distance_cm = state.distance_cm;
            } else {
                 state.distance_cm = last_distance_cm;
            }
        } else {
            // is not greater 0.5m, pass on and reset glitch counter
            last_distance_cm = state.distance_cm;
            glitch_count = 0;
        }

        last_reading_ms = now;
    }

    // update range_valid state based on distance measured
    update_status();

    // consider sending new ping
    if (now - last_ping_ms > 67) { // read ~@15Hz - recommended 60ms delay from datasheet
        last_ping_ms = now;
        // gcs().send_text(MAV_SEVERITY_INFO, "Ping!");
        // raise stop pin for n-microseconds
        hal.gpio->pinMode(trigger_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(trigger_pin, 1);
        hal.scheduler->delay_microseconds(10);
        hal.gpio->write(trigger_pin, 0);
    }
}

