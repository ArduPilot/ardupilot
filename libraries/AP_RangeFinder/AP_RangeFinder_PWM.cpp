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
}

/*
   There's no sensible way of detecting a PWM rangefinder as the pins are configurable
*/
bool AP_RangeFinder_PWM::detect()
{
    return true;
}

// interrupt handler for reading pwm value
void AP_RangeFinder_PWM::irq_handler(uint8_t pin, bool pin_high, uint32_t timestamp_us)
{
    if (pin_high) {
        irq_pulse_start_us = timestamp_us;
    } else {
        if (irq_pulse_start_us != 0) {
            irq_value_us += timestamp_us - irq_pulse_start_us;
            irq_pulse_start_us = 0;
            irq_sample_count++;
        }
    }
}

// read - return last value measured by sensor
bool AP_RangeFinder_PWM::get_reading(uint16_t &reading_cm)
{
    // disable interrupts and grab state
    void *irqstate = hal.scheduler->disable_interrupts_save();
    const uint32_t value_us = irq_value_us;
    const uint16_t sample_count = irq_sample_count;
    irq_value_us = 0;
    irq_sample_count = 0;
    hal.scheduler->restore_interrupts(irqstate);

    if (value_us == 0 || sample_count == 0) {
        return false;
    }
    reading_cm = value_us/(sample_count * 10); // correct for LidarLite.  Parameter needed?  Converts from decimetres -> cm here
    return true;
}

void AP_RangeFinder_PWM::check_pin()
{
    if (params.pin == last_pin) {
        return;
    }

    // detach last one
    if (last_pin > 0) {
        if (!hal.gpio->detach_interrupt(last_pin)) {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "RangeFinder_PWM: Failed to detach from pin %u",
                            last_pin);
            // ignore this failure or the user may be stuck
        }
    }

    // set last pin to params.pin so we don't continually try to attach
    // to it if the attach is failing
    last_pin = params.pin;

    if (params.pin <= 0) {
        // don't need to install handler
        return;
    }

    // install interrupt handler on rising and falling edge
    hal.gpio->pinMode(params.pin, HAL_GPIO_INPUT);
    if (!hal.gpio->attach_interrupt(
            params.pin,
            FUNCTOR_BIND_MEMBER(&AP_RangeFinder_PWM::irq_handler,
                                void,
                                uint8_t,
                                bool,
                                uint32_t),
            AP_HAL::GPIO::INTERRUPT_BOTH)) {
        // failed to attach interrupt
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "RangeFinder_PWM: Failed to attach to pin %u",
                        (unsigned int)params.pin);
        return;
    }
}

void AP_RangeFinder_PWM::check_stop_pin()
{
    if (params.stop_pin == last_stop_pin) {
        return;
    }

    hal.gpio->pinMode(params.stop_pin, HAL_GPIO_OUTPUT);

    last_stop_pin = params.stop_pin;
}

void AP_RangeFinder_PWM::check_pins()
{
    check_pin();
    check_stop_pin();
}


/*
   update the state of the sensor
*/
void AP_RangeFinder_PWM::update(void)
{
    // check if pin has changed and configure interrupt handlers if required:
    check_pins();

    if (last_pin <= 0) {
        // disabled (by configuration)
        return;
    }

    if (params.stop_pin != -1) {
        const bool oor = out_of_range();
        if (oor) {
            if (!was_out_of_range) {
                // we are above the power saving range. Disable the sensor
                hal.gpio->write(params.stop_pin, false);
                set_status(RangeFinder::RangeFinder_NoData);
                state.distance_cm = 0;
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

    if (!get_reading(state.distance_cm)) {
        // failure; consider changing our state
        if (AP_HAL::millis() - state.last_reading_ms > 200) {
            set_status(RangeFinder::RangeFinder_NoData);
        }
        return;
    }
    // add offset
    state.distance_cm += params.offset;

    // update range_valid state based on distance measured
    state.last_reading_ms = AP_HAL::millis();
    update_status();
}


// return true if we are beyond the power saving range
bool AP_RangeFinder_PWM::out_of_range(void) const {
    return params.powersave_range > 0 && estimated_terrain_height > params.powersave_range;
}
