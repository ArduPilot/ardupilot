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

#include <AP_HAL/AP_HAL.h>
#include "RPM_Pin.h"

#include <AP_HAL/GPIO.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;
AP_RPM_Pin::IrqState AP_RPM_Pin::irq_state[RPM_MAX_INSTANCES];

/* 
   open the sensor in constructor
*/
AP_RPM_Pin::AP_RPM_Pin(AP_RPM &_ap_rpm, uint8_t instance, AP_RPM::RPM_State &_state) :
	AP_RPM_Backend(_ap_rpm, instance, _state)
{
}

/*
  handle interrupt on an instance
 */
void AP_RPM_Pin::irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp)
{
    const uint32_t dt = timestamp - irq_state[state.instance].last_pulse_us;
    irq_state[state.instance].last_pulse_us = timestamp;
    // we don't accept pulses less than 100us. Using an irq for such
    // high RPM is too inaccurate, and it is probably just bounce of
    // the signal which we should ignore
    if (dt > 100 && dt < 1000*1000) {
        irq_state[state.instance].dt_sum += dt;
        irq_state[state.instance].dt_count++;
    }
}

void AP_RPM_Pin::update(void)
{
    if (last_pin != get_pin()) {
        // detach from last pin
        if (interrupt_attached) {
            // ignore this failure of the user may be stuck
            IGNORE_RETURN(hal.gpio->detach_interrupt(last_pin));
            interrupt_attached = false;
        }
        irq_state[state.instance].dt_count = 0;
        irq_state[state.instance].dt_sum = 0;
        // attach to new pin
        last_pin = get_pin();
        if (last_pin > 0) {
            hal.gpio->pinMode(last_pin, HAL_GPIO_INPUT);
            if (hal.gpio->attach_interrupt(
                    last_pin,
                    FUNCTOR_BIND_MEMBER(&AP_RPM_Pin::irq_handler, void, uint8_t, bool, uint32_t),
                    AP_HAL::GPIO::INTERRUPT_RISING)) {
                interrupt_attached = true;
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "RPM: Failed to attach to pin %d", last_pin);
            }
        }
    }

    if (irq_state[state.instance].dt_count > 0) {
        float dt_avg;

        // disable interrupts to prevent race with irq_handler
        void *irqstate = hal.scheduler->disable_interrupts_save();
        dt_avg = irq_state[state.instance].dt_sum / irq_state[state.instance].dt_count;
        irq_state[state.instance].dt_count = 0;
        irq_state[state.instance].dt_sum = 0;
        hal.scheduler->restore_interrupts(irqstate);

        const float scaling = ap_rpm._params[state.instance].scaling;
        float maximum = ap_rpm._params[state.instance].maximum;
        float minimum = ap_rpm._params[state.instance].minimum;
        float quality = 0;
        float rpm = scaling * (1.0e6 / dt_avg) * 60;
        float filter_value = signal_quality_filter.get();

        state.rate_rpm = signal_quality_filter.apply(rpm);

        if ((maximum <= 0 || rpm <= maximum) && (rpm >= minimum)) {
            if (is_zero(filter_value)){
                quality = 0;
            } else {
                quality = 1 - constrain_float((fabsf(rpm-filter_value))/filter_value, 0.0, 1.0);
                quality = powf(quality, 2.0);
            }
            state.last_reading_ms = AP_HAL::millis();
        } else {
            quality = 0;
        }
        state.signal_quality = (0.1 * quality) + (0.9 * state.signal_quality);
    }

    // assume we get readings at at least 1Hz, otherwise reset quality to zero
    if (AP_HAL::millis() - state.last_reading_ms > 1000) {
        state.signal_quality = 0;
        state.rate_rpm = 0;
    }
}
