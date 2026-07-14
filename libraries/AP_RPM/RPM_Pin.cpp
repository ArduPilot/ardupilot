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

#include "AP_RPM_config.h"

#if AP_RPM_PIN_ENABLED

#include "RPM_Pin.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/GPIO.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;
AP_RPM_Pin::IrqState AP_RPM_Pin::irq_state[RPM_MAX_INSTANCES];

/*
  handle interrupt on an instance
 */
void AP_RPM_Pin::irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp)
{
    auto &data = irq_state[state.instance];
    const uint32_t dt = timestamp - data.last_pulse_us;
    data.last_pulse_us = timestamp;
    // we don't accept pulses less than 100us. Using an irq for such
    // high RPM is too inaccurate, and it is probably just bounce of
    // the signal which we should ignore
    if (dt > 100) {
        data.buffer[(data.last_sample + 1) % RPM_PIN_BUFFER_SIZE] = dt;
        data.last_sample++;
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
        // set the last sample to 0, next interrupt call will put the value in the next element,
        // update will see 0 and assume it hit the uninitialized part of the buffer.
        
        irq_state[state.instance].buffer[irq_state[state.instance].last_sample] = 0; 
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
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RPM: Failed to attach to pin %d", last_pin);
            }
        }
    }
    const auto &data = irq_state[state.instance];

    if (last_sample_used != data.last_sample) {
        last_sample_used = data.last_sample;
        uint8_t samples_used = 0;
        uint32_t sample_sum = 0;
        for(uint8_t i = 0; i < RPM_PIN_BUFFER_SIZE-2; i++){ //skip two oldest samples in the buffer just in case we got unlucky with the interrupt timing
            uint32_t sample = data.buffer[(last_sample_used - i) % RPM_PIN_BUFFER_SIZE];
            if (sample < 100 || sample >= 1000 * 1000){
                break; // we have hit either unused or timed out sample;
            }
            sample_sum += sample;
            samples_used++;
            if (sample_sum > 1000 * 1000){ //don't add samples that are older than 1s compared to last sample
                break;
            }
        }
        if (samples_used > 0) {
            float dt_avg = static_cast<float>(sample_sum)/samples_used;
            const float scaling = ap_rpm._params[state.instance].scaling;
            const float maximum = ap_rpm._params[state.instance].maximum;
            const float minimum = ap_rpm._params[state.instance].minimum;
            float quality;
            const float rpm = scaling * (1.0e6 / dt_avg) * 60;
            const float filter_value = signal_quality_filter.get();

            state.rate_rpm = signal_quality_filter.apply(rpm);

            if ((maximum <= 0 || rpm <= maximum) && (rpm >= minimum)) {
                if (is_zero(filter_value)){
                    quality = 0;
                } else {
                    quality = 1 - constrain_float((fabsf(rpm - filter_value)) / filter_value, 0.0, 1.0);
                    quality = powf(quality, 2.0);
                }
                state.last_reading_ms = AP_HAL::millis();
            } else {
                quality = 0;
            }
            state.signal_quality = (0.1 * quality) + (0.9 * state.signal_quality);
        }
    }


    // assume we get readings at at least 1Hz, otherwise reset quality to zero
    if (AP_HAL::millis() - state.last_reading_ms > 1000) {
        state.signal_quality = 0;
        state.rate_rpm = 0;
    }
}

#endif  // AP_RPM_PIN_ENABLED
