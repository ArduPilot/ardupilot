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

#include "WheelEncoder_Quadrature.h"

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// check if pin has changed and initialise gpio event callback
void AP_WheelEncoder_Quadrature::update_pin(uint8_t &pin,
                                            uint8_t new_pin,
                                            uint8_t &pin_value)
{
    if (new_pin == pin) {
        // no change
        return;
    }

    // remove old gpio event callback if present
    if (pin != (uint8_t)-1 &&
        !hal.gpio->detach_interrupt(pin)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WEnc: Failed to detach from pin %u", pin);
        // ignore this failure or the user may be stuck
    }

    pin = new_pin;

    // install interrupt handler on rising or falling edge of gpio for pin a
    if (new_pin != (uint8_t)-1) {
        hal.gpio->pinMode(pin, HAL_GPIO_INPUT);
        if (!hal.gpio->attach_interrupt(
                pin,
                FUNCTOR_BIND_MEMBER(&AP_WheelEncoder_Quadrature::irq_handler,
                                    void,
                                    uint8_t,
                                    bool,
                                    uint32_t),
                AP_HAL::GPIO::INTERRUPT_BOTH)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "WEnc: Failed to attach to pin %u", pin);
        }
        pin_value = hal.gpio->read(pin);
    }
}

void AP_WheelEncoder_Quadrature::update(void)
{
    update_pin(last_pin_a, get_pin_a(), last_pin_a_value);
    update_pin(last_pin_b, get_pin_b(), last_pin_b_value);

    // disable interrupts to prevent race with irq_handler
    void *irqstate = hal.scheduler->disable_interrupts_save();

    // copy distance and error count so it is accessible to front end
    copy_state_to_frontend(irq_state.distance_count,
                           irq_state.total_count,
                           irq_state.error_count,
                           irq_state.last_reading_ms);

    // restore interrupts
    hal.scheduler->restore_interrupts(irqstate);
}

// convert pin a and pin b state to a wheel encoder phase
uint8_t AP_WheelEncoder_Quadrature::pin_ab_to_phase(bool pin_a, bool pin_b)
{
    if (!pin_a) {
        if (!pin_b) {
            // A = 0, B = 0
            return 0;
        } else {
            // A = 0, B = 1
            return 1;
        }
    } else {
        if (!pin_b) {
            // A = 1, B = 0
            return 3;
        } else {
            // A = 1, B = 1
            return 2;
        }
    }
    return (uint8_t)pin_a << 1 | (uint8_t)pin_b;
}

void AP_WheelEncoder_Quadrature::update_phase_and_error_count()
{
    // convert pin state before and after to phases
    uint8_t phase_after = pin_ab_to_phase(last_pin_a_value, last_pin_b_value);

    // look for invalid changes
    uint8_t step_forward = irq_state.phase < 3 ? irq_state.phase+1 : 0;
    uint8_t step_back = irq_state.phase > 0 ? irq_state.phase-1 : 3;
    if (phase_after == step_forward) {
        irq_state.phase = phase_after;
        irq_state.distance_count++;
    } else if (phase_after == step_back) {
        irq_state.phase = phase_after;
        irq_state.distance_count--;
    } else {
        irq_state.error_count++;
    }
    irq_state.total_count++;
}

void AP_WheelEncoder_Quadrature::irq_handler(uint8_t pin,
                                             bool pin_value,
                                             uint32_t timestamp)
{
    // sanity check
    if (last_pin_a == 0 || last_pin_b == 0) {
        return;
    }

    // update distance and error counts
    if (pin == last_pin_a) {
        last_pin_a_value = pin_value;
    } else if (pin == last_pin_b) {
        last_pin_b_value = pin_value;
    } else {
        return;
    };
    update_phase_and_error_count();

    // record update time
    irq_state.last_reading_ms = timestamp * 1e-3f;
}
