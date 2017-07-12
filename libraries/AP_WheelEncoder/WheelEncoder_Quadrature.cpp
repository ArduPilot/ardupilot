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

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <board_config.h>
#include "WheelEncoder_Quadrature.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;
AP_WheelEncoder_Quadrature::IrqState AP_WheelEncoder_Quadrature::irq_state[WHEELENCODER_MAX_INSTANCES];

// constructor
AP_WheelEncoder_Quadrature::AP_WheelEncoder_Quadrature(AP_WheelEncoder &frontend, uint8_t instance, AP_WheelEncoder::WheelEncoder_State &state) :
	AP_WheelEncoder_Backend(frontend, instance, state)
{
}

void AP_WheelEncoder_Quadrature::update(void)
{
    uint8_t instance = _state.instance;

    // check if pin a has changed and initialise gpio event callback
    if (last_pin_a != get_pin_a()) {
        last_pin_a = get_pin_a();

        // remove old gpio event callback if present
        if (irq_state[instance].last_gpio_a != 0) {
            stm32_gpiosetevent(irq_state[instance].last_gpio_a, false, false, false, nullptr);
            irq_state[instance].last_gpio_a = 0;
        }

        // install interrupt handler on rising or falling edge of gpio for pin a
        irq_state[instance].last_gpio_a = get_gpio(last_pin_a);
        if (irq_state[instance].last_gpio_a != 0) {
            stm32_gpiosetevent(irq_state[instance].last_gpio_a, true, true, false, _state.instance==0 ? irq_handler0_pina : irq_handler1_pina);
        }

    }

    // check if pin b has changed and initialise gpio event callback
    if (last_pin_b != get_pin_b()) {
        last_pin_b = get_pin_b();

        // remove old gpio event callback if present
        if (irq_state[instance].last_gpio_b != 0) {
            stm32_gpiosetevent(irq_state[instance].last_gpio_b, false, false, false, nullptr);
            irq_state[instance].last_gpio_b = 0;
        }

        // install interrupt handler on rising or falling edge of gpio for pin b
        irq_state[instance].last_gpio_b = get_gpio(last_pin_b);
        if (irq_state[instance].last_gpio_b != 0) {
            stm32_gpiosetevent(irq_state[instance].last_gpio_b, true, true, false, _state.instance==0 ? irq_handler0_pinb : irq_handler1_pinb);
        }

    }

    // disable interrupts to prevent race with irq_handler
    irqstate_t istate = irqsave();

    // copy distance and error count so it is accessible to front end
    _state.distance_count = irq_state[instance].distance_count;
    _state.total_count = irq_state[instance].total_count;
    _state.error_count = irq_state[instance].error_count;
    _state.last_reading_ms = AP_HAL::millis();

    // restore interrupts
    irqrestore(istate);
}

// interrupt handler for instance 0, pin a
int AP_WheelEncoder_Quadrature::irq_handler0_pina(int irq, void *context)
{
    irq_handler(0, true);
    return 0;
}

// interrupt handler for instance 0, pin b
int AP_WheelEncoder_Quadrature::irq_handler0_pinb(int irq, void *context)
{
    irq_handler(0, false);
    return 0;
}

// interrupt handler for instance 1, pin a
int AP_WheelEncoder_Quadrature::irq_handler1_pina(int irq, void *context)
{
    irq_handler(1, true);
    return 0;
}

// interrupt handler for instance 1, pin b
int AP_WheelEncoder_Quadrature::irq_handler1_pinb(int irq, void *context)
{
    irq_handler(1, false);
    return 0;
}

// get gpio id from pin number
uint32_t AP_WheelEncoder_Quadrature::get_gpio(uint8_t pin_number)
{
#ifdef GPIO_GPIO0_INPUT
    switch (pin_number) {
    case 50:
        return GPIO_GPIO0_INPUT;
    case 51:
        return GPIO_GPIO1_INPUT;
    case 52:
        return GPIO_GPIO2_INPUT;
    case 53:
        return GPIO_GPIO3_INPUT;
    case 54:
        return GPIO_GPIO4_INPUT;
    case 55:
        return GPIO_GPIO5_INPUT;
    }
#endif
    return 0;
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

void AP_WheelEncoder_Quadrature::update_phase_and_error_count(bool pin_a_now, bool pin_b_now, uint8_t &phase, int32_t &distance_count, uint32_t &total_count, uint32_t &error_count)
{
    // convert pin state before and after to phases
    uint8_t phase_after = pin_ab_to_phase(pin_a_now, pin_b_now);

    // look for invalid changes
    uint8_t step_forward = phase < 3 ? phase+1 : 0;
    uint8_t step_back = phase > 0 ? phase-1 : 3;
    if (phase_after == step_forward) {
        phase = phase_after;
        distance_count++;
    } else if (phase_after == step_back) {
        phase = phase_after;
        distance_count--;
    } else {
        error_count++;
    }
    total_count++;
}

// combined irq handler
void AP_WheelEncoder_Quadrature::irq_handler(uint8_t instance, bool pin_a)
{
    // sanity check
    if (irq_state[instance].last_gpio_a == 0 || irq_state[instance].last_gpio_b == 0) {
        return;
    }

    // read value of pin-a and pin-b
    bool pin_a_high = stm32_gpioread(irq_state[instance].last_gpio_a);
    bool pin_b_high = stm32_gpioread(irq_state[instance].last_gpio_b);

    // update distance and error counts
    update_phase_and_error_count(pin_a_high, pin_b_high, irq_state[instance].phase, irq_state[instance].distance_count, irq_state[instance].total_count, irq_state[instance].error_count);
}

#endif // CONFIG_HAL_BOARD
