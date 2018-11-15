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

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <board_config.h>
#endif

#include <stdio.h>

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
void AP_RPM_Pin::irq_handler(uint8_t instance)
{
    uint32_t now = AP_HAL::micros();
    uint32_t dt = now - irq_state[instance].last_pulse_us;
    irq_state[instance].last_pulse_us = now;
    // we don't accept pulses less than 100us. Using an irq for such
    // high RPM is too inaccurate, and it is probably just bounce of
    // the signal which we should ignore
    if (dt > 100 && dt < 1000*1000) {
        irq_state[instance].dt_sum += dt;
        irq_state[instance].dt_count++;
    }
}

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
/*
  interrupt handler for instance 0
 */
int AP_RPM_Pin::irq_handler0(int irq, void *context)
{
    irq_handler(0);
    return 0;
}

/*
  interrupt handler for instance 1
 */
int AP_RPM_Pin::irq_handler1(int irq, void *context)
{
    irq_handler(1);
    return 0;
}
#else // other HALs
/*
  interrupt handler for instance 0
 */
void AP_RPM_Pin::irq_handler0(void)
{
    irq_handler(0);
}

/*
  interrupt handler for instance 1
 */
void AP_RPM_Pin::irq_handler1(void)
{
    irq_handler(1);
}
#endif

void AP_RPM_Pin::update(void)
{
    if (last_pin != get_pin()) {
        last_pin = get_pin();

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        uint32_t gpio = 0;

#ifdef GPIO_GPIO0_INPUT
        switch (last_pin) {
        case 50:
            gpio = GPIO_GPIO0_INPUT;
            break;
        case 51:
            gpio = GPIO_GPIO1_INPUT;
            break;
        case 52:
            gpio = GPIO_GPIO2_INPUT;
            break;
        case 53:
            gpio = GPIO_GPIO3_INPUT;
            break;
        case 54:
            gpio = GPIO_GPIO4_INPUT;
            break;
        case 55:
            gpio = GPIO_GPIO5_INPUT;
            break;
        }
#endif // GPIO_GPIO5_INPUT
        
        // uninstall old handler if installed
        if (last_gpio != 0) {
            stm32_gpiosetevent(last_gpio, false, false, false, nullptr);
        }
        irq_state[state.instance].dt_count = 0;
        irq_state[state.instance].dt_sum = 0;

        last_gpio = gpio;

        if (gpio == 0) {
            return;
        }
        
        // install interrupt handler on rising edge of pin. This works
        // for either polarity of pulse, as all we want is the period
        stm32_gpiosetevent(gpio, true, false, false,
                           state.instance==0?irq_handler0:irq_handler1);
#else // other HALs
        hal.gpio->pinMode(last_pin, HAL_GPIO_INPUT);
        hal.gpio->attach_interrupt(last_pin, state.instance==0?irq_handler0:irq_handler1,
                                   HAL_GPIO_INTERRUPT_RISING);
#endif
    }

    if (irq_state[state.instance].dt_count > 0) {
        float dt_avg;

        // disable interrupts to prevent race with irq_handler
        void *irqstate = hal.scheduler->disable_interrupts_save();
        dt_avg = irq_state[state.instance].dt_sum / irq_state[state.instance].dt_count;
        irq_state[state.instance].dt_count = 0;
        irq_state[state.instance].dt_sum = 0;
        hal.scheduler->restore_interrupts(irqstate);

        const float scaling = ap_rpm._scaling[state.instance];
        float maximum = ap_rpm._maximum[state.instance];
        float minimum = ap_rpm._minimum[state.instance];
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
