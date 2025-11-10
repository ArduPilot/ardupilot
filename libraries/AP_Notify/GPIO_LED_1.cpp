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

#include "AP_Notify_config.h"

#if AP_NOTIFY_GPIO_LED_1_ENABLED

#include "GPIO_LED_1.h"

#include <AP_HAL/HAL.h>
#include "AP_Notify.h"

#ifndef AP_NOTIFY_GPIO_LED_1_PIN
#error "define AP_NOTIFY_GPIO_LED_1_PIN"
#endif

extern const AP_HAL::HAL& hal;

bool GPIO_LED_1::init(void)
{
    // when HAL_GPIO_LED_ON is 0 then we must not use pinMode()
    // as it could remove the OPENDRAIN attribute on the pin
#if HAL_GPIO_LED_ON != 0
    hal.gpio->pinMode(AP_NOTIFY_GPIO_LED_1_PIN, HAL_GPIO_OUTPUT);
#endif
    hal.gpio->write(AP_NOTIFY_GPIO_LED_1_PIN, HAL_GPIO_LED_OFF);
    return true;
}

/*
  main update function called at 50Hz
 */
void GPIO_LED_1::update(void)
{
    uint32_t new_pattern;
    if (AP_Notify::flags.initialising) {
        new_pattern = INITIALIZING;
    } else if (AP_Notify::flags.failsafe_radio || AP_Notify::flags.failsafe_gcs || AP_Notify::flags.failsafe_battery) {
        new_pattern = FAILSAFE;
    } else if (AP_Notify::flags.armed) {
        new_pattern = ARMED;
    } else if (AP_Notify::flags.pre_arm_check) {
        new_pattern = READY_TO_ARM;
    } else {
        new_pattern = NOT_READY_TO_ARM;
    }
    if (new_pattern != current_pattern) {
        next_bit = 0;
        current_pattern = new_pattern;
        last_timestep_ms = 0;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_timestep_ms < 100) {
        return;
    }
    last_timestep_ms = now_ms;

    const auto new_state = (current_pattern & (1U<<next_bit)) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF;
    hal.gpio->write(AP_NOTIFY_GPIO_LED_1_PIN, new_state);
    next_bit++;
    if (next_bit > 31) {
        next_bit = 0;
    }
}

#endif  // AP_NOTIFY_GPIO_LED_1_ENABLED
