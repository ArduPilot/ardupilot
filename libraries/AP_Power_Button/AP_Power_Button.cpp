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

#include "AP_Power_Button.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Arming/AP_Arming.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_Power_Button::AP_Power_Button(void)
{
    last_power_down_time_ms = 0;
}

/*
  update and report, called from main loop
 */
void AP_Power_Button::update(void)
{
#ifdef HAL_PWR_AD_KEY_GPIO
    if (AP::arming().is_armed()) {
        hal.gpio->write(HAL_POWER_CONTROL_GPIO,1);
        return;
    }

    if (pwr_button_down() && last_power_down_time_ms == 0) {
        last_power_down_time_ms = AP_HAL::millis64();
    } else {
        if (!pwr_button_down()) {
            last_power_down_time_ms = 0;
        } else if (pwr_button_down()){
            uint64_t button_down_ms = AP_HAL::millis64() - last_power_down_time_ms;

            if (button_down_ms > MS_PWR_BUTTON_DOWN) {
                hal.gpio->write(HAL_POWER_CONTROL_GPIO,0);
            }
        }
    }
#endif
}


bool AP_Power_Button::pwr_button_down()
{
#ifdef HAL_PWR_AD_KEY_GPIO
    return (hal.gpio->read(HAL_PWR_AD_KEY_GPIO) == 0);
#else
    return false;
#endif
}
