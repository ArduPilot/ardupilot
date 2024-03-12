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

#include "AP_Notify/AP_Notify.h"
#include <AP_HAL/RCOutput.h>
#include "DShotLED.h"
#include "SRV_Channel/SRV_Channel.h"

extern const AP_HAL::HAL& hal;

bool DShotLED::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    // don't play the motor LEDs while flying
    if (hal.util->get_soft_armed() || hal.rcout->get_dshot_esc_type() != AP_HAL::RCOutput::DSHOT_ESC_BLHELI) {
        return false;
    }

    hal.rcout->send_dshot_command(red ? AP_HAL::RCOutput::DSHOT_LED1_ON : AP_HAL::RCOutput::DSHOT_LED1_OFF);
    hal.rcout->send_dshot_command(green ? AP_HAL::RCOutput::DSHOT_LED2_ON : AP_HAL::RCOutput::DSHOT_LED2_OFF);
    hal.rcout->send_dshot_command(blue ? AP_HAL::RCOutput::DSHOT_LED0_ON : AP_HAL::RCOutput::DSHOT_LED0_OFF);

    return true;
}