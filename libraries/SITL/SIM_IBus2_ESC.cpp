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
  Simulator for IBUS2 ESC devices receiving Frame 1 motor commands.
*/

#include "SIM_IBus2_ESC.h"

#if AP_IBUS2_ENABLED

#include "SITL_Input.h"
#include <AP_Math/AP_Math.h>

using namespace SITL;

const AP_Param::GroupInfo IBus2ESC::var_info[] = {
    // @Param: ENA
    // @DisplayName: IBUS2 ESC simulator enable
    // @Description: Enable IBUS2 ESC simulator (overrides servo PWM with IBUS2 channel values)
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENA", 1, IBus2ESC, _enabled, 0),

    AP_GROUPEND
};

void IBus2ESC::update_sitl_input_pwm(struct sitl_input &input)
{
    for (uint8_t i = 0; i < MIN(ARRAY_SIZE(_channels), ARRAY_SIZE(input.servos)); i++) {
        if (_channels[i] != 0) {
            input.servos[i] = _channels[i];
        }
    }
}

#endif  // AP_IBUS2_ENABLED
