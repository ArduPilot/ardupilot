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
  simple parachute simulator class
*/

#include "SIM_Parachute.h"
#include "AP_HAL/AP_HAL.h"
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo Parachute::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Parachute Sim enable/disable
    // @Description: Allows you to enable (1) or disable (0) the Parachute simulation
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 0, Parachute, parachute_enable, 0),

    // @Param: PIN
    // @DisplayName: Parachute pin
    // @Description: The pin number that the Parachute pyrotechnics are connected to. (start at 1)
    // @Range: 0 15
    // @User: Advanced
    AP_GROUPINFO("PIN", 1, Parachute, parachute_pin, -1),

    AP_GROUPEND
};

/*
  update parachute state
 */
void Parachute::update(const struct sitl_input &input)
{
    const int16_t pwm = parachute_pin >= 1 ? input.servos[parachute_pin-1] : -1;
    const uint64_t now = AP_HAL::micros64();
    // const float dt = (now - last_update_us) * 1.0e-6f;
    if (pwm >= 1250) {
        if (!deployed_ms) {
            deployed_ms = AP_HAL::millis();
            gcs().send_text(MAV_SEVERITY_WARNING, "BANG!  Parachute deployed");
        }
    }
    last_update_us = now;
}

bool Parachute::should_report()
{
    if (AP_HAL::micros64() - last_report_us < report_interval) {
        return false;
    }

    return false;
}
