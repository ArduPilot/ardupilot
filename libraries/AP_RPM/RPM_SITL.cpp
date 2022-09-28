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

#include "RPM_SITL.h"

#if AP_RPM_SIM_ENABLED

#include <AP_HAL/AP_HAL.h>

/*
   open the sensor in constructor
*/
AP_RPM_SITL::AP_RPM_SITL(AP_RPM &_ap_rpm, uint8_t _instance, AP_RPM::RPM_State &_state) :
    AP_RPM_Backend(_ap_rpm, _instance, _state),
    sitl(AP::sitl())
{
    instance = _instance;
}

void AP_RPM_SITL::update(void)
{
    if (sitl == nullptr) {
        return;
    }
    if (instance == 0) {
        state.rate_rpm = sitl->state.rpm[0];
    } else {
        state.rate_rpm = sitl->state.rpm[1];
    }
    state.rate_rpm *= ap_rpm._params[state.instance].scaling;
    state.signal_quality = 0.5f;
    state.last_reading_ms = AP_HAL::millis();

}

#endif // AP_RPM_SIM_ENABLED
