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
#include <AP_ESC_Telem/AP_ESC_Telem.h>

#include "RPM_ESC_Telem.h"

#if AP_RPM_ESC_TELEM_ENABLED

extern const AP_HAL::HAL& hal;

/*
   open the sensor in constructor
*/
AP_RPM_ESC_Telem::AP_RPM_ESC_Telem(AP_RPM &_ap_rpm, uint8_t _instance, AP_RPM::RPM_State &_state) :
    AP_RPM_Backend(_ap_rpm, _instance, _state)
{
    instance = _instance;
}


void AP_RPM_ESC_Telem::update(void)
{
#if HAL_WITH_ESC_TELEM
    AP_ESC_Telem &esc_telem = AP::esc_telem();
    float esc_rpm = esc_telem.get_average_motor_rpm(ap_rpm._params[state.instance].esc_mask);
    state.rate_rpm = esc_rpm * ap_rpm._params[state.instance].scaling;
    state.signal_quality = 0.5f;
    state.last_reading_ms = AP_HAL::millis();
#endif
}

#endif  // AP_RPM_ESC_TELEM_ENABLED
