// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include "RPM_SITL.h"

extern const AP_HAL::HAL& hal;

/* 
   open the sensor in constructor
*/
AP_RPM_SITL::AP_RPM_SITL(AP_RPM &_ap_rpm, uint8_t _instance, AP_RPM::RPM_State &_state) :
	AP_RPM_Backend(_ap_rpm, _instance, _state)
{
    sitl = (SITL::SITL *)AP_Param::find_object("SIM_");
    instance = _instance;
}

void AP_RPM_SITL::update(void)
{
    if (sitl == nullptr) {
        return;
    }
    if (instance == 0) {
        state.rate_rpm = sitl->state.rpm1;
    } else {
        state.rate_rpm = sitl->state.rpm2;
    }
    state.rate_rpm *= ap_rpm._scaling[state.instance];
    state.signal_quality = 0.5f;
    state.last_reading_ms = AP_HAL::millis();
}

#endif // CONFIG_HAL_BOARD
