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

#include "AP_WindVane_RPM.h"

#include "AP_WindVane_config.h"

#if AP_WINDVANE_RPM_ENABLED

#include <AP_RPM/AP_RPM.h>

void AP_WindVane_RPM::update_speed()
{
    const AP_RPM* rpm = AP_RPM::get_singleton();
    if (rpm != nullptr) {
        float temp_speed;
        if (rpm->get_rpm(0, temp_speed) &&
            !is_negative(temp_speed)) {
            _frontend._speed_apparent_raw = temp_speed;
        }
    }
}

#endif  // AP_WINDVANE_RPM_ENABLED
