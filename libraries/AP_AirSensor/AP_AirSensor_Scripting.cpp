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

#include "AP_AirSensor_config.h"

#if AP_AIRSENSOR_SCRIPTING_ENABLED

#include "AP_AirSensor_Scripting.h"

// Require 10Hz, with 1.5x FOS.
static constexpr uint32_t TIMEOUT_MS = 100 * 1.5;

void AP_AirSensor_Scripting::update() {

    // TODO investigate why scripting checks 0 first.
    if (AP_HAL::millis() - _last_update_ms < 100 * 1.5) {
        // set_status(AP_Proximity::Status::NoData);
    } else {
        // set_status(AP_Proximity::Status::Good);
    }
}

bool AP_AirSensor_Scripting::get_wind(Vector3f& wind_uvw) const {
    if ((_last_update_ms != 0) && (AP_HAL::millis() - _last_update_ms <= TIMEOUT_MS)) {
        wind_uvw = _wind_uvw;
        return true;
    }
    return false;
}

#if AP_SCRIPTING_ENABLED
bool AP_AirSensor_Scripting::handle_script_3d_msg(const Vector3f &wind_uvw) {
    _last_update_ms = AP_HAL::millis();
    _wind_uvw = wind_uvw;
    return true;
}
#endif

#endif // AP_AIRSENSOR_SCRIPTING_ENABLED
