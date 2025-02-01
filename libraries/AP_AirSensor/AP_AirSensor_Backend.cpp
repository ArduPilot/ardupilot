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

#include "AP_AirSensor_Backend.h"

#if AP_AIRSENSOR_ENABLED

AP_AirSensor_Backend::AP_AirSensor_Backend(AP_AirSensor& frontend, AP_AirSensor::State& state, AP_AirSensor_Params& params) :
    _frontend(frontend),
    _state(state),
    _params(params)
{
    _backend_type = (AP_AirSensor::Type )_params.type.get();
}

bool AP_AirSensor_Backend::get_aoa(float& a) const {
    Vector3f wind;
    if (get_wind(wind)) {
        a = atan2(wind.y, wind.x);
        return true;
    }
    return false;
}

void AP_AirSensor_Backend::set_status(AP_AirSensor::Status status)
{
    _state.status = status;
}

#endif // AP_AIRSENSOR_ENABLED
