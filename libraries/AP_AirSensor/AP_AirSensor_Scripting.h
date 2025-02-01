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
#pragma once

#include "AP_AirSensor_config.h"

#if AP_AIRSENSOR_SCRIPTING_ENABLED

#include "AP_AirSensor_Backend.h"
#include "AP_Math/vector3.h"

class AP_AirSensor_Scripting: public AP_AirSensor_Backend {
public:
    // constructor
    using AP_AirSensor_Backend::AP_AirSensor_Backend;
    bool init() override { return true; }
    void update() override;

    // TODO add a getter like "bool get_wind(Vector3f& wind_uvw) const" - see AP_Proximity_Scripting get_upward_distance
    bool get_wind(Vector3f& wind_uvw) const override;

#if AP_SCRIPTING_ENABLED
    // this is in body frame
    bool handle_script_3d_msg(const Vector3f &wind_uvw) override;
#endif
private:
    uint32_t _last_update_ms;
    Vector3f _wind_uvw;
};

#endif // AP_AIRSENSOR_SCRIPTING_ENABLED
