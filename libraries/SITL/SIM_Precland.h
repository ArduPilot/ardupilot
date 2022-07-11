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
#include "stdint.h"
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>

namespace SITL {

class SIM_Precland {
public:
    SIM_Precland() {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // update precland state
    void update(const Location &loc, const Vector3d &position);

    // true if precland sensor is online and healthy
    bool healthy() const { return _healthy; }

    // timestamp of most recent data read from the sensor
    uint32_t last_update_ms() const { return _last_update_ms; }

    const Vector3d &get_target_position() const { return _target_pos; }
    bool is_enabled() const {return static_cast<bool>(_enable);}
    void set_default_location(float lat, float lon, int16_t yaw);
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int8 _enable;
    AP_Float _origin_lat;
    AP_Float _origin_lon;
    AP_Float _origin_height;
    AP_Int16 _orient_yaw;
    AP_Int8 _type;
    AP_Int32 _rate;
    AP_Float _alt_limit;
    AP_Float _dist_limit;
    AP_Int8 _orient;
    bool _over_precland_base;

    enum PreclandType {
        PRECLAND_TYPE_CYLINDER = 0,
        PRECLAND_TYPE_CONE = 1,
        PRECLAND_TYPE_SPHERE = 2,
    };
private:
    uint32_t _last_update_ms;
    bool _healthy;
    Vector3d _target_pos;
};

}
