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

#include "SIM_Precland.h"
#include "AP_HAL/AP_HAL.h"
#include "AP_Math/AP_Math.h"
#include "AP_Common/Location.h"
#include <stdio.h>

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo SIM_Precland::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Preland device Sim enable/disable
    // @Description: Allows you to enable (1) or disable (0) the Preland simulation
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE",  0, SIM_Precland, _enable, 0),

    // @Param: LAT
    // @DisplayName: Precland device origin's latitude
    // @Description: Precland device origin's latitude
    // @Units: deg
    // @Increment: 0.000001
    // @Range: -90 90
    // @User: Advanced
    AP_GROUPINFO("LAT", 1, SIM_Precland, _origin_lat, 0),

    // @Param: LON
    // @DisplayName: Precland device origin's longitude
    // @Description: Precland device origin's longitude
    // @Units: deg
    // @Increment: 0.000001
    // @Range: -180 180
    // @User: Advanced
    AP_GROUPINFO("LON", 2, SIM_Precland, _origin_lon, 0),

    // @Param: HEIGHT
    // @DisplayName: Precland device origin's height above sealevel
    // @Description: Precland device origin's height above sealevel assume a 2x2m square as station base
    // @Units: cm
    // @Increment: 1
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("HEIGHT", 3, SIM_Precland, _origin_height, 0),

    // @Param: YAW
    // @DisplayName: Precland device systems rotation from north
    // @Description: Precland device systems rotation from north
    // @Units: deg
    // @Increment: 1
    // @Range: -180 +180
    // @User: Advanced
    AP_GROUPINFO("YAW", 4, SIM_Precland, _orient_yaw, 0),

    // @Param: RATE
    // @DisplayName: Precland device update rate
    // @Description: Precland device rate. e.g led patter refresh rate, RF message rate, etc.
    // @Units: Hz
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("RATE", 5, SIM_Precland, _rate, 100),

    // @Param: TYPE
    // @DisplayName: Precland device radiance type
    // @Description: Precland device radiance type: it can be a cylinder, a cone, or a sphere.
    // @Values: 0:cylinder,1:cone,2:sphere
    // @User: Advanced
    AP_GROUPINFO("TYPE", 6, SIM_Precland, _type, SIM_Precland::PRECLAND_TYPE_CYLINDER),

    // @Param: ALT_LIMIT
    // @DisplayName: Precland device alt range
    // @Description: Precland device maximum range altitude
    // @Units: m
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("ALT_LMT", 7, SIM_Precland, _alt_limit, 15),

    // @Param: DIST_LIMIT
    // @DisplayName: Precland device lateral range
    // @Description: Precland device maximum lateral range
    // @Units: m
    // @Range: 5 100
    // @User: Advanced
    AP_GROUPINFO("DIST_LMT", 8, SIM_Precland, _dist_limit, 10),

    // @Param: ORIENT
    // @DisplayName: Precland device orientation
    // @Description: Precland device orientation vector
    // @Values: 0:Front, 4:Back, 24:Up
    // @User: Advanced
    AP_GROUPINFO("ORIENT", 9, SIM_Precland, _orient, ROTATION_PITCH_90),

    AP_GROUPEND
};

void SIM_Precland::update(const Location &loc, const Vector3d &position)
{
    if (!_enable) {
        _healthy = false;
        return;
    }
    if (is_zero(_alt_limit) || _dist_limit < 1.0f) {
        _healthy = false;
        return;
    }

    const Location origin_center(static_cast<int32_t>(_origin_lat * 1.0e7f),
            static_cast<int32_t>(_origin_lon * 1.0e7f),
            static_cast<int32_t>(_origin_height),
            Location::AltFrame::ABOVE_HOME);
    Vector2f centerf;
    if (!origin_center.get_vector_xy_from_origin_NE(centerf)) {
        _healthy = false;
        return;
    }
    centerf = centerf * 0.01f;        // cm to m
    Vector3d center(centerf.x, centerf.y, -_origin_height);   // convert to make the further vector operations easy

    // axis of cone or cylinder inside which the vehicle receives signals from simulated precland device
    Vector3d axis{1, 0, 0};
    axis.rotate((Rotation)_orient.get());   // unit vector in direction of axis of cone or cylinder
    Vector3d position_wrt_origin = position - center;  // position of vehicle with respect to preland device origin
    
    // longitudinal distance of vehicle from the precland device
    // this is the distance of vehicle from the plane which is passing through precland device origin and perpendicular to axis of cone/cylinder
    // this plane is the ground plane when the axis has PITCH_90 rotation
    Vector3d projection_on_axis = position_wrt_origin.projected(axis);
    const float longitudinal_dist = projection_on_axis.length();

    // lateral distance of vehicle from the precland device
    // this is the perpendicular distance of vehicle from the axis of cone/cylinder
    const float lateral_distance = safe_sqrt(MAX(0, position_wrt_origin.length_squared() - longitudinal_dist*longitudinal_dist));

    // sign of projection's dot product with axis tells if vehicle is in front of beacon
    // return false if vehicle if vehicle is  longitudinally too far away from precland device
    // for PITCH_90 orientation, longitudinal distance = alt of vehicle - origin_height (in m)
    if (projection_on_axis.dot(axis) <= 0 || longitudinal_dist > _alt_limit) {
        _healthy = false;
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (now - _last_update_ms < 1000.0f * (1.0f / _rate)) {
        return;
    }
    _last_update_ms = now;

    switch (_type) {
        case PRECLAND_TYPE_CONE: {
            // lateral_limit is the limit of how far the vehicle can laterally be from precland_device
            // in case of cone, this limit increases gradually as the vehicle moves longitudinally far away from precland device
            const float lateral_limit = longitudinal_dist * _dist_limit / _alt_limit;
            if (lateral_distance > lateral_limit) {
                _healthy = false;
                return;
            }
            break;
        }
        case PRECLAND_TYPE_SPHERE: {
            if (position_wrt_origin.length() > _dist_limit) {
                _healthy = false;
                return;
            }
            break;
        }
        default:
        case PRECLAND_TYPE_CYLINDER: {
            if (lateral_distance > _dist_limit) {
                _healthy = false;
                return;
            }
            break;
        }
    }
    _target_pos = position_wrt_origin;
    _healthy = true;
}

void SIM_Precland::set_default_location(float lat, float lon, int16_t yaw) {
    if (is_zero(_origin_lat) && is_zero(_origin_lon)) {
        _origin_lat = lat;
        _origin_lon = lon;
        _orient_yaw = yaw;
    }
}
