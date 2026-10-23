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

#include "AP_Proximity_Scripting.h"

#if HAL_PROXIMITY_ENABLED && AP_SCRIPTING_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define PROXIMITY_SCRIPTING_TIMEOUT_MS 1500 // distance messages must arrive within this many milliseconds

// update the state of the sensor
void AP_Proximity_Scripting::update(void)
{
    // check for timeout and set health status
    if ((_last_update_ms == 0 || (AP_HAL::millis() - _last_update_ms > PROXIMITY_SCRIPTING_TIMEOUT_MS)) &&
        (_last_upward_update_ms == 0 || (AP_HAL::millis() - _last_upward_update_ms > PROXIMITY_SCRIPTING_TIMEOUT_MS))) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// Set max and min range of the sensor. Only needs to be set once
bool AP_Proximity_Scripting::set_distance_min_max(float min, float max)
{
    if (min >= max) {
        return false;
    }
    _distance_min = min;
    _distance_max = max;
    return true;
}

// get distance upwards in meters. returns true on success
bool AP_Proximity_Scripting::get_upward_distance(float &distance) const
{
    if ((_last_upward_update_ms != 0) && (AP_HAL::millis() - _last_upward_update_ms <= PROXIMITY_SCRIPTING_TIMEOUT_MS)) {
        distance = _distance_upward;
        return true;
    }
    return false;
}

// handle script distance messages
bool AP_Proximity_Scripting::handle_script_distance_msg(float dist_m, float yaw_deg, float pitch_deg, bool push_to_boundary)
{
    _last_update_ms = AP_HAL::millis();

    Vector3f current_pos;
    Matrix3f body_to_ned;
    const bool database_ready = database_prepare_for_push(current_pos, body_to_ned);

    if (dist_m < distance_min_m() || dist_m > distance_max_m() || is_zero(dist_m)) {
        // message isn't healthy
        return false;
    }

    // store upward distance
    if (is_equal(pitch_deg, 90.f)) {
        _distance_upward = dist_m;
        _last_upward_update_ms = _last_update_ms;
        return true;
    }

    yaw_deg = correct_angle_for_orientation(yaw_deg);

    if (ignore_reading(pitch_deg, yaw_deg, dist_m, false)) {
        // obstacle is probably near ground or out of range
        return false;
    }

    // allot to correct layer and sector based on calculated pitch and yaw
    const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(pitch_deg, yaw_deg);

    // add to temp boundary
    temp_boundary.add_distance(face, pitch_deg, yaw_deg, dist_m);

    if (push_to_boundary) {
        temp_boundary.update_3D_boundary(state.instance, frontend.boundary);
        temp_boundary.reset();
    }

    if (database_ready) {
        database_push(yaw_deg, pitch_deg, dist_m, _last_update_ms, current_pos, body_to_ned);
    }

    return true;
}

// handle script vector messages
bool AP_Proximity_Scripting::handle_script_3d_msg(const Vector3f &vec_to_obstacle, bool push_to_boundary)
{
    // convert to FRU
    const Vector3f obstacle(vec_to_obstacle.x, vec_to_obstacle.y, vec_to_obstacle.z * -1.0f);

    // extract yaw and pitch from Obstacle Vector
    const float yaw = wrap_360(degrees(atan2f(obstacle.y, obstacle.x)));
    const float pitch = wrap_180(degrees(M_PI_2 - atan2f(obstacle.xy().length(), obstacle.z)));

    // now simply handle as a distance msg
    return handle_script_distance_msg(obstacle.length(), yaw, pitch, push_to_boundary);
}

// update the temporary (buffer) boundary
bool AP_Proximity_Scripting::update_virtual_boundary()
{
    temp_boundary.update_3D_boundary(state.instance, frontend.boundary);
    temp_boundary.reset();
    return true;
}

#endif // HAL_PROXIMITY_ENABLED && AP_SCRIPTING_ENABLED
