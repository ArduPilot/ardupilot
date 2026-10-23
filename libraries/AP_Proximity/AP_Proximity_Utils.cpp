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

#include "AP_Proximity.h"
#include <AP_Common/AP_Common.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>

#if HAL_PROXIMITY_ENABLED

#define PROXIMITY_ALT_DETECT_TIMEOUT_MS 500 // alt readings should arrive within this much time

extern const AP_HAL::HAL& hal;

// set alt as read from downward facing rangefinder. Tilt is already adjusted for.
void AP_Proximity::set_rangefinder_alt(bool use, bool healthy, float alt_cm)
{
    _rangefinder_state.use = use;
    _rangefinder_state.healthy = healthy;
    _rangefinder_state.alt_cm = alt_cm;
    _rangefinder_state.last_downward_update_ms = AP_HAL::millis();
}

// get alt from rangefinder in meters
bool AP_Proximity::get_rangefinder_alt(float &alt_m) const
{
    if (!_rangefinder_state.use || !_rangefinder_state.healthy) {
        // range finder is not healthy
        return false;
    }

    const uint32_t dt = AP_HAL::millis() - _rangefinder_state.last_downward_update_ms;
    if (dt > PROXIMITY_ALT_DETECT_TIMEOUT_MS) {
        return false;
    }

    // readings are healthy
    alt_m = _rangefinder_state.alt_cm * 0.01f;
    return true;
}

// Check if Obstacle defined by body-frame yaw and pitch is near ground
bool AP_Proximity::check_obstacle_near_ground(float pitch, float yaw, float distance) const
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_Periph)
    if (!_ign_gnd_enable) {
        return false;
    }
    if (!hal.util->get_soft_armed()) {
        // don't run this feature while vehicle is disarmed, otherwise proximity data will not show up on GCS
        return false;
    }
    if ((pitch > 90.0f) || (pitch < -90.0f)) {
        // sanity check on pitch
        return false;
    }
    // Assume object is yaw and pitch bearing and distance meters away from the vehicle
    Vector3f object_3D;
    object_3D.offset_bearing(wrap_180(yaw), (pitch * -1.0f), distance);
    const Matrix3f body_to_ned = AP::ahrs().get_rotation_body_to_ned();
    const Vector3f rotated_object_3D = body_to_ned * object_3D;

    float alt = FLT_MAX;
    if (!get_rangefinder_alt(alt)) {
        return false;
    }

    if (rotated_object_3D.z > -0.5f) {
        // obstacle is at the most 0.5 meters above vehicle
        if ((alt - _alt_min_m) < rotated_object_3D.z) {
            // obstacle is near or below ground
            return true;
        }
    }
#endif
    return false;
}


#endif // HAL_PROXIMITY_ENABLED
