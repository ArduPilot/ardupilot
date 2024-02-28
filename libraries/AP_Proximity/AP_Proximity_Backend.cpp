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

#include "AP_Proximity_Backend.h"

#if HAL_PROXIMITY_ENABLED
#include <AP_Common/Location.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AC_Avoidance/AP_OADatabase.h>

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_Proximity_Backend::AP_Proximity_Backend(AP_Proximity& _frontend, AP_Proximity::Proximity_State& _state, AP_Proximity_Params& _params) :
        frontend(_frontend),
        state(_state),
        params(_params)
{
    _backend_type = (AP_Proximity::Type )_params.type.get();
}

static_assert(PROXIMITY_MAX_DIRECTION <= 8,
              "get_horizontal_distances assumes 8-bits is enough for validity bitmask");

// set status and update valid count
void AP_Proximity_Backend::set_status(AP_Proximity::Status status)
{
    state.status = status;
}

// correct an angle (in degrees) based on the orientation and yaw correction parameters
float AP_Proximity_Backend::correct_angle_for_orientation(float angle_degrees) const
{
    const float angle_sign = (params.orientation == 1) ? -1.0f : 1.0f;
    return wrap_360(angle_degrees * angle_sign + params.yaw_correction);
}

// check if a reading should be ignored because it falls into an ignore area (check_for_ign_area should be sent as false if this check is not needed)
// pitch is the vertical body-frame angle (in degrees) to the obstacle (0=directly ahead, 90 is above the vehicle)
// yaw is the horizontal body-frame angle (in degrees) to the obstacle (0=directly ahead of the vehicle, 90 is to the right of the vehicle)
// Also checks if obstacle is near land or out of range
// angles should be in degrees and in the range of 0 to 360, distance should be in meteres
bool AP_Proximity_Backend::ignore_reading(float pitch, float yaw, float distance_m, bool check_for_ign_area) const
{
    // check if distances are supposed to be in a particular range
    if (!is_zero(params.max_m)) {
        if (distance_m > params.max_m) {
            // too far away
            return true;
        }
    }

    if (!is_zero(params.min_m)) {
        if (distance_m < params.min_m) {
            // too close
            return true;
        }
    }

    if (check_for_ign_area) {
        // check angle vs each ignore area
        for (uint8_t i=0; i < PROXIMITY_MAX_IGNORE; i++) {
            if (params.ignore_width_deg[i] != 0) {
                if (abs(yaw - params.ignore_angle_deg[i]) <= (params.ignore_width_deg[i]/2)) {
                    return true;
                }
            }
        }
    }

   // check if obstacle is near land
   return frontend.check_obstacle_near_ground(pitch, yaw, distance_m);
}

// returns true if database is ready to be pushed to and all cached data is ready
bool AP_Proximity_Backend::database_prepare_for_push(Vector3f &current_pos, Matrix3f &body_to_ned)
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_Periph)
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return false;
    }

    if (!AP::ahrs().get_relative_position_NED_origin(current_pos)) {
        return false;
    }

    body_to_ned = AP::ahrs().get_rotation_body_to_ned();

    return true;
#else
    return false;
#endif
}

// update Object Avoidance database with Earth-frame point
void AP_Proximity_Backend::database_push(float angle, float pitch, float distance)
{
    Vector3f current_pos;
    Matrix3f body_to_ned;

    if (database_prepare_for_push(current_pos, body_to_ned)) {
        database_push(angle, pitch, distance, AP_HAL::millis(), current_pos, body_to_ned);
    }
}

// update Object Avoidance database with Earth-frame point
// pitch can be optionally provided if needed
void AP_Proximity_Backend::database_push(float angle, float pitch, float distance, uint32_t timestamp_ms, const Vector3f &current_pos, const Matrix3f &body_to_ned)
{

#if !APM_BUILD_TYPE(APM_BUILD_AP_Periph)
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return;
    }
    if ((pitch > 90.0f) || (pitch < -90.0f)) {
        // sanity check on pitch
        return;
    }
    //Assume object is angle and pitch bearing and distance meters away from the vehicle 
    Vector3f object_3D;
    object_3D.offset_bearing(wrap_180(angle), (pitch * -1.0f), distance);
    const Vector3f rotated_object_3D = body_to_ned * object_3D;

    //Calculate the position vector from origin
    Vector3f temp_pos = current_pos + rotated_object_3D;
    //Convert the vector to a NEU frame from NED
    temp_pos.z = temp_pos.z * -1.0f;

    oaDb->queue_push(temp_pos, timestamp_ms, distance);
#endif
}

#endif // HAL_PROXIMITY_ENABLED
