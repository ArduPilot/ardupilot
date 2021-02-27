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

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AC_Avoidance/AP_OADatabase.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_Proximity_Backend::AP_Proximity_Backend(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state) :
        frontend(_frontend),
        state(_state)
{
}

static_assert(PROXIMITY_MAX_DIRECTION <= 8,
              "get_horizontal_distances assumes 8-bits is enough for validity bitmask");

// get distances in PROXIMITY_MAX_DIRECTION directions horizontally. used for sending distances to ground station
bool AP_Proximity_Backend::get_horizontal_distances(AP_Proximity::Proximity_Distance_Array &prx_dist_array) const
{
    AP_Proximity::Proximity_Distance_Array prx_filt_dist_array; // unused
    return boundary.get_layer_distances(PROXIMITY_MIDDLE_LAYER, distance_max(), prx_dist_array, prx_filt_dist_array);
}
// get distances in PROXIMITY_MAX_DIRECTION directions at a layer. used for logging
bool AP_Proximity_Backend::get_active_layer_distances(uint8_t layer, AP_Proximity::Proximity_Distance_Array &prx_dist_array, AP_Proximity::Proximity_Distance_Array &prx_filt_dist_array) const
{
    return boundary.get_layer_distances(layer, distance_max(), prx_dist_array, prx_filt_dist_array);
}

// set status and update valid count
void AP_Proximity_Backend::set_status(AP_Proximity::Status status)
{
    state.status = status;
}

// correct an angle (in degrees) based on the orientation and yaw correction parameters
float AP_Proximity_Backend::correct_angle_for_orientation(float angle_degrees) const
{
    const float angle_sign = (frontend.get_orientation(state.instance) == 1) ? -1.0f : 1.0f;
    return wrap_360(angle_degrees * angle_sign + frontend.get_yaw_correction(state.instance));
}

// check if a reading should be ignored because it falls into an ignore area
bool AP_Proximity_Backend::ignore_reading(uint16_t angle_deg) const
{
    // check angle vs each ignore area
    for (uint8_t i=0; i < PROXIMITY_MAX_IGNORE; i++) {
        if (frontend._ignore_width_deg[i] != 0) {
            if (abs(angle_deg - frontend._ignore_angle_deg[i]) <= (frontend._ignore_width_deg[i]/2)) {
                return true;
            }
        }
    }
    return false;
}

// returns true if database is ready to be pushed to and all cached data is ready
bool AP_Proximity_Backend::database_prepare_for_push(Vector3f &current_pos, Matrix3f &body_to_ned)
{
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return false;
    }

    if (!AP::ahrs().get_relative_position_NED_origin(current_pos)) {
        return false;
    }

    body_to_ned = AP::ahrs().get_rotation_body_to_ned();

    return true;
}

// update Object Avoidance database with Earth-frame point
void AP_Proximity_Backend::database_push(float angle, float distance)
{
    Vector3f current_pos;
    Matrix3f body_to_ned;

    if (database_prepare_for_push(current_pos, body_to_ned)) {
        database_push(angle, distance, AP_HAL::millis(), current_pos, body_to_ned);
    }
}

// update Object Avoidance database with Earth-frame point
// pitch can be optionally provided if needed
void AP_Proximity_Backend::database_push(float angle, float pitch, float distance, uint32_t timestamp_ms, const Vector3f &current_pos, const Matrix3f &body_to_ned)
{
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return;
    }

    //Assume object is angle and pitch bearing and distance meters away from the vehicle 
    Vector3f object_3D;
    object_3D.offset_bearing(wrap_180(angle), wrap_180(pitch * -1.0f), distance);
    const Vector3f rotated_object_3D = body_to_ned * object_3D;

    //Calculate the position vector from origin
    Vector3f temp_pos = current_pos + rotated_object_3D;
    //Convert the vector to a NEU frame from NED
    temp_pos.z = temp_pos.z * -1.0f;

    oaDb->queue_push(temp_pos, timestamp_ms, distance);
}
