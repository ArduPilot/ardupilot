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
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AC_Avoidance/AP_OADatabase.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_Proximity_Backend::AP_Proximity_Backend(AP_Proximity &_frontend,
                                           AP_Proximity::Proximity_State &_state,
                                           AP_Proximity_Boundary_3D &_boundary,
                                           AP_Proximity_Utils &_utility) :
        frontend(_frontend),
        state(_state),
        boundary(_boundary),
        utility(_utility)
{
}

static_assert(PROXIMITY_MAX_DIRECTION <= 8,
              "get_horizontal_distances assumes 8-bits is enough for validity bitmask");

// get distances in PROXIMITY_MAX_DIRECTION directions horizontally. used for sending distances to ground station
bool AP_Proximity_Backend::get_horizontal_distances(AP_Proximity_Boundary_3D::Proximity_Distance_Array_2D &prx_dist_array) const
{
    AP_Proximity_Boundary_3D::Proximity_Distance_Array_2D prx_filt_dist_array; // unused
    return boundary.get_layer_distances(PROXIMITY_MIDDLE_LAYER, distance_max(), prx_dist_array, prx_filt_dist_array);
}
// // get distances in PROXIMITY_MAX_DIRECTION directions at a layer. used for logging
bool AP_Proximity_Backend::get_active_layer_distances(uint8_t layer, AP_Proximity_Boundary_3D::Proximity_Distance_Array_2D &prx_dist_array, AP_Proximity_Boundary_3D::Proximity_Distance_Array_2D &prx_filt_dist_array) const
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

// check if a reading should be ignored because it falls into an ignore area (check_for_ign_area should be sent as false if this check is not needed)
// pitch is the vertical body-frame angle (in degrees) to the obstacle (0=directly ahead, 90 is above the vehicle)
// yaw is the horizontal body-frame angle (in degrees) to the obstacle (0=directly ahead of the vehicle, 90 is to the right of the vehicle)
// Also checks if obstacle is near land or out of range
// angles should be in degrees and in the range of 0 to 360, distance should be in meteres
bool AP_Proximity_Backend::ignore_reading(float pitch, float yaw, float distance_m, bool check_for_ign_area) const
{
    if (check_for_ign_area) {
        // check angle vs each ignore area
        for (uint8_t i=0; i < PROXIMITY_MAX_IGNORE; i++) {
            if (frontend._ignore_width_deg[i] != 0) {
                if (abs(yaw - frontend._ignore_angle_deg[i]) <= (frontend._ignore_width_deg[i]/2)) {
                    return true;
                }
            }
        }
    }

    return utility.ignore_reading(pitch, yaw, distance_m, check_for_ign_area, frontend._max_m, frontend._min_m);
}

#endif // HAL_PROXIMITY_ENABLED
