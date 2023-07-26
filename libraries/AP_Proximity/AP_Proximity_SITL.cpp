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

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_SITL_ENABLED

#include "AP_Proximity_SITL.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AC_Fence/AC_Fence.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define PROXIMITY_MAX_RANGE 200.0f
#define PROXIMITY_ACCURACY 0.1f

/* 
   The constructor also initialises the proximity sensor. 
*/
AP_Proximity_SITL::AP_Proximity_SITL(AP_Proximity &_frontend,
                                     AP_Proximity::Proximity_State &_state,
                                     AP_Proximity_Params& _params):
    AP_Proximity_Backend(_frontend, _state, _params),
    sitl(AP::sitl())
{
    ap_var_type ptype;
    fence_alt_max = (AP_Float *)AP_Param::find("FENCE_ALT_MAX", &ptype);
    if (fence_alt_max == nullptr || ptype != AP_PARAM_FLOAT) {
        AP_HAL::panic("Proximity_SITL: Failed to find FENCE_ALT_MAX");
    }
}

// update the state of the sensor
void AP_Proximity_SITL::update(void)
{
    current_loc.lat = sitl->state.latitude * 1.0e7;
    current_loc.lng = sitl->state.longitude * 1.0e7;
    current_loc.alt = sitl->state.altitude * 1.0e2;

#if AP_FENCE_ENABLED
    if (!AP::fence()->polyfence().breached()) {
        // only called to prompt polyfence to reload fence if required
    }
    if (AP::fence()->polyfence().inclusion_boundary_available()) {
        set_status(AP_Proximity::Status::Good);
        // update distance in each sector
        for (uint8_t sector=0; sector < PROXIMITY_NUM_SECTORS; sector++) {
            const float yaw_angle_deg = sector * 45.0f;
            AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(yaw_angle_deg);
            float fence_distance;
            if (get_distance_to_fence(yaw_angle_deg, fence_distance)) {
                frontend.boundary.set_face_attributes(face, yaw_angle_deg, fence_distance, state.instance);
                // update OA database
                database_push(yaw_angle_deg, fence_distance);
            } else {
                frontend.boundary.reset_face(face, state.instance);
            }
        }
    } else {
        set_status(AP_Proximity::Status::NoData);
    }
#else
    set_status(AP_Proximity::Status::NoData);
#endif
}

// get distance in meters to fence in a particular direction in degrees (0 is forward, angles increase in the clockwise direction)
bool AP_Proximity_SITL::get_distance_to_fence(float angle_deg, float &distance) const
{
#if AP_FENCE_ENABLED
    if (!AP::fence()->polyfence().inclusion_boundary_available()) {
        return false;
    }

    // convert to earth frame
    angle_deg = wrap_360(sitl->state.yawDeg + angle_deg);

    /*
      simple bisection search to find distance. Not really efficient,
      but we can afford the CPU in SITL
     */
    float min_dist = 0, max_dist = PROXIMITY_MAX_RANGE;
    while (max_dist - min_dist > PROXIMITY_ACCURACY) {
        float test_dist = (max_dist+min_dist)*0.5f;

        Location loc = current_loc;
        loc.offset_bearing(angle_deg, test_dist);
        if (AP::fence()->polyfence().breached(loc)) {
            max_dist = test_dist;
        } else {
            min_dist = test_dist;
        }
    }
    distance = min_dist;
    if (ignore_reading(angle_deg, distance, false)) {
        // obstacle near land, lets ignore it
        return false;
    }
    return true;
#else
    return false;
#endif
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_SITL::distance_max() const
{
    return PROXIMITY_MAX_RANGE;
}
float AP_Proximity_SITL::distance_min() const
{
    return 0.0f;
}

// get distance upwards in meters. returns true on success
bool AP_Proximity_SITL::get_upward_distance(float &distance) const
{
    // return distance to fence altitude
    distance = MAX(0.0f, fence_alt_max->get() - sitl->height_agl);
    return true;
}

#endif // AP_PROXIMITY_SITL_ENABLED
