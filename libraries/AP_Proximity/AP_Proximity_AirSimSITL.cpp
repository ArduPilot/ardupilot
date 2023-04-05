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

#if AP_PROXIMITY_AIRSIMSITL_ENABLED

#include "AP_Proximity_AirSimSITL.h"

#define PROXIMITY_MAX_RANGE 100.0f
#define PROXIMITY_ACCURACY  0.1f        // minimum distance (in meters) between objects sent to object database

// update the state of the sensor
void AP_Proximity_AirSimSITL::update(void)
{
    SITL::vector3f_array &points = sitl->state.scanner.points;
    if (points.length == 0) {
        set_status(AP_Proximity::Status::NoData);
        return;
    }

    set_status(AP_Proximity::Status::Good);

    // reset all faces to default so that it can be filled with the fresh lidar data
    frontend.boundary.reset();

    // precalculate sq of min distance
    const float distance_min_sq = sq(distance_min());

    // variables used to reduce data sent to object database
    const float accuracy_sq = sq(PROXIMITY_ACCURACY);
    bool prev_pos_valid = false;
    Vector2f prev_pos;
    // clear temp boundary since we have a new message
    temp_boundary.reset();

    for (uint16_t i=0; i<points.length; i++) {
        Vector3f &point = points.data[i];
        if (point.is_zero()) {
            continue;
        }

        // calculate distance to point and check larger than min distance
        const Vector2f new_pos = Vector2f{point.x, point.y};
        const float distance_sq = new_pos.length_squared();
        if (distance_sq > distance_min_sq) {

            // add distance to the 3D boundary
            const float yaw_angle_deg = wrap_360(degrees(atan2f(point.y, point.x)));
            const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(yaw_angle_deg);
            // store the min distance in each face in a temp boundary
            temp_boundary.add_distance(face, yaw_angle_deg, safe_sqrt(distance_sq));

            // check distance from previous point to reduce amount of data sent to object database
            if (!prev_pos_valid || ((new_pos - prev_pos).length_squared() >= accuracy_sq)) {
                // update OA database
                database_push(yaw_angle_deg, safe_sqrt(distance_sq));
                // store point
                prev_pos_valid = true;
                prev_pos = new_pos;
            }
        }
    }
    // copy temp boundary to real boundary
    temp_boundary.update_3D_boundary(state.instance, frontend.boundary);
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_AirSimSITL::distance_max() const
{
    return PROXIMITY_MAX_RANGE;
}

float AP_Proximity_AirSimSITL::distance_min() const
{
    return 0.0f;
}

// get distance upwards in meters. returns true on success
bool AP_Proximity_AirSimSITL::get_upward_distance(float &distance) const
{
    // we don't have an upward facing laser
    return false;
}

#endif // AP_PROXIMITY_AIRSIMSITL_ENABLED
