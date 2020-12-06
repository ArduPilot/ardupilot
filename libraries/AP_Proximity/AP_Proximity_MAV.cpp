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

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_MAV.h"
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define PROXIMITY_MAV_TIMEOUT_MS    500 // distance messages must arrive within this many milliseconds
#define PROXIMITY_3D_MSG_TIMEOUT_MS  50  // boundary will be reset if OBSTACLE_DISTANCE_3D message does not arrive within this many milliseconds
// update the state of the sensor
void AP_Proximity_MAV::update(void)
{
    // check for timeout and set health status
    if ((_last_update_ms == 0 || (AP_HAL::millis() - _last_update_ms > PROXIMITY_MAV_TIMEOUT_MS)) &&
        (_last_upward_update_ms == 0 || (AP_HAL::millis() - _last_upward_update_ms > PROXIMITY_MAV_TIMEOUT_MS))) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// get distance upwards in meters. returns true on success
bool AP_Proximity_MAV::get_upward_distance(float &distance) const
{
    if ((_last_upward_update_ms != 0) && (AP_HAL::millis() - _last_upward_update_ms <= PROXIMITY_MAV_TIMEOUT_MS)) {
        distance = _distance_upward;
        return true;
    }
    return false;
}

// handle mavlink DISTANCE_SENSOR messages
void AP_Proximity_MAV::handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid == MAVLINK_MSG_ID_DISTANCE_SENSOR) {
        mavlink_distance_sensor_t packet;
        mavlink_msg_distance_sensor_decode(&msg, &packet);

        // store distance to appropriate sector based on orientation field
        if (packet.orientation <= MAV_SENSOR_ROTATION_YAW_315) {
            const uint8_t sector = packet.orientation;
            // create a boundary location object based on this sector
            const boundary_location bnd_loc{sector};
            // store in meters
            const uint16_t distance = packet.current_distance * 0.01f;
            _distance_min = packet.min_distance * 0.01f;
            _distance_max = packet.max_distance * 0.01f;
            
            // reset data on this sector, to be filled with new data
            boundary.reset_sector(bnd_loc);
            if (distance <= _distance_max && distance >= _distance_max) {
                boundary.set_attributes(bnd_loc, sector * 45, distance);
                // update OA database
                database_push(boundary.get_angle(bnd_loc), distance);
            }
            
            _last_update_ms = AP_HAL::millis();
            // update boundary used for Obstacle Avoidance
            boundary.update_boundary(bnd_loc);
        }

        // store upward distance
        if (packet.orientation == MAV_SENSOR_ROTATION_PITCH_90) {
            _distance_upward = packet.current_distance * 0.01f;
            _last_upward_update_ms = AP_HAL::millis();
        }
        return;
    }

    if (msg.msgid == MAVLINK_MSG_ID_OBSTACLE_DISTANCE) {
        mavlink_obstacle_distance_t packet;
        mavlink_msg_obstacle_distance_decode(&msg, &packet);

        // check increment (message's sector width)
        float increment;
        if (!is_zero(packet.increment_f)) {
            // use increment float
            increment = packet.increment_f;
        } else if (packet.increment != 0) {
            // use increment uint8_t
            increment = packet.increment;
        } else {
            // invalid increment
            return;
        }

        const uint8_t total_distances = MIN(((360.0f / fabsf(increment)) + 0.5f), MAVLINK_MSG_OBSTACLE_DISTANCE_FIELD_DISTANCES_LEN); // usually 72

        // set distance min and max
        _distance_min = packet.min_distance * 0.01f;
        _distance_max = packet.max_distance * 0.01f;
        _last_update_ms = AP_HAL::millis();

        // get user configured yaw correction from front end
        const float param_yaw_offset = constrain_float(frontend.get_yaw_correction(state.instance), -360.0f, +360.0f);
        const float yaw_correction = wrap_360(param_yaw_offset + packet.angle_offset);
        if (frontend.get_orientation(state.instance) != 0) {
            increment *= -1;
        }

        Vector3f current_pos;
        Matrix3f body_to_ned;
        const bool database_ready = database_prepare_for_push(current_pos, body_to_ned);

        // initialise updated array and proximity sector angles (to closest object) and distances
        bool sector_updated[PROXIMITY_NUM_SECTORS];
        memset(sector_updated, 0, sizeof(sector_updated));
        boundary.reset_all_horizontal_sectors();

        // iterate over message's sectors
        for (uint8_t j = 0; j < total_distances; j++) {
            const uint16_t distance_cm = packet.distances[j];
            if (distance_cm == 0 ||
                distance_cm == 65535 ||
                distance_cm < packet.min_distance ||
                distance_cm > packet.max_distance)
            {
                // sanity check failed, ignore this distance value
                continue;
            }

            const float packet_distance_m = distance_cm * 0.01f;
            const float mid_angle = wrap_360((float)j * increment + yaw_correction);

            // iterate over proximity sectors
            for (uint8_t i = 0; i < PROXIMITY_NUM_SECTORS; i++) {
                // update distance array sector with shortest distance from message
                const float angle_diff = wrap_180(boundary._sector_middle_deg[i] - mid_angle);
                if (fabsf(angle_diff) > PROXIMITY_SECTOR_WIDTH_DEG*0.5f) {
                    // not even in this sector
                    continue;
                }
                if (is_equal(angle_diff, -PROXIMITY_SECTOR_WIDTH_DEG*0.5f)) {
                    // on the upper boundary is *out* to avoid
                    // ambiguity.  The distance is considered to be in
                    // the next sector.  We should never be within an
                    // epsilon of the boundary, so is_equal should be
                    // safe.
                    continue;
                }
                if (packet_distance_m >= boundary.get_distance(i)) {
                    // this is no closer than a previous distance
                    // found from the packet
                    continue;
                }

                // this is the shortest distance we've found in the packet so far
                // create a location packet
                const boundary_location bnd_loc{i};
                boundary.set_attributes(bnd_loc, mid_angle, packet_distance_m);
                sector_updated[i] = true;
            }

            // update Object Avoidance database with Earth-frame point
            if (database_ready) {
                database_push(mid_angle, packet_distance_m, _last_update_ms, current_pos, body_to_ned);
            }
        }

        // update proximity sectors validity and boundary point
        for (uint8_t i = 0; i < PROXIMITY_NUM_SECTORS; i++) {
            if (sector_updated[i]) {
                const boundary_location bnd_loc{i};
                boundary.update_boundary(bnd_loc);
            }
        }
    }

    if (msg.msgid == MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D) {
        mavlink_obstacle_distance_3d_t packet;
        mavlink_msg_obstacle_distance_3d_decode(&msg, &packet);

        const uint32_t previous_sys_time = _last_update_ms;
        _last_update_ms = AP_HAL::millis();
        // time_diff will check if the new message arrived significantly later than the last message
        const uint32_t time_diff = _last_update_ms - previous_sys_time;

        const uint32_t previous_msg_timestamp = _last_3d_msg_update_ms;
        _last_3d_msg_update_ms = packet.time_boot_ms;
        bool clear_fence = false;
        
        // we will add on to the last fence if the time stamp is the same
        // provided we got the new obstacle in less than PROXIMITY_3D_MSG_TIMEOUT_MS  
        if ((previous_msg_timestamp != _last_3d_msg_update_ms) || (time_diff > PROXIMITY_3D_MSG_TIMEOUT_MS)) {
            clear_fence = true;
        }

        _distance_min = packet.min_distance;
        _distance_max = packet.max_distance;
        
        Vector3f current_pos;
        Matrix3f body_to_ned;
        const bool database_ready = database_prepare_for_push(current_pos, body_to_ned);

        if (clear_fence) {
            // cleared fence back to defaults since we have a new timestamp
            boundary.reset_all_sectors_and_stacks(); 
        }

        const Vector3f obstacle(packet.x, packet.y, packet.z);
        if (obstacle.length() < _distance_min || obstacle.length() > _distance_max || obstacle.is_zero()) {
            // message isn't healthy
            return;
        }
        // extract yaw and pitch from Obstacle Vector
        const float yaw = wrap_360(degrees(atan2f(obstacle.y, obstacle.x)));
        const float pitch = wrap_180(degrees(M_PI_2 - atan2f(norm(obstacle.x, obstacle.y), obstacle.z))); 

        // allot them correct stack and sector based on calculated pitch and yaw
        const boundary_location bnd_loc = boundary.get_sector(yaw, pitch);

        if (boundary.get_distance(bnd_loc) < obstacle.length()) {
            // we already have a shorter distance in this stack and sector
            return;
        }

        boundary.set_attributes(bnd_loc, yaw, pitch, obstacle.length());
        boundary.update_boundary(bnd_loc);

        if (database_ready) {
            database_push(yaw, obstacle.length(),_last_update_ms, current_pos, body_to_ned, pitch);
        }
    }
}
