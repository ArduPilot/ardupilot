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

#include "AP_Proximity_MAV.h"

#if HAL_PROXIMITY_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define PROXIMITY_MAV_TIMEOUT_MS    500 // distance messages must arrive within this many milliseconds
#define PROXIMITY_TIMESTAMP_MSG_TIMEOUT_MS  50  // obstacles will be transferred from temp boundary to actual boundary if mavlink message does not arrive within this many milliseconds

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

// handle mavlink messages
void AP_Proximity_MAV::handle_msg(const mavlink_message_t &msg)
{   
    switch (msg.msgid) {
        case (MAVLINK_MSG_ID_DISTANCE_SENSOR):
            handle_distance_sensor_msg(msg);
            break;

        case (MAVLINK_MSG_ID_OBSTACLE_DISTANCE):
            handle_obstacle_distance_msg(msg);
            break;

        case (MAVLINK_MSG_ID_OBSTACLE_DISTANCE_3D):
            handle_obstacle_distance_3d_msg(msg);
            break;    
    }
}

// handle mavlink DISTANCE_SENSOR messages
void AP_Proximity_MAV::handle_distance_sensor_msg(const mavlink_message_t &msg)
{
    mavlink_distance_sensor_t packet;
    mavlink_msg_distance_sensor_decode(&msg, &packet);

    // store distance to appropriate sector based on orientation field
    if (packet.orientation <= MAV_SENSOR_ROTATION_YAW_315) {
        const uint32_t previous_sys_time = _last_update_ms;
        _last_update_ms = AP_HAL::millis();

        // time_diff will check if the new message arrived significantly later than the last message
        const uint32_t time_diff = _last_update_ms - previous_sys_time;

        const uint32_t previous_msg_timestamp = _last_msg_update_timestamp_ms;
        _last_msg_update_timestamp_ms = packet.time_boot_ms;

        // we will add on to the last fence if the time stamp is the same
        // provided we got the new obstacle in less than PROXIMITY_TIMESTAMP_MSG_TIMEOUT_MS
        if ((previous_msg_timestamp != _last_msg_update_timestamp_ms) || (time_diff > PROXIMITY_TIMESTAMP_MSG_TIMEOUT_MS)) {
            // push data from temp boundary to the main 3-D proximity boundary
            temp_boundary.update_3D_boundary(boundary);
            // clear temp boundary for new data
            temp_boundary.reset();
        }
        // store in meters
        const float distance = packet.current_distance * 0.01f;
        const uint8_t sector = packet.orientation;
        // get the face for this sector
        const float yaw_angle_deg = sector * 45;
        const AP_Proximity_Boundary_3D::Face face = boundary.get_face(yaw_angle_deg);
        _distance_min = packet.min_distance * 0.01f;
        _distance_max = packet.max_distance * 0.01f;
        const bool in_range = distance <= _distance_max && distance >= _distance_min;
        if (in_range && !ignore_reading(yaw_angle_deg, distance, false)) {
            temp_boundary.add_distance(face, yaw_angle_deg, distance);
            // update OA database
            database_push(yaw_angle_deg, distance);
        }
    }

    // store upward distance
    if (packet.orientation == MAV_SENSOR_ROTATION_PITCH_90) {
        _distance_upward = packet.current_distance * 0.01f;
        _last_upward_update_ms = AP_HAL::millis();
    }
    return;
}

// handle mavlink OBSTACLE_DISTANCE messages
void AP_Proximity_MAV::handle_obstacle_distance_msg(const mavlink_message_t &msg)
{
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

    uint64_t packet_time_usec = packet.time_usec;
    if (AP::rtc().convert_usec_to_system_boot_usec(packet_time_usec)) {
        _last_update_ms = (uint32_t) (packet_time_usec / 1000);
    } else {
        _last_update_ms = AP_HAL::millis();
    }

    // get user configured yaw correction from front end
    const float param_yaw_offset = constrain_float(frontend.get_yaw_correction(state.instance), -360.0f, +360.0f);
    const float yaw_correction = wrap_360(param_yaw_offset + packet.angle_offset);
    if (frontend.get_orientation(state.instance) != 0) {
        increment *= -1;
    }

    Vector3f current_pos;
    Matrix3f body_to_ned;
    const bool database_ready = database_prepare_for_push(current_pos, body_to_ned);

    // variables to calculate closest angle and distance for each face
    AP_Proximity_Boundary_3D::Face face;
    float face_distance = FLT_MAX;
    float face_yaw_deg = 0.0f;
    bool face_distance_valid = false;

    // reset this  boundary to fill with new data
    boundary.reset();

    // iterate over message's sectors
    for (uint8_t j = 0; j < total_distances; j++) {
        const uint16_t distance_cm = packet.distances[j];
        const float packet_distance_m = distance_cm * 0.01f;
        const float mid_angle = wrap_360((float)j * increment + yaw_correction);

        const bool range_check = distance_cm == 0 || distance_cm == 65535 || distance_cm < packet.min_distance ||
                                 distance_cm > packet.max_distance;
        if (range_check || ignore_reading(mid_angle, packet_distance_m, false)) {
            // sanity check failed, ignore this distance value
            continue;
        }

        // get face for this latest reading
        AP_Proximity_Boundary_3D::Face latest_face = boundary.get_face(mid_angle);
        if (latest_face != face) {
            // store previous face
            if (face_distance_valid) {
                boundary.set_face_attributes(face, face_yaw_deg, face_distance);
            } else {
                boundary.reset_face(face);
            }
            // init for latest face
            face = latest_face;
            face_distance_valid = false;
        }

        // update minimum distance found so far
        if (!face_distance_valid || (packet_distance_m < face_distance)) {
            face_yaw_deg = mid_angle;
            face_distance = packet_distance_m;
            face_distance_valid = true;
        }

        // update Object Avoidance database with Earth-frame point
        if (database_ready) {
            database_push(mid_angle, packet_distance_m, _last_update_ms, current_pos, body_to_ned);
        }
    }

    // process the last face
    if (face_distance_valid) {
        boundary.set_face_attributes(face, face_yaw_deg, face_distance);
    } else {
        boundary.reset_face(face);
    }
    return;
}

// handle mavlink OBSTACLE_DISTANCE_3D messages
void AP_Proximity_MAV::handle_obstacle_distance_3d_msg(const mavlink_message_t &msg)
{
    mavlink_obstacle_distance_3d_t packet;
    mavlink_msg_obstacle_distance_3d_decode(&msg, &packet);

    const uint32_t previous_sys_time = _last_update_ms;
    _last_update_ms = AP_HAL::millis();

    // time_diff will check if the new message arrived significantly later than the last message
    const uint32_t time_diff = _last_update_ms - previous_sys_time;

    const uint32_t previous_msg_timestamp = _last_msg_update_timestamp_ms;
    _last_msg_update_timestamp_ms = packet.time_boot_ms;

    if (packet.frame != MAV_FRAME_BODY_FRD) {
        // we do not support this frame of reference yet
        return;
    }

    if ((previous_msg_timestamp != _last_msg_update_timestamp_ms) || (time_diff > PROXIMITY_TIMESTAMP_MSG_TIMEOUT_MS)) {
        // push data from temp boundary to the main 3-D proximity boundary because a new timestamp has arrived
        temp_boundary.update_3D_boundary(boundary);
        // clear temp boundary for new data
        temp_boundary.reset();
    }

    _distance_min = packet.min_distance;
    _distance_max = packet.max_distance;

    Vector3f current_pos;
    Matrix3f body_to_ned;
    const bool database_ready = database_prepare_for_push(current_pos, body_to_ned);

    const Vector3f obstacle_FRD(packet.x, packet.y, packet.z);
    const float obstacle_distance = obstacle_FRD.length();
    if (obstacle_distance < _distance_min || obstacle_distance > _distance_max || is_zero(obstacle_distance)) {
        // message isn't healthy
        return;
    }

    // convert to FRU
    const Vector3f obstacle(obstacle_FRD.x, obstacle_FRD.y, obstacle_FRD.z * -1.0f);

    // extract yaw and pitch from Obstacle Vector
    const float yaw = wrap_360(degrees(atan2f(obstacle.y, obstacle.x)));
    const float pitch = wrap_180(degrees(M_PI_2 - atan2f(obstacle.xy().length(), obstacle.z))); 

    if (ignore_reading(pitch, yaw, obstacle_distance, false)) {
        // obstacle is probably near ground or out of range
        return;
    }

    // allot to correct layer and sector based on calculated pitch and yaw
    const AP_Proximity_Boundary_3D::Face face = boundary.get_face(pitch, yaw);
    temp_boundary.add_distance(face, pitch, yaw, obstacle.length());

    if (database_ready) {
        database_push(yaw, pitch, obstacle.length(),_last_update_ms, current_pos, body_to_ned);
    }
    return;
}

#endif // HAL_PROXIMITY_ENABLED
