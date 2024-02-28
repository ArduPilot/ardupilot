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
/*
  simple vicon simulator class

  XKFR

*/

#include "SIM_Vicon.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

Vicon::Vicon() :
    SerialDevice::SerialDevice()
{
}

void Vicon::maybe_send_heartbeat()
{
    const uint32_t now = AP_HAL::millis();

    if (now - last_heartbeat_ms < 100) {
        // we only provide a heartbeat every so often
        return;
    }
    last_heartbeat_ms = now;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(system_id,
                               component_id,
                               &msg,
                               MAV_TYPE_GCS,
                               MAV_AUTOPILOT_INVALID,
                               0,
                               0,
                               0);
}

// get unused index in msg_buf
bool Vicon::get_free_msg_buf_index(uint8_t &index)
{
    for (uint8_t i=0; i<ARRAY_SIZE(msg_buf); i++) {
        if (msg_buf[i].time_send_us == 0) {
            index = i;
            return true;
        }
    }
    return false;
}

void Vicon::update_vicon_position_estimate(const Location &loc,
                                           const Vector3d &position,
                                           const Vector3f &velocity,
                                           const Quaternion &attitude)
{
    const uint64_t now_us = AP_HAL::micros64();

    // calculate a random time offset to the time sent in the message
    // simulates a time difference between the remote computer and autopilot
    if (time_offset_us == 0) {
        time_offset_us = (unsigned(random()) % 7000) * 1000000ULL;
        printf("time_offset_us %llu\n", (long long unsigned)time_offset_us);
    }

    // send all messages in the buffer
    bool waiting_to_send = false;
    for (uint8_t i=0; i<ARRAY_SIZE(msg_buf); i++) {
        if ((msg_buf[i].time_send_us > 0) && (now_us >= msg_buf[i].time_send_us)) {
            uint8_t buf[300];
            uint16_t buf_len = mavlink_msg_to_send_buffer(buf, &msg_buf[i].obs_msg);

            if (write_to_autopilot((char*)&buf, buf_len) != buf_len) {
                hal.console->printf("Vicon: write failure\n");
            }
            msg_buf[i].time_send_us = 0;
        }
        waiting_to_send = msg_buf[i].time_send_us != 0;
    }
    if (waiting_to_send) {
        // waiting for the last msg to go out
        return;
    }

    if (now_us - last_observation_usec < 20000) {
        // create observations at 20ms intervals (matches EKF max rate)
        return;
    }

    // failure simulation
    if (_sitl->vicon_fail.get() != 0) {
        return;
    }

    float roll;
    float pitch;
    float yaw;
    attitude.to_euler(roll, pitch, yaw);

    // calculate sensor offset in earth frame
    const Vector3f& pos_offset = _sitl->vicon_pos_offset.get();
    Matrix3f rot;
    rot.from_euler(radians(_sitl->state.rollDeg), radians(_sitl->state.pitchDeg), radians(_sitl->state.yawDeg));
    Vector3f pos_offset_ef = rot * pos_offset;

    // add earth frame sensor offset and glitch to position
    Vector3d pos_corrected = position + (pos_offset_ef + _sitl->vicon_glitch.get()).todouble();

    // calculate a velocity offset due to the antenna position offset and body rotation rate
    // note: % operator is overloaded for cross product
    Vector3f gyro(radians(_sitl->state.rollRate),
                  radians(_sitl->state.pitchRate),
                  radians(_sitl->state.yawRate));
    Vector3f vel_rel_offset_bf = gyro % pos_offset;

    // rotate the velocity offset into earth frame and add to the c.g. velocity
    Vector3f vel_rel_offset_ef = rot * vel_rel_offset_bf;
    Vector3f vel_corrected = velocity + vel_rel_offset_ef + _sitl->vicon_vel_glitch.get();

    // adjust yaw, position and velocity to account for vicon's yaw
    const int16_t vicon_yaw_deg = _sitl->vicon_yaw.get();
    if (vicon_yaw_deg != 0) {
        const float vicon_yaw_rad = radians(vicon_yaw_deg);
        yaw = wrap_PI(yaw - vicon_yaw_rad);
        Matrix3d vicon_yaw_rot;
        vicon_yaw_rot.from_euler(0, 0, -vicon_yaw_rad);
        pos_corrected = vicon_yaw_rot * pos_corrected;
        vel_corrected = vicon_yaw_rot.tofloat() * vel_corrected;
    }

    // add yaw error reported to vehicle
    yaw = wrap_PI(yaw + radians(_sitl->vicon_yaw_error.get()));

    // 25ms to 124ms delay before sending
    uint32_t delay_ms = 25 + unsigned(random()) % 100;
    uint64_t time_send_us = now_us + delay_ms * 1000UL;

    // send vision position estimate message
    uint8_t msg_buf_index;
    if (should_send(ViconTypeMask::VISION_POSITION_ESTIMATE) && get_free_msg_buf_index(msg_buf_index)) {
        const mavlink_vision_position_estimate_t vision_position_estimate{
            now_us + time_offset_us,
            float(pos_corrected.x),
            float(pos_corrected.y),
            float(pos_corrected.z),
            roll,
            pitch,
            yaw
        };
        mavlink_msg_vision_position_estimate_encode_status(
            system_id,
            component_id,
            &mav_status,
            &msg_buf[msg_buf_index].obs_msg,
            &vision_position_estimate
        );
        msg_buf[msg_buf_index].time_send_us = time_send_us;
    }

    // send older vicon position estimate message
    if (should_send(ViconTypeMask::VICON_POSITION_ESTIMATE) && get_free_msg_buf_index(msg_buf_index)) {
        const mavlink_vicon_position_estimate_t vicon_position_estimate{
            now_us + time_offset_us,
            float(pos_corrected.x),
            float(pos_corrected.y),
            float(pos_corrected.z),
            roll,
            pitch,
            yaw
        };
        mavlink_msg_vicon_position_estimate_encode_status(
            system_id,
            component_id,
            &mav_status,
            &msg_buf[msg_buf_index].obs_msg,
            &vicon_position_estimate);
        msg_buf[msg_buf_index].time_send_us = time_send_us;
    }

    // send vision speed estimate
    if (should_send(ViconTypeMask::VISION_SPEED_ESTIMATE) && get_free_msg_buf_index(msg_buf_index)) {
        const mavlink_vision_speed_estimate_t vicon_speed_estimate{
            now_us + time_offset_us,
            vel_corrected.x,
            vel_corrected.y,
            vel_corrected.z
        };
        mavlink_msg_vision_speed_estimate_encode_status(
            system_id,
            component_id,
            &mav_status,
            &msg_buf[msg_buf_index].obs_msg,
            &vicon_speed_estimate
            );
        msg_buf[msg_buf_index].time_send_us = time_send_us;
    }


    // send ODOMETRY message
    if (should_send(ViconTypeMask::ODOMETRY) && get_free_msg_buf_index(msg_buf_index)) {
        const Vector3f vel_corrected_frd = attitude.inverse() * vel_corrected;
        const mavlink_odometry_t odometry{
            now_us + time_offset_us,
            float(pos_corrected.x),
            float(pos_corrected.y),
            float(pos_corrected.z),
            {attitude[0], attitude[1], attitude[2], attitude[3]},
            vel_corrected_frd.x,
            vel_corrected_frd.y,
            vel_corrected_frd.z,
            gyro.x,
            gyro.y,
            gyro.z,
            {},
            {},
            MAV_FRAME_LOCAL_FRD,
            MAV_FRAME_BODY_FRD,
            0,
            MAV_ESTIMATOR_TYPE_VIO,
            50 // quality hardcoded to 50%
        };
        mavlink_msg_odometry_encode_status(
            system_id,
            component_id,
            &mav_status,
            &msg_buf[msg_buf_index].obs_msg,
            &odometry);
        msg_buf[msg_buf_index].time_send_us = time_send_us;
    }

    // determine time, position, and angular deltas
    uint64_t time_delta = now_us - last_observation_usec;

    Quaternion attitude_curr;                   // Rotation to current MAV_FRAME_BODY_FRD from MAV_FRAME_LOCAL_NED
    attitude_curr.from_euler(roll, pitch, yaw); // Rotation to MAV_FRAME_LOCAL_NED from current MAV_FRAME_BODY_FRD
    attitude_curr.invert();

    Quaternion attitude_curr_prev = attitude_curr * _attitude_prev.inverse(); // Get rotation to current MAV_FRAME_BODY_FRD from previous MAV_FRAME_BODY_FRD

    Matrix3f body_ned_m;
    attitude_curr.rotation_matrix(body_ned_m);

    Vector3f pos_delta = body_ned_m * (pos_corrected - _position_prev).tofloat();

    // send vision position delta
    // time_usec: (usec) Current time stamp
    // time_delta_usec: (usec) Time since last reported camera frame
    // angle_delta [3]: (radians) Roll, pitch, yaw angles that define rotation to current MAV_FRAME_BODY_FRD from previous MAV_FRAME_BODY_FRD
    // delta_position [3]: (meters) Change in position: To current position from previous position rotated to current MAV_FRAME_BODY_FRD from MAV_FRAME_LOCAL_NED
    // confidence: Normalized confidence level [0, 100]
    if (should_send(ViconTypeMask::VISION_POSITION_DELTA) && get_free_msg_buf_index(msg_buf_index)) {
        const mavlink_vision_position_delta_t vision_position_delta{
            now_us + time_offset_us,
            time_delta,
            { attitude_curr_prev.get_euler_roll(),
              attitude_curr_prev.get_euler_pitch(),
              attitude_curr_prev.get_euler_yaw()
            },
            {pos_delta.x, pos_delta.y, pos_delta.z}
        };
        mavlink_msg_vision_position_delta_encode_status(
            system_id,
            component_id,
            &mav_status,
            &msg_buf[msg_buf_index].obs_msg,
            &vision_position_delta);
        msg_buf[msg_buf_index].time_send_us = time_send_us;
    }

    // set previous position & attitude
    last_observation_usec = now_us;
    _position_prev = pos_corrected;
    _attitude_prev = attitude_curr;
}

/*
  update vicon sensor state
 */
void Vicon::update(const Location &loc, const Vector3d &position, const Vector3f &velocity, const Quaternion &attitude)
{
    if (!init_sitl_pointer()) {
        return;
    }

    maybe_send_heartbeat();
    update_vicon_position_estimate(loc, position, velocity, attitude);
}
