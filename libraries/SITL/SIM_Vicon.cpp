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
*/

#include "SIM_config.h"

#if AP_SIM_VICON_ENABLED

#include "SIM_Vicon.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

const AP_Param::GroupInfo SIM::ViconParms::var_info[] = {
    // @Param: POS_X
    // @DisplayName: SITL vicon position on vehicle in Forward direction
    // @Description: SITL vicon position on vehicle in Forward direction
    // @Units: m
    // @Range: 0 10
    // @User: Advanced

    // @Param: POS_Y
    // @DisplayName: SITL vicon position on vehicle in Right direction
    // @Description: SITL vicon position on vehicle in Right direction
    // @Units: m
    // @Range: 0 10
    // @User: Advanced

    // @Param: POS_Z
    // @DisplayName: SITL vicon position on vehicle in Down direction
    // @Description: SITL vicon position on vehicle in Down direction
    // @Units: m
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("POS",     1, ViconParms,  pos_offset, 0),

    // @Param: GLIT_X
    // @DisplayName: SITL vicon position glitch North
    // @Description: SITL vicon position glitch North
    // @Units: m
    // @User: Advanced

    // @Param: GLIT_Y
    // @DisplayName: SITL vicon position glitch East
    // @Description: SITL vicon position glitch East
    // @Units: m
    // @User: Advanced

    // @Param: GLIT_Z
    // @DisplayName: SITL vicon position glitch Down
    // @Description: SITL vicon position glitch Down
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("GLIT",    2, ViconParms,  glitch, 0),

    // @Param: FAIL
    // @DisplayName: SITL vicon failure
    // @Description: SITL vicon failure
    // @Values: 0:Vicon Healthy, 1:Vicon Failed
    // @User: Advanced
    AP_GROUPINFO("FAIL",    3, ViconParms,  fail, 0),

    // @Param: YAW
    // @DisplayName: SITL vicon yaw angle in earth frame
    // @Description: SITL vicon yaw angle in earth frame
    // @Units: deg
    // @Range: 0 360
    // @User: Advanced
    AP_GROUPINFO("YAW",     4, ViconParms,  yaw, 0),

    // @Param: YAWERR
    // @DisplayName: SITL vicon yaw error
    // @Description: SITL vicon yaw added to reported yaw sent to vehicle
    // @Units: deg
    // @Range: -180 180
    // @User: Advanced
    AP_GROUPINFO("YAWERR",  5, ViconParms,  yaw_error, 0),

    // @Param: TMASK
    // @DisplayName: SITL vicon type mask
    // @Description: SITL vicon messages sent
    // @Bitmask: 0:VISION_POSITION_ESTIMATE, 1:VISION_SPEED_ESTIMATE, 2:VICON_POSITION_ESTIMATE, 3:VISION_POSITION_DELTA, 4:ODOMETRY
    // @User: Advanced
    AP_GROUPINFO("TMASK",   6, ViconParms,  type_mask, 3),

    // @Param: VGLI_X
    // @DisplayName: SITL vicon velocity glitch North
    // @Description: SITL vicon velocity glitch North
    // @Units: m/s
    // @User: Advanced

    // @Param: VGLI_Y
    // @DisplayName: SITL vicon velocity glitch East
    // @Description: SITL vicon velocity glitch East
    // @Units: m/s
    // @User: Advanced

    // @Param: VGLI_Z
    // @DisplayName: SITL vicon velocity glitch Down
    // @Description: SITL vicon velocity glitch Down
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("VGLI",    7, ViconParms,  vel_glitch, 0),

    // @Param: P_SD
    // @DisplayName: SITL vicon position standard deviation for gaussian noise
    // @Description: SITL vicon position standard deviation for gaussian noise
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("P_SD",  8, ViconParms,  pos_stddev, 0.0f),

    // @Param: V_SD
    // @DisplayName: SITL vicon velocity standard deviation for gaussian noise
    // @Description: SITL vicon velocity standard deviation for gaussian noise
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("V_SD",  9, ViconParms,  vel_stddev, 0.0f),

    // @Param: RATE
    // @DisplayName: SITL vicon rate
    // @Description: SITL vicon rate
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("RATE",  10, ViconParms,  rate_hz, 50),

    AP_GROUPEND
};

Vicon::Vicon() :
    SerialDevice::SerialDevice()
{
}

void Vicon::maybe_send_heartbeat()
{
    const uint32_t now = AP_HAL::millis();

    if (now - last_heartbeat_ms < 500) {
        // we only provide a heartbeat every so often
        return;
    }

    uint8_t msg_buf_index;
    if (!get_free_msg_buf_index(msg_buf_index)) {
        return;
    }

    last_heartbeat_ms = now;

    const mavlink_heartbeat_t heartbeat{
        custom_mode: 0,
        type : MAV_TYPE_GCS,
        autopilot : MAV_AUTOPILOT_INVALID,
        base_mode: 0,
        system_status: 0,
        mavlink_version: 0,
    };

    mavlink_msg_heartbeat_encode_status(
        system_id,
        component_id,
        &mav_status,
        &msg_buf[msg_buf_index].obs_msg,
        &heartbeat
    );
    msg_buf[msg_buf_index].time_send_us = AP_HAL::millis();
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

    if (_sitl->vicon.rate_hz == 0) {
        return;
    }
    const uint64_t vicon_interval_us = 1000000UL / _sitl->vicon.rate_hz;  // Interval in microseconds based on rate
    if (now_us - last_observation_usec < vicon_interval_us) {
        // create observations at rate specified by vicon_rate_hz
        // by default runs at 50Hz
        return;
    }

    // failure simulation
    if (_sitl->vicon.fail.get() != 0) {
        return;
    }

    float roll;
    float pitch;
    float yaw;
    attitude.to_euler(roll, pitch, yaw);

    // calculate sensor offset in earth frame
    const Vector3f& pos_offset = _sitl->vicon.pos_offset.get();
    Matrix3f rot;
    rot.from_euler(radians(_sitl->state.rollDeg), radians(_sitl->state.pitchDeg), radians(_sitl->state.yawDeg));
    Vector3f pos_offset_ef = rot * pos_offset;

    // add earth frame sensor offset and glitch to position
    Vector3d pos_corrected = position + (pos_offset_ef + _sitl->vicon.glitch.get()).todouble();
    // add some gaussian noise to the position
    pos_corrected += Vector3d(
        Aircraft::rand_normal(0, _sitl->vicon.pos_stddev.get()),
        Aircraft::rand_normal(0, _sitl->vicon.pos_stddev.get()),
        Aircraft::rand_normal(0, _sitl->vicon.pos_stddev.get())
    );

    // calculate a velocity offset due to the antenna position offset and body rotation rate
    // note: % operator is overloaded for cross product
    Vector3f gyro(radians(_sitl->state.rollRate),
                  radians(_sitl->state.pitchRate),
                  radians(_sitl->state.yawRate));
    Vector3f vel_rel_offset_bf = gyro % pos_offset;

    // rotate the velocity offset into earth frame and add to the c.g. velocity
    Vector3f vel_rel_offset_ef = rot * vel_rel_offset_bf;
    Vector3f vel_corrected = velocity + vel_rel_offset_ef + _sitl->vicon.vel_glitch.get();

    // adjust yaw, position and velocity to account for vicon's yaw
    const int16_t vicon_yaw_deg = _sitl->vicon.yaw.get();
    if (vicon_yaw_deg != 0) {
        const float vicon_yaw_rad = radians(vicon_yaw_deg);
        yaw = wrap_PI(yaw - vicon_yaw_rad);
        Matrix3d vicon_yaw_rot;
        vicon_yaw_rot.from_euler(0, 0, -vicon_yaw_rad);
        pos_corrected = vicon_yaw_rot * pos_corrected;
        vel_corrected = vicon_yaw_rot.tofloat() * vel_corrected;
    }

    // add some gaussian noise to the velocity
    vel_corrected += Vector3f(
        Aircraft::rand_normal(0, _sitl->vicon.vel_stddev.get()),
        Aircraft::rand_normal(0, _sitl->vicon.vel_stddev.get()),
        Aircraft::rand_normal(0, _sitl->vicon.vel_stddev.get())
    );

    // add yaw error reported to vehicle
    yaw = wrap_PI(yaw + radians(_sitl->vicon.yaw_error.get()));

    // 25ms to 124ms delay before sending
    uint32_t delay_ms = 25 + unsigned(random()) % 100;
    uint64_t time_send_us = now_us + delay_ms * 1000UL;


    float pose_cov[21];
    memset(pose_cov, 0, sizeof(pose_cov));
    // Set variances (diagonal elements), assume no cross-correlation. TODO: figure out attitude variances
    const float pos_variance = _sitl->vicon.pos_stddev*_sitl->vicon.pos_stddev;
    pose_cov[0]  = pos_variance;   // x
    pose_cov[6]  = pos_variance;   // y
    pose_cov[11] = pos_variance;   // z

    // send vision position estimate message
    uint8_t msg_buf_index;
    if (should_send(ViconTypeMask::VISION_POSITION_ESTIMATE) && get_free_msg_buf_index(msg_buf_index)) {
        mavlink_vision_position_estimate_t vision_position_estimate{
        usec: now_us + time_offset_us,
        x: float(pos_corrected.x),
        y: float(pos_corrected.y),
        z: float(pos_corrected.z),
        roll: roll,
        pitch: pitch,
        yaw: yaw
        };
        memcpy(vision_position_estimate.covariance, pose_cov, sizeof(pose_cov));

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
        usec: now_us + time_offset_us,
        x: float(pos_corrected.x),
        y: float(pos_corrected.y),
        z: float(pos_corrected.z),
        roll: roll,
        pitch: pitch,
        yaw: yaw
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
        float cov[9];
        memset(cov, 0, sizeof(cov));
        // Set variances (diagonal elements), assume no cross-correlation
        const float vel_variance = _sitl->vicon.vel_stddev*_sitl->vicon.vel_stddev;
        cov[0] = vel_variance;   // x
        cov[4] = vel_variance;   // y
        cov[8] = vel_variance;   // z
        mavlink_vision_speed_estimate_t vicon_speed_estimate{
        usec: now_us + time_offset_us,
        x: vel_corrected.x,
        y: vel_corrected.y,
        z: vel_corrected.z
        };
        memcpy(vicon_speed_estimate.covariance, cov, sizeof(cov));

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
        float vel_cov[21];
        memset(vel_cov, 0, sizeof(vel_cov));
        // Set variances (diagonal elements), assume no cross-correlation. TODO: figure out angular velocity variances
        const float vel_variance = _sitl->vicon.vel_stddev*_sitl->vicon.vel_stddev;
        vel_cov[0] = vel_variance;   // x
        vel_cov[6] = vel_variance;   // y
        vel_cov[11] = vel_variance;   // z

        mavlink_odometry_t odometry{
        time_usec: now_us + time_offset_us,
        x: float(pos_corrected.x),
        y: float(pos_corrected.y),
        z: float(pos_corrected.z),
        q: {attitude[0], attitude[1], attitude[2], attitude[3]},
        vx: vel_corrected_frd.x,
        vy: vel_corrected_frd.y,
        vz: vel_corrected_frd.z,
        rollspeed: gyro.x,
        pitchspeed: gyro.y,
        yawspeed: gyro.z,
        pose_covariance: {},
        velocity_covariance: {},
        frame_id: MAV_FRAME_LOCAL_FRD,
        child_frame_id: MAV_FRAME_BODY_FRD,
        reset_counter: 0,
        estimator_type: MAV_ESTIMATOR_TYPE_VIO,
        quality: 50, // quality hardcoded to 50%
        };
        memcpy(odometry.pose_covariance, pose_cov, sizeof(pose_cov));
        memcpy(odometry.velocity_covariance, vel_cov, sizeof(vel_cov));

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
        time_usec: now_us + time_offset_us,
        time_delta_usec: time_delta,
        angle_delta: { attitude_curr_prev.get_euler_roll(),
            attitude_curr_prev.get_euler_pitch(),
            attitude_curr_prev.get_euler_yaw()
        },
        position_delta: {pos_delta.x, pos_delta.y, pos_delta.z},
        confidence: 0
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

#endif  // AP_SIM_VICON_ENABLED
