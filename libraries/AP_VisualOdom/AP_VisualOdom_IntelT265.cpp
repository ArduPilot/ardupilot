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

#include "AP_VisualOdom_IntelT265.h"

#if HAL_VISUALODOM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

// consume vision position estimate data and send to EKF. distances in meters
void AP_VisualOdom_IntelT265::handle_vision_position_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude, float posErr, float angErr, uint8_t reset_counter)
{
    const float scale_factor = _frontend.get_pos_scale();
    Vector3f pos{x * scale_factor, y * scale_factor, z * scale_factor};
    Quaternion att = attitude;

    // handle user request to align camera
    if (_align_camera) {
        if (align_sensor_to_vehicle(pos, attitude)) {
            _align_camera = false;
        }
    }

    // rotate position and attitude to align with vehicle
    rotate_and_correct_position(pos);
    rotate_attitude(att);

    posErr = constrain_float(posErr, _frontend.get_pos_noise(), 100.0f);
    angErr = constrain_float(angErr, _frontend.get_yaw_noise(), 1.5f);
    // send attitude and position to EKF
    AP::ahrs().writeExtNavData(pos, att, posErr, angErr, time_ms, _frontend.get_delay_ms(), get_reset_timestamp_ms(reset_counter));

    // calculate euler orientation for logging
    float roll;
    float pitch;
    float yaw;
    att.to_euler(roll, pitch, yaw);

    // log sensor data
    AP::logger().Write_VisualPosition(remote_time_us, time_ms, pos.x, pos.y, pos.z, degrees(roll), degrees(pitch), wrap_360(degrees(yaw)), posErr, angErr, reset_counter);

    // store corrected attitude for use in pre-arm checks
    _attitude_last = att;

    // record time for health monitoring
    _last_update_ms = AP_HAL::millis();
}

// consume vision velocity estimate data and send to EKF, velocity in NED meters per second
void AP_VisualOdom_IntelT265::handle_vision_speed_estimate(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter)
{
    // rotate velocity to align with vehicle
    Vector3f vel_corrected = vel;
    rotate_velocity(vel_corrected);

    // send velocity to EKF
    AP::ahrs().writeExtNavVelData(vel_corrected, _frontend.get_vel_noise(), time_ms, _frontend.get_delay_ms());

    // record time for health monitoring
    _last_update_ms = AP_HAL::millis();

    AP::logger().Write_VisualVelocity(remote_time_us, time_ms, vel_corrected, _frontend.get_vel_noise(), reset_counter);
}

// apply rotation and correction to position
void AP_VisualOdom_IntelT265::rotate_and_correct_position(Vector3f &position) const
{
    if (_use_posvel_rotation) {
        position = _posvel_rotation * position;
    }
    position += _pos_correction;
}

// apply rotation to velocity
void AP_VisualOdom_IntelT265::rotate_velocity(Vector3f &velocity) const
{
    if (_use_posvel_rotation) {
        velocity = _posvel_rotation * velocity;
    }
}

// rotate attitude using _yaw_trim
void AP_VisualOdom_IntelT265::rotate_attitude(Quaternion &attitude) const
{
    // apply orientation rotation
    if (_use_att_rotation) {
        attitude *= _att_rotation;
    }

    // apply earth-frame yaw rotation
    if (!is_zero(_yaw_trim)) {
        attitude = _yaw_rotation * attitude;
    }
    return;
}

// use sensor provided attitude to calculate rotation to align sensor with AHRS/EKF attitude
bool AP_VisualOdom_IntelT265::align_sensor_to_vehicle(const Vector3f &position, const Quaternion &attitude)
{
    // fail immediately if ahrs cannot provide attitude
    Quaternion ahrs_quat;
    if (!AP::ahrs().get_quaternion(ahrs_quat)) {
        return false;
    }

    // if ahrs's yaw is from the compass, wait until it has been initialised
    if (!AP::ahrs().is_ext_nav_used_for_yaw() && !AP::ahrs().yaw_initialised()) {
        return false;
    }

    // clear any existing errors
    _error_orientation = false;

    // create rotation quaternion to correct for orientation
    const Rotation rot = _frontend.get_orientation();
    _att_rotation.initialise();
    _use_att_rotation = false;
    if (rot != Rotation::ROTATION_NONE) {
        _att_rotation.rotate(rot);
        _att_rotation.invert();
        _use_att_rotation = true;
    }

    Quaternion att_corrected = attitude;
    att_corrected *= _att_rotation;

    // extract sensor's corrected yaw
    const float sens_yaw = att_corrected.get_euler_yaw();

    // trim yaw by difference between ahrs and sensor yaw
    Vector3f angle_diff;
    ahrs_quat.angular_difference(att_corrected).to_axis_angle(angle_diff);
    const float yaw_trim_orig = _yaw_trim;
    _yaw_trim = angle_diff.z;
    gcs().send_text(MAV_SEVERITY_CRITICAL, "VisOdom: yaw shifted %d to %d deg",
                    (int)degrees(_yaw_trim - yaw_trim_orig),
                    (int)wrap_360(degrees(sens_yaw + _yaw_trim)));

    // convert _yaw_trim to _yaw_rotation to speed up processing later
    _yaw_rotation.from_euler(0.0f, 0.0f, _yaw_trim);

    // calculate position with current rotation and correction
    Vector3f pos_orig = position;
    rotate_and_correct_position(pos_orig);

    // create position and velocity rotation from yaw trim
    _use_posvel_rotation = false;
    if (!is_zero(_yaw_trim)) {
        _posvel_rotation.from_euler(0.0f, 0.0f, _yaw_trim);
        _use_posvel_rotation = true;
    }

    // recalculate position with new rotation
    Vector3f pos_new = position;
    rotate_and_correct_position(pos_new);

    // update position correction to remove change due to rotation
    _pos_correction += (pos_orig - pos_new);

    return true;
}

// returns false if we fail arming checks, in which case the buffer will be populated with a failure message
bool AP_VisualOdom_IntelT265::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    // exit immediately if not healthy
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "not healthy");
        return false;
    }

    // check for unsupported orientation
    if (_error_orientation) {
        hal.util->snprintf(failure_msg, failure_msg_len, "check VISO_ORIENT parameter");
        return false;
    }

    // get ahrs attitude
    Quaternion ahrs_quat;
    if (!AP::ahrs().get_quaternion(ahrs_quat)) {
        hal.util->snprintf(failure_msg, failure_msg_len, "waiting for AHRS attitude");
        return false;
    }

    // get angular difference between AHRS and camera attitude
    Vector3f angle_diff;
    _attitude_last.angular_difference(ahrs_quat).to_axis_angle(angle_diff);

    // check if roll and pitch is different by > 10deg (using NED so cannot determine whether roll or pitch specifically)
    const float rp_diff_deg = degrees(safe_sqrt(sq(angle_diff.x)+sq(angle_diff.y)));
    if (rp_diff_deg > 10.0f) {
        hal.util->snprintf(failure_msg, failure_msg_len, "roll/pitch diff %4.1f deg (>10)",(double)rp_diff_deg);
        return false;
    }

    // check if yaw is different by > 10deg
    const float yaw_diff_deg = degrees(fabsf(angle_diff.z));
    if (yaw_diff_deg > 10.0f) {
        hal.util->snprintf(failure_msg, failure_msg_len, "yaw diff %4.1f deg (>10)",(double)yaw_diff_deg);
        return false;
    }

    return true;
}

#endif
