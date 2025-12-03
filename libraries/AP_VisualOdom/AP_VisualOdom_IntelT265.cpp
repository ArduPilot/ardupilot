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

#include "AP_VisualOdom_config.h"

#if AP_VISUALODOM_INTELT265_ENABLED

#include "AP_VisualOdom_IntelT265.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

#define VISUALODOM_RESET_IGNORE_DURATION_MS 1000    // sensor data is ignored for 1sec after a position reset

extern const AP_HAL::HAL& hal;

// consume vision pose estimate data and send to EKF. distances in meters
// quality of -1 means failed, 0 means unknown, 1 is worst, 100 is best
void AP_VisualOdom_IntelT265::handle_pose_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude, float posErr, float angErr, uint8_t reset_counter, int8_t quality)
{
    const float scale_factor = _frontend.get_pos_scale();
    Vector3f pos{x * scale_factor, y * scale_factor, z * scale_factor};
    Quaternion att = attitude;

    // handle voxl camera reset jumps in attitude and position
    handle_voxl_camera_reset_jump(pos, att, reset_counter);

    // handle request to align sensor's yaw with vehicle's AHRS/EKF attitude
    if (_align_yaw) {
        if (align_yaw_to_ahrs(pos, attitude)) {
            _align_yaw = false;
        }
    }
    if (_align_posxy || _align_posz) {
        if (align_position_to_ahrs(pos, _align_posxy, _align_posz)) {
            _align_posxy = _align_posz = false;
        }
    }

    // rotate position and attitude to align with vehicle
    rotate_and_correct_position(pos);
    rotate_attitude(att);

    // record position for voxl reset jump handling
    record_voxl_position_and_reset_count(pos, reset_counter);

    posErr = constrain_float(posErr, _frontend.get_pos_noise(), 100.0f);
    angErr = constrain_float(angErr, _frontend.get_yaw_noise(), 1.5f);

    // record quality
    _quality = quality;

    // check for recent position reset
    bool consume = should_consume_sensor_data(true, reset_counter) && (_quality >= _frontend.get_quality_min());
    if (consume) {
        // send attitude and position to EKF
        AP::ahrs().writeExtNavData(pos, att, posErr, angErr, time_ms, _frontend.get_delay_ms(), get_reset_timestamp_ms(reset_counter));
    }

    // calculate euler orientation for logging
    float roll;
    float pitch;
    float yaw;
    att.to_euler(roll, pitch, yaw);

#if HAL_LOGGING_ENABLED
    // log sensor data
    Write_VisualPosition(remote_time_us, time_ms, pos.x, pos.y, pos.z, degrees(roll), degrees(pitch), wrap_360(degrees(yaw)), posErr, angErr, reset_counter, !consume, _quality);
#endif

    // store corrected attitude for use in pre-arm checks
    _attitude_last = att;

    // record time for health monitoring
    _last_update_ms = AP_HAL::millis();
}

// consume vision velocity estimate data and send to EKF, velocity in NED meters per second
// quality of -1 means failed, 0 means unknown, 1 is worst, 100 is best
void AP_VisualOdom_IntelT265::handle_vision_speed_estimate(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter, int8_t quality)
{
    // rotate velocity to align with vehicle
    Vector3f vel_corrected = vel;
    rotate_velocity(vel_corrected);

    // record quality
    _quality = quality;

    // check for recent position reset
    bool consume = should_consume_sensor_data(false, reset_counter) && (_quality >= _frontend.get_quality_min());
    if (consume) {
        // send velocity to EKF
        AP::ahrs().writeExtNavVelData(vel_corrected, _frontend.get_vel_noise(), time_ms, _frontend.get_delay_ms());
    }

    // record time for health monitoring
    _last_update_ms = AP_HAL::millis();

#if HAL_LOGGING_ENABLED
    Write_VisualVelocity(remote_time_us, time_ms, vel_corrected, reset_counter, !consume, _quality);
#endif
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
bool AP_VisualOdom_IntelT265::align_yaw_to_ahrs(const Vector3f &position, const Quaternion &attitude)
{
    // do not align to ahrs if we are its yaw source
    if (AP::ahrs().using_extnav_for_yaw()) {
        return false;
    }

    // do not align until ahrs yaw initialised
    if (!AP::ahrs().initialised()
#if AP_AHRS_DCM_ENABLED
        || !AP::ahrs().dcm_yaw_initialised()
#endif
        ) {
        return false;
    }

    align_yaw(position, attitude, AP::ahrs().get_yaw_rad());
    return true;
}

// align sensor yaw with any new yaw (in radians)
void AP_VisualOdom_IntelT265::align_yaw(const Vector3f &position, const Quaternion &attitude, float yaw_rad)
{
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
    const float yaw_trim_orig = _yaw_trim;
    _yaw_trim = wrap_2PI(yaw_rad - sens_yaw);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "VisOdom: yaw shifted %d to %d deg",
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

    // check if roll and pitch is different by > 10deg (using NED so cannot determine whether roll or pitch specifically)
    const float rp_diff_deg = degrees(ahrs_quat.roll_pitch_difference(_attitude_last));
    if (rp_diff_deg > 10.0f) {
        hal.util->snprintf(failure_msg, failure_msg_len, "roll/pitch diff %4.1f deg (>10)",(double)rp_diff_deg);
        return false;
    }

    // check if yaw is different by > 10deg
    Vector3f angle_diff;
    ahrs_quat.angular_difference(_attitude_last).to_axis_angle(angle_diff);
    const float yaw_diff_deg = degrees(fabsf(angle_diff.z));
    if (yaw_diff_deg > 10.0f) {
        hal.util->snprintf(failure_msg, failure_msg_len, "yaw diff %4.1f deg (>10)",(double)yaw_diff_deg);
        return false;
    }

    return true;
}

// returns true if sensor data should be consumed, false if it should be ignored
// set vision_position_estimate to true if reset_counter is from the VISION_POSITION_ESTIMATE source, false otherwise
// only the VISION_POSITION_ESTIMATE message's reset_counter is used to determine if sensor data should be ignored
bool AP_VisualOdom_IntelT265::should_consume_sensor_data(bool vision_position_estimate, uint8_t reset_counter)
{
    if (get_type() == AP_VisualOdom::VisualOdom_Type::VOXL) {
        // we don't discard data after a reset for VOXL
        return true;
    }

    uint32_t now_ms = AP_HAL::millis();

    // set ignore start time if reset counter has changed
    if (vision_position_estimate) {
        if (reset_counter != _pos_reset_counter_last) {
            _pos_reset_counter_last = reset_counter;
            _pos_reset_ignore_start_ms = now_ms;
        }
    }

    // check if 1 second has passed since the last reset
    if ((now_ms - _pos_reset_ignore_start_ms) > VISUALODOM_RESET_IGNORE_DURATION_MS) {
        _pos_reset_ignore_start_ms = 0;
    }

    return (_pos_reset_ignore_start_ms == 0);
}

// record voxl camera's position and reset counter for reset jump handling
// position is post scaling, offset and orientation corrections
void AP_VisualOdom_IntelT265::record_voxl_position_and_reset_count(const Vector3f &position, uint8_t reset_counter)
{
    // return immediately if not using VOXL camera
    if (get_type() != AP_VisualOdom::VisualOdom_Type::VOXL) {
        return;
    }

    _voxl_position_last = position;
    _voxl_reset_counter_last = reset_counter;
}

// handle voxl camera reset jumps in attitude and position
// sensor_pos should be the position directly from the sensor with only scaling applied (i.e. no yaw or position corrections)
// sensor_att is similarly the attitude directly from the sensor
void AP_VisualOdom_IntelT265::handle_voxl_camera_reset_jump(const Vector3f &sensor_pos, const Quaternion &sensor_att, uint8_t reset_counter)
{
    // return immediately if not using VOXL camera
    if (get_type() != AP_VisualOdom::VisualOdom_Type::VOXL) {
        return;
    }

    // return immediately if no change in reset counter
    if (reset_counter == _voxl_reset_counter_last) {
        return;
    }

    // warng user of reset
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "VisOdom: reset");

    // align sensor yaw to match current yaw estimate
    align_yaw_to_ahrs(sensor_pos, sensor_att);

    // align psoition to match last recorded position
    align_position(sensor_pos, _voxl_position_last, true, true);

    // record change in reset counter
    _voxl_reset_counter_last = reset_counter;
}

#endif  // AP_VISUALODOM_INTELT265_ENABLED
