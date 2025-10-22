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

   This class inherits from AC_AttitudeControl_Multi and provides functionality
   specific to tailsitter quadplanes.
   1) "body-frame" roll control mode for all tailsitters
   2) a relax_attitude_controller method needed for coping with vectored tailsitters
 */
#include "AC_AttitudeControl_TS.h"

void AC_AttitudeControl_TS::relax_attitude_controllers(bool exclude_pitch)
{
    // If exclude_pitch: relax roll and yaw rate controller outputs only,
    // leaving pitch controller active to let TVBS motors tilt up while in throttle_wait
    if (exclude_pitch) {
        // Get the current attitude quaternion
        Quaternion current_attitude;
        _ahrs.get_quat_body_to_ned(current_attitude);

        Vector3f current_eulers;
        current_attitude.to_euler(current_eulers.x, current_eulers.y, current_eulers.z);

        // set target attitude to zero pitch with (approximate) current roll and yaw
        // by rotating the current_attitude quaternion by the error in desired pitch
        Quaternion pitch_rotation;
        pitch_rotation.from_axis_angle(Vector3f{0, -1, 0}, current_eulers.y);
        _attitude_target = current_attitude * pitch_rotation;
        _attitude_target.normalize();
        _attitude_target.to_euler(_euler_angle_target_rad.x, _euler_angle_target_rad.y, _euler_angle_target_rad.z);
        _attitude_ang_error = current_attitude.inverse() * _attitude_target;

        // Initialize the roll and yaw angular rate variables to the current rate
        _ang_vel_target_rads = _ahrs.get_gyro();
        ang_vel_to_euler_rate(_attitude_target, _ang_vel_target_rads, _euler_rate_target_rads);
        _ang_vel_body_rads.x = _ahrs.get_gyro().x;
        _ang_vel_body_rads.z = _ahrs.get_gyro().z;

        // Reset the roll and yaw I terms
        get_rate_roll_pid().reset_I();
        get_rate_yaw_pid().reset_I();
    } else {
        // relax all attitude controllers
        AC_AttitudeControl::relax_attitude_controllers();
    }
}

// Commands a body-frame roll angle (in centidegrees), an euler pitch angle (in centidegrees), and a yaw rate (in centidegrees/s).
// See input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad() for full details.
void AC_AttitudeControl_TS::input_euler_rate_yaw_euler_angle_pitch_bf_roll_cd(bool plane_controls, float body_roll_cd, float euler_pitch_cd, float euler_yaw_rate_cds)
{
    // Convert from centidegrees on public interface to radians
    float euler_yaw_rate_rads = cd_to_rad(euler_yaw_rate_cds);
    float euler_pitch_rad    = cd_to_rad(euler_pitch_cd);
    float body_roll_rad      = cd_to_rad(body_roll_cd);

    input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad(plane_controls, body_roll_rad, euler_pitch_rad, euler_yaw_rate_rads);
}

// Commands a body-frame roll angle (in radians), an euler pitch angle (in radians), and a yaw rate (in radians/s).
// Used by tailsitter quadplanes. Optionally swaps roll and yaw effects as pitch nears 90° if plane_controls is true.
void AC_AttitudeControl_TS::input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad(bool plane_controls, float body_roll_rad, float euler_pitch_rad, float euler_yaw_rate_rads)
{
    euler_pitch_rad    = constrain_float(euler_pitch_rad, -radians(90.0), radians(90.0));
    body_roll_rad      = -body_roll_rad;

    const float cpitch = cosf(euler_pitch_rad);
    const float spitch = fabsf(sinf(euler_pitch_rad));

    // Compute attitude error
    Quaternion attitude_body;
    Quaternion error_quat;
    _ahrs.get_quat_body_to_ned(attitude_body);
    error_quat = attitude_body.inverse() * _attitude_target;
    Vector3f att_error;
    error_quat.to_axis_angle(att_error);

    // update heading
    float yaw_rate_rads = euler_yaw_rate_rads;
    if (plane_controls) {
        yaw_rate_rads = (euler_yaw_rate_rads * spitch) + (body_roll_rad * cpitch);
    }
    // limit yaw error
    float yaw_error = fabsf(att_error.z);
    float error_ratio = yaw_error / M_PI_2;
    if (error_ratio > 1) {
        yaw_rate_rads /= (error_ratio * error_ratio);
    }
    _euler_angle_target_rad.z = wrap_PI(_euler_angle_target_rad.z + yaw_rate_rads * _dt_s);

    // init attitude target to desired euler yaw and pitch with zero roll
    _attitude_target.from_euler(0, euler_pitch_rad, _euler_angle_target_rad.z);

    // apply body-frame yaw/roll (this is roll/yaw for a tailsitter in forward flight)
    // rotate body_roll_rad axis by |sin(pitch angle)|
    Quaternion bf_roll_Q;
    bf_roll_Q.from_axis_angle(Vector3f{0, 0, spitch * body_roll_rad});

    // rotate body_yaw axis by cos(pitch angle)
    Quaternion bf_yaw_Q;
    if (plane_controls) {
        bf_yaw_Q.from_axis_angle(Vector3f{cpitch, 0, 0}, euler_yaw_rate_rads);
    } else {
        bf_yaw_Q.from_axis_angle(Vector3f{-cpitch * body_roll_rad, 0, 0});
    }
    _attitude_target = _attitude_target * bf_roll_Q * bf_yaw_Q;

    // _euler_angle_target_rad roll and pitch: Note: roll/yaw will be indeterminate when pitch is near +/-90
    // These should be used only for logging target eulers, with the caveat noted above.
    // Also note that _attitude_target.from_euler() should only be used in special circumstances
    // such as when attitude is specified directly in terms of Euler angles.
    //    _euler_angle_target_rad.x = _attitude_target.get_euler_roll();
    //    _euler_angle_target_rad.y = euler_pitch_rad;

    // Set rate feedforward requests to zero
    _euler_rate_target_rads.zero();
    _ang_vel_target_rads.zero();

    // Compute attitude error
    error_quat = attitude_body.inverse() * _attitude_target;
    error_quat.to_axis_angle(att_error);

    // Compute the angular velocity target from the attitude error
    _ang_vel_body_rads = update_ang_vel_target_from_att_error(att_error);
}
