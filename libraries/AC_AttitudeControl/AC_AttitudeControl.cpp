// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AC_AttitudeControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl::var_info[] = {

    // 0, 1 were RATE_RP_MAX, RATE_Y_MAX

    // @Param: SLEW_YAW
    // @DisplayName: Yaw target slew rate
    // @Description: Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
    // @Units: Centi-Degrees/Sec
    // @Range: 500 18000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("SLEW_YAW",    2, AC_AttitudeControl, _slew_yaw, AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_CDS),

    // 3 was for ACCEL_RP_MAX

    // @Param: ACCEL_Y_MAX
    // @DisplayName: Acceleration Max for Yaw
    // @Description: Maximum acceleration in yaw axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 72000
    // @Values: 0:Disabled, 18000:Slow, 36000:Medium, 54000:Fast
    // @Increment: 1000
    // @User: Advanced
    AP_GROUPINFO("ACCEL_Y_MAX",  4, AC_AttitudeControl, _accel_yaw_max, AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS),

    // @Param: RATE_FF_ENAB
    // @DisplayName: Rate Feedforward Enable
    // @Description: Controls whether body-frame rate feedfoward is enabled or disabled
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("RATE_FF_ENAB", 5, AC_AttitudeControl, _rate_bf_ff_enabled, AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT),

    // @Param: ACCEL_R_MAX
    // @DisplayName: Acceleration Max for Roll
    // @Description: Maximum acceleration in roll axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_R_MAX", 6, AC_AttitudeControl, _accel_roll_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

    // @Param: ACCEL_P_MAX
    // @DisplayName: Acceleration Max for Pitch
    // @Description: Maximum acceleration in pitch axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_P_MAX", 7, AC_AttitudeControl, _accel_pitch_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

    // IDs 8,9,10,11 RESERVED (in use on Solo)

    // @Param: ANGLE_BOOST
    // @DisplayName: Angle Boost
    // @Description: Angle Boost increases output throttle as the vehicle leans to reduce loss of altitude
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ANGLE_BOOST", 12, AC_AttitudeControl, _angle_boost_enabled, 1),

    // @Param: ANG_RLL_P
    // @DisplayName: Roll axis angle controller P gain
    // @Description: Roll axis angle controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_roll, "ANG_RLL_", 13, AC_AttitudeControl, AC_P),

    // @Param: ANG_PIT_P
    // @DisplayName: Pitch axis angle controller P gain
    // @Description: Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_pitch, "ANG_PIT_", 14, AC_AttitudeControl, AC_P),

    // @Param: ANG_YAW_P
    // @DisplayName: Yaw axis angle controller P gain
    // @Description: Yaw axis angle controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
    // @Range: 3.000 6.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_yaw, "ANG_YAW_", 15, AC_AttitudeControl, AC_P),

    // @Param: ANG_LIM_TC
    // @DisplayName: Angle Limit (to maintain altitude) Time Constant
    // @Description: Angle Limit (to maintain altitude) Time Constant
    // @Range: 0.5 10.0
    // @User: Advanced
    AP_GROUPINFO("ANG_LIM_TC", 16, AC_AttitudeControl, _angle_limit_tc, AC_ATTITUDE_CONTROL_ANGLE_LIMIT_TC_DEFAULT),

    AP_GROUPEND
};

void AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds, float smoothing_gain)
{
    // Convert from centidegrees on public interface to radians
    float euler_roll_angle_rad = radians(euler_roll_angle_cd*0.01f);
    float euler_pitch_angle_rad = radians(euler_pitch_angle_cd*0.01f);
    float euler_yaw_rate_rads = radians(euler_yaw_rate_cds*0.01f);

    // ensure smoothing gain can not cause overshoot
    smoothing_gain = constrain_float(smoothing_gain,1.0f,1/_dt);

    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    euler_roll_angle_rad += get_roll_trim_rad();

    if (_rate_bf_ff_enabled & !_att_ctrl_use_accel_limit) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        Vector3f euler_accel = euler_accel_limit(_att_target_euler_rad, Vector3f(get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()));

        // When acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler
        // angular velocity that will cause the euler angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by smoothing_gain at the end.
        input_shaping_angle(euler_roll_angle_rad, _att_target_euler_rad.x, smoothing_gain, euler_accel.x, _att_target_euler_rate_rads.x);
        input_shaping_angle(euler_pitch_angle_rad, _att_target_euler_rad.y, smoothing_gain, euler_accel.y, _att_target_euler_rate_rads.y);

        // When yaw acceleration limiting is enabled, the yaw input shaper constrains angular acceleration about the yaw axis, slewing
        // the output rate towards the input rate.
        _att_target_euler_rate_rads.z = input_shaping_ang_vel(_att_target_euler_rate_rads.z, euler_yaw_rate_rads, euler_accel.z);
    } else {
        // When feedforward is not enabled, the target euler angle is input into the target and the feedforward rate is zeroed.
        _att_target_euler_rad.x = euler_roll_angle_rad;
        _att_target_euler_rad.y = euler_pitch_angle_rad;

        _att_target_euler_rate_rads.x = 0;
        _att_target_euler_rate_rads.y = 0;
        _att_target_euler_rate_rads.z = euler_yaw_rate_rads;
    }

    // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
    euler_rate_to_ang_vel(_att_target_euler_rad, _att_target_euler_rate_rads, _att_target_ang_vel_rads);

    // Compute quaternion target attitude
    _attitude_target_quat.from_euler(_att_target_euler_rad.x, _att_target_euler_rad.y, _att_target_euler_rad.z);

    // Call quaternion attitude controller
    attitude_controller_run_quat(_att_target_ang_vel_rads, _attitude_target_quat, _ang_vel_target_rads);
}

void AC_AttitudeControl::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw)
{
    // Convert from centidegrees on public interface to radians
    float euler_roll_angle_rad = radians(euler_roll_angle_cd*0.01f);
    float euler_pitch_angle_rad = radians(euler_pitch_angle_cd*0.01f);
    float euler_yaw_angle_rad = radians(euler_yaw_angle_cd*0.01f);

    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    euler_roll_angle_rad += get_roll_trim_rad();

    // When feedforward is not enabled, the target euler angle is input into the target and the feedforward rate is zeroed.
    _att_target_euler_rad.x = euler_roll_angle_rad;
    _att_target_euler_rad.y = euler_pitch_angle_rad;
    _att_target_euler_rad.y = euler_yaw_angle_rad;

    // If slew_yaw is enabled, constrain yaw target within get_slew_yaw_rads() of _ahrs.yaw
    if (slew_yaw) {
        // Compute constrained angle error
        float angle_error = constrain_float(wrap_PI(_att_target_euler_rad.z - _ahrs.yaw), -get_slew_yaw_rads(), get_slew_yaw_rads());

        // Update attitude target from constrained angle error
        _att_target_euler_rad.z = angle_error + _ahrs.yaw;
    }

    _att_target_euler_rate_rads.x = 0;
    _att_target_euler_rate_rads.y = 0;
    _att_target_euler_rate_rads.z = 0;

    // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
    euler_rate_to_ang_vel(_att_target_euler_rad, _att_target_euler_rate_rads, _att_target_ang_vel_rads);

    // Compute quaternion target attitude
    _attitude_target_quat.from_euler(_att_target_euler_rad.x, _att_target_euler_rad.y, _att_target_euler_rad.z);

    // Call quaternion attitude controller
    attitude_controller_run_quat(_att_target_ang_vel_rads, _attitude_target_quat, _ang_vel_target_rads);
}

void AC_AttitudeControl::input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds)
{
    // Convert from centidegrees on public interface to radians
    float euler_roll_rate_rads = radians(euler_roll_rate_cds*0.01f);
    float euler_pitch_rate_rads = radians(euler_pitch_rate_cds*0.01f);
    float euler_yaw_rate_rads = radians(euler_yaw_rate_cds*0.01f);

    if (_rate_bf_ff_enabled) {
        // translate the roll pitch and yaw acceleration limits to the euler axis
        Vector3f euler_accel = euler_accel_limit(_att_target_euler_rad, Vector3f(get_accel_roll_max_radss(), get_accel_pitch_max_radss(), get_accel_yaw_max_radss()));

        // When acceleration limiting is enabled, the input shaper constrains angular acceleration, slewing
        // the output rate towards the input rate.
        _att_target_euler_rate_rads.x = input_shaping_ang_vel(_att_target_euler_rate_rads.x, euler_roll_rate_rads, euler_accel.x);
        _att_target_euler_rate_rads.y = input_shaping_ang_vel(_att_target_euler_rate_rads.y, euler_pitch_rate_rads, euler_accel.y);
        _att_target_euler_rate_rads.z = input_shaping_ang_vel(_att_target_euler_rate_rads.z, euler_yaw_rate_rads, euler_accel.z);
    } else {
        _att_target_euler_rate_rads.x = euler_roll_rate_rads;
        _att_target_euler_rate_rads.y = euler_pitch_rate_rads;
        _att_target_euler_rate_rads.z = euler_yaw_rate_rads;
    }

    // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
    euler_rate_to_ang_vel(_att_target_euler_rad, _att_target_euler_rate_rads, _att_target_ang_vel_rads);

    // Compute quaternion target attitude
    _attitude_target_quat.from_euler(_att_target_euler_rad.x, _att_target_euler_rad.y, _att_target_euler_rad.z);

    // Call quaternion attitude controller
    attitude_controller_run_quat(_att_target_ang_vel_rads, _attitude_target_quat, _ang_vel_target_rads);
}

void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // Convert from centidegrees on public interface to radians
    float roll_rate_rads = radians(roll_rate_bf_cds*0.01f);
    float pitch_rate_rads = radians(pitch_rate_bf_cds*0.01f);
    float yaw_rate_rads = radians(yaw_rate_bf_cds*0.01f);

    if (_rate_bf_ff_enabled) {
        // Compute acceleration-limited euler rates
        // When acceleration limiting is enabled, the input shaper constrains angular acceleration about the axis, slewing
        // the output rate towards the input rate.
        _att_target_ang_vel_rads.x = input_shaping_ang_vel(_att_target_ang_vel_rads.x, roll_rate_rads, get_accel_roll_max_radss());
        _att_target_ang_vel_rads.y = input_shaping_ang_vel(_att_target_ang_vel_rads.y, pitch_rate_rads, get_accel_pitch_max_radss());
        _att_target_ang_vel_rads.z = input_shaping_ang_vel(_att_target_ang_vel_rads.z, yaw_rate_rads, get_accel_yaw_max_radss());
    } else {
        // When axis acceleration limiting is disabled, the attitude target is simply rotated using the input rate and the input rate
        // is fed forward into the rate controller.
        _att_target_ang_vel_rads.x = roll_rate_rads;
        _att_target_ang_vel_rads.y = pitch_rate_rads;
        _att_target_ang_vel_rads.z = yaw_rate_rads;
    }

    // Call quaternion attitude controller
    attitude_controller_run_quat(_att_target_ang_vel_rads, _attitude_target_quat, _ang_vel_target_rads);
}

void AC_AttitudeControl::attitude_controller_run_quat(const Vector3f& att_target_ang_vel_rads, Quaternion& att_target_quat, Vector3f& ang_vel_target_rads)
{
    // Retrieve quaternion vehicle attitude
    // TODO add _ahrs.get_quaternion()
    Quaternion att_vehicle_quat;
    att_vehicle_quat.from_rotation_matrix(_ahrs.get_rotation_body_to_ned());

    Quaternion att_error_quat = att_vehicle_quat.inverse() * att_target_quat;

    // Compute attitude error
    Vector3f attitude_error_vector;
    float thrust_error_angle;
    thrust_heading_rotation_angles(att_target_quat, att_vehicle_quat, attitude_error_vector, thrust_error_angle);

    // Compute the angular velocity target from the attitude error
    update_ang_vel_target_from_att_error(attitude_error_vector, ang_vel_target_rads);

    // Add feedforward term that attempts to ensure that roll and pitch errors rotate with the body frame rather than the reference frame.
    // NOTE: This is a small-angle approximation.
    ang_vel_target_rads.x += attitude_error_vector.y * _ahrs.get_gyro().z;
    ang_vel_target_rads.y += -attitude_error_vector.x * _ahrs.get_gyro().z;

    // Add the angular velocity feedforward, rotated into vehicle frame
    Quaternion _att_target_ang_vel_quat = Quaternion(0.0f, _att_target_ang_vel_rads.x, _att_target_ang_vel_rads.y, _att_target_ang_vel_rads.z);
    Quaternion _ang_vel_target_quat = att_error_quat.inverse()*_att_target_ang_vel_quat*att_error_quat;

    // Correct the thrust vector and smoothly add feedforward and yaw input
    if(thrust_error_angle > radians(60.0f)){
        ang_vel_target_rads.z = 0.0f;
    }else if(thrust_error_angle > radians(30.0f)){
        float flip_scalar = (1.0f - (thrust_error_angle-radians(30.0f))/radians(30.0f));
        ang_vel_target_rads.x += _ang_vel_target_quat.q2*flip_scalar;
        ang_vel_target_rads.y += _ang_vel_target_quat.q3*flip_scalar;
        ang_vel_target_rads.z += _ang_vel_target_quat.q4;
        ang_vel_target_rads.z *= flip_scalar;
    } else {
        ang_vel_target_rads.x += _ang_vel_target_quat.q2;
        ang_vel_target_rads.y += _ang_vel_target_quat.q3;
        ang_vel_target_rads.z += _ang_vel_target_quat.q4;
    }

    // rotate target and normalize
    Quaternion att_target_ang_rot_quat;
    att_target_ang_rot_quat.from_axis_angle(Vector3f(att_target_ang_vel_rads.x * _dt, att_target_ang_vel_rads.y * _dt, att_target_ang_vel_rads.z * _dt));
    att_target_quat = att_target_quat * att_target_ang_rot_quat;
    att_target_quat.normalize();
}

// get_thrust_heading_rotation - calculates two ordered rotations to move the att_from_quat quaternion to the att_to_quat quaternion.
// The first rotation corrects the thrust vector and the second rotation corrects the heading vector.
void AC_AttitudeControl::thrust_heading_rotation_angles(Quaternion att_to_quat, Quaternion att_from_quat, Vector3f& att_diff_angle, float& thrust_vec_dot)
{
    Matrix3f att_to_rot_matrix; // earth frame to target frame
    att_to_quat.rotation_matrix(att_to_rot_matrix);
    Vector3f att_to_thrust_vec = att_to_rot_matrix*Vector3f(0.0f,0.0f,1.0f);

    Matrix3f att_from_rot_matrix; // earth frame to target frame
    att_from_quat.rotation_matrix(att_from_rot_matrix);
    Vector3f att_from_thrust_vec = att_from_rot_matrix*Vector3f(0.0f,0.0f,1.0f);

    // the cross product of the desired and target thrust vector defines the rotation vector
    Vector3f thrust_vec_cross = att_from_thrust_vec % att_to_thrust_vec;

    // the dot product is used to calculate the angle between the target and desired thrust vectors
    thrust_vec_dot = acosf(constrain_float(att_from_thrust_vec * att_to_thrust_vec,-1.0f,1.0f));

    // Normalize the thrust rotation vector
    float thrust_vector_length = thrust_vec_cross.length();
    if(is_zero(thrust_vector_length) || is_zero(thrust_vec_dot)){
        thrust_vec_cross = Vector3f(0,0,1);
        thrust_vec_dot = 0.0f;
    }else{
        thrust_vec_cross /= thrust_vector_length;
    }
    Quaternion thrust_vec_correction_quat;
    thrust_vec_correction_quat.from_axis_angle(thrust_vec_cross, thrust_vec_dot);
    thrust_vec_correction_quat = att_from_quat.inverse()*thrust_vec_correction_quat*att_from_quat;

    // calculate the remaining rotation required after thrust vector is rotated
    Quaternion heading_quat = thrust_vec_correction_quat.inverse()*att_from_quat.inverse()*att_to_quat;

    Vector3f rotation;
    thrust_vec_correction_quat.to_axis_angle(rotation);
    att_diff_angle.x = rotation.x;
    att_diff_angle.y = rotation.y;

    heading_quat.to_axis_angle(rotation);
    att_diff_angle.z = rotation.z;
}

void AC_AttitudeControl::input_shaping_angle(float desired_angle, float target_angle, float smoothing_gain, float accel_max, float& target_ang_vel)
{
    // Calculate the velocity as error approaches zero with acceleration limited by accel_max_radss
    float ang_vel = sqrt_controller(desired_angle-target_angle, smoothing_gain, accel_max);

    // Acceleration is limited directly to smooth the beginning of the curve.
    float delta_ang_vel = accel_max * _dt;
    target_ang_vel = constrain_float(ang_vel, target_ang_vel-delta_ang_vel, target_ang_vel+delta_ang_vel);
}

float AC_AttitudeControl::input_shaping_ang_vel(float target_ang_vel, float desired_ang_vel, float accel_max)
{
    if ((accel_max > 0.0f) && !is_zero(accel_max)) {
        float delta_ang_vel = accel_max * _dt;
        target_ang_vel += constrain_float(desired_ang_vel - target_ang_vel, -delta_ang_vel, delta_ang_vel);
    } else {
        target_ang_vel = desired_ang_vel;
    }
    return target_ang_vel;
}

Vector3f AC_AttitudeControl::euler_accel_limit(Vector3f euler_rad, Vector3f euler_accel)
{
    float sin_phi = constrain_float(fabs(sinf(euler_rad.x)), 0.1f, 1.0f);
    float cos_phi = constrain_float(fabs(cosf(euler_rad.x)), 0.1f, 1.0f);
    float sin_theta = constrain_float(fabs(sinf(euler_rad.y)), 0.1f, 1.0f);

    Vector3f rot_accel;
    if(is_zero(euler_accel.x) || is_zero(euler_accel.y) || is_zero(euler_accel.z) || (euler_accel.x < 0.0f) || (euler_accel.y < 0.0f) || (euler_accel.z < 0.0f)) {
        rot_accel.x = euler_accel.x;
        rot_accel.y = euler_accel.y;
        rot_accel.z = euler_accel.z;
    } else {
        rot_accel.x = euler_accel.x;
        rot_accel.y = MIN(euler_accel.y/cos_phi, euler_accel.z/sin_phi);
        rot_accel.z = MIN(MIN(euler_accel.x/sin_theta, euler_accel.y/sin_phi), euler_accel.z/cos_phi);
    }
    return rot_accel;
}

void AC_AttitudeControl::relax_bf_rate_controller()
{
    // Set reference angular velocity used in angular velocity controller equal
    // to the input angular velocity and reset the angular velocity integrators.
    // This zeros the output of the angular velocity controller.
    _ang_vel_target_rads = _ahrs.get_gyro();
    get_rate_roll_pid().reset_I();
    get_rate_pitch_pid().reset_I();
    get_rate_yaw_pid().reset_I();

    // Write euler derivatives derived from vehicle angular velocity to
    // _att_target_euler_rate_rads. This resets the state of the input shapers.
    ang_vel_to_euler_rate(Vector3f(_ahrs.roll,_ahrs.pitch,_ahrs.yaw), _ang_vel_target_rads, _att_target_euler_rate_rads);
}

void AC_AttitudeControl::shift_ef_yaw_target(float yaw_shift_cd)
{
    _att_target_euler_rad.z = wrap_2PI(_att_target_euler_rad.z + radians(yaw_shift_cd*0.01f));
}

void AC_AttitudeControl::euler_rate_to_ang_vel(const Vector3f& euler_rad, const Vector3f& euler_rate_rads, Vector3f& ang_vel_rads)
{
    float sin_theta = sinf(euler_rad.y);
    float cos_theta = cosf(euler_rad.y);
    float sin_phi = sinf(euler_rad.x);
    float cos_phi = cosf(euler_rad.x);

    ang_vel_rads.x = euler_rate_rads.x - sin_theta * euler_rate_rads.z;
    ang_vel_rads.y = cos_phi  * euler_rate_rads.y + sin_phi * cos_theta * euler_rate_rads.z;
    ang_vel_rads.z = -sin_phi * euler_rate_rads.y + cos_theta * cos_phi * euler_rate_rads.z;
}

bool AC_AttitudeControl::ang_vel_to_euler_rate(const Vector3f& euler_rad, const Vector3f& ang_vel_rads, Vector3f& euler_rate_rads)
{
    float sin_theta = sinf(euler_rad.y);
    float cos_theta = cosf(euler_rad.y);
    float sin_phi = sinf(euler_rad.x);
    float cos_phi = cosf(euler_rad.x);

    // When the vehicle pitches all the way up or all the way down, the euler angles become discontinuous. In this case, we just return false.
    if (is_zero(cos_theta)) {
        return false;
    }

    euler_rate_rads.x = ang_vel_rads.x + sin_phi * (sin_theta/cos_theta) * ang_vel_rads.y + cos_phi * (sin_theta/cos_theta) * ang_vel_rads.z;
    euler_rate_rads.y = cos_phi  * ang_vel_rads.y - sin_phi * ang_vel_rads.z;
    euler_rate_rads.z = (sin_phi / cos_theta) * ang_vel_rads.y + (cos_phi / cos_theta) * ang_vel_rads.z;
    return true;
}

void AC_AttitudeControl::update_ang_vel_target_from_att_error(Vector3f att_error_rot_vec_rad, Vector3f& ang_vel_target_rads)
{
    // Compute the roll angular velocity demand from the roll angle error
    if (_att_ctrl_use_accel_limit) {
        ang_vel_target_rads.x = sqrt_controller(att_error_rot_vec_rad.x, _p_angle_roll.kP(), constrain_float(get_accel_roll_max_radss()/2.0f,  AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS));
    }else{
        ang_vel_target_rads.x = _p_angle_roll.kP() * att_error_rot_vec_rad.x;
    }

    // Compute the pitch angular velocity demand from the roll angle error
    if (_att_ctrl_use_accel_limit) {
        ang_vel_target_rads.y = sqrt_controller(att_error_rot_vec_rad.y, _p_angle_pitch.kP(), constrain_float(get_accel_pitch_max_radss()/2.0f,  AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS));
    }else{
        ang_vel_target_rads.y = _p_angle_pitch.kP() * att_error_rot_vec_rad.y;
    }

    // Compute the yaw angular velocity demand from the roll angle error
    if (_att_ctrl_use_accel_limit) {
        ang_vel_target_rads.z = sqrt_controller(att_error_rot_vec_rad.z, _p_angle_yaw.kP(), constrain_float(get_accel_yaw_max_radss()/2.0f,  AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS));
    }else{
        ang_vel_target_rads.z = _p_angle_yaw.kP() * att_error_rot_vec_rad.z;
    }
}

float AC_AttitudeControl::rate_bf_to_motor_roll(float rate_target_rads)
{
    float current_rate_rads = _ahrs.get_gyro().x;
    float rate_error_rads = rate_target_rads - current_rate_rads;

    // pass error to PID controller
    get_rate_roll_pid().set_input_filter_d(rate_error_rads);
    get_rate_roll_pid().set_desired_rate(rate_target_rads);

    float integrator = get_rate_roll_pid().get_integrator();

    // Ensure that integrator can only be reduced if the output is saturated
    if (!_motors.limit.roll_pitch || ((integrator > 0 && rate_error_rads < 0) || (integrator < 0 && rate_error_rads > 0))) {
        integrator = get_rate_roll_pid().get_i();
    }

    // Compute output in range -1 ~ +1
    float output = get_rate_roll_pid().get_p() + integrator + get_rate_roll_pid().get_d();

    // Constrain output
    return constrain_float(output, -1.0f, 1.0f);
}

float AC_AttitudeControl::rate_bf_to_motor_pitch(float rate_target_rads)
{
    float current_rate_rads = _ahrs.get_gyro().y;
    float rate_error_rads = rate_target_rads - current_rate_rads;

    // pass error to PID controller
    get_rate_pitch_pid().set_input_filter_d(rate_error_rads);
    get_rate_pitch_pid().set_desired_rate(rate_target_rads);

    float integrator = get_rate_pitch_pid().get_integrator();

    // Ensure that integrator can only be reduced if the output is saturated
    if (!_motors.limit.roll_pitch || ((integrator > 0 && rate_error_rads < 0) || (integrator < 0 && rate_error_rads > 0))) {
        integrator = get_rate_pitch_pid().get_i();
    }

    // Compute output in range -1 ~ +1
    float output = get_rate_pitch_pid().get_p() + integrator + get_rate_pitch_pid().get_d();

    // Constrain output
    return constrain_float(output, -1.0f, 1.0f);
}

float AC_AttitudeControl::rate_bf_to_motor_yaw(float rate_target_rads)
{
    float current_rate_rads = _ahrs.get_gyro().z;
    float rate_error_rads = rate_target_rads - current_rate_rads;

    // pass error to PID controller
    get_rate_yaw_pid().set_input_filter_all(rate_error_rads);
    get_rate_yaw_pid().set_desired_rate(rate_target_rads);

    float integrator = get_rate_yaw_pid().get_integrator();

    // Ensure that integrator can only be reduced if the output is saturated
    if (!_motors.limit.yaw || ((integrator > 0 && rate_error_rads < 0) || (integrator < 0 && rate_error_rads > 0))) {
        integrator = get_rate_yaw_pid().get_i();
    }

    // Compute output in range -1 ~ +1
    float output = get_rate_yaw_pid().get_p() + integrator + get_rate_yaw_pid().get_d();

    // Constrain output
    return constrain_float(output, -1.0f, 1.0f);
}

void AC_AttitudeControl::accel_limiting(bool enable_limits)
{
    if (enable_limits) {
        // If enabling limits, reload from eeprom or set to defaults
        if (is_zero(_accel_roll_max)) {
            _accel_roll_max.load();
        }
        if (is_zero(_accel_pitch_max)) {
            _accel_pitch_max.load();
        }
        if (is_zero(_accel_yaw_max)) {
            _accel_yaw_max.load();
        }
    } else {
        _accel_roll_max = 0.0f;
        _accel_pitch_max = 0.0f;
        _accel_yaw_max = 0.0f;
    }
}

void AC_AttitudeControl::set_throttle_out_unstabilized(float throttle_in, bool reset_attitude_control, float filter_cutoff)
{
    _throttle_in = throttle_in;
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (reset_attitude_control) {
        relax_bf_rate_controller();
        set_yaw_target_to_current_heading();
    }
    _motors.set_throttle(throttle_in);
    _angle_boost = 0.0f;
}

// Return tilt angle limit for pilot input that prioritises altitude hold over lean angle
float AC_AttitudeControl::get_althold_lean_angle_max() const
{
    // convert to centi-degrees for public interface
    return ToDeg(_althold_lean_angle_max) * 100.0f;
}

float AC_AttitudeControl::sqrt_controller(float error, float p, float second_ord_lim)
{
    if (second_ord_lim < 0.0f || is_zero(second_ord_lim) || is_zero(p)) {
        return error*p;
    }

    float linear_dist = second_ord_lim/sq(p);

    if (error > linear_dist) {
        return safe_sqrt(2.0f*second_ord_lim*(error-(linear_dist/2.0f)));
    } else if (error < -linear_dist) {
        return -safe_sqrt(2.0f*second_ord_lim*(-error-(linear_dist/2.0f)));
    } else {
        return error*p;
    }
}

float AC_AttitudeControl::stopping_point(float first_ord_mag, float p, float second_ord_lim)
{
    if (second_ord_lim < 0.0f || is_zero(second_ord_lim) || is_zero(p)) {
        return first_ord_mag/p;
    }

    // calculate the velocity at which we switch from calculating the stopping point using a linear function to a sqrt function
    float linear_velocity = second_ord_lim/p;

    if (fabsf(first_ord_mag) < linear_velocity) {
        // if our current velocity is below the cross-over point we use a linear function
        return first_ord_mag/p;
    } else {
        float linear_dist = second_ord_lim/sq(p);
        float overshoot = (linear_dist/2.0f) + sq(first_ord_mag)/(2.0f*second_ord_lim);
        if (first_ord_mag > 0){
            return overshoot;
        } else {
            return -overshoot;
        }
    }
}

float AC_AttitudeControl::max_rate_step_bf_roll()
{
    float alpha = get_rate_roll_pid().get_filt_alpha();
    float alpha_remaining = 1-alpha;
    return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*get_rate_roll_pid().kD())/_dt + get_rate_roll_pid().kP());
}

float AC_AttitudeControl::max_rate_step_bf_pitch()
{
    float alpha = get_rate_pitch_pid().get_filt_alpha();
    float alpha_remaining = 1-alpha;
    return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*get_rate_pitch_pid().kD())/_dt + get_rate_pitch_pid().kP());
}

float AC_AttitudeControl::max_rate_step_bf_yaw()
{
    float alpha = get_rate_yaw_pid().get_filt_alpha();
    float alpha_remaining = 1-alpha;
    return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*get_rate_yaw_pid().kD())/_dt + get_rate_yaw_pid().kP());
}
