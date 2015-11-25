// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl::var_info[] = {

    // 0, 1 were RATE_RP_MAX, RATE_Y_MAX

    // BUG: SLEW_YAW's behavior does not match its parameter documentation
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

    AP_GROUPEND
};

//
// high level controllers
//

void AC_AttitudeControl::set_dt(float delta_sec)
{
    _dt = delta_sec;

    // set attitude controller's D term filters
    _pid_rate_roll.set_dt(_dt);
    _pid_rate_pitch.set_dt(_dt);
    _pid_rate_yaw.set_dt(_dt);
}

// relax_bf_rate_controller - ensure body-frame rate controller has zero errors to relax rate controller output
void AC_AttitudeControl::relax_bf_rate_controller()
{
    // ensure zero error in body frame rate controllers
    _rate_bf_target_rads = _ahrs.get_gyro();

    // write euler derivatives to _rate_ef_desired_rads so that the input shapers are properly initialized
    ang_vel_to_euler_derivative(Vector3f(_ahrs.roll,_ahrs.pitch,_ahrs.yaw), _rate_bf_target_rads, _rate_ef_desired_rads);

    _pid_rate_roll.reset_I();
    _pid_rate_pitch.reset_I();
    _pid_rate_yaw.reset_I();
}

// shifts earth frame yaw target by yaw_shift_cd.  yaw_shift_cd should be in centi-degrees and is added to the current target heading
void AC_AttitudeControl::shift_ef_yaw_target(float yaw_shift_cd)
{
    // convert from centi-degrees on public interface to radians
    _angle_ef_target_rad.z = wrap_2PI(_angle_ef_target_rad.z + radians(yaw_shift_cd*0.01f));
}

//
// methods to be called by upper controllers to request and implement a desired attitude
//

// angle_ef_roll_pitch_rate_ef_yaw_smooth - attempts to maintain a roll and pitch angle and yaw rate (all earth frame) while smoothing the attitude based on the feel parameter
//      smoothing_gain : a number from 1 to 50 with 1 being sluggish and 50 being very crisp
void AC_AttitudeControl::angle_ef_roll_pitch_rate_ef_yaw_smooth(float roll_angle_ef_cd, float pitch_angle_ef_cd, float yaw_rate_ef_cds, float smoothing_gain)
{
    // convert from centi-degrees on public interface to radians
    float roll_angle_ef_rad = radians(roll_angle_ef_cd*0.01f);
    float pitch_angle_ef_rad = radians(pitch_angle_ef_cd*0.01f);
    float yaw_rate_ef_rads = radians(yaw_rate_ef_cds*0.01f);

    float rate_ef_desired_rads;
    float rate_change_limit_rads;
    Vector3f angle_ef_error_rad;    // earth frame angle errors

    // sanity check smoothing gain
    smoothing_gain = constrain_float(smoothing_gain,1.0f,50.0f);

    // add roll trim to compensate tail rotor thrust in heli (should return zero for multirotors)
    roll_angle_ef_rad += get_roll_trim_rad();

    // if accel limiting and feed forward enabled
    if ((get_accel_roll_max_radss() > 0.0f) && _rate_bf_ff_enabled) {
        rate_change_limit_rads = get_accel_roll_max_radss() * _dt;

        // calculate earth-frame feed forward roll rate using linear response when close to the target, sqrt response when we're further away
        rate_ef_desired_rads = sqrt_controller(roll_angle_ef_rad-_angle_ef_target_rad.x, smoothing_gain, get_accel_roll_max_radss());

        // apply acceleration limit to feed forward roll rate
        _rate_ef_desired_rads.x = constrain_float(rate_ef_desired_rads, _rate_ef_desired_rads.x-rate_change_limit_rads, _rate_ef_desired_rads.x+rate_change_limit_rads);

        // update earth-frame roll angle target using desired roll rate
        update_ef_roll_angle_and_error(_rate_ef_desired_rads.x, angle_ef_error_rad, AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX_RAD);
    } else {
        // target roll and pitch to desired input roll and pitch
        _angle_ef_target_rad.x = roll_angle_ef_rad;
        angle_ef_error_rad.x = wrap_180_cd_float(_angle_ef_target_rad.x - _ahrs.roll);

        // set roll and pitch feed forward to zero
        _rate_ef_desired_rads.x = 0;
    }
    // constrain earth-frame angle targets
    _angle_ef_target_rad.x = constrain_float(_angle_ef_target_rad.x, -get_tilt_limit_rad(), get_tilt_limit_rad());

    // if accel limiting and feed forward enabled
    if ((get_accel_pitch_max_radss() > 0.0f) && _rate_bf_ff_enabled) {
        rate_change_limit_rads = get_accel_pitch_max_radss() * _dt;

        // calculate earth-frame feed forward pitch rate using linear response when close to the target, sqrt response when we're further away
        rate_ef_desired_rads = sqrt_controller(pitch_angle_ef_rad-_angle_ef_target_rad.y, smoothing_gain, get_accel_pitch_max_radss());

        // apply acceleration limit to feed forward pitch rate
        _rate_ef_desired_rads.y = constrain_float(rate_ef_desired_rads, _rate_ef_desired_rads.y-rate_change_limit_rads, _rate_ef_desired_rads.y+rate_change_limit_rads);

        // update earth-frame pitch angle target using desired pitch rate
        update_ef_pitch_angle_and_error(_rate_ef_desired_rads.y, angle_ef_error_rad, AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX_RAD);
    } else {
        // target pitch and pitch to desired input pitch and pitch
        _angle_ef_target_rad.y = pitch_angle_ef_rad;
        angle_ef_error_rad.y = wrap_180_cd_float(_angle_ef_target_rad.y - _ahrs.pitch);

        // set pitch and pitch feed forward to zero
        _rate_ef_desired_rads.y = 0;
    }
    // constrain earth-frame angle targets
    _angle_ef_target_rad.y = constrain_float(_angle_ef_target_rad.y, -get_tilt_limit_rad(), get_tilt_limit_rad());

    if (get_accel_yaw_max_radss() > 0.0f) {
        // set earth-frame feed forward rate for yaw
        rate_change_limit_rads = get_accel_yaw_max_radss() * _dt;

        // update yaw rate target with acceleration limit
        _rate_ef_desired_rads.z += constrain_float(yaw_rate_ef_rads - _rate_ef_desired_rads.z, -rate_change_limit_rads, rate_change_limit_rads);

        // calculate yaw target angle and angle error
        update_ef_yaw_angle_and_error(_rate_ef_desired_rads.z, angle_ef_error_rad, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX_RAD);
    } else {
        // set yaw feed forward to zero
        _rate_ef_desired_rads.z = yaw_rate_ef_rads;
        // calculate yaw target angle and angle error
        update_ef_yaw_angle_and_error(_rate_ef_desired_rads.z, angle_ef_error_rad, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX_RAD);
    }

    // convert 321-intrinsic euler angle errors to a body-frame rotation vector
    // NOTE: this results in an approximation of the attitude error based on a linearization about the current attitude
    euler_derivative_to_ang_vel(Vector3f(_ahrs.roll,_ahrs.pitch,_ahrs.yaw), angle_ef_error_rad, _angle_bf_error_rad);


    // convert body-frame angle errors to body-frame rate targets
    update_rate_bf_targets();

    // add body frame rate feed forward
    if (_rate_bf_ff_enabled) {
        // convert euler angle derivative of desired attitude into a body-frame angular velocity vector
        euler_derivative_to_ang_vel(_angle_ef_target_rad, _rate_ef_desired_rads, _rate_bf_desired_rads);
        // NOTE: rotation of _rate_bf_desired_rads from desired body frame to estimated body frame is possibly omitted here
        _rate_bf_target_rads += _rate_bf_desired_rads;
    } else {
        // convert euler angle derivatives of desired attitude into a body-frame angular velocity vector
        euler_derivative_to_ang_vel(_angle_ef_target_rad, Vector3f(0,0,_rate_ef_desired_rads.z), _rate_bf_desired_rads);
        // NOTE: rotation of _rate_bf_desired_rads from desired body frame to estimated body frame is possibly omitted here
        _rate_bf_target_rads += _rate_bf_desired_rads;
    }

    // body-frame to motor outputs should be called separately
}

//
// methods to be called by upper controllers to request and implement a desired attitude
//

// angle_ef_roll_pitch_rate_ef_yaw - attempts to maintain a roll and pitch angle and yaw rate (all earth frame)
void AC_AttitudeControl::angle_ef_roll_pitch_rate_ef_yaw(float roll_angle_ef_cd, float pitch_angle_ef_cd, float yaw_rate_ef_cds)
{
    // convert from centidegrees on public interface to radians
    float roll_angle_ef_rad = radians(roll_angle_ef_cd*0.01f);
    float pitch_angle_ef_rad = radians(pitch_angle_ef_cd*0.01f);
    float yaw_rate_ef_rads = radians(yaw_rate_ef_cds*0.01f);

    Vector3f    angle_ef_error_rad;         // earth frame angle errors

    // add roll trim to compensate tail rotor thrust in heli (should return zero for multirotors)
    roll_angle_ef_rad += get_roll_trim_rad();

    // set earth-frame angle targets for roll and pitch and calculate angle error
    _angle_ef_target_rad.x = constrain_float(roll_angle_ef_rad, -get_tilt_limit_rad(), get_tilt_limit_rad());
    angle_ef_error_rad.x = wrap_PI(_angle_ef_target_rad.x - _ahrs.roll);

    _angle_ef_target_rad.y = constrain_float(pitch_angle_ef_rad, -get_tilt_limit_rad(), get_tilt_limit_rad());
    angle_ef_error_rad.y = wrap_PI(_angle_ef_target_rad.y - _ahrs.pitch);

    if (get_accel_yaw_max_radss() > 0.0f) {
        // set earth-frame feed forward rate for yaw
        float rate_change_limit_rads = get_accel_yaw_max_radss() * _dt;

        float rate_change_rads = yaw_rate_ef_rads - _rate_ef_desired_rads.z;
        rate_change_rads = constrain_float(rate_change_rads, -rate_change_limit_rads, rate_change_limit_rads);
        _rate_ef_desired_rads.z += rate_change_rads;
        // calculate yaw target angle and angle error
        update_ef_yaw_angle_and_error(_rate_ef_desired_rads.z, angle_ef_error_rad, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX_RAD);
    } else {
        // set yaw feed forward to zero
        _rate_ef_desired_rads.z = yaw_rate_ef_rads;
        // calculate yaw target angle and angle error
        update_ef_yaw_angle_and_error(_rate_ef_desired_rads.z, angle_ef_error_rad, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX_RAD);
    }

    // convert 321-intrinsic euler angle errors to a body-frame rotation vector
    // NOTE: this results in an approximation of the attitude error based on a linearization about the current attitude
    euler_derivative_to_ang_vel(Vector3f(_ahrs.roll,_ahrs.pitch,_ahrs.yaw), angle_ef_error_rad, _angle_bf_error_rad);

    // convert body-frame angle errors to body-frame rate targets
    update_rate_bf_targets();

    // set roll and pitch feed forward to zero
    _rate_ef_desired_rads.x = 0;
    _rate_ef_desired_rads.y = 0;

    // convert euler angle derivatives of desired attitude into a body-frame angular velocity vector
    euler_derivative_to_ang_vel(_angle_ef_target_rad, _rate_ef_desired_rads, _rate_bf_desired_rads);
    // NOTE: rotation of _rate_bf_desired_rads from desired body frame to estimated body frame is possibly omitted here
    _rate_bf_target_rads += _rate_bf_desired_rads;

    // body-frame to motor outputs should be called separately
}

// angle_ef_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw angle (all earth frame)
//  if yaw_slew is true then target yaw movement will be gradually moved to the new target based on the SLEW_YAW parameter
void AC_AttitudeControl::angle_ef_roll_pitch_yaw(float roll_angle_ef_cd, float pitch_angle_ef_cd, float yaw_angle_ef_cd, bool slew_yaw)
{
    // convert from centidegrees on public interface to radians
    float roll_angle_ef_rad = radians(roll_angle_ef_cd*0.01f);
    float pitch_angle_ef_rad = radians(pitch_angle_ef_cd*0.01f);
    float yaw_angle_ef_rad = radians(yaw_angle_ef_cd*0.01f);

    Vector3f    angle_ef_error_rad;

    // add roll trim to compensate tail rotor thrust in heli (should return zero for multirotors)
    roll_angle_ef_rad += get_roll_trim_rad();

    // set earth-frame angle targets
    _angle_ef_target_rad.x = constrain_float(roll_angle_ef_rad, -get_tilt_limit_rad(), get_tilt_limit_rad());
    _angle_ef_target_rad.y = constrain_float(pitch_angle_ef_rad, -get_tilt_limit_rad(), get_tilt_limit_rad());
    _angle_ef_target_rad.z = yaw_angle_ef_rad;

    // calculate earth frame errors
    angle_ef_error_rad.x = wrap_PI(_angle_ef_target_rad.x - _ahrs.roll);
    angle_ef_error_rad.y = wrap_PI(_angle_ef_target_rad.y - _ahrs.pitch);
    angle_ef_error_rad.z = wrap_PI(_angle_ef_target_rad.z - _ahrs.yaw);

    // constrain the yaw angle error
    // BUG: SLEW_YAW's behavior does not match its parameter documentation
    if (slew_yaw) {
        angle_ef_error_rad.z = constrain_float(angle_ef_error_rad.z,-get_slew_yaw_rads(),get_slew_yaw_rads());
    }

    // convert 321-intrinsic euler angle errors to a body-frame rotation vector
    // NOTE: this results in an approximation of the attitude error based on a linearization about the current attitude
    euler_derivative_to_ang_vel(Vector3f(_ahrs.roll,_ahrs.pitch,_ahrs.yaw), angle_ef_error_rad, _angle_bf_error_rad);

    // convert body-frame angle errors to body-frame rate targets
    update_rate_bf_targets();

    // body-frame to motor outputs should be called separately
}

// rate_ef_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw rate (all earth frame)
void AC_AttitudeControl::rate_ef_roll_pitch_yaw(float roll_rate_ef_cds, float pitch_rate_ef_cds, float yaw_rate_ef_cds)
{
    // convert from centidegrees on public interface to radians
    float roll_rate_ef_rads = radians(roll_rate_ef_cds*0.01f);
    float pitch_rate_ef_rads = radians(pitch_rate_ef_cds*0.01f);
    float yaw_rate_ef_rads = radians(yaw_rate_ef_cds*0.01f);

    Vector3f angle_ef_error_rad;
    float rate_change_limit_rads, rate_change_rads;

    if (get_accel_roll_max_radss() > 0.0f) {
        rate_change_limit_rads = get_accel_roll_max_radss() * _dt;

        // update feed forward roll rate after checking it is within acceleration limits
        rate_change_rads = roll_rate_ef_rads - _rate_ef_desired_rads.x;
        rate_change_rads = constrain_float(rate_change_rads, -rate_change_limit_rads, rate_change_limit_rads);
        _rate_ef_desired_rads.x += rate_change_rads;
    } else {
        _rate_ef_desired_rads.x = roll_rate_ef_rads;
    }

    if (get_accel_pitch_max_radss() > 0.0f) {
        rate_change_limit_rads = get_accel_pitch_max_radss() * _dt;

        // update feed forward pitch rate after checking it is within acceleration limits
        rate_change_rads = pitch_rate_ef_rads - _rate_ef_desired_rads.y;
        rate_change_rads = constrain_float(rate_change_rads, -rate_change_limit_rads, rate_change_limit_rads);
        _rate_ef_desired_rads.y += rate_change_rads;
    } else {
        _rate_ef_desired_rads.y = pitch_rate_ef_rads;
    }

    if (get_accel_yaw_max_radss() > 0.0f) {
        rate_change_limit_rads = get_accel_yaw_max_radss() * _dt;

        // update feed forward yaw rate after checking it is within acceleration limits
        rate_change_rads = yaw_rate_ef_rads - _rate_ef_desired_rads.z;
        rate_change_rads = constrain_float(rate_change_rads, -rate_change_limit_rads, rate_change_limit_rads);
        _rate_ef_desired_rads.z += rate_change_rads;
    } else {
        _rate_ef_desired_rads.z = yaw_rate_ef_rads;
    }

    // update earth frame angle targets and errors
    update_ef_roll_angle_and_error(_rate_ef_desired_rads.x, angle_ef_error_rad, AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX_RAD);
    update_ef_pitch_angle_and_error(_rate_ef_desired_rads.y, angle_ef_error_rad, AC_ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX_RAD);
    update_ef_yaw_angle_and_error(_rate_ef_desired_rads.z, angle_ef_error_rad, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX_RAD);

    // constrain earth-frame angle targets
    _angle_ef_target_rad.x = constrain_float(_angle_ef_target_rad.x, -get_tilt_limit_rad(), get_tilt_limit_rad());
    _angle_ef_target_rad.y = constrain_float(_angle_ef_target_rad.y, -get_tilt_limit_rad(), get_tilt_limit_rad());

    // convert 321-intrinsic euler angle errors to a body-frame rotation vector
    // NOTE: this results in an approximation of the attitude error based on a linearization about the current attitude
    euler_derivative_to_ang_vel(Vector3f(_ahrs.roll,_ahrs.pitch,_ahrs.yaw), angle_ef_error_rad, _angle_bf_error_rad);

    // convert body-frame angle errors to body-frame rate targets
    update_rate_bf_targets();

    // convert euler angle derivatives of desired attitude into a body-frame angular velocity vector
    euler_derivative_to_ang_vel(_angle_ef_target_rad, _rate_ef_desired_rads, _rate_bf_desired_rads);
    // NOTE: rotation of _rate_bf_desired_rads from desired body frame to estimated body frame is possibly omitted here

    // add body frame rate feed forward
    _rate_bf_target_rads += _rate_bf_desired_rads;

    // body-frame to motor outputs should be called separately
}

// rate_bf_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw rate (all body frame)
void AC_AttitudeControl::rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // convert from centidegrees on public interface to radians
    float roll_rate_bf_rads = radians(roll_rate_bf_cds*0.01f);
    float pitch_rate_bf_rads = radians(pitch_rate_bf_cds*0.01f);
    float yaw_rate_bf_rads = radians(yaw_rate_bf_cds*0.01f);

    Vector3f    angle_ef_error_rad;

    float rate_change_rads, rate_change_limit_rads;

    // update the rate feed forward with angular acceleration limits
    if (get_accel_roll_max_radss() > 0.0f) {
        rate_change_limit_rads = get_accel_roll_max_radss() * _dt;

        rate_change_rads = roll_rate_bf_rads - _rate_bf_desired_rads.x;
        rate_change_rads = constrain_float(rate_change_rads, -rate_change_limit_rads, rate_change_limit_rads);
        _rate_bf_desired_rads.x += rate_change_rads;
    } else {
        _rate_bf_desired_rads.x = roll_rate_bf_rads;
    }

    // update the rate feed forward with angular acceleration limits
    if (get_accel_pitch_max_radss() > 0.0f) {
        rate_change_limit_rads = get_accel_pitch_max_radss() * _dt;

        rate_change_rads = pitch_rate_bf_rads - _rate_bf_desired_rads.y;
        rate_change_rads = constrain_float(rate_change_rads, -rate_change_limit_rads, rate_change_limit_rads);
        _rate_bf_desired_rads.y += rate_change_rads;
    } else {
        _rate_bf_desired_rads.y = pitch_rate_bf_rads;
    }

    if (get_accel_yaw_max_radss() > 0.0f) {
        rate_change_limit_rads = get_accel_yaw_max_radss() * _dt;

        rate_change_rads = yaw_rate_bf_rads - _rate_bf_desired_rads.z;
        rate_change_rads = constrain_float(rate_change_rads, -rate_change_limit_rads, rate_change_limit_rads);
        _rate_bf_desired_rads.z += rate_change_rads;
    } else {
    	_rate_bf_desired_rads.z = yaw_rate_bf_rads;
    }

    // Update angle error
    if (fabsf(_ahrs.pitch)<_acro_angle_switch_rad) {
        _acro_angle_switch_rad = radians(60.0f);

        // convert body-frame demanded angular velocity into 321-intrinsic euler angle derivatives
        // NOTE: a rotation from vehicle body frame to demanded body frame is possibly omitted here
        // NOTE: this will never return false, since _ahrs.pitch cannot be +/- 90deg within this else statement
        ang_vel_to_euler_derivative(Vector3f(_ahrs.roll,_ahrs.pitch,_ahrs.yaw), _rate_bf_desired_rads, _rate_ef_desired_rads);

        // update earth frame angle targets and errors
        update_ef_roll_angle_and_error(_rate_ef_desired_rads.x, angle_ef_error_rad, AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD);
        update_ef_pitch_angle_and_error(_rate_ef_desired_rads.y, angle_ef_error_rad, AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD);
        update_ef_yaw_angle_and_error(_rate_ef_desired_rads.z, angle_ef_error_rad, AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD);

        // convert 321-intrinsic euler angle errors to a body-frame rotation vector
        // NOTE: this results in an approximation of the attitude error based on a linearization about the current attitude
        euler_derivative_to_ang_vel(Vector3f(_ahrs.roll,_ahrs.pitch,_ahrs.yaw), angle_ef_error_rad, _angle_bf_error_rad);
    } else {
        _acro_angle_switch_rad = radians(45.0f);
        integrate_bf_rate_error_to_angle_errors();

        // convert angle error rotation vector into 321-intrinsic euler angle difference
        // NOTE: this results an an approximation linearized about the vehicle's attitude
        if(ang_vel_to_euler_derivative(Vector3f(_ahrs.roll,_ahrs.pitch,_ahrs.yaw), _angle_bf_error_rad, angle_ef_error_rad)) {
            _angle_ef_target_rad.x = wrap_PI(angle_ef_error_rad.x + _ahrs.roll);
            _angle_ef_target_rad.y = wrap_PI(angle_ef_error_rad.y + _ahrs.pitch);
            _angle_ef_target_rad.z = wrap_2PI(angle_ef_error_rad.z + _ahrs.yaw);
        }
        if (_angle_ef_target_rad.y > M_PI/2.0f) {
            _angle_ef_target_rad.x = wrap_PI(_angle_ef_target_rad.x + M_PI);
            _angle_ef_target_rad.y = wrap_PI(M_PI - _angle_ef_target_rad.y);
            _angle_ef_target_rad.z = wrap_2PI(_angle_ef_target_rad.z + M_PI);
        }
        if (_angle_ef_target_rad.y < -M_PI/2.0f) {
            _angle_ef_target_rad.x = wrap_PI(_angle_ef_target_rad.x + M_PI);
            _angle_ef_target_rad.y = wrap_PI(-M_PI - _angle_ef_target_rad.y);
            _angle_ef_target_rad.z = wrap_2PI(_angle_ef_target_rad.z + M_PI);
        }
    }

    // convert body-frame angle errors to body-frame rate targets
    update_rate_bf_targets();

    // body-frame rate commands added
    _rate_bf_target_rads += _rate_bf_desired_rads;
}

//
// rate_controller_run - run lowest level body-frame rate controller and send outputs to the motors
//      should be called at 100hz or more
//
void AC_AttitudeControl::rate_controller_run()
{
    // call rate controllers and send output to motors object
    // To-Do: should the outputs from get_rate_roll, pitch, yaw be int16_t which is the input to the motors library?
    // To-Do: skip this step if the throttle out is zero?
    _motors.set_roll(rate_bf_to_motor_roll(_rate_bf_target_rads.x));
    _motors.set_pitch(rate_bf_to_motor_pitch(_rate_bf_target_rads.y));
    _motors.set_yaw(rate_bf_to_motor_yaw(_rate_bf_target_rads.z));
}

// converts a 321-intrinsic euler angle derivative to an angular velocity vector
void AC_AttitudeControl::euler_derivative_to_ang_vel(const Vector3f& euler_rad, const Vector3f& euler_dot_rads, Vector3f& ang_vel_rads)
{
    float sin_theta = sinf(euler_rad.y);
    float cos_theta = cosf(euler_rad.y);
    float sin_phi = sinf(euler_rad.x);
    float cos_phi = cosf(euler_rad.x);

    ang_vel_rads.x = euler_dot_rads.x - sin_theta * euler_dot_rads.z;
    ang_vel_rads.y = cos_phi  * euler_dot_rads.y + sin_phi * cos_theta * euler_dot_rads.z;
    ang_vel_rads.z = -sin_phi * euler_dot_rads.y + cos_theta * cos_phi * euler_dot_rads.z;
}

// converts an angular velocity vector to a 321-intrinsic euler angle derivative
// returns false if the vehicle is pitched all the way up or all the way down
bool AC_AttitudeControl::ang_vel_to_euler_derivative(const Vector3f& euler_rad, const Vector3f& ang_vel_rads, Vector3f& euler_dot_rads)
{
    float sin_theta = sinf(euler_rad.y);
    float cos_theta = cosf(euler_rad.y);
    float sin_phi = sinf(euler_rad.x);
    float cos_phi = cosf(euler_rad.x);

    // when the vehicle pitches all the way up or all the way down, the euler angles become discontinuous
    // in this case, we return false
    if (is_zero(cos_theta)) {
        return false;
    }

    // convert body frame angle or rates to earth frame
    euler_dot_rads.x = ang_vel_rads.x + sin_phi * (sin_theta/cos_theta) * ang_vel_rads.y + cos_phi * (sin_theta/cos_theta) * ang_vel_rads.z;
    euler_dot_rads.y = cos_phi  * ang_vel_rads.y - sin_phi * ang_vel_rads.z;
    euler_dot_rads.z = (sin_phi / cos_theta) * ang_vel_rads.y + (cos_phi / cos_theta) * ang_vel_rads.z;
    return true;
}

//
// protected methods
//

//
// stabilized rate controller (body-frame) methods
//

// update_ef_roll_angle_and_error - update _angle_ef_target.x using an earth frame roll rate request
void AC_AttitudeControl::update_ef_roll_angle_and_error(float roll_rate_ef_rads, Vector3f &angle_ef_error_rad, float overshoot_max_rad)
{
    // calculate angle error with maximum of +- max angle overshoot
    angle_ef_error_rad.x = wrap_PI(_angle_ef_target_rad.x - _ahrs.roll);
    angle_ef_error_rad.x  = constrain_float(angle_ef_error_rad.x, -overshoot_max_rad, overshoot_max_rad);

    // update roll angle target to be within max angle overshoot of our roll angle
    _angle_ef_target_rad.x = angle_ef_error_rad.x + _ahrs.roll;

    // increment the roll angle target
    _angle_ef_target_rad.x += roll_rate_ef_rads * _dt;
    _angle_ef_target_rad.x = wrap_PI(_angle_ef_target_rad.x);
}

// update_ef_pitch_angle_and_error - update _angle_ef_target.y using an earth frame pitch rate request
void AC_AttitudeControl::update_ef_pitch_angle_and_error(float pitch_rate_ef_rads, Vector3f &angle_ef_error_rad, float overshoot_max_rad)
{
    // calculate angle error with maximum of +- max angle overshoot
    // To-Do: should we do something better as we cross 90 degrees?
    angle_ef_error_rad.y = wrap_PI(_angle_ef_target_rad.y - _ahrs.pitch);
    angle_ef_error_rad.y  = constrain_float(angle_ef_error_rad.y, -overshoot_max_rad, overshoot_max_rad);

    // update pitch angle target to be within max angle overshoot of our pitch angle
    _angle_ef_target_rad.y = angle_ef_error_rad.y + _ahrs.pitch;

    // increment the pitch angle target
    _angle_ef_target_rad.y += pitch_rate_ef_rads * _dt;
    _angle_ef_target_rad.y = wrap_PI(_angle_ef_target_rad.y);
}

// update_ef_yaw_angle_and_error - update _angle_ef_target.z using an earth frame yaw rate request
void AC_AttitudeControl::update_ef_yaw_angle_and_error(float yaw_rate_ef_rads, Vector3f &angle_ef_error_rad, float overshoot_max_rad)
{
    // calculate angle error with maximum of +- max angle overshoot
    angle_ef_error_rad.z = wrap_PI(_angle_ef_target_rad.z - _ahrs.yaw);
    angle_ef_error_rad.z  = constrain_float(angle_ef_error_rad.z, -overshoot_max_rad, overshoot_max_rad);

    // update yaw angle target to be within max angle overshoot of our current heading
    _angle_ef_target_rad.z = angle_ef_error_rad.z + _ahrs.yaw;

    // increment the yaw angle target
    _angle_ef_target_rad.z += yaw_rate_ef_rads * _dt;
    _angle_ef_target_rad.z = wrap_2PI(_angle_ef_target_rad.z);
}

// update_rate_bf_errors - calculates body frame angle errors
//   body-frame feed forward rates (radians/s) taken from _angle_bf_error
//   angle errors in radians placed in _angle_bf_error
void AC_AttitudeControl::integrate_bf_rate_error_to_angle_errors()
{
    // roll - calculate body-frame angle error by integrating body-frame rate error
    _angle_bf_error_rad.x += (_rate_bf_desired_rads.x - _ahrs.get_gyro().x) * _dt;
    // roll - limit maximum error
    _angle_bf_error_rad.x = constrain_float(_angle_bf_error_rad.x, -AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD, AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD);


    // pitch - calculate body-frame angle error by integrating body-frame rate error
    _angle_bf_error_rad.y += (_rate_bf_desired_rads.y - _ahrs.get_gyro().y) * _dt;
    // pitch - limit maximum error
    _angle_bf_error_rad.y = constrain_float(_angle_bf_error_rad.y, -AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD, AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD);


    // yaw - calculate body-frame angle error by integrating body-frame rate error
    _angle_bf_error_rad.z += (_rate_bf_desired_rads.z - _ahrs.get_gyro().z) * _dt;
    // yaw - limit maximum error
    _angle_bf_error_rad.z = constrain_float(_angle_bf_error_rad.z, -AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD, AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD);

    // To-Do: handle case of motors being disarmed or channel_throttle == 0 and set error to zero
}

// update_rate_bf_targets - converts body-frame angle error to body-frame rate targets for roll, pitch and yaw axis
//   targets rates in radians/s taken from _angle_bf_error
//   results in radians/s put into _rate_bf_target
void AC_AttitudeControl::update_rate_bf_targets()
{

    // stab roll calculation
    // constrain roll rate request

    if (_flags.limit_angle_to_rate_request && _accel_roll_max > 0.0f) {
        _rate_bf_target_rads.x = sqrt_controller(_angle_bf_error_rad.x, _p_angle_roll.kP(), constrain_float(get_accel_roll_max_radss()/2.0f,  AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS));
    }else{
        _rate_bf_target_rads.x = _p_angle_roll.kP() * _angle_bf_error_rad.x;
    }

    // stab pitch calculation
    // constrain pitch rate request
    if (_flags.limit_angle_to_rate_request && _accel_pitch_max > 0.0f) {
        _rate_bf_target_rads.y = sqrt_controller(_angle_bf_error_rad.y, _p_angle_pitch.kP(), constrain_float(get_accel_pitch_max_radss()/2.0f,  AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS));
    }else{
        _rate_bf_target_rads.y = _p_angle_pitch.kP() * _angle_bf_error_rad.y;
    }

    // stab yaw calculation
    // constrain yaw rate request
    if (_flags.limit_angle_to_rate_request && _accel_yaw_max > 0.0f) {
        _rate_bf_target_rads.z = sqrt_controller(_angle_bf_error_rad.z, _p_angle_yaw.kP(), constrain_float(get_accel_yaw_max_radss()/2.0f,  AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS));
    }else{
        _rate_bf_target_rads.z = _p_angle_yaw.kP() * _angle_bf_error_rad.z;
    }

    // include roll and pitch rate required to account for precession of the desired attitude about the body frame yaw axes
	_rate_bf_target_rads.x += _angle_bf_error_rad.y * _ahrs.get_gyro().z;
	_rate_bf_target_rads.y += -_angle_bf_error_rad.x * _ahrs.get_gyro().z;
}

//
// body-frame rate controller
//

// rate_bf_to_motor_roll - ask the rate controller to calculate the motor outputs to achieve the target rate in radians/s
float AC_AttitudeControl::rate_bf_to_motor_roll(float rate_target_rads)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate_rads;     // this iteration's rate
    float rate_error_rads;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate_rads = _ahrs.get_gyro().x;

    // calculate error and call pid controller
    rate_error_rads = rate_target_rads - current_rate_rads;

    // For legacy reasons, we convert to centi-degrees before inputting to the PID
    _pid_rate_roll.set_input_filter_d(degrees(rate_error_rads)*100.0f);
    _pid_rate_roll.set_desired_rate(degrees(rate_target_rads)*100.0f);

    // get p value
    p = _pid_rate_roll.get_p();

    // get i term
    i = _pid_rate_roll.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.roll_pitch || ((i>0&&rate_error_rads<0)||(i<0&&rate_error_rads>0))) {
        i = _pid_rate_roll.get_i();
    }

    // get d term
    d = _pid_rate_roll.get_d();

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
}

// rate_bf_to_motor_pitch - ask the rate controller to calculate the motor outputs to achieve the target rate in radians/s
float AC_AttitudeControl::rate_bf_to_motor_pitch(float rate_target_rads)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate_rads;     // this iteration's rate
    float rate_error_rads;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate_rads = _ahrs.get_gyro().y;

    // calculate error and call pid controller
    rate_error_rads = rate_target_rads - current_rate_rads;

    // For legacy reasons, we convert to centi-degrees before inputting to the PID
    _pid_rate_pitch.set_input_filter_d(degrees(rate_error_rads)*100.0f);
    _pid_rate_pitch.set_desired_rate(degrees(rate_target_rads)*100.0f);

    // get p value
    p = _pid_rate_pitch.get_p();

    // get i term
    i = _pid_rate_pitch.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.roll_pitch || ((i>0&&rate_error_rads<0)||(i<0&&rate_error_rads>0))) {
        i = _pid_rate_pitch.get_i();
    }

    // get d term
    d = _pid_rate_pitch.get_d();

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
}

// rate_bf_to_motor_yaw - ask the rate controller to calculate the motor outputs to achieve the target rate in radians/s
float AC_AttitudeControl::rate_bf_to_motor_yaw(float rate_target_rads)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate_rads;     // this iteration's rate
    float rate_error_rads;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate_rads = _ahrs.get_gyro().z;

    // calculate error and call pid controller
    rate_error_rads  = rate_target_rads - current_rate_rads;

    // For legacy reasons, we convert to centi-degrees before inputting to the PID
    _pid_rate_yaw.set_input_filter_all(degrees(rate_error_rads)*100.0f);
    _pid_rate_yaw.set_desired_rate(degrees(rate_target_rads)*100.0f);

    // get p value
    p = _pid_rate_yaw.get_p();

    // get i term
    i = _pid_rate_yaw.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.yaw || ((i>0&&rate_error_rads<0)||(i<0&&rate_error_rads>0))) {
        i = _pid_rate_yaw.get_i();
    }

    // get d value
    d = _pid_rate_yaw.get_d();

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX);
}

// accel_limiting - enable or disable accel limiting
void AC_AttitudeControl::accel_limiting(bool enable_limits)
{
    if (enable_limits) {
        // if enabling limits, reload from eeprom or set to defaults
        if (is_zero(_accel_roll_max)) {
            _accel_roll_max.load();
        }
        // if enabling limits, reload from eeprom or set to defaults
        if (is_zero(_accel_pitch_max)) {
            _accel_pitch_max.load();
        }
        if (is_zero(_accel_yaw_max)) {
            _accel_yaw_max.load();
        }
    } else {
        // if disabling limits, set to zero
        _accel_roll_max = 0.0f;
        _accel_pitch_max = 0.0f;
        _accel_yaw_max = 0.0f;
    }
}

//
// throttle functions
//

 // set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
 // provide 0 to cut motors
void AC_AttitudeControl::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in_filt.apply(throttle_in, _dt);
    _motors.set_stabilizing(true);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (apply_angle_boost) {
        _motors.set_throttle(get_boosted_throttle(throttle_in));
    }else{
        _motors.set_throttle(throttle_in);
        // clear angle_boost for logging purposes
        _angle_boost = 0;
    }
}

// outputs a throttle to all motors evenly with no attitude stabilization
void AC_AttitudeControl::set_throttle_out_unstabilized(float throttle_in, bool reset_attitude_control, float filter_cutoff)
{
    _throttle_in_filt.apply(throttle_in, _dt);
    if (reset_attitude_control) {
        relax_bf_rate_controller();
        set_yaw_target_to_current_heading();
    }
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    _motors.set_stabilizing(false);
    _motors.set_throttle(throttle_in);
    _angle_boost = 0;
}

// sqrt_controller - response based on the sqrt of the error instead of the more common linear response
float AC_AttitudeControl::sqrt_controller(float error, float p, float second_ord_lim)
{
    if (is_zero(second_ord_lim) || is_zero(p)) {
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

// Maximum roll rate step size in centidegrees that results in maximum output after 4 time steps
// NOTE: for legacy reasons, the output of this function is in centidegrees
float AC_AttitudeControl::max_rate_step_bf_roll()
{
    float alpha = _pid_rate_roll.get_filt_alpha();
    float alpha_remaining = 1-alpha;
    return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*_pid_rate_roll.kD())/_dt + _pid_rate_roll.kP());
}

// Maximum pitch rate step size that results in maximum output after 4 time steps
// NOTE: for legacy reasons, the output of this function is in centidegrees
float AC_AttitudeControl::max_rate_step_bf_pitch()
{
    float alpha = _pid_rate_pitch.get_filt_alpha();
    float alpha_remaining = 1-alpha;
    return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*_pid_rate_pitch.kD())/_dt + _pid_rate_pitch.kP());
}

// Maximum yaw rate step size that results in maximum output after 4 time steps
// NOTE: for legacy reasons, the output of this function is in centidegrees
float AC_AttitudeControl::max_rate_step_bf_yaw()
{
    float alpha = _pid_rate_yaw.get_filt_alpha();
    float alpha_remaining = 1-alpha;
    return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*_pid_rate_yaw.kD())/_dt + _pid_rate_yaw.kP());
}
