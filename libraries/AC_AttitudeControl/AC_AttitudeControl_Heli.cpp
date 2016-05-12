// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl_Heli.h"
#include <AP_HAL/AP_HAL.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Heli::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    // @Param: PIRO_COMP
    // @DisplayName: Piro Comp Enable
    // @Description: Pirouette compensation enabled
    // @Values: 0:Disabled 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("PIRO_COMP",    0, AC_AttitudeControl_Heli, _piro_comp_enabled, 0),

    // @Param: HOVR_ROL_TRM
    // @DisplayName: Hover Roll Trim
    // @Description: Trim the hover roll angle to counter tail rotor thrust in a hover
    // @Units: Centi-Degrees
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("HOVR_ROL_TRM",    1, AC_AttitudeControl_Heli, _hover_roll_trim, AC_ATTITUDE_HELI_HOVER_ROLL_TRIM_DEFAULT),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.08 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.001 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FILT
    // @DisplayName: Roll axis rate conroller input frequency in Hz
    // @Description: Roll axis rate conroller input frequency in Hz
    // @Units: Hz
    // @Range: 1 20
    // @Increment: 1
    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 2, AC_AttitudeControl_Heli, AC_HELI_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
    // @Range: 0.08 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.001 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FILT
    // @DisplayName: Pitch axis rate conroller input frequency in Hz
    // @Description: Pitch axis rate conroller input frequency in Hz
    // @Units: Hz
    // @Range: 1 20
    // @Increment: 1
    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 3, AC_AttitudeControl_Heli, AC_HELI_PID),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
    // @Range: 0.180 0.60
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.01 0.06
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FILT
    // @DisplayName: Yaw axis rate conroller input frequency in Hz
    // @Description: Yaw axis rate conroller input frequency in Hz
    // @Units: Hz
    // @Range: 1 20
    // @Increment: 1
    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 4, AC_AttitudeControl_Heli, AC_HELI_PID),

    AP_GROUPEND
};

// passthrough_bf_roll_pitch_rate_yaw - passthrough the pilots roll and pitch inputs directly to swashplate for flybar acro mode
void AC_AttitudeControl_Heli::passthrough_bf_roll_pitch_rate_yaw(float roll_passthrough, float pitch_passthrough, float yaw_rate_bf_cds)
{
    // convert from centidegrees on public interface to radians
    float yaw_rate_bf_rads = radians(yaw_rate_bf_cds*0.01f);

    // store roll, pitch and passthroughs
    // NOTE: this abuses yaw_rate_bf_rads
    _passthrough_roll = roll_passthrough;
    _passthrough_pitch = pitch_passthrough;
    _passthrough_yaw = degrees(yaw_rate_bf_rads)*100.0f;

    // set rate controller to use pass through
    _flags_heli.flybar_passthrough = true;

    // set bf rate targets to current body frame rates (i.e. relax and be ready for vehicle to switch out of acro)
    _att_target_ang_vel_rads.x = _ahrs.get_gyro().x;
    _att_target_ang_vel_rads.y = _ahrs.get_gyro().y;

    // accel limit desired yaw rate
    if (get_accel_yaw_max_radss() > 0.0f) {
        float rate_change_limit_rads = get_accel_yaw_max_radss() * _dt;
        float rate_change_rads = yaw_rate_bf_rads - _att_target_ang_vel_rads.z;
        rate_change_rads = constrain_float(rate_change_rads, -rate_change_limit_rads, rate_change_limit_rads);
        _att_target_ang_vel_rads.z += rate_change_rads;
    } else {
        _att_target_ang_vel_rads.z = yaw_rate_bf_rads;
    }

    integrate_bf_rate_error_to_angle_errors();
    _att_error_rot_vec_rad.x = 0;
    _att_error_rot_vec_rad.y = 0;

    // update our earth-frame angle targets
    Vector3f att_error_euler_rad;

    // convert angle error rotation vector into 321-intrinsic euler angle difference
    // NOTE: this results an an approximation linearized about the vehicle's attitude
    if (ang_vel_to_euler_rate(Vector3f(_ahrs.roll,_ahrs.pitch,_ahrs.yaw), _att_error_rot_vec_rad, att_error_euler_rad)) {
        _att_target_euler_rad.x = wrap_PI(att_error_euler_rad.x + _ahrs.roll);
        _att_target_euler_rad.y = wrap_PI(att_error_euler_rad.y + _ahrs.pitch);
        _att_target_euler_rad.z = wrap_2PI(att_error_euler_rad.z + _ahrs.yaw);
    }

    // handle flipping over pitch axis
    if (_att_target_euler_rad.y > M_PI/2.0f) {
        _att_target_euler_rad.x = wrap_PI(_att_target_euler_rad.x + M_PI);
        _att_target_euler_rad.y = wrap_PI(M_PI - _att_target_euler_rad.x);
        _att_target_euler_rad.z = wrap_2PI(_att_target_euler_rad.z + M_PI);
    }
    if (_att_target_euler_rad.y < -M_PI/2.0f) {
        _att_target_euler_rad.x = wrap_PI(_att_target_euler_rad.x + M_PI);
        _att_target_euler_rad.y = wrap_PI(-M_PI - _att_target_euler_rad.x);
        _att_target_euler_rad.z = wrap_2PI(_att_target_euler_rad.z + M_PI);
    }

    // convert body-frame angle errors to body-frame rate targets
    update_ang_vel_target_from_att_error();

    // set body-frame roll/pitch rate target to current desired rates which are the vehicle's actual rates
    _ang_vel_target_rads.x = _att_target_ang_vel_rads.x;
    _ang_vel_target_rads.y = _att_target_ang_vel_rads.y;

    // add desired target to yaw
    _ang_vel_target_rads.z += _att_target_ang_vel_rads.z;
}

// subclass non-passthrough too, for external gyro, no flybar
void AC_AttitudeControl_Heli::input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    _passthrough_yaw = yaw_rate_bf_cds;

    AC_AttitudeControl::input_rate_bf_roll_pitch_yaw(roll_rate_bf_cds, pitch_rate_bf_cds, yaw_rate_bf_cds);
}

//
// rate controller (body-frame) methods
//

// rate_controller_run - run lowest level rate controller and send outputs to the motors
// should be called at 100hz or more
void AC_AttitudeControl_Heli::rate_controller_run()
{	
    // call rate controllers and send output to motors object
    // if using a flybar passthrough roll and pitch directly to motors
    if (_flags_heli.flybar_passthrough) {
        _motors.set_roll(_passthrough_roll/4500.0f);
        _motors.set_pitch(_passthrough_pitch/4500.0f);
    } else {
        rate_bf_to_motor_roll_pitch(_ang_vel_target_rads.x, _ang_vel_target_rads.y);
    }
    if (_flags_heli.tail_passthrough) {
        _motors.set_yaw(_passthrough_yaw/4500.0f);
    } else {
        _motors.set_yaw(rate_bf_to_motor_yaw(_ang_vel_target_rads.z));
    }
}

// get lean angle max for pilot input that prioritises altitude hold over lean angle
float AC_AttitudeControl_Heli::get_althold_lean_angle_max() const
{
    // calc maximum tilt angle based on throttle
    float ret = acosf(constrain_float(_throttle_in_filt.get()/0.9f, 0.0f, 1.0f));

    // TEMP: convert to centi-degrees for public interface
    return degrees(ret) * 100.0f;
}

//
// private methods
//

//
// body-frame rate controller
//

// rate_bf_to_motor_roll_pitch - ask the rate controller to calculate the motor outputs to achieve the target rate in radians/second
void AC_AttitudeControl_Heli::rate_bf_to_motor_roll_pitch(float rate_roll_target_rads, float rate_pitch_target_rads)
{
    float roll_pd, roll_i, roll_ff;             // used to capture pid values
    float pitch_pd, pitch_i, pitch_ff;          // used to capture pid values
    float rate_roll_error_rads, rate_pitch_error_rads;    // simply target_rate - current_rate
    float roll_out, pitch_out;
    const Vector3f& gyro = _ahrs.get_gyro();     // get current rates

    // calculate error
    rate_roll_error_rads = rate_roll_target_rads - gyro.x;
    rate_pitch_error_rads = rate_pitch_target_rads - gyro.y;

    // pass error to PID controller
    _pid_rate_roll.set_input_filter_all(rate_roll_error_rads);
    _pid_rate_roll.set_desired_rate(rate_roll_target_rads);
    _pid_rate_pitch.set_input_filter_all(rate_pitch_error_rads);
    _pid_rate_pitch.set_desired_rate(rate_pitch_target_rads);

    // call p and d controllers
    roll_pd = _pid_rate_roll.get_p() + _pid_rate_roll.get_d();
    pitch_pd = _pid_rate_pitch.get_p() + _pid_rate_pitch.get_d();

    // get roll i term
    roll_i = _pid_rate_roll.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_flags_heli.limit_roll || ((roll_i>0&&rate_roll_error_rads<0)||(roll_i<0&&rate_roll_error_rads>0))){
		if (_flags_heli.leaky_i){
			roll_i = _pid_rate_roll.get_leaky_i(AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
		}else{
			roll_i = _pid_rate_roll.get_i();
		}
    }

    // get pitch i term
    pitch_i = _pid_rate_pitch.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_flags_heli.limit_pitch || ((pitch_i>0&&rate_pitch_error_rads<0)||(pitch_i<0&&rate_pitch_error_rads>0))){
		if (_flags_heli.leaky_i) {
			pitch_i = _pid_rate_pitch.get_leaky_i(AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
		}else{
			pitch_i = _pid_rate_pitch.get_i();
		}
    }
    
    // For legacy reasons, we convert to centi-degrees before inputting to the feedforward
    roll_ff = roll_feedforward_filter.apply(_pid_rate_roll.get_vff(rate_roll_target_rads), _dt);
    pitch_ff = pitch_feedforward_filter.apply(_pid_rate_pitch.get_vff(rate_pitch_target_rads), _dt);

    // add feed forward and final output
    roll_out = roll_pd + roll_i + roll_ff;
    pitch_out = pitch_pd + pitch_i + pitch_ff;

    // constrain output and update limit flags
    if (fabsf(roll_out) > AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX) {
        roll_out = constrain_float(roll_out,-AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX,AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
        _flags_heli.limit_roll = true;
    }else{
        _flags_heli.limit_roll = false;
    }
    if (fabsf(pitch_out) > AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX) {
        pitch_out = constrain_float(pitch_out,-AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX,AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
        _flags_heli.limit_pitch = true;
    }else{
        _flags_heli.limit_pitch = false;
    }

    // output to motors
    _motors.set_roll(roll_out);
    _motors.set_pitch(pitch_out);

    // Piro-Comp, or Pirouette Compensation is a pre-compensation calculation, which basically rotates the Roll and Pitch Rate I-terms as the
    // helicopter rotates in yaw.  Much of the built-up I-term is needed to tip the disk into the incoming wind.  Fast yawing can create an instability
    // as the built-up I-term in one axis must be reduced, while the other increases.  This helps solve that by rotating the I-terms before the error occurs.
    // It does assume that the rotor aerodynamics and mechanics are essentially symmetrical about the main shaft, which is a generally valid assumption. 
    if (_piro_comp_enabled){

        int32_t         piro_roll_i, piro_pitch_i;            // used to hold I-terms while doing piro comp

        piro_roll_i  = roll_i;
        piro_pitch_i = pitch_i;

        Vector2f yawratevector;
        yawratevector.x     = cosf(-_ahrs.get_gyro().z * _dt);
        yawratevector.y     = sinf(-_ahrs.get_gyro().z * _dt);
        yawratevector.normalize();

        roll_i      = piro_roll_i * yawratevector.x - piro_pitch_i * yawratevector.y;
        pitch_i     = piro_pitch_i * yawratevector.x + piro_roll_i * yawratevector.y;

        _pid_rate_pitch.set_integrator(pitch_i);
        _pid_rate_roll.set_integrator(roll_i);
    }

}

// rate_bf_to_motor_yaw - ask the rate controller to calculate the motor outputs to achieve the target rate in radians/second
float AC_AttitudeControl_Heli::rate_bf_to_motor_yaw(float rate_target_rads)
{
    float pd,i,vff;     // used to capture pid values for logging
    float current_rate_rads;     // this iteration's rate
    float rate_error_rads;       // simply target_rate - current_rate
    float yaw_out;

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate_rads = _ahrs.get_gyro().z;

    // calculate error and call pid controller
    rate_error_rads  = rate_target_rads - current_rate_rads;

    // pass error to PID controller
    _pid_rate_yaw.set_input_filter_all(rate_error_rads);
    _pid_rate_yaw.set_desired_rate(rate_target_rads);

    // get p and d
    pd = _pid_rate_yaw.get_p() + _pid_rate_yaw.get_d();

    // get i term
    i = _pid_rate_yaw.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_flags_heli.limit_yaw || ((i>0&&rate_error_rads<0)||(i<0&&rate_error_rads>0))) {
        if (((AP_MotorsHeli&)_motors).rotor_runup_complete()) {
            i = _pid_rate_yaw.get_i();
        } else {
            i = ((AC_HELI_PID&)_pid_rate_yaw).get_leaky_i(AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);    // If motor is not running use leaky I-term to avoid excessive build-up
        }
    }
    
    // For legacy reasons, we convert to centi-degrees before inputting to the feedforward
    vff = yaw_velocity_feedforward_filter.apply(_pid_rate_yaw.get_vff(rate_target_rads), _dt);
    
    // add feed forward
    yaw_out = pd + i + vff;

    // constrain output and update limit flag
    if (fabsf(yaw_out) > AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX) {
        yaw_out = constrain_float(yaw_out,-AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX,AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX);
        _flags_heli.limit_yaw = true;
    }else{
        _flags_heli.limit_yaw = false;
    }

    // output to motors
    return yaw_out;
}

//
// throttle functions
//

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
float AC_AttitudeControl_Heli::get_boosted_throttle(float throttle_in)
{
    // no angle boost for trad helis
    _angle_boost = 0.0f;
    return throttle_in;
}
