#include "AC_AttitudeControl_Heli.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Heli::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    // @Param: HOVR_ROL_TRM
    // @DisplayName: Hover Roll Trim
    // @Description: Trim the hover roll angle to counter tail rotor thrust in a hover
    // @Units: cdeg
    // @Increment: 10
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("HOVR_ROL_TRM",    1, AC_AttitudeControl_Heli, _hover_roll_trim, AC_ATTITUDE_HELI_HOVER_ROLL_TRIM_DEFAULT),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Corrects in proportion to the difference between the desired roll rate vs actual roll rate
    // @Range: 0.0 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.0 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_ILMI
    // @DisplayName: Roll axis rate controller I-term leak minimum
    // @Description: Point below which I-term will not leak down
    // @Range: 0 1
    // @User: Advanced

    // @Param: RAT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.0 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0.05 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_RLL_D_FF
    // @DisplayName: Roll Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_RLL_NTF
    // @DisplayName: Roll Target notch filter index
    // @Description: Roll Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: RAT_RLL_NEF
    // @DisplayName: Roll Error notch filter index
    // @Description: Roll Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 2, AC_AttitudeControl_Heli, AC_HELI_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Corrects in proportion to the difference between the desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_ILMI
    // @DisplayName: Pitch axis rate controller I-term leak minimum
    // @Description: Point below which I-term will not leak down
    // @Range: 0 1
    // @User: Advanced

    // @Param: RAT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0.05 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_PIT_D_FF
    // @DisplayName: Pitch Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_PIT_NTF
    // @DisplayName: Pitch Target notch filter index
    // @Description: Pitch Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: RAT_PIT_NEF
    // @DisplayName: Pitch Error notch filter index
    // @Description: Pitch Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 3, AC_AttitudeControl_Heli, AC_HELI_PID),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Corrects in proportion to the difference between the desired yaw rate vs actual yaw rate
    // @Range: 0.180 0.60
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.01 0.2
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_ILMI
    // @DisplayName: Yaw axis rate controller I-term leak minimum
    // @Description: Point below which I-term will not leak down
    // @Range: 0 1
    // @User: Advanced

    // @Param: RAT_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FF
    // @DisplayName: Yaw axis rate controller feed forward
    // @Description: Yaw axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FLTT
    // @DisplayName: Yaw axis rate controller target frequency in Hz
    // @Description: Yaw axis rate controller target frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTE
    // @DisplayName: Yaw axis rate controller error frequency in Hz
    // @Description: Yaw axis rate controller error frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTD
    // @DisplayName: Yaw axis rate controller derivative frequency in Hz
    // @Description: Yaw axis rate controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_SMAX
    // @DisplayName: Yaw slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_YAW_D_FF
    // @DisplayName: Yaw Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_YAW_NTF
    // @DisplayName: Yaw Target notch filter index
    // @Description: Yaw Target notch filter index
    // @Range: 1 8
    // @Units: Hz
    // @User: Advanced

    // @Param: RAT_YAW_NEF
    // @DisplayName: Yaw Error notch filter index
    // @Description: Yaw Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 4, AC_AttitudeControl_Heli, AC_HELI_PID),

    // @Param: PIRO_COMP
    // @DisplayName: Piro Comp Enable
    // @Description: Pirouette compensation enabled
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("PIRO_COMP",    5, AC_AttitudeControl_Heli, _piro_comp_enabled, 0),
    
    AP_GROUPEND
};

AC_AttitudeControl_Heli::AC_AttitudeControl_Heli(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsHeli& motors) :
    AC_AttitudeControl(ahrs, aparm, motors),
    _pid_rate_roll(AC_ATC_HELI_RATE_RP_P, AC_ATC_HELI_RATE_RP_I, AC_ATC_HELI_RATE_RP_D, AC_ATC_HELI_RATE_RP_FF, AC_ATC_HELI_RATE_RP_IMAX, AC_ATTITUDE_HELI_RATE_RP_FF_FILTER, AC_ATC_HELI_RATE_RP_FILT_HZ, 0.0f),
    _pid_rate_pitch(AC_ATC_HELI_RATE_RP_P, AC_ATC_HELI_RATE_RP_I, AC_ATC_HELI_RATE_RP_D, AC_ATC_HELI_RATE_RP_FF, AC_ATC_HELI_RATE_RP_IMAX, AC_ATTITUDE_HELI_RATE_RP_FF_FILTER, AC_ATC_HELI_RATE_RP_FILT_HZ, 0.0f),
    _pid_rate_yaw(AC_ATC_HELI_RATE_YAW_P, AC_ATC_HELI_RATE_YAW_I, AC_ATC_HELI_RATE_YAW_D, AC_ATC_HELI_RATE_YAW_FF, AC_ATC_HELI_RATE_YAW_IMAX, AC_ATTITUDE_HELI_RATE_Y_FF_FILTER, AC_ATC_HELI_RATE_YAW_FILT_HZ, 0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise flags
    _flags_heli.leaky_i = true;
    _flags_heli.flybar_passthrough = false;
    _flags_heli.tail_passthrough = false;
#if AP_FILTER_ENABLED
    set_notch_sample_rate(AP::scheduler().get_loop_rate_hz());
#endif
}

// passthrough_bf_roll_pitch_rate_yaw - passthrough the pilots roll and pitch inputs directly to swashplate for flybar acro mode
void AC_AttitudeControl_Heli::passthrough_bf_roll_pitch_rate_yaw(float roll_passthrough, float pitch_passthrough, float yaw_rate_bf_cds)
{
    // convert from centidegrees on public interface to radians
    float yaw_rate_bf_rads = radians(yaw_rate_bf_cds * 0.01f);

    // store roll, pitch and passthroughs
    // NOTE: this abuses yaw_rate_bf_rads
    _passthrough_roll = roll_passthrough;
    _passthrough_pitch = pitch_passthrough;
    _passthrough_yaw = degrees(yaw_rate_bf_rads) * 100.0f;

    // set rate controller to use pass through
    _flags_heli.flybar_passthrough = true;

    // set bf rate targets to current body frame rates (i.e. relax and be ready for vehicle to switch out of acro)
    _ang_vel_target.x = _ahrs.get_gyro().x;
    _ang_vel_target.y = _ahrs.get_gyro().y;

    // accel limit desired yaw rate
    if (get_accel_yaw_max_radss() > 0.0f) {
        float rate_change_limit_rads = get_accel_yaw_max_radss() * _dt;
        float rate_change_rads = yaw_rate_bf_rads - _ang_vel_target.z;
        rate_change_rads = constrain_float(rate_change_rads, -rate_change_limit_rads, rate_change_limit_rads);
        _ang_vel_target.z += rate_change_rads;
    } else {
        _ang_vel_target.z = yaw_rate_bf_rads;
    }

    integrate_bf_rate_error_to_angle_errors();
    _att_error_rot_vec_rad.x = 0;
    _att_error_rot_vec_rad.y = 0;

    // update our earth-frame angle targets
    Vector3f att_error_euler_rad;

    // convert angle error rotation vector into 321-intrinsic euler angle difference
    // NOTE: this results an an approximation linearized about the vehicle's attitude
    Quaternion att;
    _ahrs.get_quat_body_to_ned(att);
    if (ang_vel_to_euler_rate(att, _att_error_rot_vec_rad, att_error_euler_rad)) {
        _euler_angle_target.x = wrap_PI(att_error_euler_rad.x + _ahrs.roll);
        _euler_angle_target.y = wrap_PI(att_error_euler_rad.y + _ahrs.pitch);
        _euler_angle_target.z = wrap_2PI(att_error_euler_rad.z + _ahrs.yaw);
    }

    // handle flipping over pitch axis
    if (_euler_angle_target.y > M_PI / 2.0f) {
        _euler_angle_target.x = wrap_PI(_euler_angle_target.x + M_PI);
        _euler_angle_target.y = wrap_PI(M_PI - _euler_angle_target.x);
        _euler_angle_target.z = wrap_2PI(_euler_angle_target.z + M_PI);
    }
    if (_euler_angle_target.y < -M_PI / 2.0f) {
        _euler_angle_target.x = wrap_PI(_euler_angle_target.x + M_PI);
        _euler_angle_target.y = wrap_PI(-M_PI - _euler_angle_target.x);
        _euler_angle_target.z = wrap_2PI(_euler_angle_target.z + M_PI);
    }

    // convert body-frame angle errors to body-frame rate targets
    _ang_vel_body = update_ang_vel_target_from_att_error(_att_error_rot_vec_rad);

    // set body-frame roll/pitch rate target to current desired rates which are the vehicle's actual rates
    _ang_vel_body.x = _ang_vel_target.x;
    _ang_vel_body.y = _ang_vel_target.y;

    // add desired target to yaw
    _ang_vel_body.z += _ang_vel_target.z;
    _thrust_error_angle = _att_error_rot_vec_rad.xy().length();
}

void AC_AttitudeControl_Heli::integrate_bf_rate_error_to_angle_errors()
{
    // Integrate the angular velocity error into the attitude error
    _att_error_rot_vec_rad += (_ang_vel_target - _ahrs.get_gyro()) * _dt;

    // Constrain attitude error
    _att_error_rot_vec_rad.x = constrain_float(_att_error_rot_vec_rad.x, -AC_ATTITUDE_HELI_ACRO_OVERSHOOT_ANGLE_RAD, AC_ATTITUDE_HELI_ACRO_OVERSHOOT_ANGLE_RAD);
    _att_error_rot_vec_rad.y = constrain_float(_att_error_rot_vec_rad.y, -AC_ATTITUDE_HELI_ACRO_OVERSHOOT_ANGLE_RAD, AC_ATTITUDE_HELI_ACRO_OVERSHOOT_ANGLE_RAD);
    _att_error_rot_vec_rad.z = constrain_float(_att_error_rot_vec_rad.z, -AC_ATTITUDE_HELI_ACRO_OVERSHOOT_ANGLE_RAD, AC_ATTITUDE_HELI_ACRO_OVERSHOOT_ANGLE_RAD);
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
    _ang_vel_body += _sysid_ang_vel_body;

    _rate_gyro = _ahrs.get_gyro_latest();
    _rate_gyro_time_us = AP_HAL::micros64();

    // call rate controllers and send output to motors object
    // if using a flybar passthrough roll and pitch directly to motors
    if (_flags_heli.flybar_passthrough) {
        _motors.set_roll(_passthrough_roll / 4500.0f);
        _motors.set_pitch(_passthrough_pitch / 4500.0f);
    } else {
        rate_bf_to_motor_roll_pitch(_rate_gyro, _ang_vel_body.x, _ang_vel_body.y);
    }
    if (_flags_heli.tail_passthrough) {
        _motors.set_yaw(_passthrough_yaw / 4500.0f);
    } else {
        _motors.set_yaw(rate_target_to_motor_yaw(_rate_gyro.z, _ang_vel_body.z));
    }

    _sysid_ang_vel_body.zero();
    _actuator_sysid.zero();

}

// Update Alt_Hold angle maximum
void AC_AttitudeControl_Heli::update_althold_lean_angle_max(float throttle_in)
{
    float althold_lean_angle_max = acosf(constrain_float(throttle_in / AC_ATTITUDE_HELI_ANGLE_LIMIT_THROTTLE_MAX, 0.0f, 1.0f));
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt / (_dt + _angle_limit_tc)) * (althold_lean_angle_max - _althold_lean_angle_max);
}

//
// private methods
//

//
// body-frame rate controller
//

// rate_bf_to_motor_roll_pitch - ask the rate controller to calculate the motor outputs to achieve the target rate in radians/second
void AC_AttitudeControl_Heli::rate_bf_to_motor_roll_pitch(const Vector3f &rate_rads, float rate_roll_target_rads, float rate_pitch_target_rads)
{

    if (_flags_heli.leaky_i) {
        _pid_rate_roll.update_leaky_i(AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
    }
    float roll_pid = _pid_rate_roll.update_all(rate_roll_target_rads, rate_rads.x, _dt, _motors.limit.roll) + _actuator_sysid.x;

    if (_flags_heli.leaky_i) {
        _pid_rate_pitch.update_leaky_i(AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
    }

    float pitch_pid = _pid_rate_pitch.update_all(rate_pitch_target_rads, rate_rads.y, _dt, _motors.limit.pitch) + _actuator_sysid.y;

    // use pid library to calculate ff
    float roll_ff = _pid_rate_roll.get_ff();
    float pitch_ff = _pid_rate_pitch.get_ff();

    // add feed forward and final output
    float roll_out = roll_pid + roll_ff;
    float pitch_out = pitch_pid + pitch_ff;

    // constrain output
    roll_out = constrain_float(roll_out, -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
    pitch_out = constrain_float(pitch_out, -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);

    // output to motors
    _motors.set_roll(roll_out);
    _motors.set_pitch(pitch_out);

    // Piro-Comp, or Pirouette Compensation is a pre-compensation calculation, which basically rotates the Roll and Pitch Rate I-terms as the
    // helicopter rotates in yaw.  Much of the built-up I-term is needed to tip the disk into the incoming wind.  Fast yawing can create an instability
    // as the built-up I-term in one axis must be reduced, while the other increases.  This helps solve that by rotating the I-terms before the error occurs.
    // It does assume that the rotor aerodynamics and mechanics are essentially symmetrical about the main shaft, which is a generally valid assumption. 
    if (_piro_comp_enabled) {

        // used to hold current I-terms while doing piro comp:
        const float piro_roll_i = _pid_rate_roll.get_i();
        const float piro_pitch_i = _pid_rate_pitch.get_i();

        Vector2f yawratevector;
        yawratevector.x     = cosf(-rate_rads.z * _dt);
        yawratevector.y     = sinf(-rate_rads.z * _dt);
        yawratevector.normalize();

        _pid_rate_roll.set_integrator(piro_roll_i * yawratevector.x - piro_pitch_i * yawratevector.y);
        _pid_rate_pitch.set_integrator(piro_pitch_i * yawratevector.x + piro_roll_i * yawratevector.y);
    }

}

// rate_bf_to_motor_yaw - ask the rate controller to calculate the motor outputs to achieve the target rate in radians/second
float AC_AttitudeControl_Heli::rate_target_to_motor_yaw(float rate_yaw_actual_rads, float rate_target_rads)
{
    if (!((AP_MotorsHeli&)_motors).rotor_runup_complete()) {
        _pid_rate_yaw.update_leaky_i(AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
    }

    float pid = _pid_rate_yaw.update_all(rate_target_rads, rate_yaw_actual_rads, _dt,  _motors.limit.yaw) + _actuator_sysid.z;

    // use pid library to calculate ff
    float vff = _pid_rate_yaw.get_ff()*_feedforward_scalar;

    // add feed forward
    float yaw_out = pid + vff;

    // constrain output
    yaw_out = constrain_float(yaw_out, -AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX);

    // output to motors
    return yaw_out;
}

//
// throttle functions
//

void AC_AttitudeControl_Heli::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);

    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (apply_angle_boost && !((AP_MotorsHeli&)_motors).get_in_autorotation()) {
        // Apply angle boost
        throttle_in = get_throttle_boosted(throttle_in);
    } else {
        // Clear angle_boost for logging purposes
        _angle_boost = 0.0f;
    }
    _motors.set_throttle(throttle_in);
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Heli::get_throttle_boosted(float throttle_in)
{
    if (!_angle_boost_enabled) {
        _angle_boost = 0;
        return throttle_in;
    }
    // inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor changes from 1 to -1 for tilt angles between 60 and 120 degrees

    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = constrain_float(2.0f * cos_tilt, -1.0f, 1.0f);
    float cos_tilt_target = fabsf(cosf(_thrust_angle));
    float boost_factor = 1.0f / constrain_float(cos_tilt_target, 0.1f, 1.0f);

    // angle boost and inverted factor applied about the zero thrust collective
    const float coll_mid = ((AP_MotorsHeli&)_motors).get_coll_mid();
    float throttle_out = ((throttle_in - coll_mid)  * inverted_factor * boost_factor) + coll_mid;
    _angle_boost = constrain_float(throttle_out - throttle_in, -1.0f, 1.0f);
    return throttle_out;
}

// get_roll_trim - angle in centi-degrees to be added to roll angle for learn hover collective. Used by helicopter to counter tail rotor thrust in hover
float AC_AttitudeControl_Heli::get_roll_trim_cd()
{
    // hover roll trim is given the opposite sign in inverted flight since the tail rotor thrust is pointed in the opposite direction. 
    float inverted_factor = constrain_float(2.0f * _ahrs.cos_roll(), -1.0f, 1.0f);
    return constrain_float(_hover_roll_trim_scalar * _hover_roll_trim * inverted_factor, -1000.0f,1000.0f);
}

// Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
void AC_AttitudeControl_Heli::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)
{
    if (_inverted_flight) {
        euler_roll_angle_cd = wrap_180_cd(euler_roll_angle_cd + 18000);
    }
    AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_rate_cds);
}

// Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing
void AC_AttitudeControl_Heli::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw)
{
    if (_inverted_flight) {
        euler_roll_angle_cd = wrap_180_cd(euler_roll_angle_cd + 18000);
    }
    AC_AttitudeControl::input_euler_angle_roll_pitch_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_angle_cd, slew_yaw);
}

void AC_AttitudeControl_Heli::set_notch_sample_rate(float sample_rate)
{
#if AP_FILTER_ENABLED
    _pid_rate_roll.set_notch_sample_rate(sample_rate);
    _pid_rate_pitch.set_notch_sample_rate(sample_rate);
    _pid_rate_yaw.set_notch_sample_rate(sample_rate);
#endif
}

// Command a thrust vector and heading rate
void AC_AttitudeControl_Heli::input_thrust_vector_rate_heading(const Vector3f& thrust_vector, float heading_rate_cds, bool slew_yaw)
{

    if (!_inverted_flight) {
        AC_AttitudeControl::input_thrust_vector_rate_heading(thrust_vector, heading_rate_cds, slew_yaw);
        return;
    }
    // convert thrust vector to a roll and pitch angles
    // this negates the advantage of using thrust vector control, but works just fine
    Vector3f angle_target = attitude_from_thrust_vector(thrust_vector, _ahrs.yaw).to_vector312();

    float euler_roll_angle_cd = degrees(angle_target.x) * 100.0f;
    euler_roll_angle_cd = wrap_180_cd(euler_roll_angle_cd + 18000);
    AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw(euler_roll_angle_cd, degrees(angle_target.y) * 100.0f, heading_rate_cds);
}

// Command a thrust vector, heading and heading rate
void AC_AttitudeControl_Heli::input_thrust_vector_heading(const Vector3f& thrust_vector, float heading_angle_cd, float heading_rate_cds)
{
    if (!_inverted_flight) {
        AC_AttitudeControl::input_thrust_vector_heading(thrust_vector, heading_angle_cd, heading_rate_cds);
        return;
    }
    // convert thrust vector to a roll and pitch angles
    Vector3f angle_target = attitude_from_thrust_vector(thrust_vector, _ahrs.yaw).to_vector312();

    float euler_roll_angle_cd = degrees(angle_target.x) * 100.0f;
    euler_roll_angle_cd = wrap_180_cd(euler_roll_angle_cd + 18000);
    // note that we are throwing away heading rate here
    AC_AttitudeControl::input_euler_angle_roll_pitch_yaw(euler_roll_angle_cd, degrees(angle_target.y) * 100.0f, heading_angle_cd, true);
}
