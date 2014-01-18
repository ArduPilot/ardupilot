// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl::var_info[] PROGMEM = {

    // @Param: DUMMY
    // @DisplayName: Dummy parameter
    // @Description: This is the dummy parameters
	// @Increment: 0.1
	// @User: User
    AP_GROUPINFO("DUMMY",    0, AC_AttitudeControl, _dummy_param, 0),

    AP_GROUPEND
};

//
// high level controllers
//

// init_targets - resets target angles to current angles
void AC_AttitudeControl::init_targets()
{
    _angle_ef_target.x = _ahrs.roll_sensor;
    _angle_ef_target.y = _ahrs.pitch_sensor;
    _angle_ef_target.z = _ahrs.yaw_sensor;
}

// angleef_rp_rateef_y - attempts to maintain a roll and pitch angle and yaw rate (all earth frame)
void AC_AttitudeControl::angleef_rp_rateef_y(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef)
{
    // set earth-frame angle targets
    _angle_ef_target.x = roll_angle_ef;
    _angle_ef_target.y = pitch_angle_ef;

    // convert earth-frame angle targets to earth-frame rate targets
    angle_to_rate_ef_roll();
    angle_to_rate_ef_pitch();

    // set earth-frame rate stabilize target for yaw
    _rate_stab_ef_target.z =  yaw_rate_ef;

    // convert earth-frame stabilize rate to regular rate target
    rate_stab_ef_to_rate_ef_yaw();

    // convert earth-frame rates to body-frame rates
    rate_ef_targets_to_bf();

    // body-frame to motor outputs should be called separately
}

// angleef_rpy - attempts to maintain a roll, pitch and yaw angle (all earth frame)
void AC_AttitudeControl::angleef_rpy(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef)
{
    // set earth-frame angle targets
    _angle_ef_target.x = roll_angle_ef;
    _angle_ef_target.y = pitch_angle_ef;

    // convert earth-frame angle targets to earth-frame rate targets
    angle_to_rate_ef_roll();
    angle_to_rate_ef_pitch();
    angle_to_rate_ef_yaw();

    // convert earth-frame rates to body-frame rates
    rate_ef_targets_to_bf();

    // body-frame to motor outputs should be called separately
}

//
// angle controller methods
//

// angle_to_rate_ef_roll - ask the angle controller to calculate the earth frame rate targets for roll
void AC_AttitudeControl::angle_to_rate_ef_roll()
{
    // calculate angle error
    // To-Do: is this being converted to int32_t as part of wrap_180_cd?
    float angle_error_cd = wrap_180_cd(_angle_ef_target.x - _ahrs.roll_sensor);

    // limit the error we're feeding to the PID
    // Does this make any sense to limit the angular error by the maximum lean angle of the copter?  Is it the responsibility of the rate controller to know it's own limits?
    // To-Do: move aparm.angle_max into the AP_Vehicles class?
    angle_error_cd = constrain_float(angle_error_cd, -AC_ATTITUDE_ANGLE_CONTROLLER_ANGLE_MAX, AC_ATTITUDE_ANGLE_CONTROLLER_ANGLE_MAX);

    // convert to desired earth-frame rate
    // To-Do: replace PI controller with just a single gain?
    _rate_ef_target.x = _pi_angle_roll.kP() * angle_error_cd;
}

// angle_to_rate_ef_pitch - ask the angle controller to calculate the earth frame rate targets for pitch
void AC_AttitudeControl::angle_to_rate_ef_pitch()
{
    // calculate angle error
    // To-Do: is this being converted to int32_t as part of wrap_180_cd?
    float angle_error_cd = wrap_180_cd(_angle_ef_target.y - _ahrs.pitch_sensor);

    // limit the error we're feeding to the PID
    // Does this make any sense to limit the angular error by the maximum lean angle of the copter?  Is it the responsibility of the rate controller to know it's own limits?
    // To-Do: move aparm.angle_max into the AP_Vehicles class?
    angle_error_cd = constrain_float(angle_error_cd, -AC_ATTITUDE_ANGLE_CONTROLLER_ANGLE_MAX, AC_ATTITUDE_ANGLE_CONTROLLER_ANGLE_MAX);

    // convert to desired earth-frame rate
    // To-Do: replace PI controller with just a single gain?
    _rate_ef_target.y = _pi_angle_pitch.kP() * angle_error_cd;
}

// angle_to_rate_ef_yaw - ask the angle controller to calculate the earth-frame yaw rate in centi-degrees/second
void AC_AttitudeControl::angle_to_rate_ef_yaw()
{
    // calculate angle error
    // To-Do: is this being converted to int32_t as part of wrap_180_cd?
    float angle_error_cd = wrap_180_cd(_angle_ef_target.z - _ahrs.yaw_sensor);

    // limit the error we're feeding to the PID
    angle_error_cd = constrain_float(angle_error_cd, -AC_ATTITUDE_ANGLE_YAW_CONTROLLER_OUT_MAX, AC_ATTITUDE_ANGLE_YAW_CONTROLLER_OUT_MAX);

    // convert to desired earth-frame rate in centi-degrees/second
    // To-Do: replace PI controller with just a single gain?
    _rate_ef_target.z = _pi_angle_yaw.kP() * angle_error_cd;

    // To-Do: deal with trad helicopter which do not use yaw rate controllers if using external gyros

    // To-Do: allow logging of PIDs?
}

//
// stabilized rate controller (earth-frame) methods
// stabilized rate controllers are better at maintaining a desired rate than the simpler earth-frame rate controllers
// because they also maintain angle-targets and increase/decrease the rate request passed to the earth-frame rate controller
// upon the errors between the actual angle and angle-target.
// 
//
// rate_stab_ef_to_rate_ef_roll - converts earth-frame stabilized rate targets to regular earth-frame rate targets for roll, pitch and yaw axis
//   targets rates in centi-degrees/second taken from _rate_stab_ef_target
//   results in centi-degrees/sec put into _rate_ef_target
void AC_AttitudeControl::rate_stab_ef_to_rate_ef_roll()
{
    float angle_error;

    // convert the input to the desired roll rate
    _angle_ef_target.x += _rate_stab_ef_target.x * _dt;
    _angle_ef_target.x = wrap_180_cd(_angle_ef_target.x);

    // ensure targets are within the lean angle limits
    // To-Do: make angle_max part of the AP_Vehicle class
    _angle_ef_target.x  = constrain_float(_angle_ef_target.x, -_aparm.angle_max, _aparm.angle_max);

    // calculate angle error with maximum of +- max_angle_overshoot
    angle_error = wrap_180_cd(_angle_ef_target.x - _ahrs.roll_sensor);
    angle_error  = constrain_float(angle_error, -AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX, AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX);

    // To-Do: handle check for traditional heli's motors.motor_runup_complete
    // To-Do: reset target angle to current angle if motors not spinning

    // update acro_roll to be within max_angle_overshoot of our current heading
    _angle_ef_target.x = wrap_180_cd(angle_error + _ahrs.roll_sensor);

    // set earth frame rate controller targets
    _rate_ef_target.x = _pi_angle_roll.get_p(angle_error) + _rate_stab_ef_target.x;
}

void AC_AttitudeControl::rate_stab_ef_to_rate_ef_pitch()
{
   float angle_error;

    // convert the input to the desired roll rate
    _angle_ef_target.y += _rate_stab_ef_target.y * _dt;
    _angle_ef_target.y = wrap_180_cd(_angle_ef_target.y);

    // ensure targets are within the lean angle limits
    // To-Do: make angle_max part of the AP_Vehicle class
    _angle_ef_target.y  = constrain_float(_angle_ef_target.y, -_aparm.angle_max, _aparm.angle_max);

    // calculate angle error with maximum of +- max_angle_overshoot
    // To-Do: should we do something better as we cross 90 degrees?
    angle_error = wrap_180_cd(_angle_ef_target.y - _ahrs.pitch_sensor);
    angle_error  = constrain_float(angle_error, -AC_ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX, AC_ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX);

    // To-Do: handle check for traditional heli's motors.motor_runup_complete
    // To-Do: reset target angle to current angle if motors not spinning

    // update acro_roll to be within max_angle_overshoot of our current heading
    _angle_ef_target.y = wrap_180_cd(angle_error + _ahrs.pitch_sensor);

    // set earth frame rate controller targets
    _rate_ef_target.y = _pi_angle_pitch.get_p(angle_error) + _rate_stab_ef_target.y;
}

void AC_AttitudeControl::rate_stab_ef_to_rate_ef_yaw()
{
   float angle_error;

    // convert the input to the desired roll rate
    _angle_ef_target.z += _rate_stab_ef_target.z * _dt;
    _angle_ef_target.z = wrap_360_cd(_angle_ef_target.z);

    // calculate angle error with maximum of +- max_angle_overshoot
    angle_error = wrap_180_cd(_angle_ef_target.z - _ahrs.yaw_sensor);
    angle_error  = constrain_float(angle_error, -AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX);

    // To-Do: handle check for traditional heli's motors.motor_runup_complete
    // To-Do: reset target angle to current angle if motors not spinning

    // update acro_roll to be within max_angle_overshoot of our current heading
    _angle_ef_target.z = wrap_360_cd(angle_error + _ahrs.yaw_sensor);

    // set earth frame rate controller targets
    _rate_ef_target.z = _pi_angle_yaw.get_p(angle_error) + _rate_stab_ef_target.z;
}

//
// stabilized rate controller (body-frame) methods
//

// rate_stab_bf_to_rate_ef_roll - converts body-frame stabilized rate targets to regular body-frame rate targets for roll, pitch and yaw axis
//   targets rates in centi-degrees/second taken from _rate_stab_bf_target
//   results in centi-degrees/sec put into _rate_bf_target
void AC_AttitudeControl::rate_stab_bf_to_rate_bf_roll()
{
    // calculate rate correction from angle errors
    // To-Do: do we still need to have this rate correction calculated from the previous iteration's errors?
    float rate_correction = _pi_angle_roll.get_p(_rate_stab_bf_error.x);

    // set body frame targets for rate controller
    _rate_bf_target.x = _rate_stab_bf_target.x + rate_correction;

    // calculate body-frame angle error by integrating body-frame rate error
    _rate_stab_bf_error.x += (_rate_stab_bf_target.x - (_ins.get_gyro().x * AC_ATTITUDE_CONTROL_DEGX100)) * _dt;

    // limit maximum error
    _rate_stab_bf_error.x = constrain_float(_rate_stab_bf_error.x, -AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX, AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX);

    // To-Do: handle case of motors being disarmed or g.rc_3.servo_out == 0 and set error to zero
}

void AC_AttitudeControl::rate_stab_bf_to_rate_bf_pitch()
{
    // calculate rate correction from angle errors
    // To-Do: do we still need to have this rate correction calculated from the previous iteration's errors?
    float rate_correction = _pi_angle_pitch.get_p(_rate_stab_bf_error.y);

    // set body frame targets for rate controller
    _rate_bf_target.y = _rate_stab_bf_target.y + rate_correction;

    // calculate body-frame angle error by integrating body-frame rate error
    _rate_stab_bf_error.y += (_rate_stab_bf_target.y - (_ins.get_gyro().y * AC_ATTITUDE_CONTROL_DEGX100)) * _dt;

    // limit maximum error
    _rate_stab_bf_error.y = constrain_float(_rate_stab_bf_error.y, -AC_ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX, AC_ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX);

    // To-Do: handle case of motors being disarmed or g.rc_3.servo_out == 0 and set error to zero
}

void AC_AttitudeControl::rate_stab_bf_to_rate_bf_yaw()
{
    // calculate rate correction from angle errors
    float rate_correction = _pi_angle_yaw.get_p(_rate_stab_bf_error.z);

    // set body frame targets for rate controller
    _rate_bf_target.y = _rate_stab_bf_target.y + rate_correction;

    // calculate body-frame angle error by integrating body-frame rate error
    _rate_stab_bf_error.z += (_rate_stab_bf_target.z - (_ins.get_gyro().z * AC_ATTITUDE_CONTROL_DEGX100)) * _dt;

    // limit maximum error
    _rate_stab_bf_error.z = constrain_float(_rate_stab_bf_error.z, -AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX);

    // To-Do: handle case of motors being disarmed or g.rc_3.servo_out == 0 and set error to zero
}

//
// rate controller (earth-frame) methods
//

// rate_ef_targets_to_bf - converts earth frame rate targets to body frame rate targets
void AC_AttitudeControl::rate_ef_targets_to_bf()
{
    // convert earth frame rates to body frame rates
    _rate_bf_target.x = _rate_ef_target.x - _sin_pitch * _rate_ef_target.z;
    _rate_bf_target.y = _cos_roll  * _rate_ef_target.y + _sin_roll * _cos_pitch * _rate_ef_target.z;
    _rate_bf_target.z = _cos_pitch * _cos_roll * _rate_ef_target.z - _sin_roll * _rate_ef_target.y;
}

//
// rate controller (body-frame) methods
//

// rate_controller_run - run lowest level rate controller and send outputs to the motors
// should be called at 100hz or more
void AC_AttitudeControl::rate_controller_run()
{	
    // call rate controllers and send output to motors object
    // To-Do: should the outputs from get_rate_roll, pitch, yaw be int16_t which is the input to the motors library?
    // To-Do: skip this step if the throttle out is zero?
    _motor_roll = rate_bf_to_motor_roll(_rate_bf_target.x);
    _motor_pitch = rate_bf_to_motor_pitch(_rate_bf_target.y);
    _motor_yaw = rate_bf_to_motor_yaw(_rate_bf_target.z);

    // To-Do: what about throttle?
    //_motor_
}

//
// private methods
//
//
// body-frame rate controller
//

// rate_bf_to_motor_roll - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float AC_AttitudeControl::rate_bf_to_motor_roll(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (_ins.get_gyro().x * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error = rate_target_cds - current_rate;
    p = _pid_rate_roll.get_p(rate_error);

    // get i term
    i = _pid_rate_roll.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = _pid_rate_roll.get_i(rate_error, _dt);
    }

    // get d term
    d = _pid_rate_roll.get_d(rate_error, _dt);

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);

    // To-Do: allow logging of PIDs?
}

// rate_bf_to_motor_pitch - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float AC_AttitudeControl::rate_bf_to_motor_pitch(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (_ins.get_gyro().y * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error = rate_target_cds - current_rate;
    p = _pid_rate_pitch.get_p(rate_error);

    // get i term
    i = _pid_rate_pitch.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = _pid_rate_pitch.get_i(rate_error, _dt);
    }

    // get d term
    d = _pid_rate_pitch.get_d(rate_error, _dt);

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);

    // To-Do: allow logging of PIDs?
}

// rate_bf_to_motor_yaw - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float AC_AttitudeControl::rate_bf_to_motor_yaw(float rate_target_cds)
{
    float p,i,d;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (_ins.get_gyro().z * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error  = rate_target_cds - current_rate;
    p = _pid_rate_yaw.get_p(rate_error);

    // separately calculate p, i, d values for logging
    p = _pid_rate_yaw.get_p(rate_error);

    // get i term
    i = _pid_rate_yaw.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_motors.limit.yaw || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        i = _pid_rate_yaw.get_i(rate_error, _dt);
    }

    // get d value
    d = _pid_rate_yaw.get_d(rate_error, _dt);

    // constrain output and return
    return constrain_float((p+i+d), -AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX);

    // To-Do: allow logging of PIDs?
}


//
// throttle functions
//

 // set_throttle_out - to be called by upper throttle controllers when they wish to provide throttle output directly to motors
 // provide 0 to cut motors
void AC_AttitudeControl::set_throttle_out(int16_t throttle_out, bool apply_angle_boost)
{
    if (apply_angle_boost) {
        _motor_throttle = get_angle_boost(throttle_out);
    }else{
        _motor_throttle = throttle_out;
        // clear angle_boost for logging purposes
        _angle_boost = 0;
    }

    // update compass with throttle value
    // To-Do: find another method to grab the throttle out and feed to the compass.  Could be done completely outside this class
    //compass.set_throttle((float)g.rc_3.servo_out/1000.0f);
}

// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
int16_t AC_AttitudeControl::get_angle_boost(int16_t throttle_pwm)
{
    float temp = _cos_pitch * _cos_roll;
    int16_t throttle_out;

    temp = constrain_float(temp, 0.5f, 1.0f);

    // reduce throttle if we go inverted
    temp = constrain_float(9000-max(labs(_ahrs.roll_sensor),labs(_ahrs.pitch_sensor)), 0, 3000) / (3000 * temp);

    // apply scale and constrain throttle
    // To-Do: move throttle_min and throttle_max into the AP_Vehicles class?
    throttle_out = constrain_float((float)(throttle_pwm-_motors.throttle_min()) * temp + _motors.throttle_min(), _motors.throttle_min(), 1000);

    // record angle boost for logging
    _angle_boost = throttle_out - throttle_pwm;

    return throttle_out;
}
