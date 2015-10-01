// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl_Heli.h"
#include <AP_HAL/AP_HAL.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Heli::var_info[] PROGMEM = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    // @Param: PIRO_COMP
    // @DisplayName: Piro Comp Enable
    // @Description: Pirouette compensation enabled
    // @Range: 0:Disabled 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("PIRO_COMP",    1, AC_AttitudeControl_Heli, _piro_comp_enabled, 0),

    // @Param: STAB_COL_1
    // @DisplayName: Stabilize Mode Collective Point 1
    // @Description: Helicopter's minimum collective pitch setting at zero throttle input in Stabilize mode
    // @Range: 0 500
    // @Units: Percent*10
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STAB_COL_1",    2, AC_AttitudeControl_Heli, _heli_stab_col_min, AC_ATTITUDE_HELI_STAB_COLLECTIVE_MIN_DEFAULT),

    // @Param: STAB_COL_2
    // @DisplayName: Stabilize Mode Collective Point 2
    // @Description: Helicopter's collective pitch setting at mid-low throttle input in Stabilize mode
    // @Range: 0 500
    // @Units: Percent*10
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STAB_COL_2",    3, AC_AttitudeControl_Heli, _heli_stab_col_low, AC_ATTITUDE_HELI_STAB_COLLECTIVE_LOW_DEFAULT),

    // @Param: STAB_COL_3
    // @DisplayName: Stabilize Mode Collective Point 3
    // @Description: Helicopter's collective pitch setting at mid-high throttle input in Stabilize mode
    // @Range: 500 1000
    // @Units: Percent*10
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STAB_COL_3",    4, AC_AttitudeControl_Heli, _heli_stab_col_high, AC_ATTITUDE_HELI_STAB_COLLECTIVE_HIGH_DEFAULT),

    // @Param: STAB_COL_4
    // @DisplayName: Stabilize Mode Collective Point 4
    // @Description: Helicopter's maximum collective pitch setting at full throttle input in Stabilize mode
    // @Range: 500 1000
    // @Units: Percent*10
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STAB_COL_4",    5, AC_AttitudeControl_Heli, _heli_stab_col_max, AC_ATTITUDE_HELI_STAB_COLLECTIVE_MAX_DEFAULT),

    // @Param: ACRO_COL_EXP
    // @DisplayName: Acro Mode Collective Expo
    // @Description: Used to soften collective pitch inputs near center point in Acro mode.
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @User: Advanced
    AP_GROUPINFO("ACRO_COL_EXP",    6, AC_AttitudeControl_Heli, _acro_col_expo, 0),

    AP_GROUPEND
};

// passthrough_bf_roll_pitch_rate_yaw - passthrough the pilots roll and pitch inputs directly to swashplate for flybar acro mode
void AC_AttitudeControl_Heli::passthrough_bf_roll_pitch_rate_yaw(float roll_passthrough, float pitch_passthrough, float yaw_rate_bf)
{
    // store roll, pitch and passthroughs
    _passthrough_roll = roll_passthrough;
    _passthrough_pitch = pitch_passthrough;
    _passthrough_yaw = yaw_rate_bf;

    // set rate controller to use pass through
    _flags_heli.flybar_passthrough = true;

    // set bf rate targets to current body frame rates (i.e. relax and be ready for vehicle to switch out of acro)
    _rate_bf_desired.x = _ahrs.get_gyro().x * AC_ATTITUDE_CONTROL_DEGX100;
    _rate_bf_desired.y = _ahrs.get_gyro().y * AC_ATTITUDE_CONTROL_DEGX100;

    // accel limit desired yaw rate
    if (_accel_yaw_max > 0.0f) {
        float rate_change_limit = _accel_yaw_max * _dt;
        float rate_change = yaw_rate_bf - _rate_bf_desired.z;
        rate_change = constrain_float(rate_change, -rate_change_limit, rate_change_limit);
        _rate_bf_desired.z += rate_change;
    } else {
        _rate_bf_desired.z = yaw_rate_bf;
    }

    integrate_bf_rate_error_to_angle_errors();
    _angle_bf_error.x = 0;
    _angle_bf_error.y = 0;

    // update our earth-frame angle targets
    Vector3f angle_ef_error;
    if (frame_conversion_bf_to_ef(_angle_bf_error, angle_ef_error)) {
        _angle_ef_target.x = wrap_180_cd_float(angle_ef_error.x + _ahrs.roll_sensor);
        _angle_ef_target.y = wrap_180_cd_float(angle_ef_error.y + _ahrs.pitch_sensor);
        _angle_ef_target.z = wrap_360_cd_float(angle_ef_error.z + _ahrs.yaw_sensor);
    }

    // handle flipping over pitch axis
    if (_angle_ef_target.y > 9000.0f) {
        _angle_ef_target.x = wrap_180_cd_float(_angle_ef_target.x + 18000.0f);
        _angle_ef_target.y = wrap_180_cd_float(18000.0f - _angle_ef_target.x);
        _angle_ef_target.z = wrap_360_cd_float(_angle_ef_target.z + 18000.0f);
    }
    if (_angle_ef_target.y < -9000.0f) {
        _angle_ef_target.x = wrap_180_cd_float(_angle_ef_target.x + 18000.0f);
        _angle_ef_target.y = wrap_180_cd_float(-18000.0f - _angle_ef_target.x);
        _angle_ef_target.z = wrap_360_cd_float(_angle_ef_target.z + 18000.0f);
    }

    // convert body-frame angle errors to body-frame rate targets
    update_rate_bf_targets();

    // set body-frame roll/pitch rate target to current desired rates which are the vehicle's actual rates
    _rate_bf_target.x = _rate_bf_desired.x;
    _rate_bf_target.y = _rate_bf_desired.y;

    // add desired target to yaw
    _rate_bf_target.z += _rate_bf_desired.z;
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
        _motors.set_roll(_passthrough_roll);
        _motors.set_pitch(_passthrough_pitch);
    } else {
        rate_bf_to_motor_roll_pitch(_rate_bf_target.x, _rate_bf_target.y);
    }
    if (_flags_heli.tail_passthrough) {
        _motors.set_yaw(_passthrough_yaw);
    } else {
        _motors.set_yaw(rate_bf_to_motor_yaw(_rate_bf_target.z));
    }
}

//
// private methods
//

//
// body-frame rate controller
//

// rate_bf_to_motor_roll_pitch - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
void AC_AttitudeControl_Heli::rate_bf_to_motor_roll_pitch(float rate_roll_target_cds, float rate_pitch_target_cds)
{
    float roll_pd, roll_i, roll_ff;             // used to capture pid values
    float pitch_pd, pitch_i, pitch_ff;          // used to capture pid values
    float rate_roll_error, rate_pitch_error;    // simply target_rate - current_rate
    float roll_out, pitch_out;
    const Vector3f& gyro = _ahrs.get_gyro();     // get current rates

    // calculate error
    rate_roll_error = rate_roll_target_cds - gyro.x * AC_ATTITUDE_CONTROL_DEGX100;
    rate_pitch_error = rate_pitch_target_cds - gyro.y * AC_ATTITUDE_CONTROL_DEGX100;

    // input to PID controller
    _pid_rate_roll.set_input_filter_all(rate_roll_error);
    _pid_rate_roll.set_desired_rate(rate_roll_target_cds);
    _pid_rate_pitch.set_input_filter_all(rate_pitch_error);
    _pid_rate_pitch.set_desired_rate(rate_pitch_target_cds);

    // call p and d controllers
    roll_pd = _pid_rate_roll.get_p() + _pid_rate_roll.get_d();
    pitch_pd = _pid_rate_pitch.get_p() + _pid_rate_pitch.get_d();

    // get roll i term
    roll_i = _pid_rate_roll.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_flags_heli.limit_roll || ((roll_i>0&&rate_roll_error<0)||(roll_i<0&&rate_roll_error>0))){
        if (((AP_MotorsHeli&)_motors).has_flybar()) {                              // Mechanical Flybars get regular integral for rate auto trim
            if (rate_roll_target_cds > -50 && rate_roll_target_cds < 50){       // Frozen at high rates
                roll_i = _pid_rate_roll.get_i();
            }
        }else{
            if (_flags_heli.leaky_i){
                roll_i = ((AC_HELI_PID&)_pid_rate_roll).get_leaky_i(AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
            }else{
                roll_i = _pid_rate_roll.get_i();
            }
        }
    }

    // get pitch i term
    pitch_i = _pid_rate_pitch.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_flags_heli.limit_pitch || ((pitch_i>0&&rate_pitch_error<0)||(pitch_i<0&&rate_pitch_error>0))){
        if (((AP_MotorsHeli&)_motors).has_flybar()) {                              // Mechanical Flybars get regular integral for rate auto trim
            if (rate_pitch_target_cds > -50 && rate_pitch_target_cds < 50){     // Frozen at high rates
                pitch_i = _pid_rate_pitch.get_i();
            }
        }else{
            if (_flags_heli.leaky_i) {
                pitch_i = ((AC_HELI_PID&)_pid_rate_pitch).get_leaky_i(AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
            }else{
                pitch_i = _pid_rate_pitch.get_i();
            }
        }
    }
    
    roll_ff = roll_feedforward_filter.apply(((AC_HELI_PID&)_pid_rate_roll).get_vff(rate_roll_target_cds), _dt);
    pitch_ff = pitch_feedforward_filter.apply(((AC_HELI_PID&)_pid_rate_pitch).get_vff(rate_pitch_target_cds), _dt);

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

// rate_bf_to_motor_yaw - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float AC_AttitudeControl_Heli::rate_bf_to_motor_yaw(float rate_target_cds)
{
    float pd,i,vff;         // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate
    float yaw_out;

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (_ahrs.get_gyro().z * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error  = rate_target_cds - current_rate;

    // send input to PID controller
    _pid_rate_yaw.set_input_filter_all(rate_error);
    _pid_rate_yaw.set_desired_rate(rate_target_cds);

    // get p and d
    pd = _pid_rate_yaw.get_p() + _pid_rate_yaw.get_d();

    // get i term
    i = _pid_rate_yaw.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_flags_heli.limit_yaw || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        if (((AP_MotorsHeli&)_motors).rotor_runup_complete()) {
            i = _pid_rate_yaw.get_i();
        } else {
            i = ((AC_HELI_PID&)_pid_rate_yaw).get_leaky_i(AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);    // If motor is not running use leaky I-term to avoid excessive build-up
        }
    }
    
    vff = yaw_velocity_feedforward_filter.apply(((AC_HELI_PID&)_pid_rate_yaw).get_vff(rate_target_cds), _dt);
    
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
    _angle_boost = 0;
    return throttle_in;
}

// get_pilot_desired_collective - rescale's pilot collective pitch input in Stabilize and Acro modes
int16_t AC_AttitudeControl_Heli::get_pilot_desired_collective(int16_t control_in)
{
    float slope_low, slope_high, slope_range, slope_run, scalar;
    int16_t stab_col_out, acro_col_out;

    // calculate stabilize collective value which scales pilot input to reduced collective range
    // code implements a 3-segment curve with knee points at 40% and 60% throttle input
    if (control_in < 400){
        slope_low = _heli_stab_col_min;
        slope_high = _heli_stab_col_low;
        slope_range = 400;
        slope_run = control_in;
    } else if(control_in <600){
        slope_low = _heli_stab_col_low;
        slope_high = _heli_stab_col_high;
        slope_range = 200;
        slope_run = control_in - 400;
    } else {
        slope_low = _heli_stab_col_high;
        slope_high = _heli_stab_col_max;
        slope_range = 400;
        slope_run = control_in - 600;
    }    

    scalar = (slope_high - slope_low)/slope_range;
    stab_col_out = slope_low + slope_run * scalar;
    stab_col_out = constrain_int16(stab_col_out, 0, 1000);

    //
    // calculate expo-scaled acro collective
    // range check expo
    if (_acro_col_expo > 1.0f) {
        _acro_col_expo = 1.0f;
    }

    if (_acro_col_expo <= 0) {
        acro_col_out = control_in;
    } else {
        // expo variables
        float col_in, col_in3, col_out;
        col_in = (float)(control_in-500)/500.0f;
        col_in3 = col_in*col_in*col_in;
        col_out = (_acro_col_expo * col_in3) + ((1-_acro_col_expo)*col_in);
        acro_col_out = 500 + col_out*500;
    }
    acro_col_out = constrain_int16(acro_col_out, 0, 1000);

    // ramp to and from stab col over 1/2 second
    if (_flags_heli.use_stab_col && (_stab_col_ramp < 1.0)){
        _stab_col_ramp += 2*_dt;
    } else if(!_flags_heli.use_stab_col && (_stab_col_ramp > 0.0)){
        _stab_col_ramp -= 2*_dt;
    }
    _stab_col_ramp = constrain_float(_stab_col_ramp, 0.0, 1.0);

    // scale collective output smoothly between acro and stab col
    int16_t collective_out;
    collective_out = (float)((1.0-_stab_col_ramp)*acro_col_out + _stab_col_ramp*stab_col_out);
    collective_out = constrain_int16(collective_out, 0, 1000);

    return collective_out;
}

