// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl_Heli.h"
#include <AP_HAL.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Heli::var_info[] PROGMEM = {

    // @Param: RATE_RP_MAX
    // @DisplayName: Angle Rate Roll-Pitch max
    // @Description: maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
    // @Units: Centi-Degrees/Sec
    // @Range: 9000 36000
    // @Increment: 500
    // @User: Advanced
    AP_GROUPINFO("RATE_RP_MAX", 0, AC_AttitudeControl_Heli, _angle_rate_rp_max, AC_ATTITUDE_CONTROL_RATE_RP_MAX_DEFAULT),

    // @Param: RATE_Y_MAX
    // @DisplayName: Angle Rate Yaw max
    // @Description: maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
    // @Units: Centi-Degrees/Sec
    // @Range: 4500 18000
    // @Increment: 500
    // @User: Advanced
    AP_GROUPINFO("RATE_Y_MAX",  1, AC_AttitudeControl_Heli, _angle_rate_y_max, AC_ATTITUDE_CONTROL_RATE_Y_MAX_DEFAULT),

    // @Param: SLEW_YAW
    // @DisplayName: Yaw target slew rate
    // @Description: Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
    // @Units: Centi-Degrees/Sec
    // @Range: 500 18000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("SLEW_YAW",    2, AC_AttitudeControl_Heli, _slew_yaw, AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT),

    // @Param: ACCEL_RP_MAX
    // @DisplayName: Acceleration Max for Roll/Pitch
    // @Description: Maximum acceleration in roll/pitch axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_RP_MAX", 3, AC_AttitudeControl_Heli, _accel_rp_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT),

    // @Param: ACCEL_Y_MAX
    // @DisplayName: Acceleration Max for Yaw
    // @Description: Maximum acceleration in yaw axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 72000
    // @Values: 0:Disabled, 18000:Slow, 36000:Medium, 54000:Fast
    // @Increment: 1000
    // @User: Advanced
    AP_GROUPINFO("ACCEL_Y_MAX",  4, AC_AttitudeControl_Heli, _accel_y_max, AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT),

    // @Param: RATE_FF_ENAB
    // @DisplayName: Rate Feedforward Enable
    // @Description: Controls whether body-frame rate feedfoward is enabled or disabled
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("RATE_FF_ENAB", 5, AC_AttitudeControl_Heli, _rate_bf_ff_enabled, AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT),

    AP_GROUPEND
};

// passthrough_bf_roll_pitch_rate_yaw - passthrough the pilots roll and pitch inputs directly to swashplate for flybar acro mode
void AC_AttitudeControl_Heli::passthrough_bf_roll_pitch_rate_yaw(float roll_passthrough, float pitch_passthrough, float yaw_rate_bf)
{
    // store roll and pitch passthroughs
    _passthrough_roll = roll_passthrough;
    _passthrough_pitch = pitch_passthrough;

    // set rate controller to use pass through
    _flags_heli.flybar_passthrough = true;

    // set bf rate targets to current body frame rates (i.e. relax and be ready for vehicle to switch out of acro)
    _rate_bf_desired.x = _ahrs.get_gyro().x * AC_ATTITUDE_CONTROL_DEGX100;
    _rate_bf_desired.y = _ahrs.get_gyro().y * AC_ATTITUDE_CONTROL_DEGX100;

    // accel limit desired yaw rate
    if (_accel_y_max > 0.0f) {
        float rate_change_limit = _accel_y_max * _dt;
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
    _motors.set_yaw(rate_bf_to_motor_yaw(_rate_bf_target.z));
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

    // call p and d controllers
    roll_pd = _pid_rate_roll.get_p(rate_roll_error) + _pid_rate_roll.get_d(rate_roll_error, _dt);
    pitch_pd = _pid_rate_pitch.get_p(rate_pitch_error) + _pid_rate_pitch.get_d(rate_pitch_error, _dt);

    // get roll i term
    roll_i = _pid_rate_roll.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_flags_heli.limit_roll || ((roll_i>0&&rate_roll_error<0)||(roll_i<0&&rate_roll_error>0))){
        if (((AP_MotorsHeli&)_motors).has_flybar()) {                              // Mechanical Flybars get regular integral for rate auto trim
            if (rate_roll_target_cds > -50 && rate_roll_target_cds < 50){       // Frozen at high rates
                roll_i = _pid_rate_roll.get_i(rate_roll_error, _dt);
            }
        }else{
            if (_flags_heli.leaky_i){
                roll_i = ((AC_HELI_PID&)_pid_rate_roll).get_leaky_i(rate_roll_error, _dt, AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
            }else{
                roll_i = _pid_rate_roll.get_i(rate_roll_error, _dt);
            }
        }
    }

    // get pitch i term
    pitch_i = _pid_rate_pitch.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_flags_heli.limit_pitch || ((pitch_i>0&&rate_pitch_error<0)||(pitch_i<0&&rate_pitch_error>0))){
        if (((AP_MotorsHeli&)_motors).has_flybar()) {                              // Mechanical Flybars get regular integral for rate auto trim
            if (rate_pitch_target_cds > -50 && rate_pitch_target_cds < 50){     // Frozen at high rates
                pitch_i = _pid_rate_pitch.get_i(rate_pitch_error, _dt);
            }
        }else{
            if (_flags_heli.leaky_i) {
                pitch_i = ((AC_HELI_PID&)_pid_rate_pitch).get_leaky_i(rate_pitch_error, _dt, AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
            }else{
                pitch_i = _pid_rate_pitch.get_i(rate_pitch_error, _dt);
            }
        }
    }
    
    roll_ff = roll_feedforward_filter.apply(((AC_HELI_PID&)_pid_rate_roll).get_ff(rate_roll_target_cds));
    pitch_ff = pitch_feedforward_filter.apply(((AC_HELI_PID&)_pid_rate_pitch).get_ff(rate_pitch_target_cds));

    // add feed forward and final output
    roll_out = roll_pd + roll_i + roll_ff;
    pitch_out = pitch_pd + pitch_i + pitch_ff;

    // constrain output and update limit flags
    if ((float)fabs(roll_out) > AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX) {
        roll_out = constrain_float(roll_out,-AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX,AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
        _flags_heli.limit_roll = true;
    }else{
        _flags_heli.limit_roll = false;
    }
    if ((float)fabs(pitch_out) > AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX) {
        pitch_out = constrain_float(pitch_out,-AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX,AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
        _flags_heli.limit_pitch = true;
    }else{
        _flags_heli.limit_pitch = false;
    }

    // output to motors
    _motors.set_roll(roll_out);
    _motors.set_pitch(pitch_out);

/*
#if HELI_CC_COMP == ENABLED
static LowPassFilterFloat rate_dynamics_filter;     // Rate Dynamics filter
#endif

#if HELI_CC_COMP == ENABLED
    rate_dynamics_filter.set_cutoff_frequency(0.01f, 4.0f);
#endif

#if AC_ATTITUDE_HELI_CC_COMP == ENABLED
// Do cross-coupling compensation for low rpm helis
// Credit: Jolyon Saunders
// Note: This is not widely tested at this time.  Will not be used by default yet.
    float cc_axis_ratio = 2.0f; // Ratio of compensation on pitch vs roll axes. Number >1 means pitch is affected more than roll
    float cc_kp = 0.0002f;      // Compensation p term. Setting this to zero gives h_phang only, while increasing it will increase the p term of correction
    float cc_kd = 0.127f;       // Compensation d term, scaled. This accounts for flexing of the blades, dampers etc. Originally was (motors.ext_gyro_gain * 0.0001)
    float cc_angle, cc_total_output;
    uint32_t cc_roll_d, cc_pitch_d, cc_sum_d;
    int32_t cc_scaled_roll;
    int32_t cc_roll_output;     // Used to temporarily hold output while rotation is being calculated
    int32_t cc_pitch_output;    // Used to temporarily hold output while rotation is being calculated
    static int32_t last_roll_output = 0;
    static int32_t last_pitch_output = 0;

    cc_scaled_roll  = roll_output / cc_axis_ratio; // apply axis ratio to roll
    cc_total_output = safe_sqrt(cc_scaled_roll * cc_scaled_roll + pitch_output * pitch_output) * cc_kp;

    // find the delta component
    cc_roll_d  = (roll_output - last_roll_output) / cc_axis_ratio;
    cc_pitch_d = pitch_output - last_pitch_output;
    cc_sum_d = safe_sqrt(cc_roll_d * cc_roll_d + cc_pitch_d * cc_pitch_d);

    // do the magic.
    cc_angle = cc_kd * cc_sum_d * cc_total_output - cc_total_output * motors.get_phase_angle();

    // smooth angle variations, apply constraints
    cc_angle = rate_dynamics_filter.apply(cc_angle);
    cc_angle = constrain_float(cc_angle, -90.0f, 0.0f);
    cc_angle = radians(cc_angle);

    // Make swash rate vector
    Vector2f swashratevector;
    swashratevector.x = cosf(cc_angle);
    swashratevector.y = sinf(cc_angle);
    swashratevector.normalize();

    // rotate the output
    cc_roll_output  = roll_output;
    cc_pitch_output = pitch_output;
    roll_output     = - (cc_pitch_output * swashratevector.y - cc_roll_output * swashratevector.x);
    pitch_output    =    cc_pitch_output * swashratevector.x + cc_roll_output * swashratevector.y;

    // make current outputs old, for next iteration
    last_roll_output  = cc_roll_output;
    last_pitch_output = cc_pitch_output;
# endif // HELI_CC_COMP

#if AC_ATTITUDE_HELI_PIRO_COMP == ENABLED
    if (control_mode <= ACRO){

        int32_t         piro_roll_i, piro_pitch_i;            // used to hold i term while doing prio comp

        piro_roll_i  = roll_i;
        piro_pitch_i = pitch_i;

        Vector2f yawratevector;
        yawratevector.x     = cos(-omega.z/100);
        yawratevector.y     = sin(-omega.z/100);
        yawratevector.normalize();

        roll_i      = piro_roll_i * yawratevector.x - piro_pitch_i * yawratevector.y;
        pitch_i     = piro_pitch_i * yawratevector.x + piro_roll_i * yawratevector.y;

        g.pid_rate_pitch.set_integrator(pitch_i);
        g.pid_rate_roll.set_integrator(roll_i);
    }
#endif //HELI_PIRO_COMP
*/
}

// rate_bf_to_motor_yaw - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
float AC_AttitudeControl_Heli::rate_bf_to_motor_yaw(float rate_target_cds)
{
    float pd,i,ff;            // used to capture pid values for logging
    float current_rate;     // this iteration's rate
    float rate_error;       // simply target_rate - current_rate
    float yaw_out;

    // get current rate
    // To-Do: make getting gyro rates more efficient?
    current_rate = (_ahrs.get_gyro().z * AC_ATTITUDE_CONTROL_DEGX100);

    // calculate error and call pid controller
    rate_error  = rate_target_cds - current_rate;
    pd = _pid_rate_yaw.get_p(rate_error) + _pid_rate_yaw.get_d(rate_error, _dt);

    // get i term
    i = _pid_rate_yaw.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_flags_heli.limit_yaw || ((i>0&&rate_error<0)||(i<0&&rate_error>0))) {
        if (((AP_MotorsHeli&)_motors).motor_runup_complete()) {
            i = _pid_rate_yaw.get_i(rate_error, _dt);
        } else {
            i = ((AC_HELI_PID&)_pid_rate_yaw).get_leaky_i(rate_error, _dt, AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);    // If motor is not running use leaky I-term to avoid excessive build-up
        }
    }
    
    ff = yaw_feedforward_filter.apply(((AC_HELI_PID&)_pid_rate_yaw).get_ff(rate_target_cds));
    
    // add feed forward
    yaw_out = pd + i + ff;

    // constrain output and update limit flag
    if ((float)fabs(yaw_out) > AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX) {
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

// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
int16_t AC_AttitudeControl_Heli::get_angle_boost(int16_t throttle_pwm)
{
    // no angle boost for trad helis
    _angle_boost = 0;
    return throttle_pwm;
}

// update_feedforward_filter_rate - Sets LPF cutoff frequency
void AC_AttitudeControl_Heli::update_feedforward_filter_rates(float time_step)
{
    pitch_feedforward_filter.set_cutoff_frequency(time_step, AC_ATTITUDE_HELI_RATE_FF_FILTER);
    roll_feedforward_filter.set_cutoff_frequency(time_step, AC_ATTITUDE_HELI_RATE_FF_FILTER);
    yaw_feedforward_filter.set_cutoff_frequency(time_step, AC_ATTITUDE_HELI_RATE_FF_FILTER);
}

