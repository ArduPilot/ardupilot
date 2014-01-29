// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl_Heli.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Heli::var_info[] PROGMEM = {

    // @Param: DUMMY
    // @DisplayName: Dummy parameter
    // @Description: This is the dummy parameters
    // @Increment: 0.1
    // @User: User
    AP_GROUPINFO("DUMMY",       0,  AC_AttitudeControl_Heli,    _dummy_param, 0),

    // @Param: RATE_RLL_FF
    // @DisplayName: Rate Roll Feed Forward
    // @Description: Rate Roll Feed Forward (for TradHeli Only)
    // @Range: 0 10
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("RATE_RLL_FF", 1, AC_AttitudeControl_Heli,     _heli_roll_ff,  AC_ATTITUDE_HELI_ROLL_FF),

    // @Param: RATE_PIT_FF
    // @DisplayName: Rate Pitch Feed Forward
    // @Description: Rate Pitch Feed Forward (for TradHeli Only)
    // @Range: 0 10
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("RATE_PIT_FF", 2,  AC_AttitudeControl_Heli,    _heli_pitch_ff, AC_ATTITUDE_HELI_ROLL_FF),

    // @Param: RATE_YAW_FF
    // @DisplayName: Rate Yaw Feed Forward
    // @Description: Rate Yaw Feed Forward (for TradHeli Only)
    // @Range: 0 10
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("RATE_YAW_FF", 3, AC_AttitudeControl_Heli,     _heli_yaw_ff,   AC_ATTITUDE_HELI_YAW_FF),

    AP_GROUPEND
};

//
// rate controller (body-frame) methods
//

// rate_controller_run - run lowest level rate controller and send outputs to the motors
// should be called at 100hz or more
void AC_AttitudeControl_Heli::rate_controller_run()
{	
    // call rate controllers and send output to motors object
    // To-Do: should the outputs from get_rate_roll, pitch, yaw be int16_t which is the input to the motors library?
    // To-Do: skip this step if the throttle out is zero?
    rate_bf_to_motor_roll_pitch(_rate_bf_target.x, _rate_bf_target.y, _motor_roll, _motor_pitch);
    _motor_yaw = rate_bf_to_motor_yaw(_rate_bf_target.z);
}

//
// private methods
//

//
// body-frame rate controller
//

// rate_bf_to_motor_roll_pitch - ask the rate controller to calculate the motor outputs to achieve the target rate in centi-degrees / second
void AC_AttitudeControl_Heli::rate_bf_to_motor_roll_pitch(float rate_roll_target_cds, float rate_pitch_target_cds, int16_t& motor_roll, int16_t& motor_pitch)
{
    float roll_pd, roll_i;                      // used to capture pid values
    float pitch_pd, pitch_i;                    // used to capture pid values
    float current_rate;                         // this iteration's rate
    float rate_roll_error, rate_pitch_error;    // simply target_rate - current_rate
    float roll_out, pitch_out;
    const Vector3f& gyro = _ins.get_gyro();     // get current rates
    AP_MotorsHeli& heli_motors = (AP_MotorsHeli&)_motors;

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
        if (heli_motors.has_flybar()) {                              // Mechanical Flybars get regular integral for rate auto trim
            if (rate_roll_target_cds > -50 && rate_roll_target_cds < 50){       // Frozen at high rates
                roll_i = _pid_rate_roll.get_i(rate_roll_error, _dt);
            }
        }else{
            if (_flags_heli.leaky_i){
                roll_i = _pid_rate_roll.get_leaky_i(rate_roll_error, _dt, AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
            }else{
                roll_i = _pid_rate_roll.get_i(rate_roll_error, _dt);
            }
        }
    }

    // get pitch i term
    pitch_i = _pid_rate_pitch.get_integrator();

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    if (!_flags_heli.limit_pitch || ((pitch_i>0&&rate_pitch_error<0)||(pitch_i<0&&rate_pitch_error>0))){
        if (heli_motors.has_flybar()) {                              // Mechanical Flybars get regular integral for rate auto trim
            if (rate_pitch_target_cds > -50 && rate_pitch_target_cds < 50){     // Frozen at high rates
                pitch_i = _pid_rate_pitch.get_i(rate_pitch_error, _dt);
            }
        }else{
            if (_flags_heli.leaky_i){
                pitch_i = _pid_rate_pitch.get_leaky_i(rate_pitch_error, _dt, AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
            }else{
                pitch_i = _pid_rate_pitch.get_i(rate_pitch_error, _dt);
            }
        }
    }

    // add feed forward and final output
    roll_out = (_heli_roll_ff * rate_roll_target_cds) + roll_pd + roll_i;
    pitch_out = (_heli_pitch_ff * rate_pitch_target_cds) + pitch_pd + pitch_i;

    // constrain output and update limit flags
    if (fabs(roll_out) > AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX) {
        roll_out = constrain_float(roll_out,-AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX,AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
        _flags_heli.limit_roll = true;
    }else{
        _flags_heli.limit_roll = false;
    }
    if (fabs(pitch_out) > AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX) {
        pitch_out = constrain_float(pitch_out,-AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX,AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
        _flags_heli.limit_pitch = true;
    }else{
        _flags_heli.limit_pitch = false;
    }

    // output to motors
    motor_roll = roll_out;
    motor_pitch = pitch_out;

/*
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

// get_angle_boost - returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1000
int16_t AC_AttitudeControl_Heli::get_angle_boost(int16_t throttle_pwm)
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
