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

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_MotorsHeli_RSC.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo RSCThrCrvParam::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable settings for RSC Setpoint
    // @Description: Automatically set when RSC Setpoint mode is selected. Should not be set manually.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, RSCThrCrvParam, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: 000
    // @DisplayName: Throttle Servo Position in percent for 0 percent collective
    // @Description: Throttle Servo Position in percent for 0 percent collective. This is on a scale from 0 to 100, where 100 is full throttle and 0 is zero throttle. Actual PWM values are controlled by SERVOX_MIN and SERVOX_MAX. The 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("000", 2, RSCThrCrvParam, thrcrv[0], AP_MOTORS_HELI_RSC_THRCRV_0_DEFAULT),

    // @Param: 025
    // @DisplayName: Throttle Servo Position in percent for 25 percent collective
    // @Description: Throttle Servo Position in percent for 25 percent collective. This is on a scale from 0 to 100, where 100 is full throttle and 0 is zero throttle. Actual PWM values are controlled by SERVOX_MIN and SERVOX_MAX. The 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("025", 3, RSCThrCrvParam, thrcrv[1], AP_MOTORS_HELI_RSC_THRCRV_25_DEFAULT),

    // @Param: 050
    // @DisplayName: Throttle Servo Position in percent for 50 percent collective
    // @Description: Throttle Servo Position in percent for 50 percent collective. This is on a scale from 0 to 100, where 100 is full throttle and 0 is zero throttle. Actual PWM values are controlled by SERVOX_MIN and SERVOX_MAX. The 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("050", 4, RSCThrCrvParam, thrcrv[2], AP_MOTORS_HELI_RSC_THRCRV_50_DEFAULT),

    // @Param: 075
    // @DisplayName: Throttle Servo Position in percent for 75 percent collective
    // @Description: Throttle Servo Position in percent for 75 percent collective. This is on a scale from 0 to 100, where 100 is full throttle and 0 is zero throttle. Actual PWM values are controlled by SERVOX_MIN and SERVOX_MAX. The 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("075", 5, RSCThrCrvParam, thrcrv[3], AP_MOTORS_HELI_RSC_THRCRV_75_DEFAULT),

    // @Param: 100
    // @DisplayName: Throttle Servo Position in percent for 100 percent collective
    // @Description: Throttle Servo Position in percent for 100 percent collective. This is on a scale from 0 to 100, where 100 is full throttle and 0 is zero throttle. Actual PWM values are controlled by SERVOX_MIN and SERVOX_MAX. The 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("100", 6, RSCThrCrvParam, thrcrv[4], AP_MOTORS_HELI_RSC_THRCRV_100_DEFAULT),

    AP_GROUPEND
};

const AP_Param::GroupInfo RSCGovParam::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable settings for RSC Governor
    // @Description: Automatically set when RSC Governor mode is selected. Should not be set manually.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, RSCGovParam, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: SETPNT
    // @DisplayName: Governor RPM Reference Setting
    // @Description: Main rotor rpm setting that governor maintains when engaged
    // @Range: 800 3500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("SETPNT", 2, RSCGovParam, reference, AP_MOTORS_HELI_RSC_GOVERNOR_SETPNT_DEFAULT),

    // @Param: DISGAG
    // @DisplayName: Throttle Percentage for Governor Disengage
    // @Description: Percentage of throttle where the governor will disenage to allow return to flight idle power
    // @Range: 0 50
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DISGAG", 3, RSCGovParam, disengage, AP_MOTORS_HELI_RSC_GOVERNOR_DISENGAGE_DEFAULT),

    // @Param: DROOP
    // @DisplayName: Governor Droop Response Setting
    // @Description: Governor droop response under load, 0-100%. Higher value is quicker response but may cause surging
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DROOP", 4, RSCGovParam, droop_response, AP_MOTORS_HELI_RSC_GOVERNOR_DROOP_DEFAULT),

    // @Param: THRCURVE
    // @DisplayName: Governor Throttle Curve Gain
    // @Description: Percentage of throttle curve gain in governor output
    // @Range: 50 100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THRCURVE", 5, RSCGovParam, thrcurve, AP_MOTORS_HELI_RSC_GOVERNOR_THRCURVE_DEFAULT),
    
    // @Param: RANGE
    // @DisplayName: Governor Operational Range
    // @Description: RPM range +/- governor rpm reference setting where governor is operational. If speed sensor fails or rpm falls outside of this range, the governor will disengage and return to throttle curve. Recommended range is 100
    // @Range: 50 200
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RANGE", 6, RSCGovParam, range, AP_MOTORS_HELI_RSC_GOVERNOR_RANGE_DEFAULT),

    AP_GROUPEND
};

RSCThrCrvParam::RSCThrCrvParam(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

RSCGovParam::RSCGovParam(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// init_servo - servo initialization on start-up
void AP_MotorsHeli_RSC::init_servo()
{
    // setup RSC on specified channel by default
    SRV_Channels::set_aux_channel_default(_aux_fn, _default_channel);

    // set servo range 
    SRV_Channels::set_range(SRV_Channels::get_motor_function(_aux_fn), 1000);

}

// set_power_output_range
// TODO: Look at possibly calling this at a slower rate.  Doesn't need to be called every cycle.
void AP_MotorsHeli_RSC::set_throttle_curve(float thrcrv[5])
{

    // Ensure user inputs are within parameter limits
    for (uint8_t i = 0; i < 5; i++) {
        thrcrv[i] = constrain_float(thrcrv[i], 0.0f, 1.0f);
    }
    // Calculate the spline polynomials for the throttle curve
    splinterp5(thrcrv,_thrcrv_poly);

}

// output - update value to send to ESC/Servo
void AP_MotorsHeli_RSC::output(RotorControlState state)
{
    float dt;
    uint64_t now = AP_HAL::micros64();
    float last_control_output = _control_output;

    if (_last_update_us == 0) {
        _last_update_us = now;
        dt = 0.001f;
    } else {
        dt = 1.0e-6f * (now - _last_update_us);
        _last_update_us = now;
    }

    switch (state){
        case ROTOR_CONTROL_STOP:
            // set rotor ramp to decrease speed to zero, this happens instantly inside update_rotor_ramp()
            update_rotor_ramp(0.0f, dt);

            // control output forced to zero
            _control_output = 0.0f;
            break;

        case ROTOR_CONTROL_IDLE:
            // set rotor ramp to decrease speed to zero
            update_rotor_ramp(0.0f, dt);

            // set rotor control speed to idle speed parameter, this happens instantly and ignore ramping
            _control_output = _idle_output;
            break;

        case ROTOR_CONTROL_ACTIVE:
            // set main rotor ramp to increase to full speed
            update_rotor_ramp(1.0f, dt);

            if ((_control_mode == ROTOR_CONTROL_MODE_SPEED_PASSTHROUGH) || (_control_mode == ROTOR_CONTROL_MODE_SPEED_SETPOINT)) {
                // set control rotor speed to ramp slewed value between idle and desired speed
                _control_output = _idle_output + (_rotor_ramp_output * (_desired_speed - _idle_output));
            } else if (_control_mode == ROTOR_CONTROL_MODE_OPEN_LOOP_POWER_OUTPUT) {
                // throttle output from throttle curve based on collective position
                float desired_throttle = calculate_desired_throttle(_collective_in);
                _control_output = _idle_output + (_rotor_ramp_output * (desired_throttle - _idle_output));
            } else if (_control_mode == ROTOR_CONTROL_MODE_CLOSED_LOOP_POWER_OUTPUT) {
                // governor provides two modes of throttle control - governor engaged
                // or throttle curve if governor is out of range or sensor failed
            	float desired_throttle = calculate_desired_throttle(_collective_in);
            	// governor is active if within user-set range from reference speed
                if ((_rotor_rpm >= (_governor_reference - _governor_range)) && (_rotor_rpm <= (_governor_reference + _governor_range))) {
            	    float governor_droop = constrain_float(_governor_reference - _rotor_rpm,0.0f,_governor_range);
            	    // if rpm has not reached 40% of the operational range from reference speed, governor
            	    // remains in pre-engage status, no reference speed compensation due to droop
            	    // this provides a soft-start function that engages the governor less aggressively
            	    if (_governor_engage && _rotor_rpm < (_governor_reference - (_governor_range * 0.4f))) {
                        _governor_output = ((_rotor_rpm - _governor_reference) * desired_throttle) * _governor_droop_response * -0.01f;
                    } else {
            	        // normal flight status, governor fully engaged with reference speed compensation for droop
            	        _governor_engage = true;
                        _governor_output = ((_rotor_rpm - (_governor_reference + governor_droop)) * desired_throttle) * _governor_droop_response * -0.01f;
                    }
                    // check for governor disengage for return to flight idle power
                    if (desired_throttle <= _governor_disengage) {
                        _governor_output = 0.0f;
                        _governor_engage = false;
                    }
                    // throttle output with governor on is constrained from minimum called for from throttle curve
                    // to maximum WOT. This prevents outliers on rpm signal from closing the throttle in flight due
                    // to rpm sensor failure or bad signal quality
            	    _control_output = constrain_float(_idle_output + (_rotor_ramp_output * (((desired_throttle * _governor_thrcurve) + _governor_output) - _idle_output)), _idle_output + (_rotor_ramp_output * ((desired_throttle * _governor_thrcurve)) - _idle_output), 1.0f);
            	} else {
            	    // hold governor output at zero, engage status is false and use the throttle curve
            	    // this is failover for in-flight failure of the speed sensor
            	    _governor_output = 0.0f;
            	    _governor_engage = false;
                    _control_output = _idle_output + (_rotor_ramp_output * (desired_throttle - _idle_output));
                }
            }
            break;
    }

    // update rotor speed run-up estimate
    update_rotor_runup(dt);

    if (_power_slewrate > 0) {
        // implement slew rate for throttle
        float max_delta = dt * _power_slewrate * 0.01f;
        _control_output = constrain_float(_control_output, last_control_output-max_delta, last_control_output+max_delta);
    }

    // output to rsc servo
    write_rsc(_control_output);
}

// update_rotor_ramp - slews rotor output scalar between 0 and 1, outputs float scalar to _rotor_ramp_output
void AP_MotorsHeli_RSC::update_rotor_ramp(float rotor_ramp_input, float dt)
{
    // sanity check ramp time
    if (_ramp_time <= 0) {
        _ramp_time = 1;
    }

    // ramp output upwards towards target
    if (_rotor_ramp_output < rotor_ramp_input) {
        // allow control output to jump to estimated speed
        if (_rotor_ramp_output < _rotor_runup_output) {
            _rotor_ramp_output = _rotor_runup_output;
        }
        // ramp up slowly to target
        _rotor_ramp_output += (dt / _ramp_time);
        if (_rotor_ramp_output > rotor_ramp_input) {
            _rotor_ramp_output = rotor_ramp_input;
        }
    }else{
        // ramping down happens instantly
        _rotor_ramp_output = rotor_ramp_input;
    }
}

// update_rotor_runup - function to slew rotor runup scalar, outputs float scalar to _rotor_runup_ouptut
void AP_MotorsHeli_RSC::update_rotor_runup(float dt)
{
    // sanity check runup time
    if (_runup_time < _ramp_time) {
        _runup_time = _ramp_time;
    }
    if (_runup_time <= 0 ) {
        _runup_time = 1;
    }

    // ramp speed estimate towards control out
    float runup_increment = dt / _runup_time;
    if (_rotor_runup_output < _rotor_ramp_output) {
        _rotor_runup_output += runup_increment;
        if (_rotor_runup_output > _rotor_ramp_output) {
            _rotor_runup_output = _rotor_ramp_output;
        }
    }else{
        _rotor_runup_output -= runup_increment;
        if (_rotor_runup_output < _rotor_ramp_output) {
            _rotor_runup_output = _rotor_ramp_output;
        }
    }

    // update run-up complete flag

    // if control mode is disabled, then run-up complete always returns true
    if ( _control_mode == ROTOR_CONTROL_MODE_DISABLED ){
        _runup_complete = true;
        return;
    }

    // if rotor ramp and runup are both at full speed, then run-up has been completed
    if (!_runup_complete && (_rotor_ramp_output >= 1.0f) && (_rotor_runup_output >= 1.0f)) {
        _runup_complete = true;
    }
    // if rotor speed is less than critical speed, then run-up is not complete
    // this will prevent the case where the target rotor speed is less than critical speed
    if (_runup_complete && (get_rotor_speed() <= _critical_speed)) {
        _runup_complete = false;
    }
}

// get_rotor_speed - gets rotor speed either as an estimate, or (ToDO) a measured value
float AP_MotorsHeli_RSC::get_rotor_speed() const
{
    // if no actual measured rotor speed is available, estimate speed based on rotor runup scalar.
    return _rotor_runup_output;
}

// write_rsc - outputs pwm onto output rsc channel
// servo_out parameter is of the range 0 ~ 1
void AP_MotorsHeli_RSC::write_rsc(float servo_out)
{
    if (_control_mode == ROTOR_CONTROL_MODE_DISABLED){
        // do not do servo output to avoid conflicting with other output on the channel
        // ToDo: We should probably use RC_Channel_Aux to avoid this problem
        return;
    } else {
        SRV_Channels::set_output_scaled(_aux_fn, (uint16_t) (servo_out * 1000));
    }
}

    // calculate_desired_throttle - uses throttle curve and collective input to determine throttle setting
float AP_MotorsHeli_RSC::calculate_desired_throttle(float collective_in)
{

    const float inpt = collective_in * 4.0f + 1.0f;
    uint8_t idx = constrain_int16(int8_t(collective_in * 4), 0, 3);
    const float a = inpt - (idx + 1.0f);
    const float b = (idx + 1.0f) - inpt + 1.0f;
    float throttle = _thrcrv_poly[idx][0] * a + _thrcrv_poly[idx][1] * b + _thrcrv_poly[idx][2] * (powf(a,3.0f) - a) / 6.0f + _thrcrv_poly[idx][3] * (powf(b,3.0f) - b) / 6.0f;

    throttle = constrain_float(throttle, 0.0f, 1.0f);
    return throttle;

}
