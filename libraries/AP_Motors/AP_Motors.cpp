/*
 *       AP_Motors.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */

#include "AP_Motors.h"

// parameters for the motor class
const AP_Param::GroupInfo AP_Motors::var_info[] PROGMEM = {
    AP_GROUPINFO("TB_RATIO", 0, AP_Motors,  top_bottom_ratio, AP_MOTORS_TOP_BOTTOM_RATIO),      // not used
    AP_GROUPINFO("TCRV_ENABLE", 1, AP_Motors, _throttle_curve_enabled, THROTTLE_CURVE_ENABLED),
    AP_GROUPINFO("TCRV_MIDPCT", 2, AP_Motors, _throttle_curve_mid, THROTTLE_CURVE_MID_THRUST),
    AP_GROUPINFO("TCRV_MAXPCT", 3, AP_Motors, _throttle_curve_max, THROTTLE_CURVE_MAX_THRUST),
    AP_GROUPEND
};

// Constructor
AP_Motors::AP_Motors( uint8_t APM_version, APM_RC_Class* rc_out, RC_Channel* rc_roll, RC_Channel* rc_pitch, RC_Channel* rc_throttle, RC_Channel* rc_yaw, uint16_t speed_hz ) :
    _rc(rc_out),
    _rc_roll(rc_roll),
    _rc_pitch(rc_pitch),
    _rc_throttle(rc_throttle),
    _rc_yaw(rc_yaw),
    _speed_hz(speed_hz),
    _armed(false),
    _auto_armed(false),
    _frame_orientation(0),
    _min_throttle(AP_MOTORS_DEFAULT_MIN_THROTTLE),
    _max_throttle(AP_MOTORS_DEFAULT_MAX_THROTTLE)
{
    uint8_t i;

    top_bottom_ratio = AP_MOTORS_TOP_BOTTOM_RATIO;

    // initialise motor map
    if( APM_version == AP_MOTORS_APM1 ) {
        set_motor_to_channel_map(APM1_MOTOR_TO_CHANNEL_MAP);
    } else {
        set_motor_to_channel_map(APM2_MOTOR_TO_CHANNEL_MAP);
    }

    // clear output arrays
    for(i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        motor_out[i] = 0;
    }
};

// init
void AP_Motors::Init()
{
    // set-up throttle curve - motors classes will decide whether to use it based on _throttle_curve_enabled parameter
    setup_throttle_curve();
};

// throttle_pass_through - passes throttle through to motors - dangerous but used for initialising ESCs
void AP_Motors::throttle_pass_through()
{
    if( armed() ) {
        for( int16_t i=0; i < AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
            _rc->OutputCh(_motor_to_channel_map[i], _rc_throttle->radio_in);
        }
    }
}

// setup_throttle_curve - used to linearlise thrust output by motors
// returns true if set up successfully
bool AP_Motors::setup_throttle_curve()
{
    int16_t min_pwm = _rc_throttle->radio_min;
    int16_t max_pwm = _rc_throttle->radio_max;
	int16_t mid_throttle_pwm = (max_pwm + min_pwm) / 2;
    int16_t mid_thrust_pwm = min_pwm + (float)(max_pwm - min_pwm) * ((float)_throttle_curve_mid/100.0);
    int16_t max_thrust_pwm = min_pwm + (float)(max_pwm - min_pwm) * ((float)_throttle_curve_max/100.0);
    bool retval = true;

    // some basic checks that the curve is valid
    if( mid_thrust_pwm >= (min_pwm+_min_throttle) && mid_thrust_pwm <= max_pwm && max_thrust_pwm >= mid_thrust_pwm && max_thrust_pwm <= max_pwm ) {
        // clear curve
        _throttle_curve.clear();

        // curve initialisation
        retval &= _throttle_curve.add_point(min_pwm, min_pwm);
        retval &= _throttle_curve.add_point(min_pwm+_min_throttle, min_pwm+_min_throttle);
        retval &= _throttle_curve.add_point(mid_throttle_pwm, mid_thrust_pwm);
        retval &= _throttle_curve.add_point(max_pwm, max_thrust_pwm);

        // return success
        return retval;
    }else{
        retval = false;
    }

    // disable throttle curve if not set-up corrrectly
    if( !retval ) {
        _throttle_curve_enabled = false;
        Serial.printf_P(PSTR("AP_Motors: failed to create throttle curve"));
    }

    return retval;
}