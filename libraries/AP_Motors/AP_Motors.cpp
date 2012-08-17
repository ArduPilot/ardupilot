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

// throttle_pass_through - passes throttle through to motors - dangerous but used for initialising ESCs
void AP_Motors::throttle_pass_through() {
    if( armed() ) {
        for( int16_t i=0; i < AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
            _rc->OutputCh(_motor_to_channel_map[i], _rc_throttle->radio_in);
        }
    }
}
