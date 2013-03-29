// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       RC_Channel.cpp - Radio library for Arduino
 *       Code by Jason Short. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and / or
 *               modify it under the terms of the GNU Lesser General Public
 *               License as published by the Free Software Foundation; either
 *               version 2.1 of the License, or (at your option) any later version.
 *
 */

#include <stdlib.h>
#include <math.h>

#include <AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include <AP_Math.h>

#include "RC_Channel.h"

#define NUM_CHANNELS 8
/// global array with pointers to all APM RC channels, will be used by AP_Mount
/// and AP_Camera classes / It points to RC input channels, both APM1 and APM2
/// only have 8 input channels.
RC_Channel* rc_ch[NUM_CHANNELS];

const AP_Param::GroupInfo RC_Channel::var_info[] PROGMEM = {
    // @Param: MIN
    // @DisplayName: RC min PWM
    // @Description: RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: ms
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MIN",  0, RC_Channel, radio_min, 1100),

    // @Param: TRIM
    // @DisplayName: RC trim PWM
    // @Description: RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: ms
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TRIM", 1, RC_Channel, radio_trim, 1500),

    // @Param: MAX
    // @DisplayName: RC max PWM
    // @Description: RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: ms
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAX",  2, RC_Channel, radio_max, 1900),

    // @Param: REV
    // @DisplayName: RC reverse
    // @Description: Reverse servo operation. Ignored on APM1 unless dip-switches are disabled.
    // @Values: -1:Reversed,1:Normal
    // @User: Advanced
    AP_GROUPINFO("REV",  3, RC_Channel, _reverse, 1),

    // @Param: DZ
    // @DisplayName: RC dead-zone
    // @Description: dead zone around trim.
    // @User: Advanced
    AP_GROUPINFO("DZ",   4, RC_Channel, _dead_zone, 0),
    AP_GROUPEND
};

// setup the control preferences
void
RC_Channel::set_range(int16_t low, int16_t high)
{
    _type           = RC_CHANNEL_TYPE_RANGE;
    _high           = high;
    _low            = low;
    _high_out       = high;
    _low_out        = low;
}

void
RC_Channel::set_range_out(int16_t low, int16_t high)
{
    _high_out       = high;
    _low_out        = low;
}

void
RC_Channel::set_angle(int16_t angle)
{
    _type   = RC_CHANNEL_TYPE_ANGLE;
    _high   = angle;
}

void
RC_Channel::set_dead_zone(int16_t dzone)
{
    _dead_zone.set_and_save(abs(dzone >>1));
}

void
RC_Channel::set_reverse(bool reverse)
{
    if (reverse) _reverse = -1;
    else _reverse = 1;
}

bool
RC_Channel::get_reverse(void)
{
    if (_reverse==-1) return 1;
    else return 0;
}

void
RC_Channel::set_type(uint8_t t)
{
    _type = t;
}

// call after first read
void
RC_Channel::trim()
{
    radio_trim = radio_in;
}

// read input from APM_RC - create a control_in value
void
RC_Channel::set_pwm(int16_t pwm)
{
    radio_in = pwm;

    if(_type == RC_CHANNEL_TYPE_RANGE) {
        control_in = pwm_to_range();
        control_in = (control_in < _dead_zone) ? 0 : control_in;
    } else {
        //RC_CHANNEL_TYPE_ANGLE, RC_CHANNEL_TYPE_ANGLE_RAW
        control_in = pwm_to_angle();
    }
}

// read input from APM_RC - create a control_in value, but use a 
// zero value for the dead zone. When done this way the control_in
// value can be used as servo_out to give the same output as input
void
RC_Channel::set_pwm_no_deadzone(int16_t pwm)
{
    radio_in = pwm;

    if (_type == RC_CHANNEL_TYPE_RANGE) {
        control_in = pwm_to_range_dz(0);
    } else {
        //RC_CHANNEL_ANGLE, RC_CHANNEL_ANGLE_RAW
        control_in = pwm_to_angle_dz(0);
    }
}

int16_t
RC_Channel::control_mix(float value)
{
    return (1 - abs(control_in / _high)) * value + control_in;
}

// are we below a threshold?
bool
RC_Channel::get_failsafe(void)
{
    return (radio_in < (radio_min - 50));
}

// returns just the PWM without the offset from radio_min
void
RC_Channel::calc_pwm(void)
{
    if(_type == RC_CHANNEL_TYPE_RANGE) {
        pwm_out         = range_to_pwm();
        radio_out       = (_reverse >= 0) ? (radio_min + pwm_out) : (radio_max - pwm_out);

    }else if(_type == RC_CHANNEL_TYPE_ANGLE_RAW) {
        pwm_out         = (float)servo_out * 0.1f;
        radio_out       = (pwm_out * _reverse) + radio_trim;

    }else{     // RC_CHANNEL_TYPE_ANGLE
        pwm_out         = angle_to_pwm();
        radio_out       = pwm_out + radio_trim;
    }

    radio_out = constrain_int16(radio_out, radio_min.get(), radio_max.get());
}

// ------------------------------------------

void
RC_Channel::load_eeprom(void)
{
    radio_min.load();
    radio_trim.load();
    radio_max.load();
    _reverse.load();
    _dead_zone.load();
}

void
RC_Channel::save_eeprom(void)
{
    radio_min.save();
    radio_trim.save();
    radio_max.save();
    _reverse.save();
    _dead_zone.save();
}

// ------------------------------------------

void
RC_Channel::zero_min_max()
{
    radio_min = radio_max = radio_in;
}

void
RC_Channel::update_min_max()
{
    radio_min = min(radio_min.get(), radio_in);
    radio_max = max(radio_max.get(), radio_in);
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value using the specified dead_zone
 */
int16_t
RC_Channel::pwm_to_angle_dz(uint16_t dead_zone)
{
    int16_t radio_trim_high = radio_trim + dead_zone;
    int16_t radio_trim_low  = radio_trim - dead_zone;

    // prevent div by 0
    if ((radio_trim_low - radio_min) == 0 || (radio_max - radio_trim_high) == 0)
        return 0;

    if(radio_in > radio_trim_high) {
        return _reverse * ((long)_high * (long)(radio_in - radio_trim_high)) / (long)(radio_max  - radio_trim_high);
    }else if(radio_in < radio_trim_low) {
        return _reverse * ((long)_high * (long)(radio_in - radio_trim_low)) / (long)(radio_trim_low - radio_min);
    }else
        return 0;
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value
 */
int16_t
RC_Channel::pwm_to_angle()
{
	return pwm_to_angle_dz(_dead_zone);
}


int16_t
RC_Channel::angle_to_pwm()
{
    if((servo_out * _reverse) > 0)
        return _reverse * ((long)servo_out * (long)(radio_max - radio_trim)) / (long)_high;
    else
        return _reverse * ((long)servo_out * (long)(radio_trim - radio_min)) / (long)_high;
}

/*
  convert a pulse width modulation value to a value in the configured
  range, using the specified deadzone
 */
int16_t
RC_Channel::pwm_to_range_dz(uint16_t dead_zone)
{
    int16_t r_in = constrain_int16(radio_in, radio_min.get(), radio_max.get());

    if (_reverse == -1) {
	    r_in = radio_max.get() - (r_in - radio_min.get());
    }

    int16_t radio_trim_low  = radio_min + dead_zone;

    if (r_in > radio_trim_low)
        return (_low + ((long)(_high - _low) * (long)(r_in - radio_trim_low)) / (long)(radio_max - radio_trim_low));
    else if (dead_zone > 0)
        return 0;
    else
        return _low;
}

/*
  convert a pulse width modulation value to a value in the configured
  range
 */
int16_t
RC_Channel::pwm_to_range()
{
    return pwm_to_range_dz(_dead_zone);
}


int16_t
RC_Channel::range_to_pwm()
{
    return ((long)(servo_out - _low_out) * (long)(radio_max - radio_min)) / (long)(_high_out - _low_out);
}

// ------------------------------------------

float
RC_Channel::norm_input()
{
    if(radio_in < radio_trim)
        return _reverse * (float)(radio_in - radio_trim) / (float)(radio_trim - radio_min);
    else
        return _reverse * (float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim);
}

float
RC_Channel::norm_output()
{
    int16_t mid = (radio_max + radio_min) / 2;
    float ret;
    if(radio_out < mid)
        ret = (float)(radio_out - mid) / (float)(mid - radio_min);
    else
        ret = (float)(radio_out - mid) / (float)(radio_max  - mid);
    if (_reverse == -1) {
	    ret = -ret;
    }
    return ret;
}

void RC_Channel::output()
{
    hal.rcout->write(_ch_out, radio_out);
}

void
RC_Channel::input()
{
    radio_in = hal.rcin->read(_ch_out);
}

void
RC_Channel::enable_out()
{
    hal.rcout->enable_ch(_ch_out);
}
