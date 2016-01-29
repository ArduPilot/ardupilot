// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

/*
 *       RC_Channel.cpp - Radio library for Arduino
 *       Code by Jason Short. DIYDrones.com
 *
 */

#include <stdlib.h>
#include <math.h>

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

#include <AP_Math/AP_Math.h>

#include "RC_Channel.h"

/// global array with pointers to all APM RC channels, will be used by AP_Mount
/// and AP_Camera classes / It points to RC input channels.
RC_Channel *RC_Channel::rc_ch[RC_MAX_CHANNELS];

const AP_Param::GroupInfo RC_Channel::var_info[] = {
    // @Param: MIN
    // @DisplayName: RC min PWM
    // @Description: RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MIN",  0, RC_Channel, radio_min, 1100),

    // @Param: TRIM
    // @DisplayName: RC trim PWM
    // @Description: RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TRIM", 1, RC_Channel, radio_trim, 1500),

    // @Param: MAX
    // @DisplayName: RC max PWM
    // @Description: RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAX",  2, RC_Channel, radio_max, 1900),

    // @Param: REV
    // @DisplayName: RC reverse
    // @Description: Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
    // @Values: -1:Reversed,1:Normal
    // @User: Advanced
    AP_GROUPINFO("REV",  3, RC_Channel, _reverse, 1),

    // Note: index 4 was used by the previous _dead_zone value. We
    // changed it to 5 as dead zone values had previously been
    // incorrectly saved, overriding user values. They were also
    // incorrectly interpreted for the throttle on APM:Plane

    // @Param: DZ
    // @DisplayName: RC dead-zone
    // @Description: dead zone around trim or bottom
    // @Units: pwm
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("DZ",   5, RC_Channel, _dead_zone, 0),

    AP_GROUPEND
};

// setup the control preferences
void
RC_Channel::set_range(int16_t low, int16_t high)
{
    set_range_in(low, high);
    set_range_out(low, high);
}

void
RC_Channel::set_range_out(int16_t low, int16_t high)
{
    _type_out       = RC_CHANNEL_TYPE_RANGE;
    _high_out       = high;
    _low_out        = low;
}

void
RC_Channel::set_range_in(int16_t low, int16_t high)
{
    _type_in       = RC_CHANNEL_TYPE_RANGE;
    _high_in       = high;
    _low_in        = low;
}

void
RC_Channel::set_angle(int16_t angle)
{
    set_angle_in(angle);
    set_angle_out(angle);
}

void
RC_Channel::set_angle_out(int16_t angle)
{
    _type_out   = RC_CHANNEL_TYPE_ANGLE;
    _high_out   = angle;
}

void
RC_Channel::set_angle_in(int16_t angle)
{
    _type_in   = RC_CHANNEL_TYPE_ANGLE;
    _high_in   = angle;
}

void
RC_Channel::set_default_dead_zone(int16_t dzone)
{
    _dead_zone.set_default(abs(dzone));
}

void
RC_Channel::set_reverse(bool reverse)
{
    if (reverse) _reverse = -1;
    else _reverse = 1;
}

bool
RC_Channel::get_reverse(void) const
{
    if (_reverse == -1) {
        return true;
    }
    return false;
}

void
RC_Channel::set_type(uint8_t t)
{
    set_type_in(t);
    set_type_out(t);
}

void
RC_Channel::set_type_in(uint8_t t)
{
    _type_in  = t;
}

void
RC_Channel::set_type_out(uint8_t t)
{
    _type_out = t;
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

    if (_type_in == RC_CHANNEL_TYPE_RANGE) {
        control_in = pwm_to_range();
    } else {
        //RC_CHANNEL_TYPE_ANGLE, RC_CHANNEL_TYPE_ANGLE_RAW
        control_in = pwm_to_angle();
    }
}

/*
  call read() and set_pwm() on all channels
 */
void
RC_Channel::set_pwm_all(void)
{
    for (uint8_t i=0; i<RC_MAX_CHANNELS; i++) {
        if (rc_ch[i] != NULL) {
            rc_ch[i]->set_pwm(rc_ch[i]->read());
        }
    }
}

// read input from APM_RC - create a control_in value, but use a 
// zero value for the dead zone. When done this way the control_in
// value can be used as servo_out to give the same output as input
void
RC_Channel::set_pwm_no_deadzone(int16_t pwm)
{
    radio_in = pwm;

    if (_type_in == RC_CHANNEL_TYPE_RANGE) {
        control_in = pwm_to_range_dz(0);
    } else {
        //RC_CHANNEL_ANGLE, RC_CHANNEL_ANGLE_RAW
        control_in = pwm_to_angle_dz(0);
    }
}

int16_t
RC_Channel::control_mix(float value)
{
    return (1 - abs(control_in / _high_in)) * value + control_in;
}

// returns just the PWM without the offset from radio_min
void
RC_Channel::calc_pwm(void)
{
    if(_type_out == RC_CHANNEL_TYPE_RANGE) {
        pwm_out         = range_to_pwm();
        radio_out       = (_reverse >= 0) ? (radio_min + pwm_out) : (radio_max - pwm_out);

    }else if(_type_out == RC_CHANNEL_TYPE_ANGLE_RAW) {
        pwm_out         = (float)servo_out * 0.1f;
        int16_t reverse_mul = (_reverse==-1?-1:1);
        radio_out       = (pwm_out * reverse_mul) + radio_trim;

    }else{     // RC_CHANNEL_TYPE_ANGLE
        pwm_out         = angle_to_pwm();
        radio_out       = pwm_out + radio_trim;
    }

    radio_out = constrain_int16(radio_out, radio_min.get(), radio_max.get());
}


/*
  return the center stick position expressed as a control_in value
  used for thr_mid in copter
 */
int16_t
RC_Channel::get_control_mid() const {
    if (_type_in == RC_CHANNEL_TYPE_RANGE) {
        int16_t r_in = (radio_min.get()+radio_max.get())/2;

        if (_reverse == -1) {
            r_in = radio_max.get() - (r_in - radio_min.get());
        }

        int16_t radio_trim_low  = radio_min + _dead_zone;

        return (_low_in + ((int32_t)(_high_in - _low_in) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_max - radio_trim_low));
    } else {
        return 0;
    }
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
    radio_min = MIN(radio_min.get(), radio_in);
    radio_max = MAX(radio_max.get(), radio_in);
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value using the specified dead_zone
 */
int16_t
RC_Channel::pwm_to_angle_dz_trim(uint16_t dead_zone, uint16_t _trim)
{
    int16_t radio_trim_high = _trim + dead_zone;
    int16_t radio_trim_low  = _trim - dead_zone;

    // prevent div by 0
    if ((radio_trim_low - radio_min) == 0 || (radio_max - radio_trim_high) == 0)
        return 0;

    int16_t reverse_mul = (_reverse==-1?-1:1);
    if(radio_in > radio_trim_high) {
        return reverse_mul * ((int32_t)_high_in * (int32_t)(radio_in - radio_trim_high)) / (int32_t)(radio_max  - radio_trim_high);
    }else if(radio_in < radio_trim_low) {
        return reverse_mul * ((int32_t)_high_in * (int32_t)(radio_in - radio_trim_low)) / (int32_t)(radio_trim_low - radio_min);
    }else
        return 0;
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value using the specified dead_zone
 */
int16_t
RC_Channel::pwm_to_angle_dz(uint16_t dead_zone)
{
    return pwm_to_angle_dz_trim(dead_zone, radio_trim);
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
    int16_t reverse_mul = (_reverse==-1?-1:1);
    if((servo_out * reverse_mul) > 0) {
        return reverse_mul * ((int32_t)servo_out * (int32_t)(radio_max - radio_trim)) / (int32_t)_high_out;
    } else {
        return reverse_mul * ((int32_t)servo_out * (int32_t)(radio_trim - radio_min)) / (int32_t)_high_out;
    }
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
        return (_low_in + ((int32_t)(_high_in - _low_in) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_max - radio_trim_low));
    else if (dead_zone > 0)
        return 0;
    else
        return _low_in;
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
    if (_high_out == _low_out) {
        return radio_trim;
    }
    return ((int32_t)(servo_out - _low_out) * (int32_t)(radio_max - radio_min)) / (int32_t)(_high_out - _low_out);
}

// ------------------------------------------

float
RC_Channel::norm_input()
{
    float ret;
    int16_t reverse_mul = (_reverse==-1?-1:1);
    if (radio_in < radio_trim) {
        if (radio_min >= radio_trim) {
            return 0.0f;
        }
        ret = reverse_mul * (float)(radio_in - radio_trim) / (float)(radio_trim - radio_min);
    } else {
        if (radio_max <= radio_trim) {
            return 0.0f;
        }
        ret = reverse_mul * (float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim);
    }
    return constrain_float(ret, -1.0f, 1.0f);
}

float
RC_Channel::norm_input_dz()
{
    int16_t dz_min = radio_trim - _dead_zone;
    int16_t dz_max = radio_trim + _dead_zone;
    float ret;
    int16_t reverse_mul = (_reverse==-1?-1:1);
    if (radio_in < dz_min && dz_min > radio_min) {
        ret = reverse_mul * (float)(radio_in - dz_min) / (float)(dz_min - radio_min);
    } else if (radio_in > dz_max && radio_max > dz_max) {
        ret = reverse_mul * (float)(radio_in - dz_max) / (float)(radio_max  - dz_max);
    } else {
        ret = 0;
    }
    return constrain_float(ret, -1.0f, 1.0f);
}

/*
  get percentage input from 0 to 100. This ignores the trim value.
 */
uint8_t
RC_Channel::percent_input()
{
    if (radio_in <= radio_min) {
        return _reverse==-1?100:0;
    }
    if (radio_in >= radio_max) {
        return _reverse==-1?0:100;
    }
    uint8_t ret = 100.0f * (radio_in - radio_min) / (float)(radio_max - radio_min);
    if (_reverse == -1) {
        ret = 100 - ret;
    }
    return ret;
}

float
RC_Channel::norm_output()
{
    int16_t mid = (radio_max + radio_min) / 2;
    float ret;
    if (mid <= radio_min) {
        return 0;
    }
    if (radio_out < mid) {
        ret = (float)(radio_out - mid) / (float)(mid - radio_min);
    } else if (radio_out > mid) {
        ret = (float)(radio_out - mid) / (float)(radio_max  - mid);
    } else {
        ret = 0;
    }
    if (_reverse == -1) {
	    ret = -ret;
    }
    return ret;
}

void RC_Channel::output() const
{
    hal.rcout->write(_ch_out, radio_out);
}

void RC_Channel::output_trim() const
{
    hal.rcout->write(_ch_out, radio_trim);
}

void RC_Channel::output_trim_all()
{
    for (uint8_t i=0; i<RC_MAX_CHANNELS; i++) {
        if (rc_ch[i] != NULL) {
            rc_ch[i]->output_trim();
        }
    }
}

/*
  setup the failsafe value to the trim value for all channels
 */
void RC_Channel::setup_failsafe_trim_all()
{
    for (uint8_t i=0; i<RC_MAX_CHANNELS; i++) {
        if (rc_ch[i] != NULL) {
            hal.rcout->set_failsafe_pwm(1U<<i, rc_ch[i]->radio_trim);
        }
    }
}

void
RC_Channel::input()
{
    radio_in = hal.rcin->read(_ch_out);
}

uint16_t
RC_Channel::read() const
{
    return hal.rcin->read(_ch_out);
}

void
RC_Channel::enable_out()
{
    hal.rcout->enable_ch(_ch_out);
}

void
RC_Channel::disable_out()
{
    hal.rcout->disable_ch(_ch_out);
}

RC_Channel *RC_Channel::rc_channel(uint8_t i)
{
    if (i >= RC_MAX_CHANNELS) {
        return NULL;
    }
    return rc_ch[i];
}

// return a limit PWM value
uint16_t RC_Channel::get_limit_pwm(LimitValue limit) const
{
    switch (limit) {
    case RC_CHANNEL_LIMIT_TRIM:
        return radio_trim;
    case RC_CHANNEL_LIMIT_MAX:
        return get_reverse() ? radio_min : radio_max;
    case RC_CHANNEL_LIMIT_MIN:
        return get_reverse() ? radio_max : radio_min;
    }
    // invalid limit value, return trim
    return radio_trim;
}

/*
  Return true if the channel is at trim and within the DZ
*/
bool RC_Channel::in_trim_dz()
{
    return is_bounded_int32(radio_in, radio_trim - _dead_zone, radio_trim + _dead_zone);
}
