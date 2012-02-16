/*
	RC_Channel.cpp - Radio library for Arduino
	Code by Jason Short. DIYDrones.com

	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

*/

#include <math.h>
#include <avr/eeprom.h>
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#include "RC_Channel.h"

#define RC_CHANNEL_ANGLE 0
#define RC_CHANNEL_RANGE 1
#define RC_CHANNEL_ANGLE_RAW 2

APM_RC_Class *RC_Channel::_apm_rc;

const AP_Param::GroupInfo RC_Channel::var_info[] PROGMEM = {
	AP_GROUPINFO("MIN",  0, RC_Channel, radio_min),
	AP_GROUPINFO("TRIM", 1, RC_Channel, radio_trim),
	AP_GROUPINFO("MAX",  2, RC_Channel, radio_max),
	AP_GROUPINFO("REV",  3, RC_Channel, _reverse),
	AP_GROUPINFO("DZ",   4, RC_Channel, _dead_zone),
	AP_GROUPEND
};

// setup the control preferences
void
RC_Channel::set_range(int low, int high)
{
	_type 	= RC_CHANNEL_RANGE;
	_high 	= high;
	_low 	= low;
}

void
RC_Channel::set_angle(int angle)
{
	_type 	= RC_CHANNEL_ANGLE;
	_high 	= angle;
}

void
RC_Channel::set_dead_zone(int dzone)
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
RC_Channel::set_filter(bool filter)
{
	_filter = filter;
}

void
RC_Channel::set_type(uint8_t t)
{
	_type = t;
	//Serial.print("type1: ");
	//Serial.println(t,DEC);
}

// call after first read
void
RC_Channel::trim()
{
	radio_trim = radio_in;
}

// read input from APM_RC - create a control_in value
void
RC_Channel::set_pwm(int pwm)
{
	//Serial.print(pwm,DEC);

	if(_filter){
		if(radio_in == 0)
			radio_in = pwm;
		else
			radio_in = (pwm + radio_in) >> 1;		// Small filtering
	}else{
		radio_in = pwm;
	}

	if(_type == RC_CHANNEL_RANGE){
		//Serial.print("range ");
		control_in = pwm_to_range();
		control_in = (control_in < _dead_zone) ? 0 : control_in;

		if (fabs(scale_output) != 1){
			control_in *= scale_output;
		}

	}else{

		//RC_CHANNEL_ANGLE, RC_CHANNEL_ANGLE_RAW
		control_in = pwm_to_angle();


		if (fabs(scale_output) != 1){
			control_in *= scale_output;
		}

		/*
		// coming soon ??
		if(expo) {
			long temp = control_in;
			temp = (temp * temp) / (long)_high;
			control_in = (int)((control_in >= 0) ? temp : -temp);
		}*/
	}
}

int
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
	if(_type == RC_CHANNEL_RANGE){
		pwm_out 	= range_to_pwm();
		radio_out 	= (_reverse >= 0) ? (radio_min + pwm_out) : (radio_max - pwm_out);

	}else if(_type == RC_CHANNEL_ANGLE_RAW){
		pwm_out 	= (float)servo_out * .1;
		radio_out 	= (pwm_out * _reverse) + radio_trim;

	}else{ // RC_CHANNEL_ANGLE
		pwm_out 	= angle_to_pwm();
		radio_out 	= pwm_out + radio_trim;
	}

	radio_out = constrain(radio_out, radio_min.get(), radio_max.get());
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

// ------------------------------------------

int16_t
RC_Channel::pwm_to_angle()
{
	int radio_trim_high = radio_trim + _dead_zone;
	int radio_trim_low  = radio_trim - _dead_zone;
    
    // prevent div by 0
    if ((radio_trim_low - radio_min) == 0 || (radio_max - radio_trim_high) == 0) 
        return 0;

	if(radio_in > radio_trim_high){
		return _reverse * ((long)_high * (long)(radio_in - radio_trim_high)) / (long)(radio_max  - radio_trim_high);
	}else if(radio_in < radio_trim_low){
		return _reverse * ((long)_high * (long)(radio_in - radio_trim_low)) / (long)(radio_trim_low - radio_min);
	}else
		return 0;
}


int16_t
RC_Channel::angle_to_pwm()
{
	if((servo_out * _reverse) > 0)
		return _reverse * ((long)servo_out * (long)(radio_max - radio_trim)) / (long)_high;
	else
		return _reverse * ((long)servo_out * (long)(radio_trim - radio_min)) / (long)_high;
}

// ------------------------------------------

int16_t
RC_Channel::pwm_to_range()
{
	int radio_trim_low  = radio_min + _dead_zone;

	if(radio_in > radio_trim_low)
		return (_low + ((long)(_high - _low) * (long)(radio_in - radio_trim_low)) / (long)(radio_max - radio_trim_low));
	else
		return 0;
}


int16_t
RC_Channel::range_to_pwm()
{
	return ((long)(servo_out - _low) * (long)(radio_max - radio_min)) / (long)(_high - _low);
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
        
	if(radio_out < mid)
		return (float)(radio_out - mid) / (float)(mid - radio_min);
	else
		return (float)(radio_out - mid) / (float)(radio_max  - mid);
}

void RC_Channel::set_apm_rc( APM_RC_Class * apm_rc )
{
    _apm_rc = apm_rc;
}
