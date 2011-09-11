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
#include <APM_RC.h>
#include "WProgram.h"
#include "RC_Channel.h"

#define RC_CHANNEL_ANGLE 0
#define RC_CHANNEL_RANGE 1
#define RC_CHANNEL_ANGLE_RAW 2


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
		control_in = (control_in < dead_zone) ? 0 : control_in;
		//if (fabs(scale_output) > 0){
		//	control_in *= scale_output;
		//}
	}else{
		control_in = pwm_to_angle();
		control_in = (abs(control_in) < dead_zone) ? 0 : control_in;
		//if (fabs(scale_output) > 0){
		//	control_in *= scale_output;
		//}
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
		radio_out 	= (_reverse >=0 ? pwm_out + radio_min : radio_max - pwm_out);

	}else if(_type == RC_CHANNEL_ANGLE_RAW){
		pwm_out 	= (float)servo_out * .1;
		radio_out 	= (pwm_out * _reverse) + 1500;

	}else{
		pwm_out 	= angle_to_pwm();
		radio_out 	= pwm_out + radio_trim;
	}
	radio_out = constrain(radio_out, radio_min.get(), radio_max.get());
}

// ------------------------------------------

void
RC_Channel::load_eeprom(void)
{
    _group.load();
}

void
RC_Channel::save_eeprom(void)
{
    _group.save();
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
	if(radio_in > radio_trim)
		return _reverse * ((long)_high * (long)(radio_in - radio_trim)) / (long)(radio_max  - radio_trim);
	else
		return _reverse * ((long)_high * (long)(radio_in - radio_trim)) / (long)(radio_trim - radio_min);
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
	//return (_low + ((_high - _low) * ((float)(radio_in - radio_min) / (float)(radio_max - radio_min))));
	return (_low + ((long)(_high - _low) * (long)(radio_in - radio_min)) / (long)(radio_max - radio_min));
}

int16_t
RC_Channel::range_to_pwm()
{
	//return (((float)(servo_out - _low) / (float)(_high - _low)) * (float)(radio_max - radio_min));
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
	if(radio_out < radio_trim)
		return (float)(radio_out - radio_trim) / (float)(radio_trim - radio_min);
	else
		return (float)(radio_out - radio_trim) / (float)(radio_max  - radio_trim);
}

int16_t
RC_Channel_aux::closest_limit(int16_t angle)
{
	// Change scaling to 0.1 degrees in order to avoid overflows in the angle arithmetic
	int16_t min = angle_min / 10;
	int16_t max = angle_max / 10;

	// Make sure the angle lies in the interval [-180 .. 180[ degrees
	while (angle < -1800) angle += 3600;
	while (angle >= 1800) angle -= 3600;

	// Make sure the angle limits lie in the interval [-180 .. 180[ degrees
	while (min < -1800) min += 3600;
	while (min >= 1800) min -= 3600;
	while (max < -1800) max += 3600;
	while (max >= 1800) max -= 3600;
	set_range(min, max);

	// If the angle is outside servo limits, saturate the angle to the closest limit
	// On a circle the closest angular position must be carefully calculated to account for wrap-around
	if ((angle < min) && (angle > max)){
		// angle error if min limit is used
		int16_t err_min = min - angle + (angle<min?0:3600); // add 360 degrees if on the "wrong side"
		// angle error if max limit is used
		int16_t err_max = angle - max + (angle>max?0:3600); // add 360 degrees if on the "wrong side"
		angle = err_min<err_max?min:max;
	}

	servo_out = angle;
	// convert angle to PWM using a linear transformation (ignores trimming because the camera limits might not be symmetric)
	calc_pwm();

	return angle;
}

// map a function to a servo channel and output it
void
RC_Channel_aux::output_ch(unsigned char ch_nr)
{
	switch(function)
	{
	case k_none: 		// disabled
		return;
		break;
	case k_manual:		// manual
		radio_out = radio_in;
		break;
	case k_flap:		// flaps
	case k_flap_auto:	// flaps automated
	case k_aileron:		// aileron
	case k_flaperon:	// flaperon (flaps and aileron combined, needs two independent servos one for each wing)
	case k_mount_yaw:	// mount yaw (pan)
	case k_mount_pitch:	// mount pitch (tilt)
	case k_mount_roll:	// mount roll
	case k_cam_trigger:	// camera trigger
	case k_cam_open:	// camera open
	case k_egg_drop:	// egg drop
	case k_nr_aux_servo_functions: // dummy, just to avoid a compiler warning
	default:
		break;
	}

	APM_RC.OutputCh(ch_nr, radio_out);
}
