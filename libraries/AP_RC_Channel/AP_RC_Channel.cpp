/*
	AP_RC_Channel.cpp - Radio library for Arduino Legacy Hardware
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
#include "AP_RC_Channel.h"

#define ANGLE 0
#define RANGE 1

// setup the control preferences
void
AP_RC_Channel::set_range(int low, int high)
{
	_type 	= RANGE;
	_high 	= high;
	_low 	= low;
}

void
AP_RC_Channel::set_angle(int angle)
{
	_type 	= ANGLE;
	_high 	= angle;
}

void
AP_RC_Channel::set_reverse(bool reverse)
{
	if (reverse) _reverse = -1;
	else _reverse = 1;
}

bool
AP_RC_Channel::get_reverse(void)
{
	if (_reverse==-1) return 1;
	else return 0;
}

void
AP_RC_Channel::set_filter(bool filter)
{
	_filter = filter;
}

// call after first read
void
AP_RC_Channel::trim()
{
	radio_trim = radio_in;
}

// read input from APM_RC - create a control_in value
void
AP_RC_Channel::set_pwm(int pwm)
{
	//Serial.print(pwm,DEC);

	if(_filter){
		if(radio_in == 0)
			radio_in = pwm;
		else
			radio_in = ((pwm + radio_in) >> 1);		// Small filtering
	}else{
		radio_in = pwm;
	}

	if(_type == RANGE){
		//Serial.print("range ");
		control_in = pwm_to_range();
		control_in = (control_in < dead_zone) ? 0 : control_in;

	}else{
		control_in = pwm_to_angle();
		control_in = (abs(control_in) < dead_zone) ? 0 : control_in;

		/*
		// coming soon ??
		if(expo) {
			long temp = control_in;
			temp = (temp * temp) / (long)_high;
			control_in = (int)((control_in >= 0) ? temp : -temp);
		}
		*/
	}
}

int
AP_RC_Channel::control_mix(float value)
{
	return (1 - abs(control_in / _high)) * value + control_in;
}

// are we below a threshold?
bool
AP_RC_Channel::get_failsafe(void)
{
	return (radio_in < (radio_min - 50));
}

// returns just the PWM without the offset from radio_min
void
AP_RC_Channel::calc_pwm(void)
{

	if(_type == RANGE){
		pwm_out 	= range_to_pwm();
		radio_out 	= pwm_out + radio_min;
	}else{
		pwm_out 	= angle_to_pwm();
		radio_out 	= pwm_out + radio_trim;
	}
	radio_out = constrain(radio_out, radio_min, radio_max);
}

// ------------------------------------------

void
AP_RC_Channel::load_eeprom(void)
{
	radio_min 	= eeprom_read_word((uint16_t *)	_address);
	radio_max	= eeprom_read_word((uint16_t *)	(_address + 2));
	load_trim();
}

void
AP_RC_Channel::save_eeprom(void)
{
	eeprom_write_word((uint16_t *)	_address, 			radio_min);
	eeprom_write_word((uint16_t *)	(_address + 2), 	radio_max);
	save_trim();
}

// ------------------------------------------
void
AP_RC_Channel::save_trim(void)
{
	eeprom_write_word((uint16_t *)	(_address + 4), 	radio_trim);
	//_ee.write_int((_address + 4), 	radio_trim);
}

void
AP_RC_Channel::load_trim(void)
{
	radio_trim 	= eeprom_read_word((uint16_t *)	(_address + 4));
	//_ee.write_int((_address + 4), 	radio_trim);
}

// ------------------------------------------

void
AP_RC_Channel::zero_min_max()
{
	radio_min = radio_max = radio_in;
}

void
AP_RC_Channel::update_min_max()
{
	radio_min = min(radio_min, radio_in);
	radio_max = max(radio_max, radio_in);
}

// ------------------------------------------

int16_t
AP_RC_Channel::pwm_to_angle()
{
	if(radio_in < radio_trim)
		return _reverse * ((long)_high * (long)(radio_in - radio_trim)) / (long)(radio_trim - radio_min);
	else
		return _reverse * ((long)_high * (long)(radio_in - radio_trim)) / (long)(radio_max  - radio_trim);

		//return _reverse * _high * ((float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim));
		//return _reverse * _high * ((float)(radio_in - radio_trim) / (float)(radio_trim - radio_min));
}


int16_t
AP_RC_Channel::angle_to_pwm()
{
	if(_reverse == -1)
	{
		if(servo_out < 0)
			return ( -1 * ((long)servo_out * (long)(radio_max - radio_trim)) / (long)_high);
		else
			return ( -1 * ((long)servo_out * (long)(radio_trim - radio_min)) / (long)_high);
	} else {
		if(servo_out > 0)
			return ((long)servo_out * (long)(radio_max - radio_trim)) / (long)_high;
		else
			return ((long)servo_out * (long)(radio_trim - radio_min)) / (long)_high;
	}

		//return (((float)servo_out / (float)_high) * (float)(radio_max - radio_trim));
		//return (((float)servo_out / (float)_high) * (float)(radio_trim - radio_min));
}

// ------------------------------------------

int16_t
AP_RC_Channel::pwm_to_range()
{
	//return (_low + ((_high - _low) * ((float)(radio_in - radio_min) / (float)(radio_max - radio_min))));
	return (_low + ((long)(_high - _low) * (long)(radio_in - radio_min)) / (long)(radio_max - radio_min));
}

int16_t
AP_RC_Channel::range_to_pwm()
{
	//return (((float)(servo_out - _low) / (float)(_high - _low)) * (float)(radio_max - radio_min));
	return ((long)(servo_out - _low) * (long)(radio_max - radio_min)) / (long)(_high - _low);
}

// ------------------------------------------

float
AP_RC_Channel::norm_input()
{
	if(radio_in < radio_trim)
		return _reverse * (float)(radio_in - radio_trim) / (float)(radio_trim - radio_min);
	else
		return _reverse * (float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim);
}

float
AP_RC_Channel::norm_output()
{
	if(radio_out < radio_trim)
		return (float)(radio_out - radio_trim) / (float)(radio_trim - radio_min);
	else
		return (float)(radio_out - radio_trim) / (float)(radio_max  - radio_trim);
}
