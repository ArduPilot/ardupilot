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
#include "WProgram.h"
#include "RC_Channel.h"

#define ANGLE 0
#define RANGE 1

// setup the control preferences
void 	
RC_Channel::set_range(int low, int high)
{
	_type 	= RANGE;
	_high 	= high;
	_low 	= low;
}

void
RC_Channel::set_angle(int angle)
{
	_type 	= ANGLE;
	_high 	= angle;
}

void
RC_Channel::set_filter(bool filter)
{
	_filter = filter;
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

	if(_type == RANGE){
		pwm_out = range_to_pwm();
	}else{
		pwm_out = angle_to_pwm();
	}
	radio_out = pwm_out + radio_min;
}

// ------------------------------------------

void
RC_Channel::load_eeprom(void)
{
	radio_min 	= eeprom_read_word((uint16_t *)	_address);
	radio_max	= eeprom_read_word((uint16_t *)	(_address + 2));
	radio_trim 	= eeprom_read_word((uint16_t *)	(_address + 4));
}

void
RC_Channel::save_eeprom(void)
{
	eeprom_write_word((uint16_t *)	_address, 			radio_min);
	eeprom_write_word((uint16_t *)	(_address + 2), 	radio_max);
	eeprom_write_word((uint16_t *)	(_address + 4), 	radio_trim);
}

// ------------------------------------------
void
RC_Channel::save_trim(void)
{
	eeprom_write_word((uint16_t *)	(_address + 4), 	radio_trim);
}

// ------------------------------------------

void
RC_Channel::update_min_max()
{
	radio_min = min(radio_min, radio_in);
	radio_max = max(radio_max, radio_in);
}

// ------------------------------------------

int16_t 
RC_Channel::pwm_to_angle()
{
	if(radio_in < radio_trim)
		return _high * ((float)(radio_in - radio_trim) / (float)(radio_trim - radio_min));
	else
		return _high * ((float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim));
}

int16_t
RC_Channel::angle_to_pwm()
{
	if(servo_out < 0)
		return (((float)servo_out / (float)_high) * (float)(radio_max - radio_trim));
	else
		return (((float)servo_out / (float)_high) * (float)(radio_trim - radio_min));
}

// ------------------------------------------

int16_t
RC_Channel::pwm_to_range()
{
	return _low + _high * ((float)(radio_in - radio_min) / (float)(radio_max - radio_min));
}

int16_t
RC_Channel::range_to_pwm()
{
	return (((float)servo_out / (float)(_high - _low)) * (float)(radio_max - radio_min));
}

