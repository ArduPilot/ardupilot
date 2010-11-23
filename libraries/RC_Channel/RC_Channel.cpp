/*
	RC_Channel.cpp - Radio library for Arduino
	Code by Jason Short. DIYDrones.com
	
	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

*/

#include "RC_Channel.h"

#define ANGLE 0
#define RANGE 1

RC_Channel::RC_Channel() : _high(1)
{
}

void 
RC_Channel::set_radio_range(int r_min, int r_max)
{
	_radio_min 	= r_min;
	_radio_max 	= r_max;
}

// setup the control preferences
void 	
RC_Channel::set_range(int high, int low)
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

// call after first read
void
RC_Channel::set_trim(int pwm)
{
	_radio_trim = pwm;
}

// read input from APM_RC - create a control_in value
void
RC_Channel::set_pwm(int pwm)
{	
	if(_radio_in == 0)
		_radio_in = pwm;
	else
		_radio_in = ((pwm + _radio_in) >> 1);		// Small filtering

	if(_type == RANGE){
		control_in = pwm_to_range();
	}else{
		control_in = pwm_to_angle();
	}
}

// are we below a threshold?
boolean
RC_Channel::get_failsafe(void)
{
	return (_radio_in < (_radio_min - 50));
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
	radio_out = pwm_out + _radio_min;
}

void
RC_Channel::load_eeprom(int address)
{
	//Serial.println("load gains ");
	//Serial.println(address, DEC);
	//_kp 	= (float)(eeprom_read_word((uint16_t *)	address)) / 1000.0;
	//_ki 	= (float)(eeprom_read_word((uint16_t *)	(address + 2))) / 1000.0;
	//_kd 	= (float)(eeprom_read_word((uint16_t *)	(address + 4))) / 1000.0;
	//_imax 	= eeprom_read_word((uint16_t *)	(address + 6)) * 100;
}

void
RC_Channel::save_eeprom(int address)
{
	//eeprom_write_word((uint16_t *)	address, 		(int)(_kp * 1000));
	//eeprom_write_word((uint16_t *)	(address + 2), 	(int)(_ki * 1000));
	//eeprom_write_word((uint16_t *)	(address + 4), 	(int)(_kd * 1000));
	//eeprom_write_word((uint16_t *)	(address + 6), 	(int)_imax/100);
}

// ------------------------------------------

int16_t 
RC_Channel::pwm_to_angle()
{
	if(_radio_in < _radio_trim)
		return _high * ((float)(_radio_in - _radio_trim) / (float)(_radio_trim - _radio_min));
	else
		return _high * ((float)(_radio_in - _radio_trim) / (float)(_radio_max  - _radio_trim));
}

int16_t
RC_Channel::angle_to_pwm()
{
	if(servo_out < 0)
		return (((float)servo_out / (float)_high) * (float)(_radio_max - _radio_trim));
	else
		return (((float)servo_out / (float)_high) * (float)(_radio_trim - _radio_min));
}

// ------------------------------------------

int16_t
RC_Channel::pwm_to_range()
{
	return _low + _high * ((float)(_radio_in - _radio_min) / (float)(_radio_max - _radio_min));
}

int16_t
RC_Channel::range_to_pwm()
{
	return (((float)servo_out / (float)(_high - _low)) * (float)(_radio_max - _radio_min));
}

