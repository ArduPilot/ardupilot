/*
	AP_RcChannel.cpp - Radio library for Arduino
	Code by Jason Short, James Goppert. DIYDrones.com
	
	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

*/

#include <math.h>
#include <avr/eeprom.h>
#include "AP_RcChannel.h"
#include <AP_Common.h>

AP_RcChannel::AP_RcChannel(const char * name, const APM_RC_Class & rc, const uint8_t & ch,
			const float & scale, const float & center, 
			const uint16_t & pwmMin, 
			const uint16_t & pwmNeutral, const uint16_t & pwmMax,
			const uint16_t & pwmDeadZone,
			const bool & filter, const bool & reverse) :
		_name(name),
		_rc(rc),
		_ch(new AP_EEPROM_Uint8(ch,"CH",name)),
		_scale(new AP_EEPROM_Float(scale,"SCALE",name)),
		_center(new AP_EEPROM_Float(center,"CNTR",name)),
		_pwmMin(new AP_EEPROM_Uint16(pwmMin,"PMIN",name)), 
		_pwmMax(new AP_EEPROM_Uint16(pwmMax,"PMAX",name)),
		_pwmNeutral(new AP_EEPROM_Uint16(pwmNeutral,"PNTRL",name)),
		_pwmDeadZone(new AP_EEPROM_Uint16(pwmDeadZone,"PDEAD",name)),
		_pwm(0),
		_filter(new AP_EEPROM_Bool(filter,"FLTR",name)),
		_reverse(new AP_EEPROM_Bool(reverse,"REV",name))
	{
	}


void AP_RcChannel::readRadio() {
	// apply reverse
	uint16_t pwmRadio = APM_RC.InputCh(getCh());
	setPwm(pwmRadio);
}

void
AP_RcChannel::setPwm(uint16_t pwm)
{
	//Serial.printf("pwm in setPwm: %d\n", pwm);
	//Serial.printf("reverse: %s\n", (getReverse())?"true":"false");
	
	// apply reverse
	if(getReverse()) pwm = int16_t(getPwmNeutral()-pwm) + getPwmNeutral();

	//Serial.printf("pwm after reverse: %d\n", pwm);

	// apply filter
	if(getFilter()){
		if(_pwm == 0)
			_pwm = pwm;
		else
			_pwm = ((pwm + _pwm) >> 1);		// Small filtering
	}else{
		_pwm = pwm;
	}

	//Serial.printf("pwm after filter: %d\n", _pwm);

	// apply deadzone
	_pwm = (abs(_pwm - getPwmNeutral()) < getPwmDeadZone()) ? getPwmNeutral() : _pwm;

	//Serial.printf("pwm after deadzone: %d\n", _pwm);
	APM_RC.OutputCh(getCh(),_pwm);
}

void
AP_RcChannel::setPosition(float position)
{
	setPwm(_positionToPwm(position));
}

void
AP_RcChannel::mixRadio(uint16_t infStart)
{
	uint16_t pwmRadio = APM_RC.InputCh(getCh());
	float inf = abs( int16_t(pwmRadio - getPwmNeutral()) );
	inf = min(inf, infStart);
	inf = ((infStart - inf) /infStart);
	setPwm(_pwm*inf + pwmRadio); 
}

uint16_t
AP_RcChannel::_positionToPwm(const float & position)
{
	uint16_t pwm;
	//Serial.printf("position: %f\n", position);
	float p = position - getCenter();	
	if(p < 0)
		pwm = p * int16_t(getPwmNeutral() - getPwmMin()) / 
			getScale() + getPwmNeutral();
	else
		pwm = p * int16_t(getPwmMax() - getPwmNeutral()) / 
			getScale() + getPwmNeutral();
	constrain(pwm,getPwmMin(),getPwmMax());
	return pwm;
}

float
AP_RcChannel::_pwmToPosition(const uint16_t & pwm)
{
	float position;
	if(pwm < getPwmNeutral())
		position = getScale() * int16_t(pwm - getPwmNeutral())/ 
			int16_t(getPwmNeutral() - getPwmMin()) + getCenter();
	else
		position = getScale() * int16_t(pwm -getPwmNeutral())/ 
			int16_t(getPwmMax() - getPwmNeutral()) + getCenter();
	constrain(position,-getScale()+getCenter(),
			getScale()+getCenter());
	return position;
}

// ------------------------------------------
