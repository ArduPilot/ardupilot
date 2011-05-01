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
#include "../AP_Common/AP_Common.h"
#include <HardwareSerial.h>


namespace apo {

AP_RcChannel::AP_RcChannel(AP_Var::Key key, const prog_char_t * name, APM_RC_Class & rc, const uint8_t & ch,
			const uint16_t & pwmMin,
			const uint16_t & pwmNeutral, const uint16_t & pwmMax,
			const rcMode_t & rcMode, const bool & reverse) :
		AP_Var_group(key,name),
		_rc(rc),
		_ch(this,0,ch,PSTR("CH")),
		_pwmMin(this,1,pwmMin,PSTR("PMIN")),
		_pwmMax(this,2,pwmMax,PSTR("PMAX")),
		_pwmNeutral(this,3,pwmNeutral,PSTR("PNTRL")),
		_rcMode(rcMode),
		_reverse(this,4,reverse,PSTR("REV")),
		_pwm(pwmNeutral)
	{
		//Serial.print("pwm after ctor: "); Serial.println(pwmNeutral);
		if (rcMode == RC_MODE_IN) return;
		//Serial.print("pwm set for ch: "); Serial.println(int(ch));
		rc.OutputCh(ch,pwmNeutral);

	}


uint16_t AP_RcChannel::readRadio() {
	if (_rcMode == RC_MODE_OUT) {
		Serial.print("tryng to read from output channel: "); Serial.println(int(_ch));
		return _pwmNeutral; // if this happens give a safe value of neutral
	}
	return _rc.InputCh(_ch);
}

void
AP_RcChannel::setPwm(uint16_t pwm)
{
	//Serial.printf("pwm in setPwm: %d\n", pwm);
	//Serial.printf("reverse: %s\n", (reverse)?"true":"false");
	
	// apply reverse
	if(_reverse) pwm = int16_t(_pwmNeutral-pwm) + _pwmNeutral;
	//Serial.printf("pwm after reverse: %d\n", pwm);

	// apply saturation
	if (_pwm > _pwmMax) _pwm = _pwmMax;
	if (_pwm < _pwmMin) _pwm = _pwmMin;
	_pwm = pwm;

	//Serial.print("ch: "); Serial.print(ch); Serial.print(" pwm: "); Serial.println(pwm);
	if (_rcMode == RC_MODE_IN) return;
	_rc.OutputCh(_ch,_pwm);
}

void
AP_RcChannel::setPosition(float position)
{
	if (position > 1.0) position = 1.0;
	else if (position < -1.0) position = -1.0;
	setPwm(_positionToPwm(position));
}

uint16_t
AP_RcChannel::_positionToPwm(const float & position)
{
	uint16_t pwm;
	//Serial.printf("position: %f\n", position);
	if(position < 0)
		pwm = position * int16_t(_pwmNeutral - _pwmMin) + _pwmNeutral;
	else
		pwm = position * int16_t(_pwmMax - _pwmNeutral) + _pwmNeutral;

	if (pwm > _pwmMax) pwm = _pwmMax;
	if (pwm < _pwmMin) pwm = _pwmMin;
	return pwm;
}

float
AP_RcChannel::_pwmToPosition(const uint16_t & pwm)
{
	float position;
	if(pwm < _pwmNeutral)
		position = 1.0 * int16_t(pwm - _pwmNeutral)/
			int16_t(_pwmNeutral - _pwmMin);
	else
		position = 1.0 * int16_t(pwm - _pwmNeutral)/
			int16_t(_pwmMax - _pwmNeutral) ;
	if (position > 1) position = 1;
	if (position < -1) position = -1;
	return position;
}

} // namespace apo
