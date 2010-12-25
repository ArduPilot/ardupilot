// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	RC_ChannelB.h
/// @brief	RC_ChannelB manager, with EEPROM-backed storage of constants.

#ifndef RC_ChannelB_h
#define RC_ChannelB_h

#include <stdint.h>
#include <FastSerial.h>

/// @class	RC_ChannelB
/// @brief	Object managing one RC channel
//
class RC_ChannelB{
 
public:	

	/// Constructor
	///
	RC_ChannelB(const float & scale, const uint16_t & pwmMin, const uint16_t & pwmNeutral,
			const uint16_t & pwmMax, const uint16_t & pwmDeadZone,
			const bool & filter, const bool & reverse) :
		_scale(scale),
		_pwmMin(pwmMin),
		_pwmMax(pwmMax),
		_pwmNeutral(pwmNeutral),
		_pwmDeadZone(pwmDeadZone),
		_pwm(0),
		_pwmRadio(0),
		_filter(filter),
		_reverse(reverse)
	{
	}

	// set servo state
	void readRadio(uint16_t pwmRadio);
	void setPwm(uint16_t pwm);
	void setPosition(float position);
	void mixRadio(uint16_t infStart);

	// get servo state
	uint16_t getPwm() { return _pwm; }
	uint16_t getPwmRadio() { return _pwmRadio; }
	float getPosition() { return _pwmToPosition(_pwm); }
	float getNormalized() { return getPosition()/_scale; }
	
	// did our read come in 50µs below the min?
	bool failSafe() { _pwm < (_pwmMin - 50); }

private:

	// configuration
	const float & _scale;
	const uint16_t & _pwmMin;
	const uint16_t & _pwmNeutral;
	const uint16_t & _pwmMax;
	const uint16_t & _pwmDeadZone;
	const bool & _filter;
	const int8_t & _reverse;

	// internal states
	uint16_t _pwm; // this is the internal state, positino is just created when needed
	uint16_t _pwmRadio; // radio pwm input

	// private methods
	uint16_t _positionToPwm(float position);
	float _pwmToPosition(uint16_t pwm);
};

#endif	




