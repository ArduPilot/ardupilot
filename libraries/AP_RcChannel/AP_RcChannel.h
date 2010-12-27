// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_RcChannel.h
/// @brief	AP_RcChannel manager

#ifndef AP_RcChannel_h
#define AP_RcChannel_h

#include <stdint.h>
#include <FastSerial.h>
#include <APM_RC.h>

/// @class	AP_RcChannel
/// @brief	Object managing one RC channel
class AP_RcChannel{
 
public:	

	/// Constructor
	AP_RcChannel(const APM_RC_Class & rc, const uint16_t & ch,
			const float & scale, const uint16_t & pwmMin, const uint16_t & pwmNeutral,
			const uint16_t & pwmMax, const uint16_t & pwmDeadZone,
			const bool & filter, const bool & reverse) :
		_rc(rc),
		_ch(ch),
		_scale(scale),
		_pwmMin(pwmMin),
		_pwmMax(pwmMax),
		_pwmNeutral(pwmNeutral),
		_pwmDeadZone(pwmDeadZone),
		_pwm(0),
		_filter(filter),
		_reverse(reverse)
	{
	}

	// set servo state
	void readRadio();
	void setPwm(uint16_t pwm);
	void setPosition(float position);
	void mixRadio(uint16_t infStart);

	// get servo state
	uint16_t getPwm() { return _pwm; }
	float getPosition() { return _pwmToPosition(_pwm); }
	float getNormalized() { return getPosition()/_scale; }
	
	// did our read come in 50µs below the min?
	bool failSafe() { _pwm < (_pwmMin - 50); }

private:

	// configuration
	const APM_RC_Class & _rc;
	const uint16_t _ch;
	const float & _scale;
	const uint16_t & _pwmMin;
	const uint16_t & _pwmNeutral;
	const uint16_t & _pwmMax;
	const uint16_t & _pwmDeadZone;
	const bool & _filter;
	const bool & _reverse;

	// internal states
	uint16_t _pwm; // this is the internal state, position is just created when needed

	// private methods
	uint16_t _positionToPwm(const float & position);
	float _pwmToPosition(const uint16_t & pwm);
};

#endif	




