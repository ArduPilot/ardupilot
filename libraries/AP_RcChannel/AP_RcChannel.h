// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_RcChannel.h
/// @brief	AP_RcChannel manager

#ifndef AP_RcChannel_h
#define AP_RcChannel_h

#include <stdint.h>
#include <APM_RC.h>
#include <AP_Var.h>
#include <AP_Common.h>

/// @class	AP_RcChannel
/// @brief	Object managing one RC channel
class AP_RcChannel : public AP_Var_scope {
 
public:	

	/// Constructor
	AP_RcChannel(const prog_char * name, APM_RC_Class & rc, const uint8_t & ch,
			const float & scale=45.0, const float & center=0.0, 
			const uint16_t & pwmMin=1200, 
			const uint16_t & pwmNeutral=1500, const uint16_t & pwmMax=1800,
			const uint16_t & pwmDeadZone=10,
			const bool & filter=false, const bool & reverse=false);

	// configuration
	AP_Uint8 ch;
	AP_Float scale;
	AP_Float center;
	AP_Uint16 pwmMin;
	AP_Uint16 pwmNeutral;
	AP_Uint16 pwmMax;
	AP_Uint16 pwmDeadZone;
	AP_Bool filter;
	AP_Bool reverse;

	// set
	void readRadio();
	void setPwm(uint16_t pwm);
	void setPosition(float position);
	void setNormalized(float normPosition) { setPosition(normPosition*scale); }
	void mixRadio(uint16_t infStart);

	// get
	uint16_t getPwm() { return _pwm; }
	float getPosition() { return _pwmToPosition(_pwm); }
	float getNormalized() { return getPosition()/scale; }
	const char * getName() { return _name; }

	// did our read come in 50µs below the min?
	bool failSafe() { _pwm < (pwmMin - 50); }

private:

	// configuration
	const char * _name;
	APM_RC_Class & _rc;
		
	// internal states
	uint16_t _pwm; // this is the internal state, position is just created when needed

	// private methods
	uint16_t _positionToPwm(const float & position);
	float _pwmToPosition(const uint16_t & pwm);
};

#endif	
