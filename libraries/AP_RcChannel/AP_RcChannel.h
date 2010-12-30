// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_RcChannel.h
/// @brief	AP_RcChannel manager

#ifndef AP_RcChannel_h
#define AP_RcChannel_h

#include <stdint.h>
#include <FastSerial.h>
#include <APM_RC.h>
#include <AP_Var.h>
#include <AP_Common.h>
#include <AP_EEProm.h>

/// @class	AP_RcChannel
/// @brief	Object managing one RC channel
class AP_RcChannel{
 
public:	

	/// Constructor
	AP_RcChannel(const char * name, const APM_RC_Class & rc, const uint8_t & ch,
			const float & scale=45.0, const float & center=0.0, 
			const uint16_t & pwmMin=1200, 
			const uint16_t & pwmNeutral=1500, const uint16_t & pwmMax=1800,
			const uint16_t & pwmDeadZone=10,
			const bool & filter=false, const bool & reverse=false);

	// set
	void readRadio();
	void setPwm(uint16_t pwm);
	void setPosition(float position);
	void setNormalized(float normPosition) { setPosition(normPosition*getScale()); }
	void mixRadio(uint16_t infStart);
	void setCh(const uint8_t & ch) { _ch->set(ch); }
	void setScale(const float & scale) { _scale->set(scale); }
	void setCenter(const float & center) { _center->set(center); }
	void setPwmMin(const uint16_t & pwmMin) { _pwmMin->set(pwmMin); }
	void setPwmNeutral(const uint16_t & pwmNeutral) { _pwmNeutral->set(pwmNeutral); }
	void setPwmMax(const uint16_t & pwmMax) { _pwmMax->set(pwmMax); }
	void setPwmDeadZone(const uint16_t & pwmDeadZone) { _pwmDeadZone->set(pwmDeadZone); }
	void setFilter(const bool & filter) { _filter->set(filter); }

	// get
	uint16_t getPwm() { return _pwm; }
	float getPosition() { return _pwmToPosition(_pwm); }
	float getNormalized() { return getPosition()/_scale->get(); }
	const char * getName() { return _name; }
	const uint8_t & getCh() { return _ch->get(); }
	const float & getScale() { return _scale->get(); }
	const float & getCenter() { return _center->get(); }
	const uint16_t & getPwmMin() { return _pwmMin->get(); }
	const uint16_t & getPwmNeutral() { return _pwmNeutral->get(); }
	const uint16_t & getPwmMax() { return _pwmMax->get(); }
	const uint16_t & getPwmDeadZone() { return _pwmDeadZone->get(); }
	const bool & getFilter() { return _filter->get(); }
	const bool & getReverse() { return _reverse->get(); }

	// did our read come in 50µs below the min?
	bool failSafe() { _pwm < (_pwmMin->get() - 50); }

private:

	// configuration
	const char * _name;
	const APM_RC_Class & _rc;
	AP_Uint8 * _ch;
	AP_Float * _scale;
	AP_Float * _center;
	AP_Uint16 * _pwmMin;
	AP_Uint16 * _pwmNeutral;
	AP_Uint16 * _pwmMax;
	AP_Uint16 * _pwmDeadZone;
	AP_Bool * _filter;
	AP_Bool * _reverse;

	// internal states
	uint16_t _pwm; // this is the internal state, position is just created when needed

	// private methods
	uint16_t _positionToPwm(const float & position);
	float _pwmToPosition(const uint16_t & pwm);
};

#endif	




