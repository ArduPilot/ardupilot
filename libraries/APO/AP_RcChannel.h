// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_RcChannel.h
/// @brief	AP_RcChannel manager

#ifndef AP_RCCHANNEL_H
#define AP_RCCHANNEL_H

#include <stdint.h>
#include "../APM_RC/APM_RC.h"
#include "../AP_Common/AP_Common.h"
#include "../AP_Common/AP_Var.h"

namespace apo {

enum rcMode_t {
	RC_MODE_IN, RC_MODE_OUT, RC_MODE_INOUT
};

/// @class	AP_RcChannel
/// @brief	Object managing one RC channel
class AP_RcChannel: public AP_Var_group {

public:

	/// Constructor
	AP_RcChannel(AP_Var::Key keyValue, const prog_char_t * name, APM_RC_Class & rc,
			const uint8_t & ch, const uint16_t & pwmMin,
			const uint16_t & pwmNeutral, const uint16_t & pwmMax,
			const rcMode_t & rcMode,
			const bool & reverse);

	// configuration
	AP_Uint8 _ch;
	AP_Uint16 _pwmMin;
	AP_Uint16 _pwmNeutral;
	AP_Uint16 _pwmMax;
	AP_Uint8 _deg2mPwm;
	rcMode_t _rcMode;
	AP_Bool _reverse;

	// get
	uint16_t getPwm() {
		return _pwm;
	}
	uint16_t getRadioPwm();
	float getPosition() {
		return _pwmToPosition(getPwm());
	}
	float getRadioPosition() {
		return _pwmToPosition(getRadioPwm());
	}

	// set
	void setUsingRadio() {
		setPwm(getRadioPwm());
	}
	void setPwm(uint16_t pwm);
	void setPosition(float position) {
		setPwm(_positionToPwm(position));
	}

protected:

	// configuration
	APM_RC_Class & _rc;

	// internal states
	uint16_t _pwm; // this is the internal state, position is just created when needed

	// private methods
	uint16_t _positionToPwm(const float & position);
	float _pwmToPosition(const uint16_t & pwm);
};

class AP_RcChannel_Scaled : public AP_RcChannel {
public:
	AP_RcChannel_Scaled(AP_Var::Key keyValue, const prog_char_t * name, APM_RC_Class & rc,
			const uint8_t & ch, const uint16_t & pwmMin,
			const uint16_t & pwmNeutral, const uint16_t & pwmMax,
			const rcMode_t & rcMode,
			const bool & reverse,
			const float & scale) :
		AP_RcChannel(keyValue,name,rc,ch,pwmMin,pwmNeutral,pwmMax,rcMode,reverse), 
		_scale(this, 6, pwmNeutral, PSTR("scale"))
	{
	}
	AP_Float _scale;
	void setScaled(float val) {
		setPwm(val/_scale);
	}
	float getScaled() {
		return _scale*getPwm();
	}
};

} // apo

#endif	// AP_RCCHANNEL_H
