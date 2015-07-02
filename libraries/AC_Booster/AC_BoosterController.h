// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AC_BOOSTER_CONTROLLER_H__
#define __AC_BOOSTER_CONTROLLER_H__

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <RC_Channel.h>

#include "AC_BoosterBackend.h"

#define BOOSTER_CONTROLLER_FORWARD_PITCH		1000 	// 10 degrees default forward pitch

extern const AP_HAL::HAL& hal;

/// @class AC_BoosterController
class AC_BoosterController {
public:
	
	// default constructor
	AC_BoosterController (AC_BoosterBackend* booster);
	
	// set_boost - sets the requested boost, between 0 and 100
	void set_boost (int16_t boost_in);

	// set_active - sets the state of the booster
	void set_active (bool active);

	// set_boost_and_scale_pitch
	float set_boost_and_scale_pitch (float pitch_in, float pitch_max);

	// calculate_boost_and_scale_pitch
	void calculate_boost_and_scale_pitch(float &pitch, float pitch_max, int16_t &boost);

	// is_active - returns the state of the booster
	bool is_active ();

	// var_info - parameters
	static const struct AP_Param::GroupInfo var_info[];

private:

	// parameters

	AP_Float			_forward_pitch;

	// internal variables

	AC_BoosterBackend*	_booster;
	bool 				_active;
};

#endif // __AC_BOOSTER_CONTROLLER_H__