// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AC_BOOSTER_BACKEND_H__
#define __AC_BOOSTER_BACKEND_H__

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <RC_Channel.h>

/// @class AC_BoosterBackend
class AC_BoosterBackend {
public:
	AC_BoosterBackend() {};

	// set_boost - sets the requested boost, between 0 and 1000
	virtual void set_boost (int16_t boost_in) = 0;

	// is_enabled - allows the booster implementation to override if the controller can activate the booster
	virtual bool is_enabled () { return true; }
};

#endif // AC_BOOSTER_BACKEND_H