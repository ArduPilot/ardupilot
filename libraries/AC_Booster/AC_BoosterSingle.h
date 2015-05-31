// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AC_BOOSTER_SINGLE_H__
#define __AC_BOOSTER_SINGLE_H__

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <RC_Channel.h>

#include "AC_BoosterBackend.h"

extern const AP_HAL::HAL& hal;

/// @class AC_BoosterSingle
class AC_BoosterSingle : public AC_BoosterBackend {
public:
	
	// default constructor
	AC_BoosterSingle ();

	// set_boost - sets the requested boost, between 0 and 1000
	void set_boost (int16_t boost_in);

	// is_enabled - let's the booster controller know if the aux booster channel is enabled
	bool is_enabled ();

};

#endif // __AC_BOOSTER_SINGLE_H__