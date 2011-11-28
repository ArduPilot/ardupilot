// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-
/*
	AP_RangeFinder_SharpGP2Y.cpp - Arduino Library for Sharpe GP2Y0A02YK0F
	infrared proximity sensor
	Code by Jose Julio and Randy Mackay. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Sensor should be conected to one of the analog ports

	Sparkfun URL: http://www.sparkfun.com/products/8958
	datasheet: http://www.sparkfun.com/datasheets/Sensors/Infrared/gp2y0a02yk_e.pdf

	Variables:
		int raw_value : raw value from the sensor
		int distance : distance in cm
		int max_distance : maximum measurable distance (in cm)
		int min_distance : minimum measurable distance (in cm)

	Methods:
		read() : read value from analog port

*/

// AVR LibC Includes
#include "WConstants.h"
#include "AP_RangeFinder_SharpGP2Y.h"

// Constructor //////////////////////////////////////////////////////////////

AP_RangeFinder_SharpGP2Y::AP_RangeFinder_SharpGP2Y(AP_AnalogSource *source,
                                                   ModeFilter *filter) :
	RangeFinder(source, filter)
{
    max_distance = AP_RANGEFINDER_SHARPEGP2Y_MAX_DISTANCE;
	min_distance = AP_RANGEFINDER_SHARPEGP2Y_MIN_DISTANCE;
}

// Public Methods //////////////////////////////////////////////////////////////
