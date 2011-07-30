// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-
/*
	AP_RangeFinder_MaxsonarXL.cpp - Arduino Library for Sharpe GP2Y0A02YK0F
	infrared proximity sensor
	Code by Jose Julio and Randy Mackay. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Sparkfun URL: http://www.sparkfun.com/products/9491
	datasheet: http://www.sparkfun.com/datasheets/Sensors/Proximity/XL-EZ0-Datasheet.pdf

	Sensor should be connected to one of the analog ports

	Variables:
		int raw_value : raw value from the sensor
		int distance : distance in cm
		int max_distance : maximum measurable distance (in cm)
		int min_distance : minimum measurable distance (in cm)

	Methods:
		init(int analogPort) : Initialization of sensor
		read() : read value from analog port and returns the distance (in cm)

*/

// AVR LibC Includes
#include "WConstants.h"
#include "AP_RangeFinder_MaxsonarXL.h"

// Constructor //////////////////////////////////////////////////////////////
//AP_GPS_MTK16::AP_GPS_MTK16(Stream *s) : GPS(s)
//{
//}

AP_RangeFinder_MaxsonarXL::AP_RangeFinder_MaxsonarXL(AP_ADC *adc, ModeFilter *filter) :
	RangeFinder(adc, filter)
{
    max_distance = AP_RANGEFINDER_MAXSONARXL_MAX_DISTANCE;
	min_distance = AP_RANGEFINDER_MAXSONARXL_MIN_DISTANCE;
}

// Public Methods //////////////////////////////////////////////////////////////
