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
		int distance : distance in meters
		int max_distance : maximum measurable distance (in cm)
		int min_distance : minimum measurable distance (in cm)
	
	Methods:
		init(int analogPort) : Initialization of sensor
		read() : read value from analog port	
		
*/

// AVR LibC Includes
#include "WConstants.h"

#include "AP_RangeFinder_SharpGP2Y.h"

// Public Methods //////////////////////////////////////////////////////////////
void AP_RangeFinder_SharpGP2Y::init(int analogPort)
{
    // initialise everything
    _analogPort = analogPort;
	max_distance = AP_RANGEFINDER_SHARPEGP2Y_MAX_DISTANCE;
	min_distance = AP_RANGEFINDER_SHARPEGP2Y_MIN_DISTANCE;
	
	// make first call to read to get initial distance
	read();
}

// Read Sensor data
int AP_RangeFinder_SharpGP2Y::read()
{
    // read raw sensor value and convert to distance
    raw_value = analogRead(_analogPort);
	distance = constrain(14500/raw_value,min_distance,max_distance); 
	
	// implement filter
    //switch( _filterType ) {
	//    case AP_RANGEFINDER_FILTER_LIMITED_CHANGE:
	//	    distance = constrain(
	//	    break;
	//	case AP_RANGEFINDER_FILTER_NONE:
	//    default:
	//	    distance = tempDistance;
	//	    break;
	//}
	
	// return distance
	return distance;
}
