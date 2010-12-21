// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-
/*
	AP_RangeFinder_MaxsonarXL.cpp - Arduino Library for Sharpe GP2Y0A02YK0F 
	infrared proximity sensor
	Code by Jose Julio and Randy Mackay. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Sparkfun URL: http://www.sparkfun.com/products/9495
	datasheet: http://www.sparkfun.com/datasheets/Sensors/Proximity/XL-EZ4-Datasheet.pdf
	
	Sensor should be connected to one of the analog ports
	
	Variables:
		int raw_value : raw value from the sensor
		int distance : distance in meters
		int max_distance : maximum measurable distance (in cm)
		int min_distance : minimum measurable distance (in cm)
	
	Methods:
		init(int analogPort) : Initialization of sensor
		read() : read value from analog port and returns the distance (in cm)
		
*/

// AVR LibC Includes
#include "WConstants.h"
#include "AP_RangeFinder_MaxsonarXL.h"

// Public Methods //////////////////////////////////////////////////////////////
void AP_RangeFinder_MaxsonarXL::init(int analogPort)
{
    // local variables
    int i;
	
	// set the given analog port to an input
	pinMode(analogPort, INPUT);
	
    // initialise everything
    _analogPort = analogPort;
	max_distance = AP_RANGEFINDER_MAXSONARXL_MAX_DISTANCE;
	min_distance = AP_RANGEFINDER_MAXSONARXL_MIN_DISTANCE;
	
	// make first call to read to get initial distance
	read();
	
	// initialise history
	for( i=0; i<AP_RANGEFINDER_NUM_AVERAGES; i++ )
	    _history[i] = distance;	
}

// Read Sensor data
int AP_RangeFinder_MaxsonarXL::read()
{
    // read raw sensor value and convert to distance
    raw_value = analogRead(_analogPort);
	
	// for this sensor, the sensor value is the distance in cm! nice and easy!
	distance = constrain(raw_value,min_distance,max_distance);
	
	// return distance
	return filter(distance);
}
