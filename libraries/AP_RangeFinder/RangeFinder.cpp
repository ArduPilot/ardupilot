// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-
/*
	AP_RangeFinder.cpp - Arduino Library for Sharpe GP2Y0A02YK0F
	infrared proximity sensor
	Code by Jose Julio and Randy Mackay. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	This has the basic functions that all RangeFinders need implemented
*/

// AVR LibC Includes
#include "WConstants.h"
#include "RangeFinder.h"


// Constructor /////////////////////////////////////////////////////////////////
RangeFinder::RangeFinder() :
	_ap_adc(NULL),
	_num_averages(AP_RANGEFINDER_NUM_AVERAGES),
	_history_ptr(0)
{
}

// Public Methods //////////////////////////////////////////////////////////////
void RangeFinder::init(int analogPort, AP_ADC *ap_adc)
{
    // local variables
    int i;

    // store the analog port to be used
    _analogPort = analogPort;

	// set the given analog port to an input
	if( analogPort != AP_RANGEFINDER_PITOT_TUBE ){
	    pinMode(analogPort, INPUT);
    }else{
	    _num_averages = 0;  // turn off averaging for pitot tube because AP_ADC does this for us
	}

	// capture the AP_ADC object if passed in
	if( ap_adc != NULL )
	    _ap_adc = ap_adc;

	// make first call to read to get initial distance
	read();

	// initialise history
	for(i = 0; i < AP_RANGEFINDER_NUM_AVERAGES; i++)
	    _history[i] = distance;
}

void RangeFinder::set_orientation(int x, int y, int z)
{
    orientation_x = x;
	orientation_y = y;
	orientation_z = z;
}

// Read Sensor data - only the raw_value is filled in by this parent class
int RangeFinder::read()
{
    // local variables
	int temp_dist;
	int total = 0;

	// read from the analog port or pitot tube
	if( _analogPort == AP_RANGEFINDER_PITOT_TUBE ) {
	    if( _ap_adc != NULL ){
	        raw_value = _ap_adc->Ch(AP_RANGEFINDER_PITOT_TUBE_ADC_CHANNEL) >> 2;  // values from ADC are twice as big as you'd expect
	    }else{
			raw_value = 0;
		}
	}else{
		// read raw sensor value and convert to distance
    	raw_value = analogRead(_analogPort);
	}

	// convert analog value to distance in cm (using child implementation most likely)
	raw_value = convert_raw_to_distance(raw_value);
	// ensure distance is within min and max
	raw_value = constrain(raw_value, min_distance, max_distance);

	//distance = raw_value;

	// slew rate
	if(raw_value > distance){
		temp_dist = min(distance + 20, raw_value);
	}else{
		temp_dist = max(distance - 20, raw_value);
	}

	if(_num_averages > 0){
		// filter the results
		// ------------------

		// add to filter
		_history[_history_ptr] = temp_dist;

		// increment our filter
		_history_ptr++;

		// loop our filter
		if(_history_ptr == AP_RANGEFINDER_NUM_AVERAGES)
			_history_ptr = 0;


		// sum our filter
		for(uint8_t i = 0; i < AP_RANGEFINDER_NUM_AVERAGES; i++){
			total += _history[i];
		}

		// average our sampels
		distance = total / AP_RANGEFINDER_NUM_AVERAGES;

	}else{
		distance = temp_dist;
	}
	// return distance
	return distance;
}

