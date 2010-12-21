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

// Public Methods //////////////////////////////////////////////////////////////
void RangeFinder::set_orientation(int x, int y, int z)
{
    orientation_x = x; 
	orientation_y = y; 
	orientation_z = z;
}

// Protected Methods //////////////////////////////////////////////////////////
int RangeFinder::filter(int latestValue)
{
    int i;
    int total = 0;
    _history_ptr = (_history_ptr + 1) % AP_RANGEFINDER_NUM_AVERAGES;
    _history[_history_ptr] = latestValue;
	for(i=0; i<AP_RANGEFINDER_NUM_AVERAGES; i++ )
	    total += _history[i];
    return total / AP_RANGEFINDER_NUM_AVERAGES;
}
