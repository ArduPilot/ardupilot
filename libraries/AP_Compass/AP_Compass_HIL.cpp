/*
	AP_Compass_HIL.cpp - Arduino Library for HIL model of HMC5843 I2C Magnetometer
	Code by James Goppert. DIYDrones.com

	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.
*/


#include "AP_Compass_HIL.h"

// Public Methods //////////////////////////////////////////////////////////////

bool AP_Compass_HIL::read()
{
	// values set by setHIL function
	return true;
}

// Update raw magnetometer values from HIL data
//
void AP_Compass_HIL::setHIL(float _mag_x, float _mag_y, float _mag_z)
{
    // TODO: map floats to raw
    mag_x = _mag_x;
    mag_y = _mag_y;
    mag_z = _mag_z;
    healthy = true;
}
