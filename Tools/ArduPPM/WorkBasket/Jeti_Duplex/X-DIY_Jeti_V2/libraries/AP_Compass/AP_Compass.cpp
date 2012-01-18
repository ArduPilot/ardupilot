/*
	AP_Compass.cpp - Arduino Library for HMC5843 I2C Magnetometer
	Code by Jordi Muñoz and Jose Julio. DIYDrones.com

	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

	Sensor is conected to I2C port
	Sensor is initialized in Continuos mode (10Hz)
	
	Variables:
		heading : Magnetic heading
		heading_X : Magnetic heading X component
		heading_Y : Magnetic heading Y component
		mag_X : Raw X axis magnetometer data
		mag_Y : Raw Y axis magnetometer data
		mag_Z : Raw Z axis magnetometer data		
	
	Methods:
		init() : initialization of I2C and sensor
		update() : update Sensor data		

	To do : Calibration of the sensor, code optimization
	Mount position : UPDATED
	Big capacitor pointing backward, connector forward
		
*/


extern "C" {
	// AVR LibC Includes
	#include <math.h>
	#include "WConstants.h"
}

#include <Wire.h>
#include "AP_Compass.h"

#define COMPASS_ADDRESS 0x1E

// Constructors ////////////////////////////////////////////////////////////////
AP_Compass::AP_Compass()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void
AP_Compass::init(void)
{
	Wire.begin();	
	Wire.beginTransmission(COMPASS_ADDRESS);
	Wire.send(0x02); 
	Wire.send(0x00);				// Set continouos mode (default to 10Hz)
	Wire.endTransmission(); // end transmission
}

// update Sensor data
void
AP_Compass::update()
{
	int i = 0;
	byte buff[6];
 
	Wire.beginTransmission(COMPASS_ADDRESS); 
	Wire.send(0x03);							// sends address to read from
	Wire.endTransmission(); 					// end transmission
	
	//Wire.beginTransmission(COMPASS_ADDRESS); 
	Wire.requestFrom(COMPASS_ADDRESS, 6);		// request 6 bytes from device
	while(Wire.available()){ 
		buff[i] = Wire.receive();				// receive one byte
		i++;
	}
	Wire.endTransmission(); 					// end transmission
	
	// All bytes received?
	if (i == 6)	{
		// MSB byte first, then LSB, X,Y,Z
		mag_X = -((((int)buff[0]) << 8) | buff[1]);		// X axis
		mag_Y =  ((((int)buff[2]) << 8) | buff[3]);		// Y axis
		mag_Z = -((((int)buff[4]) << 8) | buff[5]);		// Z axis
	}
}

void
AP_Compass::calculate(float roll, float pitch)
{
	float head_X;
	float head_Y;
	float cos_roll;
	float sin_roll;
	float cos_pitch;
	float sin_pitch;
	
	cos_roll = cos(roll);							// Optimization, you can get this out of the matrix DCM?
	sin_roll = sin(roll);
	cos_pitch = cos(pitch);
	sin_pitch = sin(pitch);
	
	// Tilt compensated Magnetic field X component:
	head_X = mag_X * cos_pitch + mag_Y * sin_roll * sin_pitch + mag_Z * cos_roll * sin_pitch;
	
	// Tilt compensated Magnetic field Y component:
	head_Y = mag_Y * cos_roll - mag_Z * sin_roll;
	
	// Magnetic heading
	heading = atan2(-head_Y, head_X);
	ground_course = (degrees(heading) + 180) * 100;
	
	// Optimization for external DCM use. calculate normalized components
	heading_X = cos(heading);
	heading_Y = sin(heading);
}
