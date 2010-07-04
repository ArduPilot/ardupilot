/*
	APM_Compass.cpp - Arduino Library for HMC5843 I2C Magnetometer
	Code by Jordi Muñoz and Jose Julio. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Sensor is conected to I2C port
	Sensor is initialized in Continuos mode (10Hz)
	
	Variables:
		Heading : Magnetic heading
		Heading_X : Magnetic heading X component
		Heading_Y : Magnetic heading Y component
		Mag_X : Raw X axis magnetometer data
		Mag_Y : Raw Y axis magnetometer data
		Mag_Z : Raw Z axis magnetometer data		
	
	Methods:
		Init() : Initialization of I2C and sensor
		Read() : Read Sensor data		

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
#include "APM_Compass.h"

#define CompassAddress 0x1E

// Constructors ////////////////////////////////////////////////////////////////
APM_Compass_Class::APM_Compass_Class()
{
}

// Public Methods //////////////////////////////////////////////////////////////
void APM_Compass_Class::Init(void)
{
  Wire.begin();  
  Wire.beginTransmission(CompassAddress);
  Wire.send(0x02); 
  Wire.send(0x00);        // Set continouos mode (default to 10Hz)
  Wire.endTransmission(); //end transmission
}

// Read Sensor data
void APM_Compass_Class::Read()
{
  int i = 0;
  byte buff[6];
 
  Wire.beginTransmission(CompassAddress); 
  Wire.send(0x03);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  //Wire.beginTransmission(CompassAddress); 
  Wire.requestFrom(CompassAddress, 6);    // request 6 bytes from device
  while(Wire.available())     
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission
  
  if (i==6)  // All bytes received?
    {
    // MSB byte first, then LSB, X,Y,Z
    Mag_X = -((((int)buff[0]) << 8) | buff[1]);    // X axis
    Mag_Y = ((((int)buff[2]) << 8) | buff[3]);    // Y axis
    Mag_Z = -((((int)buff[4]) << 8) | buff[5]);    // Z axis
    }
}

void APM_Compass_Class::Calculate(float roll, float pitch)
{
  float Head_X;
  float Head_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);  // Optimizacion, se puede sacar esto de la matriz DCM?
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  // Tilt compensated Magnetic field X component:
  Head_X = Mag_X*cos_pitch+Mag_Y*sin_roll*sin_pitch+Mag_Z*cos_roll*sin_pitch;
  // Tilt compensated Magnetic field Y component:
  Head_Y = Mag_Y*cos_roll-Mag_Z*sin_roll;
  // Magnetic Heading
  Heading = atan2(-Head_Y,Head_X);
  // Optimization for external DCM use. Calculate normalized components
  Heading_X = cos(Heading);
  Heading_Y = sin(Heading);
}


// make one instance for the user to use
APM_Compass_Class APM_Compass;