// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-
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
		lastUpdate : the time of the last successful reading		
	
	Methods:
		Init() : Initialization of I2C and sensor
		Read() : Read Sensor data	
		Calculate(float roll, float pitch) : Calculate tilt adjusted heading
		SetOrientation(const Matrix3f &rotationMatrix) : Set orientation of compass
		SetOffsets(int x, int y, int z) : Set adjustments for HardIron disturbances
		SetDeclination(float radians) : Set heading adjustment between true north and magnetic north

	To do : code optimization
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

#define CompassAddress       0x1E
#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define MagGain              0x20
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12
#define NormalOperation      0x10
#define ModeRegister         0x02
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// Constructors ////////////////////////////////////////////////////////////////
APM_Compass_Class::APM_Compass_Class() : orientation(0), declination(0.0)
{
  // mag x y z offset initialisation
  offset[0] = 0;
  offset[1] = 0;
  offset[2] = 0;
  
  // initialise orientation matrix
  orientationMatrix = ROTATION_NONE;
}

// Public Methods //////////////////////////////////////////////////////////////
bool APM_Compass_Class::Init(void)
{
  unsigned long currentTime = millis();  // record current time
  int numAttempts = 0;
  int success = 0;
  
  Wire.begin();
  
  delay(10);
  
  // calibration initialisation
  calibration[0] = 1.0;
  calibration[1] = 1.0;
  calibration[2] = 1.0;
  
  while( success == 0 && numAttempts < 5 )
  {
      // record number of attempts at initialisation
	  numAttempts++;
  
	  // force positiveBias (compass should return 715 for all channels)
	  Wire.beginTransmission(CompassAddress);
	  Wire.send(ConfigRegA);
	  Wire.send(PositiveBiasConfig);
	  if (0 != Wire.endTransmission())
		 continue;			// compass not responding on the bus
	  delay(50);
	  
	  // set gains
	  Wire.beginTransmission(CompassAddress);
	  Wire.send(ConfigRegB);
	  Wire.send(MagGain);
	  Wire.endTransmission();
	  delay(10);  

	  Wire.beginTransmission(CompassAddress);
	  Wire.send(ModeRegister);
	  Wire.send(SingleConversion);
	  Wire.endTransmission();
	  delay(10);
	  
	  // read values from the compass
	  Read();
	  delay(10);

	  // calibrate
	  if( abs(Mag_X) > 500 && abs(Mag_X) < 1000 && abs(Mag_Y) > 500 && abs(Mag_Y) < 1000 && abs(Mag_Z) > 500 && abs(Mag_Z) < 1000)
	  {
		  calibration[0] = abs(715.0 / Mag_X);
		  calibration[1] = abs(715.0 / Mag_Y);
		  calibration[2] = abs(715.0 / Mag_Z);
		  
		  // mark success
		  success = 1;
	  }
		
	  // leave test mode
	  Wire.beginTransmission(CompassAddress);
	  Wire.send(ConfigRegA);
	  Wire.send(NormalOperation);
	  Wire.endTransmission();
	  delay(50);

	  Wire.beginTransmission(CompassAddress);
	  Wire.send(ModeRegister);
	  Wire.send(ContinuousConversion);        // Set continuous mode (default to 10Hz)
	  Wire.endTransmission();                 // End transmission
	  delay(50);
  }
  return(success);
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
    Mag_X = -((((int)buff[0]) << 8) | buff[1]) * calibration[0];    // X axis
    Mag_Y = ((((int)buff[2]) << 8) | buff[3]) * calibration[1];    // Y axis
    Mag_Z = -((((int)buff[4]) << 8) | buff[5]) * calibration[2];    // Z axis
    lastUpdate = millis();  // record time of update
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
  Vector3f rotMagVec;
  
  cos_roll = cos(roll);  // Optimizacion, se puede sacar esto de la matriz DCM?
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // rotate the magnetometer values depending upon orientation
  if( orientation == 0 )
      rotMagVec = Vector3f(Mag_X+offset[0],Mag_Y+offset[1],Mag_Z+offset[2]);  
  else
      rotMagVec = orientationMatrix*Vector3f(Mag_X+offset[0],Mag_Y+offset[1],Mag_Z+offset[2]); 
  
  // Tilt compensated Magnetic field X component:
  Head_X = rotMagVec.x*cos_pitch+rotMagVec.y*sin_roll*sin_pitch+rotMagVec.z*cos_roll*sin_pitch;
  // Tilt compensated Magnetic field Y component:
  Head_Y = rotMagVec.y*cos_roll-rotMagVec.z*sin_roll;
  // Magnetic Heading
  Heading = atan2(-Head_Y,Head_X);
  
  // Declination correction (if supplied)
  if( declination != 0.0 ) 
  {
      Heading = Heading + declination;
      if (Heading > M_PI)    // Angle normalization (-180 deg, 180 deg)
          Heading -= (2.0 * M_PI);
      else if (Heading < -M_PI)
          Heading += (2.0 * M_PI);
  }
	
  // Optimization for external DCM use. Calculate normalized components
  Heading_X = cos(Heading);
  Heading_Y = sin(Heading);
}

void APM_Compass_Class::SetOrientation(const Matrix3f &rotationMatrix)
{
    orientationMatrix = rotationMatrix;
	if( orientationMatrix == ROTATION_NONE )
	    orientation = 0;
	else
	    orientation = 1;
}

void APM_Compass_Class::SetOffsets(int x, int y, int z)
{
    offset[0] = x;
	offset[1] = y;
	offset[2] = z;
}

void APM_Compass_Class::SetDeclination(float radians)
{
    declination = radians;
}

// make one instance for the user to use
APM_Compass_Class APM_Compass;
