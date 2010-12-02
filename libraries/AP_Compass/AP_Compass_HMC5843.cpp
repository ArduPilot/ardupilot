// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-
/*
	AP_Compass_HMC5843.cpp - Arduino Library for HMC5843 I2C magnetometer
	Code by Jordi Muñoz and Jose Julio. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Sensor is conected to I2C port
	Sensor is initialized in Continuos mode (10Hz)
	
	Variables:
		heading : magnetic heading
		heading_x : magnetic heading X component
		heading_y : magnetic heading Y component
		mag_x : Raw X axis magnetometer data
		mag_y : Raw Y axis magnetometer data
		mag_z : Raw Z axis magnetometer data		
		last_update : the time of the last successful reading		
	
	Methods:
		init() : Initialization of I2C and sensor
		read() : Read Sensor data	
		calculate(float roll, float pitch) : Calculate tilt adjusted heading
		set_orientation(const Matrix3f &rotation_matrix) : Set orientation of compass
		set_offsets(int x, int y, int z) : Set adjustments for HardIron disturbances
		set_declination(float radians) : Set heading adjustment between true north and magnetic north

	To do : code optimization
	Mount position : UPDATED
                Big capacitor pointing backward, connector forward
		
*/

// AVR LibC Includes
#include <math.h>
#include "WConstants.h"

#include <Wire.h>
#include "AP_Compass_HMC5843.h"

#define COMPASS_ADDRESS       0x1E
#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define magGain              0x20
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12
#define NormalOperation      0x10
#define ModeRegister         0x02
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// Constructors ////////////////////////////////////////////////////////////////
AP_Compass_HMC5843::AP_Compass_HMC5843() : orientation(0), declination(0.0)
{
  // mag x y z offset initialisation
  offset[0] = 0;
  offset[1] = 0;
  offset[2] = 0;
  
  // initialise orientation matrix
  orientation_matrix = ROTATION_NONE;
}

// Public Methods //////////////////////////////////////////////////////////////
bool AP_Compass_HMC5843::init(int initialise_wire_lib)
{
  unsigned long currentTime = millis();  // record current time
  int numAttempts = 0;
  int success = 0;
  
  if( initialise_wire_lib != 0 )
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
	  Wire.beginTransmission(COMPASS_ADDRESS);
	  Wire.send(ConfigRegA);
	  Wire.send(PositiveBiasConfig);
	  if (0 != Wire.endTransmission())
		 continue;			// compass not responding on the bus
	  delay(50);
	  
	  // set gains
	  Wire.beginTransmission(COMPASS_ADDRESS);
	  Wire.send(ConfigRegB);
	  Wire.send(magGain);
	  Wire.endTransmission();
	  delay(10);  

	  Wire.beginTransmission(COMPASS_ADDRESS);
	  Wire.send(ModeRegister);
	  Wire.send(SingleConversion);
	  Wire.endTransmission();
	  delay(10);
	  
	  // read values from the compass
	  read();
	  delay(10);

	  // calibrate
	  if( abs(mag_x) > 500 && abs(mag_x) < 1000 && abs(mag_y) > 500 && abs(mag_y) < 1000 && abs(mag_z) > 500 && abs(mag_z) < 1000)
	  {
		  calibration[0] = fabs(715.0 / mag_x);
		  calibration[1] = fabs(715.0 / mag_y);
		  calibration[2] = fabs(715.0 / mag_z);
		  
		  // mark success
		  success = 1;
	  }
		
	  // leave test mode
	  Wire.beginTransmission(COMPASS_ADDRESS);
	  Wire.send(ConfigRegA);
	  Wire.send(NormalOperation);
	  Wire.endTransmission();
	  delay(50);

	  Wire.beginTransmission(COMPASS_ADDRESS);
	  Wire.send(ModeRegister);
	  Wire.send(ContinuousConversion);        // Set continuous mode (default to 10Hz)
	  Wire.endTransmission();                 // End transmission
	  delay(50);
  }
  return(success);
}

// Read Sensor data
void AP_Compass_HMC5843::read()
{
  int i = 0;
  byte buff[6];
 
  Wire.beginTransmission(COMPASS_ADDRESS); 
  Wire.send(0x03);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  //Wire.beginTransmission(COMPASS_ADDRESS); 
  Wire.requestFrom(COMPASS_ADDRESS, 6);    // request 6 bytes from device
  while(Wire.available())     
  { 
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  Wire.endTransmission(); //end transmission
  
  if (i==6)  // All bytes received?
  {
    // MSB byte first, then LSB, X,Y,Z
    mag_x = -((((int)buff[0]) << 8) | buff[1]) * calibration[0];    // X axis
    mag_y = ((((int)buff[2]) << 8) | buff[3]) * calibration[1];    // Y axis
    mag_z = -((((int)buff[4]) << 8) | buff[5]) * calibration[2];    // Z axis
    last_update = millis();  // record time of update
  }
}

void AP_Compass_HMC5843::calculate(float roll, float pitch)
{
  float headX;
  float headY;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  Vector3f rotmagVec;
  
  cos_roll = cos(roll);  // Optimizacion, se puede sacar esto de la matriz DCM?
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // rotate the magnetometer values depending upon orientation
  if( orientation == 0 )
      rotmagVec = Vector3f(mag_x+offset[0],mag_y+offset[1],mag_z+offset[2]);  
  else
      rotmagVec = orientation_matrix*Vector3f(mag_x+offset[0],mag_y+offset[1],mag_z+offset[2]); 
  
  // Tilt compensated magnetic field X component:
  headX = rotmagVec.x*cos_pitch+rotmagVec.y*sin_roll*sin_pitch+rotmagVec.z*cos_roll*sin_pitch;
  // Tilt compensated magnetic field Y component:
  headY = rotmagVec.y*cos_roll-rotmagVec.z*sin_roll;
  // magnetic heading
  heading = atan2(-headY,headX);
  
  // Declination correction (if supplied)
  if( declination != 0.0 ) 
  {
      heading = heading + declination;
      if (heading > M_PI)    // Angle normalization (-180 deg, 180 deg)
          heading -= (2.0 * M_PI);
      else if (heading < -M_PI)
          heading += (2.0 * M_PI);
  }
	
  // Optimization for external DCM use. Calculate normalized components
  heading_x = cos(heading);
  heading_y = sin(heading);
}

void AP_Compass_HMC5843::set_orientation(const Matrix3f &rotation_matrix)
{
    orientation_matrix = rotation_matrix;
	if( orientation_matrix == ROTATION_NONE )
	    orientation = 0;
	else
	    orientation = 1;
}

void AP_Compass_HMC5843::set_offsets(int x, int y, int z)
{
    offset[0] = x;
	offset[1] = y;
	offset[2] = z;
}

void AP_Compass_HMC5843::set_declination(float radians)
{
    declination = radians;
}
