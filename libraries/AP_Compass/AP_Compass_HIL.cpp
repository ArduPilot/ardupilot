/*
	AP_Compass_HIL.cpp - Arduino Library for HIL model of HMC5843 I2C Magnetometer
	Code by James Goppert. DIYDrones.com

	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.
*/


#include "AP_Compass_HIL.h"

// Constructors ////////////////////////////////////////////////////////////////
AP_Compass_HIL::AP_Compass_HIL() : orientation(0), declination(0.0)
{
  // mag x y z offset initialisation
  offset[0] = 0;
  offset[1] = 0;
  offset[2] = 0;
  
  // initialise orientation matrix
  orientation_matrix = ROTATION_NONE;
}

// Public Methods //////////////////////////////////////////////////////////////
bool AP_Compass_HIL::init(int initialise_wire_lib)
{
  unsigned long currentTime = millis();  // record current time
  int numAttempts = 0;
  int success = 0;
  
  // calibration initialisation
  calibration[0] = 1.0;
  calibration[1] = 1.0;
  calibration[2] = 1.0;
  
  while( success == 0 && numAttempts < 5 )
  {
      // record number of attempts at initialisation
          numAttempts++;
  
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
  }
  return(success);
}

// Read Sensor data
void AP_Compass_HIL::read()
{
    // values set by setHIL function
}

void AP_Compass_HIL::calculate(float roll, float pitch)
{
  float headX;
  float headY;
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
      rotMagVec = Vector3f(mag_x+offset[0],mag_y+offset[1],mag_z+offset[2]);  
  else
      rotMagVec = orientation_matrix*Vector3f(mag_x+offset[0],mag_y+offset[1],mag_z+offset[2]); 
  
  // Tilt compensated Magnetic field X component:
  headX = rotMagVec.x*cos_pitch+rotMagVec.y*sin_roll*sin_pitch+rotMagVec.z*cos_roll*sin_pitch;
  // Tilt compensated Magnetic field Y component:
  headY = rotMagVec.y*cos_roll-rotMagVec.z*sin_roll;
  // Magnetic heading
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

void AP_Compass_HIL::set_orientation(const Matrix3f &rotation_matrix)
{
    orientation_matrix = rotation_matrix;
        if( orientation_matrix == ROTATION_NONE )
            orientation = 0;
        else
            orientation = 1;
}

void AP_Compass_HIL::set_offsets(int x, int y, int z)
{
    offset[0] = x;
        offset[1] = y;
        offset[2] = z;
}

void AP_Compass_HIL::set_declination(float radians)
{
    declination = radians;
}

void AP_Compass_HIL::setHIL(float _mag_x, float _mag_y, float _mag_z)
{
    // TODO: map floats to raw
    mag_x = _mag_x;
    mag_y = _mag_y;
    mag_z = _mag_z;
}
