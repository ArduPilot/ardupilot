// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-
/*
	AP_Compass_HMC5883L.cpp - Arduino Library for HMC5883L I2C magnetometer
	Code by Jordi Muñoz and Jose Julio. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Sensor is conected to I2C port
	Sensor is initialized in Continuos mode (10Hz)

*/

// AVR LibC Includes
#include <math.h>
#include "WConstants.h"

#include <Wire.h>
#include "AP_Compass_HMC5883L.h"

#define COMPASS_ADDRESS      0x1E
#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define ModeRegister         0x02
#define DataOutputXMSB       0x03
#define DataOutputXLSB       0x04
#define DataOutputZMSB       0x05
#define DataOutputZLSB       0x06
#define DataOutputYMSB       0x07
#define DataOutputYLSB       0x08
#define StatusRegister       0x09
#define IDRegisterA          0x0A
#define IDRegisterB          0x0B
#define IDRegisterC          0x0C

// default gain value
#define magGain              0x20

// ModeRegister valid modes
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// ConfigRegA valid sample averaging
#define SampleAveraging_1    0x00
#define SampleAveraging_2    0x01
#define SampleAveraging_4    0x02
#define SampleAveraging_8    0x03

// ConfigRegA valid data output rates
#define DataOutputRate_0_75HZ 0x00
#define DataOutputRate_1_5HZ  0x01
#define DataOutputRate_3HZ    0x02
#define DataOutputRate_7_5HZ  0x03
#define DataOutputRate_15HZ   0x04
#define DataOutputRate_30HZ   0x05
#define DataOutputRate_75HZ   0x06

// ConfigRegA valid measurement configuration bits
#define NormalOperation      0x10
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12


// Public Methods //////////////////////////////////////////////////////////////
bool AP_Compass_HMC5883L::init()
{
  int numAttempts = 0;
  int success = 0;

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
	  Wire.send(SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | PositiveBiasConfig);
	  if (0 != Wire.endTransmission())
		 continue;			// compass not responding on the bus
	  delay(50);

	  // set gains
	  Wire.beginTransmission(COMPASS_ADDRESS);
	  Wire.send(ConfigRegB);
	  Wire.send(magGain);
	  Wire.endTransmission();
	  delay(10);

	  // put compass into single-measurement mode
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
	  Wire.send(SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation);
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
void AP_Compass_HMC5883L::read()
{
  int i = 0;
  byte buff[6];
  Vector3f rot_mag;

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
    mag_y = ((((int)buff[4]) << 8) | buff[5]) * calibration[1];    // Y axis
    mag_z = -((((int)buff[2]) << 8) | buff[3]) * calibration[2];    // Z axis
    last_update = millis();  // record time of update
	// rotate and offset the magnetometer values
    // XXX this could well be done in common code...
	rot_mag = _orientation_matrix.get() * Vector3f(mag_x,mag_y,mag_z);
	rot_mag = rot_mag + _offset.get();
	mag_x = rot_mag.x;
	mag_y = rot_mag.y;
	mag_z = rot_mag.z;
  }
}

