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

// ConfigRegA valid sample averaging for 5883L
#define SampleAveraging_1    0x00
#define SampleAveraging_2    0x01
#define SampleAveraging_4    0x02
#define SampleAveraging_8    0x03

// ConfigRegA valid data output rates for 5883L
#define DataOutputRate_0_75HZ 0x00
#define DataOutputRate_1_5HZ  0x01
#define DataOutputRate_3HZ    0x02
#define DataOutputRate_7_5HZ  0x03
#define DataOutputRate_15HZ   0x04
#define DataOutputRate_30HZ   0x05
#define DataOutputRate_75HZ   0x06


// Public Methods //////////////////////////////////////////////////////////////
bool 
AP_Compass_HMC5843::init()
{
  int numAttempts = 0;
  int success = 0;
  byte orig_value, new_value;  // used to test compass type

  delay(10);

  // determine if we are using 5843 or 5883L
  orig_value = read_register(ConfigRegA);  // read config register A
  new_value = orig_value | 0x60;           // turn on sample averaging turned on (only avaiable in 5883L)
  write_register(ConfigRegA, new_value);   // write config register A
  if( read_register(ConfigRegA) == new_value ) {  // if we've successfully updated it then it's a 5883L
      product_id = AP_COMPASS_TYPE_HMC5883L;
	  write_register(ConfigRegA, orig_value);   // restore config register A to it's original state
  }else
      product_id = AP_COMPASS_TYPE_HMC5843;
  
  while( success == 0 && numAttempts < 5 )
  {
      // record number of attempts at initialisation
	  numAttempts++;

	  // force positiveBias (compass should return 715 for all channels)
	  Wire.beginTransmission(COMPASS_ADDRESS);
	  Wire.send(ConfigRegA);
	  if (product_id == AP_COMPASS_TYPE_HMC5843) {
		 Wire.send(PositiveBiasConfig);
	  } else {
		 Wire.send(SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | PositiveBiasConfig);
	  }
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

	  // calibration initialisation
	  calibration[0] = 1.0;
	  calibration[1] = 1.0;
	  calibration[2] = 1.0;

	  // read values from the compass
	  read();
	  delay(10);

	  // calibrate
	  if( abs(mag_x) > 500 && abs(mag_x) < 2000 && abs(mag_y) > 500 && abs(mag_y) < 2000 && abs(mag_z) > 500 && abs(mag_z) < 2000)
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
	  if (product_id == AP_COMPASS_TYPE_HMC5843) {
		 Wire.send(NormalOperation);
	  } else {
		 Wire.send(SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation);
	  }
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
void 
AP_Compass_HMC5843::read()
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
	if( product_id == AP_COMPASS_TYPE_HMC5883L ) {
		mag_y = ((((int)buff[4]) << 8) | buff[5]) * calibration[1];    // Y axis
		mag_z = -((((int)buff[2]) << 8) | buff[3]) * calibration[2];    // Z axis
	}else{
		mag_y = ((((int)buff[2]) << 8) | buff[3]) * calibration[1];    // Y axis
		mag_z = -((((int)buff[4]) << 8) | buff[5]) * calibration[2];    // Z axis
	}	
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

// set orientation
void
AP_Compass_HMC5843::set_orientation(const Matrix3f &rotation_matrix)
{
    if( product_id == AP_COMPASS_TYPE_HMC5883L ) {
        _orientation_matrix.set_and_save(rotation_matrix * Matrix3f(ROTATION_YAW_90));
	}else{
	    _orientation_matrix.set_and_save(rotation_matrix);
	}
}

// read_register - read a register value
byte 
AP_Compass_HMC5843::read_register(int address)
{
  byte result;
  byte buff[1];
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.send(address);     //sends address to read from
  Wire.endTransmission(); //end transmission

  Wire.requestFrom(COMPASS_ADDRESS, 1);    // request 1 byte from device
  if( Wire.available() )
      result = Wire.receive();  // receive one byte
  Wire.endTransmission(); //end transmission
  
  return result;
}

// write_register - update a register value
void 
AP_Compass_HMC5843::write_register(int address, byte value)
{
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.send(address);
  Wire.send(value);
  Wire.endTransmission();
  delay(10);
}
