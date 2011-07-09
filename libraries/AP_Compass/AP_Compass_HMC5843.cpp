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

// read_register - read a register value
static bool
read_register(int address, byte *value)
{
  bool ret = false;

  *value = 0;

  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.send(address);     //sends address to read from
  if (0 != Wire.endTransmission())
	 return false;

  Wire.requestFrom(COMPASS_ADDRESS, 1);    // request 1 byte from device
  if( Wire.available() ) {
      *value = Wire.receive();  // receive one byte
	  ret = true;
  }
  if (0 != Wire.endTransmission())
	 return false;
  
  return ret;
}

// write_register - update a register value
static bool
write_register(int address, byte value)
{
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.send(address);
  Wire.send(value);
  if (0 != Wire.endTransmission())
	 return false;
  delay(10);
  return true;
}


// Public Methods //////////////////////////////////////////////////////////////
bool 
AP_Compass_HMC5843::init()
{
  int numAttempts = 0;
  bool success = false;
  byte base_config;  // used to test compass type
  byte calibration_gain = 0x20;
  uint16_t expected_xy = 715;
  uint16_t expected_z = 715;

  delay(10);

  // determine if we are using 5843 or 5883L
  if (! write_register(ConfigRegA, SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation) ||
	  ! read_register(ConfigRegA, &base_config)) {
	 return false;
  }
  if ( base_config == (SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation)) {
	 // a 5883L supports the sample averaging config
      product_id = AP_COMPASS_TYPE_HMC5883L;
	  calibration_gain = 0x60;
	  expected_xy = 766;
	  expected_z  = 713;
  } else if (base_config == NormalOperation | DataOutputRate_75HZ<<2) {
      product_id = AP_COMPASS_TYPE_HMC5843;
  } else {
	 // not behaving like either supported compass type
	 return false;
  }
  
  while( success == 0 && numAttempts < 5 )
  {
	 unsigned long update_stamp = last_update;

      // record number of attempts at initialisation
	  numAttempts++;

	  // force positiveBias (compass should return 715 for all channels)
	  if (! write_register(ConfigRegA, PositiveBiasConfig))
		 continue; // compass not responding on the bus
	  delay(50);

	  // set gains
	  if (! write_register(ConfigRegB, calibration_gain) ||
		  ! write_register(ModeRegister, SingleConversion))
		 continue;

	  // calibration initialisation
	  calibration[0] = 1.0;
	  calibration[1] = 1.0;
	  calibration[2] = 1.0;

	  // read values from the compass
	  read();
	  if (last_update == update_stamp) 
		 continue; // we didn't read valid values

	  delay(10);

	  // calibrate
	  if( abs(mag_x) > 500 && abs(mag_x) < 2000 && abs(mag_y) > 500 && abs(mag_y) < 2000 && abs(mag_z) > 500 && abs(mag_z) < 2000)
	  {
		 calibration[0] = fabs(expected_xy / (float)mag_x);
		 calibration[1] = fabs(expected_xy / (float)mag_y);
		 calibration[2] = fabs(expected_z / (float)mag_z);

		 // mark success
		 success = true;
	  }
  }

  // leave test mode
  if (! write_register(ConfigRegA, base_config))
	 return false;
  delay(50);
  if (! write_register(ConfigRegB, magGain) ||
	  ! write_register(ModeRegister, ContinuousConversion))
	 return false;
  delay(50);

  return success;
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
  if (0 != Wire.endTransmission())
	 return;

  //Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.requestFrom(COMPASS_ADDRESS, 6);    // request 6 bytes from device
  while(Wire.available())
  {
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }
  if (0 != Wire.endTransmission())
	 return;

  if (i==6)  // All bytes received?
  {
	 int16_t rx, ry, rz;
	 rx = (int16_t)(buff[0] << 8) | buff[1];
	 if (product_id == AP_COMPASS_TYPE_HMC5883L) {
		rz = (int16_t)(buff[2] << 8) | buff[3];
		ry = (int16_t)(buff[4] << 8) | buff[5];
	 } else {
		ry = (int16_t)(buff[2] << 8) | buff[3];
		rz = (int16_t)(buff[4] << 8) | buff[5];
	 }
	 if (rx == -4096 || ry == -4096 || rz == -4096) {
		// no valid data available, last_update is not updated
		return;
	 }

	 mag_x = -rx * calibration[0];
	 mag_y =  ry * calibration[1];
	 mag_z = -rz * calibration[2];

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
