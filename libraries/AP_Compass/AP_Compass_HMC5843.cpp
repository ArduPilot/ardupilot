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
#include <FastSerial.h>

#include <I2C.h>
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
bool AP_Compass_HMC5843::read_register(uint8_t address, uint8_t *value)
{
   if (I2c.read((uint8_t)COMPASS_ADDRESS, address, 1, value) != 0) {
	  healthy = false;
	  return false;
   }
   return true;
}

// write_register - update a register value
bool AP_Compass_HMC5843::write_register(uint8_t address, byte value)
{
   if (I2c.write((uint8_t)COMPASS_ADDRESS, address, value) != 0) {
	  healthy = false;
	  return false;
   }
   return true;
}

/*
  the 5883L has a different orientation to the 5843. This allows us to
  use a single MAG_ORIENTATION for both
 */
static void rotate_for_5883L(AP_VarS<Matrix3f> *_orientation_matrix)
{
		_orientation_matrix->set_and_save(_orientation_matrix->get() * Matrix3f(ROTATION_YAW_90));
}


// Read Sensor data
bool AP_Compass_HMC5843::read_raw()
{
   uint8_t buff[6];

   if (I2c.read(COMPASS_ADDRESS, 0x03, 6, buff) != 0) {
	  healthy = false;
	  return false;
   }

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
	  // no valid data available
	  return false;
   }

   mag_x = -rx;
   mag_y = ry;
   mag_z = -rz;
   
   return true;
}



/*
  re-initialise after a IO error
 */
bool AP_Compass_HMC5843::re_initialise()
{
   if (! write_register(ConfigRegA, _base_config) ||
	   ! write_register(ConfigRegB, magGain) ||
	   ! write_register(ModeRegister, ContinuousConversion))
	  return false;
   return true;
}


// Public Methods //////////////////////////////////////////////////////////////
bool 
AP_Compass_HMC5843::init()
{
  int numAttempts = 0, good_count = 0;
  bool success = false;
  byte calibration_gain = 0x20;
  uint16_t expected_x = 715;
  uint16_t expected_yz = 715;
  float gain_multiple = 1.0;

  delay(10);

  // determine if we are using 5843 or 5883L
  if (! write_register(ConfigRegA, SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation) ||
	  ! read_register(ConfigRegA, &_base_config)) {
	 healthy = false;
	 return false;
  }
  if ( _base_config == (SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation)) {
	 // a 5883L supports the sample averaging config
	 int old_product_id = product_id;

	 product_id = AP_COMPASS_TYPE_HMC5883L;
	 calibration_gain = 0x60;
	 expected_x = 766;
	 expected_yz  = 713;
	 gain_multiple = 660.0 / 1090; // adjustment for runtime vs calibration gain

	 if (old_product_id != product_id) {
		/* now we know the compass type we need to rotate the
		 * orientation matrix that we were given
		 */
		rotate_for_5883L(&_orientation_matrix);
	 }
  } else if (_base_config == (NormalOperation | DataOutputRate_75HZ<<2)) {
      product_id = AP_COMPASS_TYPE_HMC5843;
  } else {
	 // not behaving like either supported compass type
	 return false;
  }

  calibration[0] = 0;
  calibration[1] = 0;
  calibration[2] = 0;
  
  while ( success == 0 && numAttempts < 20 && good_count < 5)
  {
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

	  // read values from the compass
	  delay(50);
	  if (!read_raw())
		 continue; // we didn't read valid values

	  delay(10);

	  float cal[3];

	  cal[0] = fabs(expected_x / (float)mag_x);
	  cal[1] = fabs(expected_yz / (float)mag_y);
	  cal[2] = fabs(expected_yz / (float)mag_z);

	  if (cal[0] > 0.7 && cal[0] < 1.3 && 
		  cal[1] > 0.7 && cal[1] < 1.3 && 
		  cal[2] > 0.7 && cal[2] < 1.3) {
		 good_count++;
		 calibration[0] += cal[0];
		 calibration[1] += cal[1];
		 calibration[2] += cal[2];
	  }

#if 0
	  /* useful for debugging */
	  Serial.print("mag_x: ");
	  Serial.print(mag_x);
	  Serial.print(" mag_y: ");
	  Serial.print(mag_y);
	  Serial.print(" mag_z: ");
	  Serial.println(mag_z);
	  Serial.print("CalX: ");
	  Serial.print(calibration[0]/good_count);
	  Serial.print(" CalY: ");
	  Serial.print(calibration[1]/good_count);
	  Serial.print(" CalZ: ");
	  Serial.println(calibration[2]/good_count);
#endif
  }

  if (good_count >= 5) {
	 calibration[0] = calibration[0] * gain_multiple / good_count;
	 calibration[1] = calibration[1] * gain_multiple / good_count;
	 calibration[2] = calibration[2] * gain_multiple / good_count;
	 success = true;
  } else {
	 /* best guess */
	 calibration[0] = 1.0;
	 calibration[1] = 1.0;
	 calibration[2] = 1.0;
  }

  // leave test mode
  if (!re_initialise()) {
	 return false;
  }

  return success;
}

// Read Sensor data
bool AP_Compass_HMC5843::read()
{
   if (!healthy && !re_initialise()) {
	  return false;
   }

   if (!read_raw()) {
	  return false;
   }

   mag_x *= calibration[0];
   mag_y *= calibration[1];
   mag_z *= calibration[2];

   last_update = millis();  // record time of update
   // rotate and offset the magnetometer values
   // XXX this could well be done in common code...

   Vector3f rot_mag = _orientation_matrix.get() * Vector3f(mag_x,mag_y,mag_z);
   rot_mag = rot_mag + _offset.get();
   mag_x = rot_mag.x;
   mag_y = rot_mag.y;
   mag_z = rot_mag.z;
   healthy = true;

   return true;
}

// set orientation
void
AP_Compass_HMC5843::set_orientation(const Matrix3f &rotation_matrix)
{
   _orientation_matrix.set_and_save(rotation_matrix);
    if (product_id == AP_COMPASS_TYPE_HMC5883L) {
		rotate_for_5883L(&_orientation_matrix);
	}
}
