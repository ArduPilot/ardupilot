// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//
//	AP_IMU_INS.cpp - IMU Sensor Library for Ardupilot Mega
//		Code by Michael Smith, Doug Weibel, Jordi Muñoz and Jose Julio. DIYDrones.com
//
//	This library is free software; you can redistribute it and/or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//

/// @file	AP_IMU_INS.cpp
/// @brief	IMU driver on top of an INS driver. Provides calibration for the
//          inertial sensors (gyro and accel)

#include <FastSerial.h>
#include <AP_Common.h>

#include <avr/eeprom.h>

#include "AP_IMU_INS.h"

void
AP_IMU_INS::init( Start_style style,
                  void (*delay_cb)(unsigned long t),
                  void (*flash_leds_cb)(bool on),
                  AP_PeriodicProcess * scheduler )
{
    _ins->init(scheduler);
    // if we are warm-starting, load the calibration data from EEPROM and go
    //
    if (WARM_START == style) {
        _sensor_cal.load();
    } else {

        // do cold-start calibration for both accel and gyro
        _init_gyro(delay_cb, flash_leds_cb);

        // save calibration
        _sensor_cal.save();
    }
}

/**************************************************/

void
AP_IMU_INS::init_gyro(void (*delay_cb)(unsigned long t), void (*flash_leds_cb)(bool on))
{
    _init_gyro(delay_cb, flash_leds_cb);
    _sensor_cal.save();
}

#define FLASH_LEDS(on) do { if (flash_leds_cb != NULL) flash_leds_cb(on); } while (0)

void
AP_IMU_INS::_init_gyro(void (*delay_cb)(unsigned long t), void (*flash_leds_cb)(bool on))
{
	int flashcount = 0;
	float adc_in;
	float prev[3] = {0,0,0};
	float total_change;
	float max_offset;
    float ins_gyro[6];

	// cold start
	delay_cb(500);
	Serial.printf_P(PSTR("Init Gyro"));

	for(int c = 0; c < 25; c++){
    // Mostly we are just flashing the LED's here
    // to tell the user to keep the IMU still
        FLASH_LEDS(true);
		delay_cb(20);

        _ins->update();
        _ins->get_gyros(ins_gyro);

        FLASH_LEDS(false);
		delay_cb(20);
	}

	for (int j = 0; j <= 2; j++)
	    _sensor_cal[j] = 500;		// Just a large value to load prev[j] the first time

	do {

    _ins->update();
    _ins->get_gyros(ins_gyro);

		for (int j = 0; j <= 2; j++){
			prev[j]     = _sensor_cal[j];
			adc_in      = ins_gyro[j];
			_sensor_cal[j]	= adc_in;
		}

		for(int i = 0; i < 50; i++){

      _ins->update();
      _ins->get_gyros(ins_gyro);

			for (int j = 0; j < 3; j++){
				adc_in = ins_gyro[j];
				// filter
				_sensor_cal[j] = _sensor_cal[j] * 0.9 + adc_in * 0.1;
			}

			delay_cb(20);
			if(flashcount == 5) {
				Serial.printf_P(PSTR("*"));
                FLASH_LEDS(true);
			}

			if(flashcount >= 10) {
				flashcount = 0;
                FLASH_LEDS(false);
			}
			flashcount++;
		}

		total_change    = fabs(prev[0] - _sensor_cal[0]) + fabs(prev[1] - _sensor_cal[1]) +fabs(prev[2] - _sensor_cal[2]);
		max_offset      = (_sensor_cal[0] > _sensor_cal[1]) ? _sensor_cal[0] : _sensor_cal[1];
		max_offset      = (max_offset > _sensor_cal[2]) ? max_offset : _sensor_cal[2];
		delay_cb(500);
	} while (  total_change > _gyro_total_cal_change || max_offset > _gyro_max_cal_offset);
}

void
AP_IMU_INS::save()
{
    _sensor_cal.save();
}

void
AP_IMU_INS::init_accel(void (*delay_cb)(unsigned long t), void (*flash_leds_cb)(bool on))
{
    _init_accel(delay_cb, flash_leds_cb);
    _sensor_cal.save();
}

void
AP_IMU_INS::_init_accel(void (*delay_cb)(unsigned long t), void (*flash_leds_cb)(bool on))
{
	int flashcount = 0;
	float adc_in;
	float prev[6] = {0,0,0};
	float total_change;
	float max_offset;
    float ins_accel[3];


	// cold start
	delay_cb(500);

	Serial.printf_P(PSTR("Init Accel"));

	for (int j=3; j<=5; j++) _sensor_cal[j] = 500;		// Just a large value to load prev[j] the first time

	do {
    _ins->update();
    _ins->get_accels(ins_accel);

		for (int j = 3; j <= 5; j++){
			prev[j] = _sensor_cal[j];
			adc_in 		    = ins_accel[j-3];
			_sensor_cal[j]	= adc_in;
		}

		for(int i = 0; i < 50; i++){		// We take some readings...

			delay_cb(20);
      _ins->update();
      _ins->get_accels(ins_accel);

			for (int j = 3; j < 6; j++){
				adc_in 	    	= ins_accel[j-3];
				_sensor_cal[j]	= _sensor_cal[j] * 0.9 + adc_in * 0.1;
			}

			if(flashcount == 5) {
				Serial.printf_P(PSTR("*"));
                FLASH_LEDS(true);
			}

			if(flashcount >= 10) {
				flashcount = 0;
                FLASH_LEDS(false);
			}
			flashcount++;
		}

		// null gravity from the Z accel
		_sensor_cal[5] += 9.805;

		total_change = fabs(prev[3] - _sensor_cal[3]) + fabs(prev[4] - _sensor_cal[4]) +fabs(prev[5] - _sensor_cal[5]);
		max_offset = (_sensor_cal[3] > _sensor_cal[4]) ? _sensor_cal[3] : _sensor_cal[4];
		max_offset = (max_offset > _sensor_cal[5]) ? max_offset : _sensor_cal[5];

		delay_cb(500);
	} while (  total_change > _accel_total_cal_change || max_offset > _accel_max_cal_offset);

	Serial.printf_P(PSTR(" "));
}

  float
AP_IMU_INS::_calibrated(uint8_t channel, float ins_value)
{
    return ins_value - _sensor_cal[channel];
}


bool
AP_IMU_INS::update(void)
{
  float gyros[3];
  float accels[3];

  _ins->update();
  _ins->get_gyros(gyros);
  _ins->get_accels(accels);
  _sample_time = _ins->sample_time();

	// convert corrected gyro readings to delta acceleration
	//
	_gyro.x = _calibrated(0, gyros[0]);
	_gyro.y = _calibrated(1, gyros[1]);
	_gyro.z = _calibrated(2, gyros[2]);

	// convert corrected accelerometer readings to acceleration
	//
	_accel.x = _calibrated(3, accels[0]);
	_accel.y = _calibrated(4, accels[1]);
	_accel.z = _calibrated(5, accels[2]);

	// always updated
	return true;
}
