// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//
//	AP_IMU.cpp - IMU Sensor Library for Ardupilot Mega
//		Code by Michael Smith, Doug Weibel, Jordi Muñoz and Jose Julio. DIYDrones.com
//
//	This library works with the ArduPilot Mega and "Oilpan"
//
//	This library is free software; you can redistribute it and/or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.
//

/// @file	AP_IMU.h
/// @brief	IMU driver for the APM oilpan

#include <FastSerial.h>
#include <AP_Common.h>

#include <avr/eeprom.h>

#include "AP_IMU_Oilpan.h"

// XXX secret knowledge about the APM/oilpan wiring
//
#define A_LED_PIN   37
#define C_LED_PIN   35

// Sensors: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
const uint8_t AP_IMU_Oilpan::_sensors[6]        = { 1, 2, 0, 4, 5, 6};	// Channel assignments on the APM oilpan
const int8_t  AP_IMU_Oilpan::_sensor_signs[6]	= {	1,-1,-1, 1,-1,-1};  // Channel orientation vs. normal

// Temp compensation curve constants
// These must be produced by measuring data and curve fitting
// [X/Y/Z gyro][A/B/C or 0 order/1st order/2nd order constants]
//
const float   AP_IMU_Oilpan::_gyro_temp_curve[3][3] = {
	{1665,0,0},
	{1665,0,0},
	{1665,0,0}
};

void
AP_IMU_Oilpan::init(Start_style style)
{
    // if we are warm-starting, load the calibration data from EEPROM and go
    //
    if (WARM_START == style) {
        _sensor_cal.load();
    } else {

        // do cold-start calibration for both accel and gyro
        _init_gyro();
        _init_accel();

        // save calibration
        _sensor_cal.save();
    }
}

/**************************************************/

void
AP_IMU_Oilpan::init_gyro()
{
    _init_gyro();
    _sensor_cal.save();
}

void
AP_IMU_Oilpan::_init_gyro()
{
	int flashcount = 0;
	int tc_temp;
	float adc_in;

	// cold start
	tc_temp = _adc->Ch(_gyro_temp_ch);
 	delay(500);
	Serial.println("Init Gyro");

	for(int c = 0; c < 25; c++){				// Mostly we are just flashing the LED's here to tell the user to keep the IMU still
		digitalWrite(A_LED_PIN, LOW);
		digitalWrite(C_LED_PIN, HIGH);
		delay(20);

		for (int i = 0; i < 6; i++)
			adc_in = _adc->Ch(_sensors[i]);
			
		digitalWrite(A_LED_PIN, HIGH);
		digitalWrite(C_LED_PIN, LOW);
		delay(20);
	}

	for (int j = 0; j <= 2; j++){
		adc_in -= _sensor_compensation(j, tc_temp);
		_sensor_cal[j]	= adc_in;
	}

	for(int i = 0; i < 50; i++){
		for (int j = 0; j < 3; j++){
			adc_in = _adc->Ch(_sensors[j]);
			// Subtract temp compensated typical gyro bias
			adc_in -= _sensor_compensation(j, tc_temp);
			// filter
			_sensor_cal[j] = _sensor_cal[j] * 0.9 + adc_in * 0.1;
		}

		delay(20);
		if(flashcount == 5) {
			Serial.print("*");
			digitalWrite(A_LED_PIN, LOW);
			digitalWrite(C_LED_PIN, HIGH);
		}

		if(flashcount >= 10) {
			flashcount = 0;
			digitalWrite(C_LED_PIN, LOW);
			digitalWrite(A_LED_PIN, HIGH);
		}
		flashcount++;
	}
}

void
AP_IMU_Oilpan::init_accel()
{
    _init_accel();
    _sensor_cal.save();
}

void
AP_IMU_Oilpan::_init_accel()
{
	int flashcount = 0;
	float adc_in;

	// cold start
 	delay(500);

	Serial.println("Init Accel");

	// init to initial reading (unlike gyro which presumes zero...)
	//
	for (int j = 3; j < 6; j++){
		adc_in 		    = _adc->Ch(_sensors[j]);
		adc_in 		    -= _sensor_compensation(j, 0);  // XXX secret knowledge, temperature ignored
		_sensor_cal[j]	= adc_in;
	}

	for(int i = 0; i < 50; i++){		// We take some readings...

		delay(20);

		for (int j = 3; j < 6; j++){
			adc_in 	    	= _adc->Ch(_sensors[j]);
			adc_in 		    -= _sensor_compensation(j, 0);  // XXX secret knowledge, temperature ignored
			_sensor_cal[j]	= _sensor_cal[j] * 0.9 + adc_in * 0.1;
		}

		if(flashcount == 5) {
			Serial.print("*");
			digitalWrite(A_LED_PIN, LOW);
			digitalWrite(C_LED_PIN, HIGH);
		}

		if(flashcount >= 10) {
			flashcount = 0;
			digitalWrite(C_LED_PIN, LOW);
			digitalWrite(A_LED_PIN, HIGH);
		}
		flashcount++;
	}
	Serial.println(" ");

	// null gravity from the Z accel
	_sensor_cal[5] += _gravity * _sensor_signs[5];
}

/**************************************************/
// Returns the temperature compensated raw gyro value
//---------------------------------------------------

float
AP_IMU_Oilpan::_sensor_compensation(uint8_t channel, int temperature) const
{
    // do gyro temperature compensation
    if (channel < 3) {

        return  _gyro_temp_curve[channel][0] +
                _gyro_temp_curve[channel][1] * temperature +
                _gyro_temp_curve[channel][2] * temperature * temperature;
    }

    // do fixed-offset accelerometer compensation
    return 2025;    // XXX magic number!
}

float
AP_IMU_Oilpan::_sensor_in(uint8_t channel, int temperature)
{
    float   adc_in;

    // get the compensated sensor value
    //
    adc_in = _adc->Ch(_sensors[channel]) - _sensor_compensation(channel, temperature);

    // adjust for sensor sign and apply calibration offset
    //
    if (_sensor_signs[channel] < 0) {
        adc_in = _sensor_cal[channel] - adc_in;
    } else {
        adc_in = adc_in - _sensor_cal[channel];
    }

    // constrain sensor readings to the sensible range
    //
    if (fabs(adc_in) > _adc_constraint) {
        adc_constraints++;                                              // We keep track of the number of times
        adc_in = constrain(adc_in, -_adc_constraint, _adc_constraint);    // Throw out nonsensical values
    }
    return adc_in;
}


bool
AP_IMU_Oilpan::update(void)
{
	int tc_temp = _adc->Ch(_gyro_temp_ch);

	// convert corrected gyro readings to delta acceleration
	//
	_gyro.x = ToRad(_gyro_gain_x) * _sensor_in(0, tc_temp);
	_gyro.y = ToRad(_gyro_gain_y) * _sensor_in(1, tc_temp);
	_gyro.z = ToRad(_gyro_gain_z) * _sensor_in(2, tc_temp);

	// convert corrected accelerometer readings to acceleration
	//
	_accel.x = _accel_scale * _sensor_in(3, tc_temp);
	_accel.y = _accel_scale * _sensor_in(4, tc_temp);
	_accel.z = _accel_scale * _sensor_in(5, tc_temp);

	// always updated
	return true;
}
