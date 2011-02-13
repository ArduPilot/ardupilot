// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
//
//	AP_IMU.cpp - IMU Sensor Library for Ardupilot Mega
//		Code by Doug Weibel, Jordi Muñoz and Jose Julio. DIYDrones.com
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

#define A_LED_PIN 37			//37 = A,	35 = C
#define C_LED_PIN 35

// ADC : Voltage reference 3.3v / 12bits(4096 steps) => 0.8mV/ADC step
// ADXL335 Sensitivity(from datasheet) => 330mV/g, 0.8mV/ADC step => 330/0.8 = 412
// Tested value : 418
#define GRAVITY 		418.0 				// 1G in the raw data coming from the accelerometer
#define accel_scale(x)	(x*9.80665/GRAVITY)	// Scaling the raw data of the accel to actual acceleration in m/s/s

// IDG500 Sensitivity (from datasheet) => 2.0mV/º/s, 0.8mV/ADC step => 0.8/3.33 = 0.4
// Tested values : 0.4026, ?, 0.4192
#define _gyro_gain_x	0.4					//X axis Gyro gain
#define _gyro_gain_y	0.41 				//Y axis Gyro gain
#define _gyro_gain_z	0.41				//Z axis Gyro

#define ADC_CONSTRAINT	900

// Sensors: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
const uint8_t AP_IMU_Oilpan::_sensors[6]       = { 1, 2, 0, 4, 5, 6};	// For ArduPilot Mega Sensor Shield Hardware
const int8_t  AP_IMU_Oilpan::_sensor_signs[6]	= {	1,-1,-1, 1,-1,-1};

// Temp compensation curve constants
// These must be produced by measuring data and curve fitting
// [X/Y/Z gyro][A/B/C or 0 order/1st order/2nd order constants]
const float   AP_IMU_Oilpan::_gyro_temp_curve[3][3] = {
	{1665,0,0},
	{1665,0,0},
	{1665,0,0}
};	// To Do - make additional constructors to pass this in.

void
AP_IMU_Oilpan::init(Start_style style)
{
	init_gyro(style);
	init_accel(style);
}

/**************************************************/

void
AP_IMU_Oilpan::init_gyro(Start_style style)
{
	float temp;
	int flashcount = 0;
	int tc_temp;
	float adc_in[6];

	// warm start, load saved cal from EEPROM
	if ((WARM_START == style) && (0 != _address)) {
		_adc_offset[0] = read_EE_float(_address );
		_adc_offset[1] = read_EE_float(_address + 4);
		_adc_offset[2] = read_EE_float(_address + 8);
		return;
	}

	// cold start
	tc_temp = _adc->Ch(_gyro_temp_ch);
 	delay(500);
	Serial.println("Init Gyro");

	for(int c = 0; c < 25; c++){				// Mostly we are just flashing the LED's here to tell the user to keep the IMU still
		digitalWrite(A_LED_PIN, LOW);
		digitalWrite(C_LED_PIN, HIGH);
		delay(20);

		for (int i = 0; i < 6; i++)
			adc_in[i] = _adc->Ch(_sensors[i]);
			
		digitalWrite(A_LED_PIN, HIGH);
		digitalWrite(C_LED_PIN, LOW);
		delay(20);
	}

	for (int j = 0; j <= 2; j++){
		adc_in[j] -= _gyro_temp_comp(j, tc_temp);
		_adc_offset[j]	= adc_in[j];
	}

	for(int i = 0; i < 50; i++){
		for (int j = 0; j <= 2; j++){
			adc_in[j] = _adc->Ch(_sensors[j]);
			// Subtract temp compensated typical gyro bias
			adc_in[j] -= _gyro_temp_comp(j, tc_temp);
			// filter
			_adc_offset[j] = _adc_offset[j] * 0.9 + adc_in[j] * 0.1;
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

	_save_gyro_cal();
}


void
AP_IMU_Oilpan::init_accel(Start_style style) // 3, 4, 5
{
	float temp;
	int flashcount = 0;
	float adc_in[6];

	// warm start, load our saved cal from EEPROM
	if ((WARM_START == style) && (0 != _address)) {
		_adc_offset[3] = read_EE_float(_address + 12);
		_adc_offset[4] = read_EE_float(_address + 16);
		_adc_offset[5] = read_EE_float(_address + 20);
		return;
	}

	// cold start
 	delay(500);

	Serial.println("Init Accel");

	for (int j = 3; j <= 5; j++){
		adc_in[j] 		= _adc->Ch(_sensors[j]);
		adc_in[j] 		-= 2025;		// Typical accel bias value - subtracted in _accel_in() and update()
		_adc_offset[j]	= adc_in[j];
	}

	for(int i = 0; i < 50; i++){		// We take some readings...

		delay(20);

		for (int j = 3; j <= 5; j++){
			adc_in[j] 		= _adc->Ch(_sensors[j]);
			adc_in[j] 		-= 2025;		// Typical accel bias value - subtracted in _accel_in() and update()
			_adc_offset[j]	= _adc_offset[j] * 0.9 + adc_in[j] * 0.1;
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
	_adc_offset[5] += GRAVITY * _sensor_signs[5];

	_save_accel_cal();
}

void
AP_IMU_Oilpan::zero_accel(void) // 3, 4, 5
{
	_adc_offset[3] = 0;
	_adc_offset[4] = 0;
	_adc_offset[5] = 0;
	_save_accel_cal();
}

void
AP_IMU_Oilpan::_save_gyro_cal(void)
{
	// save cal to EEPROM for warm start
	if (0 != _address) {
		write_EE_float(_adc_offset[0], _address);
		write_EE_float(_adc_offset[1], _address + 4);
		write_EE_float(_adc_offset[2], _address + 8);
	}
}

void
AP_IMU_Oilpan::_save_accel_cal(void)
{
	// save cal to EEPROM for warm start
	if (0 != _address) {
		write_EE_float(_adc_offset[3], _address + 12);
		write_EE_float(_adc_offset[4], _address + 16);
		write_EE_float(_adc_offset[5], _address + 20);
	}
}

/**************************************************/
// Returns the temperature compensated raw gyro value
//---------------------------------------------------
float
AP_IMU_Oilpan::_gyro_temp_comp(int i, int temp) const
{
	// We use a 2nd order curve of the form Gtc = A + B * Graw + C * (Graw)**2
	//------------------------------------------------------------------------
	return _gyro_temp_curve[i][0] + _gyro_temp_curve[i][1] * temp + _gyro_temp_curve[i][2] * temp * temp;
}

float
AP_IMU_Oilpan::_gyro_in(uint8_t channel, int temperature)
{
	float	adc_in;

	adc_in = _adc->Ch(_sensors[channel]);
	adc_in -= _gyro_temp_comp(channel, temperature);		// Subtract temp compensated typical gyro bias
	if (_sensor_signs[channel] < 0) {
		adc_in = _adc_offset[channel] - adc_in;
	} else {
		adc_in = adc_in - _adc_offset[channel];
	}

	if (fabs(adc_in) > ADC_CONSTRAINT) {
		adc_constraints++; 												// We keep track of the number of times
		adc_in = constrain(adc_in, -ADC_CONSTRAINT, ADC_CONSTRAINT);	// Throw out nonsensical values
	}
	return adc_in;
}

float
AP_IMU_Oilpan::_accel_in(uint8_t channel)
{
	float	adc_in;

	adc_in = _adc->Ch(_sensors[channel]);
	adc_in -= 2025;										// Subtract typical accel bias

	if (_sensor_signs[channel] < 0) {
		adc_in = _adc_offset[channel] - adc_in;
	} else {
		adc_in = adc_in - _adc_offset[channel];
	}

	if (fabs(adc_in) > ADC_CONSTRAINT) {
		adc_constraints++; 												// We keep track of the number of times
		adc_in = constrain(adc_in, -ADC_CONSTRAINT, ADC_CONSTRAINT);	// Throw out nonsensical values
	}
	return adc_in;
}

bool
AP_IMU_Oilpan::update(void)
{
	int tc_temp = _adc->Ch(_gyro_temp_ch);
	float adc_in[6];
#if 0
	// get current gyro readings
	for (int i = 0; i < 3; i++) {
		adc_in[i] = _adc->Ch(_sensors[i]);
		adc_in[i] -= _gyro_temp_comp(i,tc_temp);		// Subtract temp compensated typical gyro bias
		if (_sensor_signs[i] < 0)
			adc_in[i] = (_adc_offset[i] - adc_in[i]);
		else
			adc_in[i] = (adc_in[i] - _adc_offset[i]);

		if (fabs(adc_in[i]) > ADC_CONSTRAINT) {
			adc_constraints++; 													// We keep track of the number of times
			adc_in[i] = constrain(adc_in[i], -ADC_CONSTRAINT, ADC_CONSTRAINT);	// Throw out nonsensical values
		}
	}
#endif
	_gyro.x = ToRad(_gyro_gain_x) * _gyro_in(0, tc_temp);
	_gyro.y = ToRad(_gyro_gain_y) * _gyro_in(1, tc_temp);
	_gyro.z = ToRad(_gyro_gain_z) * _gyro_in(2, tc_temp);
#if 0
	// get current accelerometer readings
	for (int i = 3; i < 6; i++) {
		adc_in[i] = _adc->Ch(_sensors[i]);
		adc_in[i] -= 2025;								// Subtract typical accel bias

		if (_sensor_signs[i] < 0)
			adc_in[i] = _adc_offset[i] - adc_in[i];
		else
			adc_in[i] = adc_in[i] - _adc_offset[i];

		if (fabs(adc_in[i]) > ADC_CONSTRAINT) {
			adc_constraints++; 													// We keep track of the number of times
			adc_in[i] = constrain(adc_in[i], -ADC_CONSTRAINT, ADC_CONSTRAINT);	// Throw out nonsensical values
		}
	}
#endif
	_accel.x = accel_scale(_accel_in(3));
	_accel.y = accel_scale(_accel_in(4));
	_accel.z = accel_scale(_accel_in(5));

	// always updated
	return true;
}

/********************************************************************************/

void
AP_IMU_Oilpan::print_accel_offsets(void)
{
	Serial.print("Accel offsets: ");
	Serial.print(_adc_offset[3], 2);
	Serial.print(", ");
	Serial.print(_adc_offset[4], 2);
	Serial.print(", ");
	Serial.println(_adc_offset[5], 2);
}

void
AP_IMU_Oilpan::print_gyro_offsets(void)
{
	Serial.print("Gyro offsets: ");
	Serial.print(_adc_offset[0], 2);
	Serial.print(", ");
	Serial.print(_adc_offset[1], 2);
	Serial.print(", ");
	Serial.println(_adc_offset[2], 2);
}

/********************************************************************************/

float
AP_IMU_Oilpan::read_EE_float(int address)
{
	union {
		byte bytes[4];
		float value;
	} _floatOut;

	for (int i = 0; i < 4; i++)
		_floatOut.bytes[i] = eeprom_read_byte((uint8_t *) (address + i));
	return _floatOut.value;
}

void
AP_IMU_Oilpan::write_EE_float(float value, int address)
{
	union {
		byte bytes[4];
		float value;
	} _floatIn;

	_floatIn.value = value;
	for (int i = 0; i < 4; i++)
		eeprom_write_byte((uint8_t *) (address + i), _floatIn.bytes[i]);
}

