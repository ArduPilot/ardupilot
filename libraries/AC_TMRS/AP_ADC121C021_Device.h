#pragma once
/*
 * AP_ADC121C021Device.h
 *
 *  Created on: Apr 10, 2018
 *      Author: Rob
 */

#ifndef AP_ADC121C021DEVICE_H_
#define AP_ADC121C021DEVICE_H_

#include <inttypes.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <GCS_MAVLink/GCS.h>

//struct adc_report {
//	uint8_t id;
//	float data;
//};

class AP_ADC121C021_Device {

public:
	AP_ADC121C021_Device();
	AP_ADC121C021_Device(uint8_t i2cBus, uint8_t i2cAddress, float reference_voltage);
	AP_ADC121C021_Device(uint8_t i2cBusId, uint8_t i2cAddress, float reference_value, bool bidirectional_type);

	virtual ~AP_ADC121C021_Device();

	bool init();

	float get_scaled_value();

	uint16_t get_bit_value();

//	size_t read(adc_report_s *report, size_t length) const;

//	uint8_t get_channels_number() const {
//		return _channels_number;
//	}

private:
	static const uint8_t _channels_number;
	uint8_t i2c_bus_id;
	uint8_t i2c_address;
	float scale_factor;
	uint16_t raw_bit_value = 0;
	uint16_t sample_count = 0;
	float scaled_value = 0;
	float scaled_value_m1 = 0;
	bool bidirectional;
	bool loaded;
	bool first;
	bool debug=true;

	AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

//	adc_report_s *_samples;

	void _update();
	float get_scaled_value(uint16_t raw_adc);
	float filter(float x_measured, float x_previous);
	void init_vars();

};

#endif /* AP_ADC121C021DEVICE_H_ */
