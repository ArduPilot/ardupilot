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

//struct adc_report {
//	uint8_t id;
//	float data;
//};

class AP_ADC121C021_Device {

public:
	AP_ADC121C021_Device();

	virtual ~AP_ADC121C021_Device();

	bool init();

//	size_t read(adc_report_s *report, size_t length) const;

	uint8_t get_channels_number() const {
		return _channels_number;
	}

private:
	static const uint8_t _channels_number;

	AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

//	adc_report_s *_samples;

	void _update();

//    float _convert_register_data_to_mv(int16_t word) const;
};

#endif /* AP_ADC121C021DEVICE_H_ */
