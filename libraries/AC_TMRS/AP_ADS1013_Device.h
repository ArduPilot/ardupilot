#pragma once
/*
 * AP_ADS1013Device.h
 *
 *  Created on: Apr 10, 2018
 *      Author: Rob
 */

#ifndef AP_ADS1013DEVICE_H_
#define AP_ADS1013DEVICE_H_

#include <inttypes.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

//struct adc_report {
//	uint8_t id;
//	float data;
//};

/**
11. EXTERNAL PAYLOAD CURRENT MONITOR
11.1.   ADS1013
11.2.   Address 1001000 // 0x48
11.3.   12bits
11.4.   I = 10A/4096 = 2.44mA/bit

12. INTERNAL PAYLOAD CURRENT MONITOR
12.1.   ADS1013
12.2.   Address 1001001 // 0x49
12.3.   12bits
12.4.   I = 10A/4096 = 2.44mA/bit
*/


class AP_ADS1013_Device {

public:
	AP_ADS1013_Device();
	AP_ADS1013_Device(uint8_t i2cBus, uint8_t i2cAddress, float reference_quantity);

	virtual ~AP_ADS1013_Device();

	bool init();

//	size_t read(adc_report_s *report, size_t length) const;

//	uint8_t get_channels_number() const {
//		return _channels_number;
//	}

private:
	static const uint8_t _channels_number;
	uint8_t i2c_bus_id;
	uint8_t i2c_address;
	float scale_factor;

	AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

//	adc_report_s *_samples;

	void _update();
	float get_scaled_value(uint16_t raw_adc);
};

#endif /* AP_ADS1013DEVICE_H_ */
