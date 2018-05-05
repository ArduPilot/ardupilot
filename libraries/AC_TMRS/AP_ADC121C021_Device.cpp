/*
 * AP_ADC121C021Device.cpp
 *
 *  Created on: Apr 10, 2018
 *      Author: Rob
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include "AP_ADC121C021_Device.h"

#define ADC121C021_I2C_BUS 1
#define ADC121C021_I2C_ADDR 0x50

#define ADC121C021_DEBUG 0
#if ADC121C021_DEBUG
#include <cstdio>
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#define error(fmt, args ...)
#endif

extern const AP_HAL::HAL &hal;

AP_ADC121C021_Device::AP_ADC121C021_Device() {

}

AP_ADC121C021_Device::~AP_ADC121C021_Device() {

}

bool AP_ADC121C021_Device::init() {

	_dev = hal.i2c_mgr->get_device(ADC121C021_I2C_BUS, ADC121C021_I2C_ADDR);
	if (!_dev) {
		return false;
	}

	// Enable automatic conversion mode
	_dev->write_register(0x02, 0x20, true);

	_dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_ADC121C021_Device::_update, void));

	return true;
}

void AP_ADC121C021_Device::_update() {
	uint8_t data[2];
//	be16_t val;

	if (!_dev->read_registers(0x00, data, sizeof(data))) {
		error("_dev->read_registers failed in ADC121C021");
		return;
	}

//	unsigned int raw_adc = ((data[0] & 0x0F) << 8 | (data[1] & 0xFF));
	uint16_t raw_adc = ((data[0] & 0x0F) * 128 + (data[1] & 0xFF));

}
