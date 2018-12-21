/*
 * AP_ADS1013Device.cpp
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include "AP_ADS1013_Device.h"
#include <GCS_MAVLink/GCS.h>


#define ADS1013_DEBUG 0
#if ADS1013_DEBUG
#include <cstdio>
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#define error(fmt, args ...)
#endif

extern const AP_HAL::HAL &hal;

AP_ADS1013_Device::AP_ADS1013_Device() {
	this->i2c_bus_id = 0;  // seems to be the default I2C bus, although there are 1 and 2 as well
	this->i2c_address = 0x48; // default address
	this->scale_factor =  10000.0/4096.0;
}

AP_ADS1013_Device::AP_ADS1013_Device(uint8_t i2cBusId, uint8_t i2cAddress, float reference_quantity) {
    this->i2c_address = i2cAddress;
    this->i2c_bus_id = i2cBusId;
	this->scale_factor = reference_quantity/4096.0;
}

AP_ADS1013_Device::~AP_ADS1013_Device() {
//    _dev->unregister_callback(FUNCTOR_BIND_MEMBER(&AP_ADS1013_Device::_update, void));
}

bool AP_ADS1013_Device::init() {
	hal.console->printf("Calling ADS1013 init method\n");

	_dev = hal.i2c_mgr->get_device(i2c_bus_id, i2c_address);
//    hal.console->printf("load_all took %uus\n", (unsigned)(micros() - before));

	if (!_dev) {
	    hal.console->printf("ADS1013 could not be found\n");
		return false;
	}
	hal.console->printf("ADS1013 was found\n");

	// Enable automatic conversion mode
	_dev->write_register(0x02, 0x20, true);

	_dev->register_periodic_callback(1000000, FUNCTOR_BIND_MEMBER(&AP_ADS1013_Device::_update, void));

	return true;
}

void AP_ADS1013_Device::_update() {
	uint8_t data[2];
//	be16_t val;

//	gcs().send_text(MAV_SEVERITY_INFO," ADS1013: bus address = %u",_dev->get_bus_address());

//	bool status = _dev->write_register(0x02, 0x20, true);
//	gcs().send_text(MAV_SEVERITY_INFO," ADS1013: conversion mode %B",status);

	if (!_dev->read_registers(0x00, data, sizeof(data))) {
		gcs().send_text(MAV_SEVERITY_ERROR,"_dev->read_registers failed d[0]=%u",data[0]);
//		error("_dev->read_registers failed in ADS1013");
		return;
	}

//	gcs().send_text(MAV_SEVERITY_INFO," ADS1013: read registers");

//	unsigned int raw_adc = ((data[0] & 0x0F) << 8 | (data[1] & 0xFF));

	uint16_t raw_adc = ((data[0] & 0x0F) * 256 + (data[1] & 0xFF));
	gcs().send_text(MAV_SEVERITY_INFO,"ADS1013: raw_adc = %u",raw_adc);
	float scaled_value = this->get_scaled_value(raw_adc);
	gcs().send_text(MAV_SEVERITY_INFO,"ADS1013: scaled_value = %f",scaled_value);
//	hal.console->printf("ADS1013: raw_adc = %u\n",raw_adc);

}

float AP_ADS1013_Device::get_scaled_value(uint16_t raw_adc) {
    float scaled_value = raw_adc * this->scale_factor;
    return scaled_value;
}
