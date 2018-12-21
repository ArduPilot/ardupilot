/*
 * AP_TCA9534ADevice.cpp
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include "AP_TCA9534A_Device.h"
#include <GCS_MAVLink/GCS.h>


#define TCA9534A_DEBUG 0
#if TCA9534A_DEBUG
#include <cstdio>
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#define error(fmt, args ...)
#endif

extern const AP_HAL::HAL &hal;

AP_TCA9534A_Device::AP_TCA9534A_Device() {
	this->i2c_bus_id = 0;  // seems to be the default I2C bus, although there are 1 and 2 as well
	this->i2c_address = 0x38; // default address of 0111000
	this->payload = 0;
}

AP_TCA9534A_Device::AP_TCA9534A_Device(uint8_t i2cBusId, uint8_t i2cAddress) {
    this->i2c_address = i2cAddress;
    this->i2c_bus_id = i2cBusId;
	this->payload = 0;
}

AP_TCA9534A_Device::~AP_TCA9534A_Device() {
}

bool AP_TCA9534A_Device::init() {
	hal.console->printf("Calling TCA9534A init method\n");

	_dev = hal.i2c_mgr->get_device(i2c_bus_id, i2c_address);

	if (!_dev) {
	    hal.console->printf("TCA9534A could not be found\n");
		return false;
	}
	hal.console->printf("TCA9534A was found\n");

	// Enable automatic conversion mode
	uint8_t all_write = 0xFF; // all write
	_dev->write_register(REGISTER_CONFIGURATION,all_write,true);

	return true;
}

void AP_TCA9534A_Device::set_payload(uint8_t payload_in) {
    this->payload = payload_in;
    _dev->write_register(REGISTER_OUTPUT,payload_in,true);
    update();
}

uint8_t AP_TCA9534A_Device::get_payload()
{
//    _dev->read_registers(REGISTER_INPUT, &payload, sizeof(payload));
    _dev->read_registers(REGISTER_OUTPUT, &payload, sizeof(payload));
    return this->payload;
}

void AP_TCA9534A_Device::update() {
    bool sightline5VOn = (payload & 0x01) ==0x01;
    bool externalPayload12VOn=(payload & 0x02) == 0x02;
    bool sparePowerConnector5VOn= (payload & 0x04) == 0x04;
    bool internalPayload12VOn  = (payload & 0x08) == 0x08;
    bool unusedOn = (payload & 0x10) == 0x10;
    bool lna5VOn = (payload & 0x20) == 0x20;
    bool rfSwitchPositionControl = (payload & 0x40) == 0x40;
    bool rfSwitchPosition = (payload & 0x80) == 0x80;
	gcs().send_text(MAV_SEVERITY_INFO,"sightline5VOn = %s",sightline5VOn ? "true":"false");
	gcs().send_text(MAV_SEVERITY_INFO,"externalPayload12VOn =%s", externalPayload12VOn ? "true":"false");
	gcs().send_text(MAV_SEVERITY_INFO,"sparePowerConnector5VOn = %s", sparePowerConnector5VOn ? "true":"false");
	gcs().send_text(MAV_SEVERITY_INFO,"internalPayload12VOn = %s", internalPayload12VOn ? "true":"false");
	gcs().send_text(MAV_SEVERITY_INFO,"unusedOn = %s",unusedOn ? "true":"false");
	gcs().send_text(MAV_SEVERITY_INFO,"lna5VOn = %s", lna5VOn ? "true":"false");
	gcs().send_text(MAV_SEVERITY_INFO,"rfSwitchPositionControl = %s", rfSwitchPositionControl ? "true":"false");
	gcs().send_text(MAV_SEVERITY_INFO,"rfSwitchPosition = %s", rfSwitchPosition ? "true":"false");
	gcs().send_text(MAV_SEVERITY_INFO,"new payload setting = %u", get_payload());
//	_dev->write_register(REGISTER_OUTPUT,payload,true);
}
