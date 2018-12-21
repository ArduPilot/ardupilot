/*
 * AP_ADC121C021Device.cpp
 *
 *  Created on: Apr 10, 2018
 *      Author: Rob
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include "AP_ADC121C021_Device.h"

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
/*
 * Class to read ADC121C021 I2C devices
 */

AP_ADC121C021_Device::AP_ADC121C021_Device()
{
    this->i2c_bus_id = 0; // seems to be the default I2C bus, although there are 1 and 2 as well
    this->i2c_address = 0x50; // default address
    this->scale_factor = 2.0 * 4.89 / 4096.0;
    this->bidirectional = false;
    this->init_vars();
}

AP_ADC121C021_Device::AP_ADC121C021_Device(uint8_t i2cBusId, uint8_t i2cAddress,
        float reference_value)
{
    this->i2c_address = i2cAddress;
    this->i2c_bus_id = i2cBusId;
    this->bidirectional = false;
    this->scale_factor = reference_value / 4096.0;
    this->init_vars();
}

AP_ADC121C021_Device::AP_ADC121C021_Device(uint8_t i2cBusId, uint8_t i2cAddress,
        float reference_value, bool bidirectional_type)
{
    this->bidirectional = bidirectional_type;
    this->i2c_address = i2cAddress;
    this->i2c_bus_id = i2cBusId;
    if (this->bidirectional)
    {
        this->scale_factor = reference_value / 2048.0;
    }
    else
    {
        this->scale_factor = reference_value / 4096.0;
    }
    this->init_vars();
}

AP_ADC121C021_Device::~AP_ADC121C021_Device()
{
//    _dev->unregister_callback(FUNCTOR_BIND_MEMBER(&AP_ADC121C021_Device::_update, void));
}

void AP_ADC121C021_Device::init_vars()
{
    this->raw_bit_value = 0;
    this->scaled_value = 0.0;
    this->scaled_value_m1 = 0.0;
    this->loaded = false;
    this->sample_count = 0;
    this->first = true;
}

bool AP_ADC121C021_Device::init()
{
    hal.console->printf("Calling ADC121C021 init method\n");

    _dev = hal.i2c_mgr->get_device(i2c_bus_id, i2c_address);
//    hal.console->printf("load_all took %uus\n", (unsigned)(micros() - before));

    if (!_dev)
    {
        hal.console->printf("ADC121C021 could not be found\n");
        return false;
    }

    // Enable automatic conversion mode
    bool status = _dev->write_register(0x02, 0x20, true);
    if (status)
    {
        hal.console->printf("ADC121C021 was found\n");
        this->loaded = true;
        _dev->register_periodic_callback(100000,
                FUNCTOR_BIND_MEMBER(&AP_ADC121C021_Device::_update, void));
        return true;
    }
    else
    {
        hal.console->printf("ADC121C021 not operable with ID=%X\n",
                i2c_address);
        return false;
    }
}

float AP_ADC121C021_Device::filter(float x_measured, float x_previous)
{
    float factor = .10; // dt/tau = sample_period_seconds/time_constant;
    float x_filtered = x_previous + factor * (x_measured - x_previous);
    return x_filtered;
}

void AP_ADC121C021_Device::_update()
{
    uint8_t data[2];
//	be16_t val;

//	bool status = _dev->write_register(0x02, 0x20, true);
//	gcs().send_text(MAV_SEVERITY_INFO," ADC121C021: conversion mode %B",status);

    if (!_dev->read_registers(0x00, data, sizeof(data)))
    {
        if (debug)
        {
            gcs().send_text(MAV_SEVERITY_ERROR,
                    "_dev->read_registers for ID=%X", this->i2c_address);
        }
//		error("_dev->read_registers failed in ADC121C021");
        return;
    }

//	this->raw_bit_value = ((data[0] & 0x0F) * 256 + (data[1] & 0xFF));
    this->raw_bit_value = ((data[0] & 0x0F) << 8 | (data[1] & 0xFF));
    float measured_value = this->get_scaled_value(this->raw_bit_value);
    if (this->bidirectional)
    {
        measured_value = measured_value - this->get_scaled_value(2048);
    }
    float filtered_value = measured_value;
    if (!first)
    {
        filtered_value = filter(measured_value, this->scaled_value);
    }
    this->scaled_value = filtered_value;

//    sample_count++;
//    if(sample_count >= 5) {
//        gcs().send_text(MAV_SEVERITY_INFO, "ADC121C021: ID=%X, loaded=%d",this->i2c_address, this->loaded);
//        gcs().send_text(MAV_SEVERITY_INFO, "ADC121C021: ID=%X, raw_adc = %u", this->i2c_address, this->raw_bit_value);
//        gcs().send_text(MAV_SEVERITY_INFO, "ADC121C021: ID=%X, scaled_value = %f", this->i2c_address, this->scaled_value);
//        sample_count = 0;
//    }
//    this->scaled_value_m1 = this->scaled_value;
    first = false;

}

float AP_ADC121C021_Device::get_scaled_value(uint16_t raw_adc)
{
    float sv = raw_adc * this->scale_factor;
    return sv;
}

float AP_ADC121C021_Device::get_scaled_value()
{
    return this->scaled_value;
}

uint16_t AP_ADC121C021_Device::get_bit_value()
{
    return this->raw_bit_value;
}
