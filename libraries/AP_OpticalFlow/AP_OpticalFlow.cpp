/*
	ADC.cpp - Analog Digital Converter Base Class for Ardupilot Mega
	Code by James Goppert. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.
*/

#include "AP_OpticalFlow.h"

AP_OpticalFlow::AP_OpticalFlow() : x(0),y(0),surface_quality(0),dx(0),dy(0)
{
}

// init - initCommAPI parameter controls whether I2C/SPI interface is initialised (set to false if other devices are on the I2C/SPI bus and have already initialised the interface)
void AP_OpticalFlow::init(boolean initCommAPI)
{
}

// read latest values from sensor and fill in x,y and totals
int AP_OpticalFlow::read()
{
}

// reads a value from the sensor (will be sensor specific)
byte AP_OpticalFlow::read_register(byte address)
{
}

// writes a value to one of the sensor's register (will be sensor specific)
void AP_OpticalFlow::write_register(byte address, byte value)
{
}