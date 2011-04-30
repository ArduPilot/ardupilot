#ifndef AP_OPTICALFLOW_H
#define AP_OPTICALFLOW_H

/*
	AP_OpticalFlow.cpp - OpticalFlow Base Class for Ardupilot Mega
	Code by Randy Mackay. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Methods:
		init()           : initializate sensor and library.
		read             : reads latest value from OpticalFlow and stores values in x,y, surface_quality parameter
		read_register()  : reads a value from the sensor (will be sensor specific)
		write_register() : writes a value to one of the sensor's register (will be sensor specific)

*/

#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include "WConstants.h"

// return value definition
#define OPTICALFLOW_FAIL 0
#define OPTICALFLOW_SUCCESS 1

class AP_OpticalFlow
{
  public: 
  	int x, y;    // total x,y position
	int dx, dy;  // change in x and y position
	int surface_quality;  // image quality (below 15 you really can't trust the x,y values returned)
  public:
	AP_OpticalFlow();  // Constructor
	virtual void init(boolean initCommAPI = true); // parameter controls whether I2C/SPI interface is initialised (set to false if other devices are on the I2C/SPI bus and have already initialised the interface)
    virtual int read(); // read latest values from sensor and fill in x,y and totals
	virtual byte read_register(byte address);
	virtual void write_register(byte address, byte value);
	
  private:
};

#include "AP_OpticalFlow_ADNS3080.h"

#endif
