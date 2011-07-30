#ifndef AP_ADC_H
#define AP_ADC_H

/*
	AP_ADC.cpp - Analog Digital Converter Base Class for Ardupilot Mega
	Code by James Goppert. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Methods:
		Init() : Initialization of ADC. (interrupts etc)
		Ch(ch_num) : Return the ADC channel value

*/

class AP_ADC
{
  public:
	AP_ADC() {};  // Constructor
	virtual void Init() {};
	virtual int Ch(unsigned char ch_num) = 0;
	virtual int Ch_raw(unsigned char ch_num) = 0;
  private:
};

#include "AP_ADC_ADS7844.h"
#include "AP_ADC_HIL.h"

#endif
