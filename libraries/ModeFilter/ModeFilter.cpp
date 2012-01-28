/*
	ModeFilter.cpp - Mode Filter Library for Ardupilot Mega. Arduino
	Code by Jason Short. DIYDrones.com
	Adapted from code by Jason Lessels(June 6, 2011), Bill Gentles (Nov. 12, 2010)


	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.


*/
#include "ModeFilter.h"

#include <avr/interrupt.h>
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


// Constructors ////////////////////////////////////////////////////////////////

ModeFilter::ModeFilter() :
	_sample_index(0)
{
}

// Public Methods //////////////////////////////////////////////////////////////
	//Sorting function
	// sort function (Author: Bill Gentles, Nov. 12, 2010)
	//	*a is an array pointer function

int ModeFilter::get_filtered_with_sample(int _sample){
	_samples[_sample_index] = _sample;

	_sample_index++;

	if (_sample_index >= MOD_FILTER_SIZE)
		_sample_index = 0;

	isort();

	return mode();
}


void ModeFilter::isort()
{
	for (int i = 1; i < MOD_FILTER_SIZE; ++i)   {
		int j = _samples[i];
		int k;
		for (k = i - 1; (k >= 0) && (j < _samples[k]); k--){
			_samples[k + 1] = _samples[k];
		}
		_samples[k + 1] = j;
	}
}

//Mode function, returning the mode or median.
int16_t ModeFilter::mode(){
	int fmode 		= 0;
	byte i 			= 0;
	byte count 		= 0;
	byte maxCount 	= 0;
	byte bimodal 	= 0;

	while(count > maxCount){
		fmode 		= _samples[i];
		maxCount 	= count;
		bimodal 	= 0;
	}

	if(count == 0) i++;

	if(count == maxCount){ //If the dataset has 2 or more modes.
		bimodal = 1;
	}

	if(fmode == 0 || bimodal == 1){ //Return the median if there is no mode.
		fmode = _samples[(MOD_FILTER_SIZE / 2)];
	}

	return fmode;
}
