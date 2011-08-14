// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	PID.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#ifndef Trig_LUT_h
#define Trig_LUT_h

#include <stdlib.h>
#include <avr/pgmspace.h>


/// @class	Trig_LUT
/// @brief	Simple look up table for Sin and Cos
class Trig_LUT {
public:

	Trig_LUT(){};

	float 		sin_lut(int16_t angle);
	float 		cos_lut(int16_t angle);

};

#endif

