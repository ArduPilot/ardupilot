// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	Trig_LUT.cpp
/// @brief	Generic Sin and Cos lookup table


#include "Trig_LUT.h"
static const uint16_t 	pgm_sinLUT[91] PROGMEM = {0, 17, 35, 52, 70, 87, 105, 122, 139, 156, 174, 191, 208, 225, 242, 259, 276, 292, 309, 326, 342, 358, 375, 391, 407, 423, 438, 454, 469, 485, 500, 515, 530, 545, 559, 574, 588, 602, 616, 629, 643, 656, 669, 682, 695, 707, 719, 731, 743, 755, 766, 777, 788, 799, 809, 819, 829, 839, 848, 857, 866, 875, 883, 891, 899, 906, 914, 921, 927, 934, 940, 946, 951, 956, 961, 966, 970, 974, 978, 982, 985, 988, 990, 993, 995, 996, 998, 999, 999, 1000, 1000};

float
Trig_LUT::sin_lut(int16_t angle)
{
	int8_t m,n;
	int16_t _sin;

	if (angle < 0){
		m 		= -1;
		angle 	= -angle;
	}else{
		m 		= 1;
	}

	// 0 - 360 only
	angle %= 360;

	// check quadrant
	if (angle <= 90){
		n = 1; // first quadrant
	} else if ((angle > 90) && (angle <= 180)) {
		angle = 180 - angle;
		n = 1;	// second quadrant
	} else if ((angle > 180) && (angle <= 270)) {
		angle = angle - 180;
		n = -1; // third quadrant
	} else {
		angle = 360 - angle;
		n = -1;
	}		 // fourth quadrant

	// get lookup value
	_sin = pgm_read_word(&pgm_sinLUT[angle]);

	// calculate sinus value
	return (float)(_sin * m * n) / 1000;
}

float
Trig_LUT::cos_lut(int16_t angle)
{
	return (sin_lut(90 - angle));
}

