/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "wiring.h"
#include "AP_AnalogSource_Arduino.h"

float AP_AnalogSource_Arduino::read(void)
{
	float fullscale = analogRead(_pin);
	float scaled = _prescale * fullscale;
	return scaled;
}
