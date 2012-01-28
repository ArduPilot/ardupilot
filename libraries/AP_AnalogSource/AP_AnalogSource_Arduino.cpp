/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "wiring.h"
#endif
#include "AP_AnalogSource_Arduino.h"

float AP_AnalogSource_Arduino::read(void)
{
	float fullscale = analogRead(_pin);
	float scaled = _prescale * fullscale;
	return scaled;
}
