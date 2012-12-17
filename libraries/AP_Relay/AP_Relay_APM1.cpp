#include <avr/io.h>
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "wiring.h"
#endif

#include "AP_Relay_APM1.h"

void AP_Relay_APM1::on()
{
	PORTL |= B00000100;
}

void AP_Relay_APM1::off()
{
	PORTL &= ~B00000100;
}

void AP_Relay_APM1::toggle()
{
	PORTL ^= B00000100;
}

void AP_Relay_APM1::set(bool status)
{
	if (status)
		on();
	else
		off();
}

bool AP_Relay_APM1::get()
{
	return PORTL & B00000100;
}
