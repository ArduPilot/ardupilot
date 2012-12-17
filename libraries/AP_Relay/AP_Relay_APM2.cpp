#include <avr/io.h>
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "wiring.h"
#endif

#include "AP_Relay_APM2.h"

void AP_Relay_APM2::on()
{
	PORTB |= B10000000;
}

void AP_Relay_APM2::off()
{
	PORTB &= ~B10000000;
}

void AP_Relay_APM2::toggle()
{
	PORTB ^= B10000000;
}

void AP_Relay_APM2::set(bool status)
{
	if (status)
		on();
	else
		off();
}

bool AP_Relay_APM2::get()
{
	return PORTB & B10000000;
}