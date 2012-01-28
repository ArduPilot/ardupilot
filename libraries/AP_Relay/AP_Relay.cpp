// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * AP_Relay.cpp
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

#include <avr/io.h>
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "wiring.h"
#endif

#include "AP_Relay.h"

void AP_Relay::on()
{
	PORTL |= B00000100;
}


void AP_Relay::off()
{
	PORTL &= ~B00000100;
}


void AP_Relay::toggle()
{
	PORTL ^= B00000100;
}


void AP_Relay::set(bool status)
{
	if (status)
		on();
	else
		off();
}


bool AP_Relay::get()
{
	return PORTL & B00000100;
}
