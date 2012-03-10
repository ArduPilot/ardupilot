/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// AVR runtime
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <FastSerial.h>
#include <AP_Declination.h>

FastSerialPort(Serial, 0);

void setup(void)
{
	float declination = AP_Declination::get_declination(43.064191, -87.997498);
	Serial.printf("declination: %d", declination);
}

void loop(void)
{
}