/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// AVR runtime
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Declination.h>

FastSerialPort(Serial, 0);

void setup(void)
{
	float declination;

	Serial.begin(115200);

    declination = AP_Declination::get_declination(43.064191, -87.997498);
	Serial.printf("declination: %f\n", declination);
}

void loop(void)
{
}
