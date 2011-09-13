// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// Test for AP_GPS_AUTO
//

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_GPS.h>

FastSerialPort0(Serial);
FastSerialPort1(Serial1);

GPS	    *gps;
AP_GPS_Auto GPS(&Serial1, &gps);

#define T6 1000000
#define T7 10000000

void setup()
{
	Serial.begin(115200);
	Serial1.begin(38400);

	Serial.println("GPS AUTO library test");
	gps = &GPS;
	gps->init();
}

void loop()
{
	gps->update();
	if (gps->new_data){
		if (gps->fix) {
			Serial.printf("\nLat: %.7f Lon: %.7f Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %lu",
						  (float)gps->latitude / T7,
						  (float)gps->longitude / T7,
						  (float)gps->altitude / 100.0,
						  (float)gps->ground_speed / 100.0,
						  (int)gps->ground_course / 100,
						  gps->num_sats,
						  gps->time);
		} else {
			Serial.println("No fix");
		}
		gps->new_data = false;
	}
}

