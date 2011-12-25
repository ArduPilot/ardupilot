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

// print_latlon - prints an latitude or longitude value held in an int32_t 
// probably this should be moved to AP_Common
void print_latlon(BetterStream *s, int32_t lat_or_lon)
{
    int32_t dec_portion = lat_or_lon / T7;
	int32_t frac_portion = labs(lat_or_lon - dec_portion*T7);
	s->printf("%ld.%07ld",dec_portion,frac_portion);
}

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
    if (gps->new_data) {
        if (gps->fix) {
            Serial.print("\nLat: ");
            print_latlon(&Serial,gps->latitude);
            Serial.print(" Lon: ");
            print_latlon(&Serial,gps->longitude);
            Serial.printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %lu",
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

