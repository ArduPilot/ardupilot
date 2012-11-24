// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
/*
 *       Example of GPS 406 library.
 *       Code by Jordi Munoz and Jose Julio. DIYDrones.com
 *
 *       Works with Ardupilot Mega Hardware (GPS on Serial Port1)
 */

#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_GPS.h>

FastSerialPort0(Serial);
FastSerialPort1(Serial1);

AP_GPS_406 gps(&Serial1);
#define T6 1000000
#define T7 10000000

void setup()
{
    tone(A6, 1000, 200);

    Serial.begin(38400, 16, 128);
    Serial1.begin(57600, 128, 16);
    stderr = stdout;
    gps.print_errors = true;

    Serial.println("GPS 406 library test");
    gps.init();  // GPS Initialization
    delay(1000);
}
void loop()
{
    delay(20);
    gps.update();
    if (gps.new_data) {
        Serial.print("gps:");
        Serial.print(" Lat:");
        Serial.print((float)gps.latitude / T7, DEC);
        Serial.print(" Lon:");
        Serial.print((float)gps.longitude / T7, DEC);
        Serial.print(" Alt:");
        Serial.print((float)gps.altitude / 100.0, DEC);
        Serial.print(" GSP:");
        Serial.print(gps.ground_speed / 100.0);
        Serial.print(" COG:");
        Serial.print(gps.ground_course / 100, DEC);
        Serial.print(" SAT:");
        Serial.print(gps.num_sats, DEC);
        Serial.print(" FIX:");
        Serial.print(gps.fix, DEC);
        Serial.print(" TIM:");
        Serial.print(gps.time, DEC);
        Serial.println();
        gps.new_data = 0; // We have readed the data
    }
}

