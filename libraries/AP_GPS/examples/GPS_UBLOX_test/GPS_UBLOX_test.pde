// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
/*
 *       Example of GPS UBlox library.
 *       Code by Jordi Munoz and Jose Julio. DIYDrones.com
 *
 *       Works with Ardupilot Mega Hardware (GPS on Serial Port1)
 */

#include <stdlib.h>
#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_GPS.h>
#include <AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

AP_GPS_UBLOX gps(hal.uart1);
#define T6 1000000
#define T7 10000000

void setup()
{
    hal.uart1->begin(38400);
    gps.print_errors = true;

    hal.uart0->println("GPS UBLOX library test");
    gps.init(GPS::GPS_ENGINE_AIRBORNE_2G);       // GPS Initialization
}

void loop()
{
    hal.scheduler->delay(20);
    gps.update();
    if (gps.new_data) {
        hal.uart0->print("gps:");
        hal.uart0->print(" Lat:");
        hal.uart0->print((float)gps.latitude / T7, DEC);
        hal.uart0->print(" Lon:");
        hal.uart0->print((float)gps.longitude / T7, DEC);
        hal.uart0->print(" Alt:");
        hal.uart0->print((float)gps.altitude / 100.0, DEC);
        hal.uart0->print(" GSP:");
        hal.uart0->print(gps.ground_speed / 100.0);
        hal.uart0->print(" COG:");
        hal.uart0->print(gps.ground_course / 100.0, DEC);
        hal.uart0->printf(" VEL: %.2f %.2f %.2f",
                      gps.velocity_north(),
                      gps.velocity_east(),
                      sqrt(sq(gps.velocity_north())+sq(gps.velocity_east())));
        hal.uart0->print(" SAT:");
        hal.uart0->print(gps.num_sats, DEC);
        hal.uart0->print(" FIX:");
        hal.uart0->print(gps.fix, DEC);
        hal.uart0->print(" TIM:");
        hal.uart0->print(gps.time, DEC);
        hal.uart0->println();
        gps.new_data = 0; // We have readed the data
    }
}

AP_HAL_MAIN();
