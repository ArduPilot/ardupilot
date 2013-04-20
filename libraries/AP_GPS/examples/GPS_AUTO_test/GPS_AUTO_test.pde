// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// Test for AP_GPS_AUTO
//

#include <stdlib.h>

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_GPS.h>
#include <AP_Math.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

GPS         *gps;
AP_GPS_Auto GPS(&gps);

#define T6 1000000
#define T7 10000000

void setup()
{
    hal.uartB->begin(38400);

    hal.console->println("GPS AUTO library test");
    gps = &GPS;
    gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_2G);
}

void loop()
{
    gps->update();
    if (gps->new_data) {
        if (gps->fix) {
            hal.console->print("Lat: ");
            print_latlon(hal.console,gps->latitude);
            hal.console->print(" Lon: ");
            print_latlon(hal.console,gps->longitude);
            hal.console->printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %lu STATUS: %u\n",
                          (float)gps->altitude / 100.0,
                          (float)gps->ground_speed / 100.0,
                          (int)gps->ground_course / 100,
                          gps->num_sats,
                          gps->time,
                          gps->status());
        } else {
            hal.console->println("No fix");
        }
        gps->new_data = false;
    }
}

AP_HAL_MAIN();
