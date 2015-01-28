// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_GPS.h>
#include <DataFlash.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <GCS_MAVLink.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Math.h>
#include <AP_Notify.h>
#include <AP_BoardLED.h>
#include <AP_NavEKF.h>
#include <AP_Rally.h>
#include <AP_Scheduler.h>
#include <AP_BattMonitor.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// create board led object
AP_BoardLED board_led;

AP_GPS gps;

#define T6 1000000
#define T7 10000000

void setup()
{
    hal.console->println("GPS AUTO library test");

    // initialise the leds
    board_led.init();
}

void loop()
{
    static uint32_t last_msg_ms;
    gps.update();
    if (last_msg_ms != gps.last_message_time_ms()) {
        last_msg_ms = gps.last_message_time_ms();
        const Location &loc = gps.location();
        hal.console->print("Lat: ");
        print_latlon(hal.console, loc.lat);
        hal.console->print(" Lon: ");
        print_latlon(hal.console, loc.lng);
        hal.console->printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %u/%lu STATUS: %u\n",
                            loc.alt * 0.01f,
                            gps.ground_speed(),
                            (int)gps.ground_course_cd() / 100,
                            gps.num_sats(),
                            gps.time_week(),
                            (unsigned long)gps.time_week_ms(),
                            gps.status());
    }
    hal.scheduler->delay(10);
}

AP_HAL_MAIN();
