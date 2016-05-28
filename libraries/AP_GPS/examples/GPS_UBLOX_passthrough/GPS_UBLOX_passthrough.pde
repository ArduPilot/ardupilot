// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       GPS UBlox passthrough sketch
 *       Code by DIYDrones.com
 *
 *       Works with APM2.x
 */

#include <stdlib.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
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
#include <AP_NavEKF.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void setup()
{
    // initialise console uart to 38400 baud
    hal.console->begin(38400);

    // initialise gps uart to 38400 baud
    hal.uartB->begin(38400);
}

void loop()
{
    // send characters received from the console to the GPS
    while (hal.console->available()) {
        hal.uartB->write(hal.console->read());
    }
    // send GPS characters to the console
    while (hal.uartB->available()) {
        hal.console->write(hal.uartB->read());
    }
}

AP_HAL_MAIN();
