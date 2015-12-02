// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       GPS UBlox passthrough sketch
 *       Code by DIYDrones.com
 */

#include <stdlib.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <DataFlash/DataFlash.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_ADC/AP_ADC.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Baro/AP_Baro.h>
#include <Filter/Filter.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_Rally/AP_Rally.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

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
