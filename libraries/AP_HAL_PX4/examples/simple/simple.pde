/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  simple hello world sketch
  Andrew Tridgell September 2011
*/

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>
#include <Filter.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_GPS.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Param.h>
#include <AP_Baro.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <SITL.h>
#include <AP_Notify.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

void setup() {
	hal.console->println_P(PSTR("hello world"));
}

void loop()
{
	hal.scheduler->delay(1000);
	hal.console->println("*");
}

AP_HAL_MAIN();
