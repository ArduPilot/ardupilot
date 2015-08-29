/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  simple hello world sketch
  Andrew Tridgell September 2011
*/

#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_Math/AP_Math.h>
#include <Filter/Filter.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_ADC/AP_ADC.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Param/AP_Param.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <SITL/SITL.h>
#include <DataFlash/DataFlash.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <AP_Notify/AP_Notify.h>

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
