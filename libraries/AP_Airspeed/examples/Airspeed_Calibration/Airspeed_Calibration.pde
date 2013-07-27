/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *   Airspeed_Calibration.pde - airspeed example sketch
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_PX4.h>
#include <AP_Math.h>
#include <Filter.h>
#include <AP_ADC.h>
#include <SITL.h>
#include <AP_Compass.h>
#include <AP_Baro.h>
#include <GCS_MAVLink.h>
#include <AP_Declination.h> // ArduPilot Mega Declination Helper Library
#include <AP_Airspeed.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static Airspeed_Calibration acal;

void setup()
{
    hal.console->println("Airspeed Calibration library test");
}

void loop(void)
{
    uint32_t tstart = hal.scheduler->micros();
    acal.update(15, Vector3f(10,20,13));
    hal.console->printf_P(PSTR("update took %u usec\n"),
                          hal.scheduler->micros() - tstart);
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
