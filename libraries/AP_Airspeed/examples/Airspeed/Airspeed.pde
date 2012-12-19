/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *   Airspeed.pde - airspeed example sketch
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License
 *   as published by the Free Software Foundation; either version 2.1
 *   of the License, or (at your option) any later version.
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <Filter.h>
#include <AP_Buffer.h>
#include <AP_Airspeed.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_Airspeed airspeed;

void setup()
{
    hal.console->println("ArduPilot Airspeed library test");

    airspeed.init(hal.analogin->channel(0));
    airspeed.calibrate();
}

void loop(void)
{
    static uint32_t timer;
    if((hal.scheduler->millis() - timer) > 100) {
        timer = hal.scheduler->millis();
        airspeed.read();
        hal.console->printf("airspeed %.2f\n", airspeed.get_airspeed());
    }
}

AP_HAL_MAIN();
