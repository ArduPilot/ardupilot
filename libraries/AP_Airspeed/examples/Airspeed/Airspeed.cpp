/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *   Airspeed.cpp - airspeed example sketch
 *
 */

#include <AP_ADC/AP_ADC.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Vehicle::FixedWing aparm;

AP_Airspeed airspeed(aparm);

void setup()
{
    hal.console->println("ArduPilot Airspeed library test");

    AP_Param::set_object_value(&airspeed, airspeed.var_info, "_PIN", 65);
    AP_Param::set_object_value(&airspeed, airspeed.var_info, "_ENABLE", 1);
    AP_Param::set_object_value(&airspeed, airspeed.var_info, "_USE", 1);

    airspeed.init();
    airspeed.calibrate(false);
}

void loop(void)
{
    static uint32_t timer;
    if((AP_HAL::millis() - timer) > 100) {
        timer = AP_HAL::millis();
        airspeed.read();
        hal.console->printf("airspeed %.2f healthy=%u\n", airspeed.get_airspeed(), airspeed.healthy());
    }
    hal.scheduler->delay(1);
}

AP_HAL_MAIN();
