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

#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS_Dummy.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

float temperature;

// create airspeed object
AP_Airspeed airspeed;

static AP_BoardConfig board_config;

namespace {
// try to set the object value but provide diagnostic if it failed
void set_object_value(const void *object_pointer,
                      const struct AP_Param::GroupInfo *group_info,
                      const char *name, float value)
{
    if (!AP_Param::set_object_value(object_pointer, group_info, name, value)) {
        hal.console->printf("WARNING: AP_Param::set object value \"%s::%s\" Failed.\n",
                            group_info->name, name);
    }
}
}

// to be called only once on boot for initializing objects
void setup()
{
    hal.console->printf("ArduPilot Airspeed library test\n");

    // set airspeed pin to 65, enable and use to true
    set_object_value(&airspeed, airspeed.var_info, "PIN", 65);
    set_object_value(&airspeed, airspeed.var_info, "ENABLE", 1);
    set_object_value(&airspeed, airspeed.var_info, "USE", 1);

    board_config.init();

    // initialize airspeed
    airspeed.init();

    airspeed.calibrate(false);
}

// loop
void loop(void)
{
    static uint32_t timer;

    // run read() and get_temperature() in 10Hz
    if ((AP_HAL::millis() - timer) > 100) {

        // current system time in milliseconds
        timer = AP_HAL::millis();
        airspeed.update(false);
        airspeed.get_temperature(temperature);

        // print temperature and airspeed to console
        hal.console->printf("airspeed %5.2f temperature %6.2f healthy = %u\n",
                            (double)airspeed.get_airspeed(), (double)temperature, airspeed.healthy());
    }
    hal.scheduler->delay(1);
}

const struct AP_Param::GroupInfo        GCS_MAVLINK::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
