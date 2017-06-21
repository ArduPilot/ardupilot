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
#include <AP_BoardConfig/AP_BoardConfig.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

float temperature;

AP_Airspeed airspeed;

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

void setup()
{
    hal.console->printf("ArduPilot Airspeed library test\n");

    set_object_value(&airspeed, airspeed.var_info, "PIN", 65);
    set_object_value(&airspeed, airspeed.var_info, "ENABLE", 1);
    set_object_value(&airspeed, airspeed.var_info, "USE", 1);

    AP_BoardConfig{}.init();

    airspeed.init();
    airspeed.calibrate(false);
}

void loop(void)
{
    static uint32_t timer;
    if ((AP_HAL::millis() - timer) > 100) {
        timer = AP_HAL::millis();
        airspeed.read();
        airspeed.get_temperature(temperature);

        hal.console->printf("airspeed %5.2f temperature %6.2f healthy = %u\n",
                            (double)airspeed.get_airspeed(), (double)temperature, airspeed.healthy());
    }
    hal.scheduler->delay(1);
}

AP_HAL_MAIN();
