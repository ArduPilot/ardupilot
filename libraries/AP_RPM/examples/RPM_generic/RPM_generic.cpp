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
 *   RPM_generic.cpp - RPM library example sketch
 *
 */

#include <AP_RPM/AP_RPM.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_RPM RPM;

char sensor_state;

void setup()
{
    hal.console->println("APM RPM library test\n\n");
    RPM.init();

    hal.console->printf("Detected %u RPM sensors\n\n", RPM.num_sensors());
}

void loop(void)
{
    RPM.update();

    for (uint8_t ii = 0; ii<RPM.num_sensors(); ii++) {

        // Determine sensor state
        if (RPM.healthy(ii)) {
            // Healthy sensor
            sensor_state = 'h';
        } else if (RPM.enabled(ii)) {
            // Enabled but not healthy
            sensor_state = 'e';
        } else {
            // Not enabled, not healthy
            sensor_state = '-';
        }

        hal.console->printf("%u - (%c) RPM: %8.2f  Quality: %.2f  ",
                ii, sensor_state, RPM.get_rpm(ii), RPM.get_signal_quality(ii));

        if (ii+1<RPM.num_sensors()) {
            // Print a seperating bar if more sensors to process
            hal.console->printf("|  ");
        }

    }

    hal.scheduler->delay(100);

    hal.console->printf("\n");
}

AP_HAL_MAIN();
