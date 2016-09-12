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
 * AP_IRLock_PX4.cpp
 *
 *  Created on: Nov 16, 2014
 *      Author: MLandes
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "AP_IRLock_PX4.h"

#include <fcntl.h>
#include <unistd.h>
#include "drivers/drv_irlock.h"

extern const AP_HAL::HAL& hal;

AP_IRLock_PX4::AP_IRLock_PX4() :
    _fd(0)
{}

extern "C" int irlock_main(int, char **);

void AP_IRLock_PX4::init()
{
    if (!AP_BoardConfig::px4_start_driver(irlock_main, "irlock", "start")) {
        hal.console->printf("irlock driver start failed\n");
#if CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        if (!AP_BoardConfig::px4_start_driver(irlock_main, "irlock", "start -b 2")) {
            hal.console->printf("irlock driver start failed (bus2)\n");
        } else {
            // give it time to initialise
            hal.scheduler->delay(500);
        }
#endif
    } else {
        // give it time to initialise
        hal.scheduler->delay(500);
    }

    _fd = open(IRLOCK0_DEVICE_PATH, O_RDONLY);
    if (_fd < 0) {
        hal.console->printf("Unable to open " IRLOCK0_DEVICE_PATH "\n");
        return;
    }

    _flags.healthy = true;
}

// retrieve latest sensor data - returns true if new data is available
bool AP_IRLock_PX4::update()
{
    // return immediately if not healthy
    if (!_flags.healthy) {
        return false;
    }

    // read position of all objects
    bool new_data = false;
    struct irlock_s report;
    while(::read(_fd, &report, sizeof(struct irlock_s)) == sizeof(struct irlock_s)) {
        new_data = true;
        _num_targets = report.num_targets;
        for (uint8_t i=0; i<report.num_targets; i++) {
            _target_info[i].timestamp = report.timestamp;
            _target_info[i].pos_x = report.targets[i].pos_x;
            _target_info[i].pos_y = report.targets[i].pos_y;
            _target_info[i].size_x = report.targets[i].size_x;
            _target_info[i].size_y = report.targets[i].size_y;
        }
        _last_update_ms = AP_HAL::millis();
    }

    // return true if new data found
    return new_data;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
