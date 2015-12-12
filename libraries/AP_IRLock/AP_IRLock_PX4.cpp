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

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include "AP_IRLock_PX4.h"

#include <fcntl.h>
#include <unistd.h>
#include "drivers/drv_irlock.h"

extern const AP_HAL::HAL& hal;

AP_IRLock_PX4::AP_IRLock_PX4() :
    _fd(0),
    _last_timestamp(0)
{}

void AP_IRLock_PX4::init()
{
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
    struct irlock_s report;
    uint16_t count = 0;
    while(::read(_fd, &report, sizeof(struct irlock_s)) == sizeof(struct irlock_s) && report.timestamp >_last_timestamp) {
        _target_info[count].timestamp = report.timestamp;
        _target_info[count].target_num = report.target_num;
        _target_info[count].angle_x = report.angle_x;
        _target_info[count].angle_y = report.angle_y;
        _target_info[count].size_x = report.size_x;
        _target_info[count].size_y = report.size_y;
        count++;
        _last_timestamp = report.timestamp;
        _last_update = AP_HAL::millis();
    }

    // update num_blocks and implement timeout
    if (count > 0) {
        _num_targets = count;
    } else if ((AP_HAL::millis() - _last_update) > IRLOCK_TIMEOUT_MS) {
        _num_targets = 0;
    }

    // return true if new data found
    return (_num_targets > 0);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
