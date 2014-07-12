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
 *       AP_OpticalFlow_PX4.cpp - ardupilot library for PX4Flow sensor
 *
 */

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <AP_HAL.h>
#include "AP_OpticalFlow_PX4.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_px4flow.h>
#include <stdio.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

// Public Methods //////////////////////////////////////////////////////////////

void AP_OpticalFlow_PX4::init(void)
{
    _fd = open(PX4FLOW_DEVICE_PATH, O_RDONLY);

    // check for failure to open device
    if (_fd < 0) {
        hal.console->printf("Unable to open " PX4FLOW_DEVICE_PATH "\n");
        return;
    }

    // if we got this far, the sensor must be healthy
    _flags.healthy = true;
}

// update - read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_PX4::update(void)
{
    struct px4flow_report report;
    while (::read(_fd, &report, sizeof(px4flow_report)) == sizeof(px4flow_report) && report.timestamp != _last_timestamp) {
        _device_id = report.sensor_id;
        _surface_quality = report.quality;
        _raw.x = report.flow_raw_x;
        _raw.y = report.flow_raw_y;
        _velocity.x = report.flow_comp_x_m;
        _velocity.y = report.flow_comp_y_m;
        _last_timestamp = report.timestamp;
        _last_update = hal.scheduler->millis();
    }

    // To-Do: add support for PX4Flow's sonar by retrieving ground_distance_m from report
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
