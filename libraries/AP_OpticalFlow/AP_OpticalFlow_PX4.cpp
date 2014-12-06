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

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "AP_OpticalFlow_PX4.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_px4flow.h>
#include <uORB/topics/optical_flow.h>

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

    // change to 10Hz update
    if (ioctl(_fd, SENSORIOCSPOLLRATE, 10) != 0) {
        hal.console->printf("Unable to set flow rate to 10Hz\n");        
    }

    // if we got this far, the sensor must be healthy
    _flags.healthy = true;
}

// update - read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_PX4::update(void)
{
    // return immediately if not healthy
    if (!_flags.healthy) {
        return;
    }

    struct optical_flow_s report;
    while (::read(_fd, &report, sizeof(optical_flow_s)) == sizeof(optical_flow_s) && report.timestamp != _last_timestamp) {
        _device_id = report.sensor_id;
        _surface_quality = report.quality;
        if (report.integration_timespan > 0) {
            float flowScaleFactorX = 1.0f + 0.001f * float(_flowScalerX);
            float flowScaleFactorY = 1.0f + 0.001f * float(_flowScalerY);
            float integralToRate = 1e6f / float(report.integration_timespan);
            _flowRate.x = flowScaleFactorX * integralToRate * float(report.pixel_flow_x_integral); // rad/sec measured optically about the X sensor axis
            _flowRate.y = flowScaleFactorY * integralToRate * float(report.pixel_flow_y_integral); // rad/sec measured optically about the Y sensor axis
            _bodyRate.x = integralToRate * float(report.gyro_x_rate_integral); // rad/sec measured inertially about the X sensor axis
            _bodyRate.y = integralToRate * float(report.gyro_y_rate_integral); // rad/sec measured inertially about the Y sensor axis
        } else {
            _flowRate.x = 0.0f;
            _flowRate.y = 0.0f;
            _bodyRate.x = 0.0f;
            _bodyRate.y = 0.0f;
        }
        _last_timestamp = report.timestamp;
        _last_update = hal.scheduler->millis();
    }
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
