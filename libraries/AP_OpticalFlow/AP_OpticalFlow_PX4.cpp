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
#include "OpticalFlow.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_px4flow.h>
#include <uORB/topics/optical_flow.h>

extern const AP_HAL::HAL& hal;

AP_OpticalFlow_PX4::AP_OpticalFlow_PX4(OpticalFlow &_frontend) : 
OpticalFlow_backend(_frontend) 
{}


void AP_OpticalFlow_PX4::init(void)
{
    _fd = open(PX4FLOW_DEVICE_PATH, O_RDONLY);

    // check for failure to open device
    if (_fd == -1) {
        return;
    }

    // change to 10Hz update
    if (ioctl(_fd, SENSORIOCSPOLLRATE, 10) != 0) {
        hal.console->printf("Unable to set flow rate to 10Hz\n");        
    }
}

// update - read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_PX4::update(void)
{
    // return immediately if not initialised
    if (_fd == -1) {
        return;
    }

    struct optical_flow_s report;
    while (::read(_fd, &report, sizeof(optical_flow_s)) == sizeof(optical_flow_s) && 
           report.timestamp != _last_timestamp) {
        struct OpticalFlow::OpticalFlow_state state;
        state.device_id = report.sensor_id;
        state.surface_quality = report.quality;
        if (report.integration_timespan > 0) {
            const Vector2f flowScaler = _flowScaler();
            float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
            float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
            float integralToRate = 1e6f / float(report.integration_timespan);
            state.flowRate.x = flowScaleFactorX * integralToRate * float(report.pixel_flow_x_integral); // rad/sec measured optically about the X sensor axis
            state.flowRate.y = flowScaleFactorY * integralToRate * float(report.pixel_flow_y_integral); // rad/sec measured optically about the Y sensor axis
            state.bodyRate.x = integralToRate * float(report.gyro_x_rate_integral); // rad/sec measured inertially about the X sensor axis
            state.bodyRate.y = integralToRate * float(report.gyro_y_rate_integral); // rad/sec measured inertially about the Y sensor axis
        } else {
            state.flowRate.zero();
            state.bodyRate.zero();
        }
        _last_timestamp = report.timestamp;

        _update_frontend(state);
    }
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
