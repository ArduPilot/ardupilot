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

#include <AP_HAL/AP_HAL.h>
#include "OpticalFlow.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include <AP_BoardConfig/AP_BoardConfig.h>
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


extern "C" int px4flow_main(int, char **);

void AP_OpticalFlow_PX4::init(void)
{
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1) || defined(CONFIG_ARCH_BOARD_PX4FMU_V2) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V51) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V52) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V54) || defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51) || defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52) || defined(CONFIG_ARCH_BOARD_VRCORE_V10)
    if (!AP_BoardConfig::px4_start_driver(px4flow_main, "px4flow", "start")) {
        hal.console->printf("Unable to start px4flow driver\n");
    } else {
        // give it time to initialise
        hal.scheduler->delay(500);
    }
#endif
    _fd = open(PX4FLOW0_DEVICE_PATH, O_RDONLY);

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
            float yawAngleRad = _yawAngleRad();
            float cosYaw = cosf(yawAngleRad);
            float sinYaw = sinf(yawAngleRad);
            const Vector2f flowScaler = _flowScaler();
            float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
            float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
            float integralToRate = 1e6f / float(report.integration_timespan);
            // rotate sensor measurements from sensor to body frame through sensor yaw angle
            state.flowRate.x = flowScaleFactorX * integralToRate * (cosYaw * float(report.pixel_flow_x_integral) - sinYaw * float(report.pixel_flow_y_integral)); // rad/sec measured optically about the X body axis
            state.flowRate.y = flowScaleFactorY * integralToRate * (sinYaw * float(report.pixel_flow_x_integral) + cosYaw * float(report.pixel_flow_y_integral)); // rad/sec measured optically about the Y body axis
            state.bodyRate.x = integralToRate * (cosYaw * float(report.gyro_x_rate_integral) - sinYaw * float(report.gyro_y_rate_integral)); // rad/sec measured inertially about the X body axis
            state.bodyRate.y = integralToRate * (sinYaw * float(report.gyro_x_rate_integral) + cosYaw * float(report.gyro_y_rate_integral)); // rad/sec measured inertially about the Y body axis
        } else {
            state.flowRate.zero();
            state.bodyRate.zero();
        }
        _last_timestamp = report.timestamp;

        _update_frontend(state);
    }
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
