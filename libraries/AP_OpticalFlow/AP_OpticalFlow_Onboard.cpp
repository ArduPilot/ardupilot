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
#include "AP_OpticalFlow_Onboard.h"

#include <AP_HAL/AP_HAL.h>

#include "OpticalFlow.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX &&\
    (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP ||\
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE ||\
     CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI)

#ifndef OPTICALFLOW_ONBOARD_DEBUG
#define OPTICALFLOW_ONBOARD_DEBUG 0
#endif

#define OPTICALFLOW_ONBOARD_ID 1
extern const AP_HAL::HAL& hal;

AP_OpticalFlow_Onboard::AP_OpticalFlow_Onboard(OpticalFlow &_frontend,
                                               AP_AHRS_NavEKF& ahrs) :
    OpticalFlow_backend(_frontend),
    _ahrs(ahrs)
{}

void AP_OpticalFlow_Onboard::init(void)
{
    /* register callback to get gyro data */
    hal.opticalflow->init(
            FUNCTOR_BIND_MEMBER(&AP_OpticalFlow_Onboard::_get_gyro,
                                void, float&, float&, float&));
}

void AP_OpticalFlow_Onboard::update()
{
    AP_HAL::OpticalFlow::Data_Frame data_frame;
    // read at maximum 10Hz
    uint32_t now = AP_HAL::millis();
    if (now - _last_read_ms < 100) {
        return;
    }
    _last_read_ms = now;

    if (!hal.opticalflow->read(data_frame)) {
        return;
    }

    struct OpticalFlow::OpticalFlow_state state;
    state.device_id = OPTICALFLOW_ONBOARD_ID;
    state.surface_quality = data_frame.quality;
    if (data_frame.delta_time > 0) {
        const Vector2f flowScaler = _flowScaler();
        float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
        float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;

        state.flowRate.x = flowScaleFactorX * 1000.0f /
                           float(data_frame.delta_time) *
                           data_frame.pixel_flow_x_integral;

        state.flowRate.y = flowScaleFactorY * 1000.0f /
                           float(data_frame.delta_time) *
                           data_frame.pixel_flow_y_integral;

        state.bodyRate.x = 1.0f / float(data_frame.delta_time) *
                           data_frame.gyro_x_integral;

        state.bodyRate.y = 1.0f / float(data_frame.delta_time) *
                           data_frame.gyro_y_integral;
    } else {
        state.flowRate.zero();
        state.bodyRate.zero();
    }

    // copy results to front end
    _update_frontend(state);

#if OPTICALFLOW_ONBOARD_DEBUG
    hal.console->printf("FLOW_ONBOARD qual:%u FlowRateX:%4.2f Y:%4.2f"
                        "BodyRateX:%4.2f Y:%4.2f, delta_time = %u\n",
                        (unsigned)state.surface_quality,
                        (double)state.flowRate.x,
                        (double)state.flowRate.y,
                        (double)state.bodyRate.x,
                        (double)state.bodyRate.y,
                        data_frame.delta_time);
#endif
}

void AP_OpticalFlow_Onboard::_get_gyro(float &rate_x, float &rate_y,
                                       float &rate_z)
{
    Vector3f rates = _ahrs.get_gyro();
    rate_x = rates.x;
    rate_y = rates.y;
    rate_z = rates.z;
}

#endif
