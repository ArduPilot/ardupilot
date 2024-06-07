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

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AP_OpticalFlow_MSP.h"

#if HAL_MSP_OPTICALFLOW_ENABLED

#define OPTFLOW_MSP_TIMEOUT_SEC 0.5f // 2Hz

extern const AP_HAL::HAL& hal;

using namespace MSP;

// detect the device
AP_OpticalFlow_MSP *AP_OpticalFlow_MSP::detect(AP_OpticalFlow &_frontend)
{
    // we assume msp messages will be sent into this driver
    return NEW_NOTHROW AP_OpticalFlow_MSP(_frontend);
}

// read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_MSP::update(void)
{
    // record gyro values as long as they are being used
    // the sanity check of dt below ensures old gyro values are not used
    if (gyro_sum_count < 1000) {
        const Vector3f& gyro = AP::ahrs().get_gyro();
        gyro_sum.x += gyro.x;
        gyro_sum.y += gyro.y;
        gyro_sum_count++;
    }

    // return without updating state if no readings
    if (count == 0) {
        return;
    }

    struct AP_OpticalFlow::OpticalFlow_state state {};

    state.surface_quality = quality_sum / count;

    // calculate dt
    const float dt = (latest_frame_us - prev_frame_us) * 1.0e-6;
    prev_frame_us = latest_frame_us;

    // sanity check dt
    if (is_positive(dt) && (dt < OPTFLOW_MSP_TIMEOUT_SEC)) {
        // calculate flow values
        const float flow_scale_factor_x = 1.0f + 0.001f * _flowScaler().x;
        const float flow_scale_factor_y = 1.0f + 0.001f * _flowScaler().y;

        // copy flow rates to state structure
        state.flowRate = { ((float)flow_sum.x / count) * flow_scale_factor_x * dt,
                           ((float)flow_sum.y / count) * flow_scale_factor_y * dt};

        // copy average body rate to state structure
        state.bodyRate = { gyro_sum.x / gyro_sum_count, gyro_sum.y / gyro_sum_count };

        // invert flow vector to align it with default sensor orientation (required on matek 3901)
        state.flowRate *= -1;

        // we only apply yaw to flowRate as body rate comes from AHRS
        _applyYaw(state.flowRate);
    } else {
        // first frame received in some time so cannot calculate flow values
        state.flowRate.zero();
        state.bodyRate.zero();
    }

    _update_frontend(state);

    // reset local buffers
    flow_sum.zero();
    quality_sum = 0;
    count = 0;

    // reset gyro sum
    gyro_sum.zero();
    gyro_sum_count = 0;
}

// handle OPTICAL_FLOW msp messages
void AP_OpticalFlow_MSP::handle_msp(const MSP::msp_opflow_data_message_t &pkt)
{
    // record time message was received
    // ToDo: add jitter correction
    latest_frame_us = AP_HAL::micros64();

    // add sensor values to sum
    flow_sum.x += pkt.motion_x;
    flow_sum.y += pkt.motion_y;
    quality_sum += (int)pkt.quality * 100 / 255;
    count++;
}

#endif // HAL_MSP_OPTICALFLOW_ENABLED
