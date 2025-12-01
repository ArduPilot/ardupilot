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

#include "AP_OpticalFlow_MAV.h"

#if AP_OPTICALFLOW_MAV_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#define OPTFLOW_MAV_TIMEOUT_SEC 0.5f

// detect the device
AP_OpticalFlow_MAV *AP_OpticalFlow_MAV::detect(AP_OpticalFlow &_frontend)
{
    // we assume mavlink messages will be sent into this driver
    AP_OpticalFlow_MAV *sensor = NEW_NOTHROW AP_OpticalFlow_MAV(_frontend);
    return sensor;
}

// read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_MAV::update(void)
{
    // record gyro values as long as they are being used
    // the sanity check of dt below ensures old gyro values are not used
    if (gyro_sum_count < 1000 ) {
        const Vector3f& gyro = AP::ahrs().get_gyro();
        gyro_sum.x += gyro.x;
        gyro_sum.y += gyro.y;
        gyro_sum_count++;
    }

    struct AP_OpticalFlow::OpticalFlow_state state{};
    if (count != 0) {
        // obtain data from OPTICAL_FLOW mavlink message
        state = update_msg_optical_flow();

        // reset local variables for OPTICAL_FLOW
        flow_sum.zero();
        quality_sum = 0;
        count = 0;
        gyro_sum.zero();
        gyro_sum_count = 0;
    }

#if AP_MAVLINK_MSG_OPTICAL_FLOW_RAD_ENABLED
    if (count_rad != 0 && (quality_rad_sum / count_rad) > state.surface_quality) {
        // state quality is better from OPTICAL_FLOW_RAD, so we can take it instead
        state = update_msg_optical_flow_rad();

        // reset local variables for OPTICAL_FLOW_RAD
        flow_rad_integral_sum.zero();
        rate_gyro_rad_integral_sum.zero();
        count_rad = 0;
        quality_rad_sum = 0;
        integration_time_sum_rad_us = 0;
    }
#endif //AP_MAVLINK_MSG_OPTICAL_FLOW_RAD_ENABLED

    _update_frontend(state);

}

// handle OPTICAL_FLOW mavlink messages
void AP_OpticalFlow_MAV::handle_msg(const mavlink_message_t &msg)
{
    mavlink_optical_flow_t packet;
    mavlink_msg_optical_flow_decode(&msg, &packet);

    // record time message was received
    // ToDo: add jitter correction
    latest_frame_us = AP_HAL::micros64();

    // use flow_rate_x/y fields if non-zero values are ever provided
    if (!flow_sum_is_rads && (!is_zero(packet.flow_rate_x) || !is_zero(packet.flow_rate_y))) {
        flow_sum_is_rads = true;
        flow_sum.zero();
        quality_sum = 0;
        count = 0;
    }

    // add sensor values to sum
    if (flow_sum_is_rads) {
        // higher precision flow_rate_x/y fields are used
        flow_sum.x += packet.flow_rate_x;
        flow_sum.y += packet.flow_rate_y;
    } else {
        // lower precision flow_x/y fields are used
        flow_sum.x += packet.flow_x;
        flow_sum.y += packet.flow_y;
    }
    quality_sum += packet.quality;
    count++;

    // take sensor id from message
    sensor_id = packet.sensor_id;
}

AP_OpticalFlow::OpticalFlow_state AP_OpticalFlow_MAV::update_msg_optical_flow()
{
    struct AP_OpticalFlow::OpticalFlow_state state;

    state.surface_quality = quality_sum / count;

    // calculate dt
    const float dt = (latest_frame_us - prev_frame_us) * 1.0e-6;
    prev_frame_us = latest_frame_us;

    // sanity check dt
    if (is_positive(dt) && (dt < OPTFLOW_MAV_TIMEOUT_SEC)) {
        // calculate flow values
        const float flow_scale_factor_x = 1.0f + 0.001f * _flowScaler().x;
        const float flow_scale_factor_y = 1.0f + 0.001f * _flowScaler().y;

        // scale and copy flow rates to state structure
        // if using flow_rates these are in rad/s (as opposed to pixels) and do not need to be multiplied by dt
        const float dt_used = flow_sum_is_rads ? 1.0f : dt;
        state.flowRate = { ((float)flow_sum.x / count) * flow_scale_factor_x * dt_used,
                           ((float)flow_sum.y / count) * flow_scale_factor_y * dt_used };

        // copy body-rates from gyro or set to zero
        if (option_is_enabled(Option::Stabilised)) {
            // if the sensor is stabilised then body rates are always zero
            state.bodyRate.zero();
        } else {
            // copy average body rate to state structure
            state.bodyRate = { gyro_sum.x / gyro_sum_count, gyro_sum.y / gyro_sum_count };
        }

        // we only apply yaw to flowRate as body rate comes from AHRS
        _applyYaw(state.flowRate);
    } else {
        // first frame received in some time so cannot calculate flow values
        state.flowRate.zero();
        state.bodyRate.zero();
    }

    return state;
}

#if AP_MAVLINK_MSG_OPTICAL_FLOW_RAD_ENABLED
void AP_OpticalFlow_MAV::handle_msg_optical_flow_rad(const mavlink_message_t &msg)
{
    mavlink_optical_flow_rad_t packet;
    mavlink_msg_optical_flow_rad_decode(&msg, &packet);

    flow_rad_integral_sum.x += packet.integrated_x;
    flow_rad_integral_sum.y += packet.integrated_y;

    rate_gyro_rad_integral_sum.x += packet.integrated_xgyro;
    rate_gyro_rad_integral_sum.y += packet.integrated_ygyro;

    quality_rad_sum += packet.quality;
    sensor_id_rad = packet.sensor_id;

    integration_time_sum_rad_us += packet.integration_time_us;

    count_rad++;

}

AP_OpticalFlow::OpticalFlow_state AP_OpticalFlow_MAV::update_msg_optical_flow_rad() {
    struct AP_OpticalFlow::OpticalFlow_state state;
    const Vector2f flowScaler = _flowScaler();

    float scale_factorx = 1.0f + 0.001f * flowScaler.x;
    float scale_factory = 1.0f + 0.001f * flowScaler.y;

    float integral_to_rate = 1.0f / (integration_time_sum_rad_us * 1e-6) / count_rad;
    state.flowRate = Vector2f{
        flow_rad_integral_sum.x * scale_factorx,
        flow_rad_integral_sum.y * scale_factory,
        } / count_rad * integral_to_rate;
    state.bodyRate = rate_gyro_rad_integral_sum / count_rad * integral_to_rate;
    state.surface_quality = quality_rad_sum / count_rad;

    _applyYaw(state.flowRate);
    _applyYaw(state.bodyRate);

    return state;

}

#endif // AP_MAVLINK_MSG_OPTICAL_FLOW_RAD_ENABLED

#endif  // AP_OPTICALFLOW_MAV_ENABLED
