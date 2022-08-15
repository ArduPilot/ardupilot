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
 * AP_OpticalFlow_SITL.cpp - SITL emulation of optical flow sensor.
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_OpticalFlow_SITL.h"

extern const AP_HAL::HAL& hal;

AP_OpticalFlow_SITL::AP_OpticalFlow_SITL(AP_OpticalFlow &_frontend) :
    OpticalFlow_backend(_frontend),
    _sitl(AP::sitl())
{
}

void AP_OpticalFlow_SITL::init(void)
{
}

void AP_OpticalFlow_SITL::update(void)
{
    if (!_sitl->flow_enable) {
        return;
    }

    // update at the requested rate
    uint32_t now = AP_HAL::millis();
    if (now - last_flow_ms < 1000*(1.0f/_sitl->flow_rate)) {
        return;
    }
    last_flow_ms = now;

    Vector3f gyro(radians(_sitl->state.rollRate), 
                  radians(_sitl->state.pitchRate), 
                  radians(_sitl->state.yawRate));

    AP_OpticalFlow::OpticalFlow_state state;

    // NED velocity vector in m/s
    Vector3f velocity(_sitl->state.speedN,
                      _sitl->state.speedE,
                      _sitl->state.speedD);

    // a rotation matrix following DCM conventions
    Matrix3f rotmat;
    rotmat.from_euler(radians(_sitl->state.rollDeg),
                      radians(_sitl->state.pitchDeg),
                      radians(_sitl->state.yawDeg));


    state.surface_quality = 51;

    // sensor position offset in body frame
    Vector3f posRelSensorBF = _sitl->optflow_pos_offset;

    // estimate range to centre of image
    float range;
    if (rotmat.c.z > 0.05f && _sitl->height_agl > 0) {
        Vector3f relPosSensorEF = rotmat * posRelSensorBF;
        range = (_sitl->height_agl - relPosSensorEF.z) / rotmat.c.z;
    } else {
        range = 1e38f;
    }

    // Calculate relative velocity in sensor frame assuming no misalignment between sensor and vehicle body axes
    Vector3f relVelSensor = rotmat.mul_transpose(velocity);

    // correct relative velocity for rotation rates and sensor offset
    relVelSensor += gyro % posRelSensorBF;

    // scaling based on parameters
    const Vector2f flowScaler = _flowScaler();
    const float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
    const float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;

    // Divide velocity by range and add body rates to get predicted sensed angular
    // optical rates relative to X and Y sensor axes assuming no misalignment or scale
    // factor error. Note - these are instantaneous values. The sensor sums these values across the interval from the last
    // poll to provide a delta angle across the interface
    state.flowRate.x = (-relVelSensor.y/range + gyro.x + _sitl->flow_noise * rand_float()) * flowScaleFactorX;
    state.flowRate.y =  (relVelSensor.x/range + gyro.y + _sitl->flow_noise * rand_float()) * flowScaleFactorY;

    // The flow sensors body rates are assumed to be the same as the vehicle body rates (ie no misalignment)
    // Note - these are instantaneous values. The sensor sums these values across the interval from the last
    // poll to provide a delta angle across the interface.
    state.bodyRate = Vector2f(gyro.x, gyro.y);

    optflow_data[next_optflow_index++] = state;
    if (next_optflow_index >= optflow_delay+1) {
        next_optflow_index = 0;
    }

    state = optflow_data[next_optflow_index];

    if (_sitl->flow_delay != optflow_delay) {
        // cope with updates to the delay control
        if (_sitl->flow_delay > 0 &&
            (uint8_t)(_sitl->flow_delay) > ARRAY_SIZE(optflow_data)) {
            _sitl->flow_delay.set(ARRAY_SIZE(optflow_data));
        }
        optflow_delay = _sitl->flow_delay;
        for (uint8_t i=0; i<optflow_delay; i++) {
            optflow_data[i] = state;
        }
    }

    _applyYaw(state.flowRate);
    
    // copy results to front end
    _update_frontend(state);
}

#endif // CONFIG_HAL_BOARD
