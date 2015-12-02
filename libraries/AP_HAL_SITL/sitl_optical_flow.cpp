/*
  SITL handling

  This simulates a optical flow sensor

  Andrew Tridgell November 2011
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#define MAX_OPTFLOW_DELAY 20
static uint8_t next_optflow_index;
static uint8_t optflow_delay;
static OpticalFlow::OpticalFlow_state optflow_data[MAX_OPTFLOW_DELAY];

/*
  update the optical flow with new data
 */
void SITL_State::_update_flow(void)
{
    Vector3f gyro;
    static uint32_t last_flow_ms;

    if (!_optical_flow ||
            !_sitl->flow_enable) {
        return;
    }

    // update at the requested rate
    uint32_t now = AP_HAL::millis();
    if (now - last_flow_ms < 1000*(1.0f/_sitl->flow_rate)) {
        return;
    }
    last_flow_ms = now;

    gyro(radians(_sitl->state.rollRate), 
         radians(_sitl->state.pitchRate), 
         radians(_sitl->state.yawRate));

    OpticalFlow::OpticalFlow_state state;

    // NED velocity vector in m/s
    Vector3f velocity(_sitl->state.speedN,
                      _sitl->state.speedE,
                      _sitl->state.speedD);

    // a rotation matrix following DCM conventions
    Matrix3f rotmat;
    rotmat.from_euler(radians(_sitl->state.rollDeg),
                      radians(_sitl->state.pitchDeg),
                      radians(_sitl->state.yawDeg));


    state.device_id = 1;
    state.surface_quality = 51;

    // estimate range to centre of image
    float range;
    if (rotmat.c.z > 0.05f && height_agl() > 0) {
        range = height_agl() / rotmat.c.z;
    } else {
        range = 1e38f;
    }

    // Calculate relative velocity in sensor frame assuming no misalignment between sensor and vehicle body axes
    Vector3f relVelSensor = rotmat.mul_transpose(velocity);

    // Divide velocity by range and add body rates to get predicted sensed angular
    // optical rates relative to X and Y sensor axes assuming no misalignment or scale
    // factor error. Note - these are instantaneous values. The sensor sums these values across the interval from the last
    // poll to provide a delta angle across the interface
    state.flowRate.x =  -relVelSensor.y/range + gyro.x;
    state.flowRate.y =   relVelSensor.x/range + gyro.y;

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
        if (_sitl->flow_delay > MAX_OPTFLOW_DELAY) {
            _sitl->flow_delay = MAX_OPTFLOW_DELAY;
        }
        optflow_delay = _sitl->flow_delay;
        for (uint8_t i=0; i<optflow_delay; i++) {
            optflow_data[i] = state;
        }
    }

    _optical_flow->setHIL(state);
}

#endif
