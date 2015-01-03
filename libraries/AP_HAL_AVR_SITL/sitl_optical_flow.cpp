/*
  SITL handling

  This simulates a optical flow sensor

  Andrew Tridgell November 2011
 */

#include <AP_HAL.h>
#include <AP_Math.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include "AP_HAL_AVR_SITL.h"

using namespace AVR_SITL;

extern const AP_HAL::HAL& hal;

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/*
  update the optical flow with new data
 */
void SITL_State::_update_flow(void)
{
    double p, q, r;
    Vector3f gyro;

    if (!_optical_flow ||
        !_terrain ||
        !_sitl->flow_enable) {
        return;
    }

    // convert roll rates to body frame
    SITL::convert_body_frame(radians(_sitl->state.rollDeg),
                             radians(_sitl->state.pitchDeg),
                             radians(_sitl->state.rollRate), 
                             radians(_sitl->state.pitchRate), 
                             radians(_sitl->state.yawRate),
                             &p, &q, &r);
    gyro(p, q, r);

    OpticalFlow::OpticalFlow_state state;

    // get height above terrain from AP_Terrain. This assumes
    // AP_Terrain is working
    float terrain_height_amsl;
    struct Location location;
    location.lat = _sitl->state.latitude*1.0e7;
    location.lng = _sitl->state.longitude*1.0e7;

    if (!_terrain->height_amsl(location, terrain_height_amsl)) {
        // no terrain height available
        return;
    }

    float height_agl = _sitl->state.altitude - terrain_height_amsl;

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
    state.surface_quality = 0;

    // estimate range to centre of image
    float range;
    if (rotmat.c.z > 0.05f) {
        range = height_agl / rotmat.c.z;
    } else {
        range = 1e38f;
    }

    // Calculate relative velocity in sensor frame assuming no misalignment between sensor and vehicle body axes
    Vector3f relVelSensor = rotmat*velocity;

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

    _optical_flow->setHIL(state);
}

#endif
