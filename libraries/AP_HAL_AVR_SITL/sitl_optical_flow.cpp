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

    state.device_id = 1;
    state.surface_quality = 0;

    // rubbish calculation for Paul to fill in
    state.flowRate = Vector2f(gyro.x, gyro.y) * height_agl;
    state.bodyRate = Vector2f(gyro.x, gyro.y) * height_agl;

    _optical_flow->setHIL(state);
}

#endif
