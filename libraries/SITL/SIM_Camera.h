/*
   Camera state class for SITL combined gimbal+camera simulations.
   Analogous to SIM_Gimbal.h but for camera state; no physics simulation.
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_MAVLINKCAMV2_ENABLED

#include <stdint.h>

namespace SITL {

class Camera {
public:
    void     trigger_shutter() { _shot_count++; }
    uint16_t shot_count() const { return _shot_count; }

private:
    uint16_t _shot_count {};
};

}  // namespace SITL

#endif  // AP_SIM_MAVLINKCAMV2_ENABLED
