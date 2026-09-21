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

    void  set_zoom_pct(float pct)  { _zoom_pct = pct; }
    float zoom_pct()   const       { return _zoom_pct; }

    void  set_focus_pct(float pct) { _focus_pct = pct; }
    float focus_pct()  const       { return _focus_pct; }

private:
    uint16_t _shot_count {};
    float    _zoom_pct   {NAN};
    float    _focus_pct  {NAN};
};

}  // namespace SITL

#endif  // AP_SIM_MAVLINKCAMV2_ENABLED
