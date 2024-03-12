#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQLand::_enter()
{
    plane.mode_qloiter._enter();
    quadplane.throttle_wait = false;
    quadplane.setup_target_position();
    poscontrol.set_state(QuadPlane::QPOS_LAND_DESCEND);
    poscontrol.pilot_correction_done = false;
    quadplane.last_land_final_agl = plane.relative_ground_altitude(plane.g.rangefinder_landing);
    quadplane.landing_detect.lower_limit_start_ms = 0;
    quadplane.landing_detect.land_start_ms = 0;
#if AP_LANDINGGEAR_ENABLED
    plane.g2.landing_gear.deploy_for_landing();
#endif
#if AP_FENCE_ENABLED
    plane.fence.auto_disable_fence_for_landing();
#endif

    return true;
}

void ModeQLand::update()
{
    plane.mode_qstabilize.update();
}

void ModeQLand::run()
{
    /*
      use QLOITER to do the main control
     */
    plane.mode_qloiter.run();
}

#endif
