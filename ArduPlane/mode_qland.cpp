#include "mode.h"
#include "Plane.h"

bool ModeQLand::_enter()
{
    return plane.mode_qstabilize._enter();
}

void ModeQLand::init()
{
    plane.mode_qloiter.init();
    quadplane.throttle_wait = false;
    quadplane.setup_target_position();
    poscontrol.set_state(QuadPlane::QPOS_LAND_DESCEND);
    poscontrol.pilot_correction_done = false;
    quadplane.last_land_final_agl = plane.relative_ground_altitude(plane.g.rangefinder_landing);
    quadplane.landing_detect.lower_limit_start_ms = 0;
    quadplane.landing_detect.land_start_ms = 0;
#if LANDING_GEAR_ENABLED == ENABLED
    plane.g2.landing_gear.deploy_for_landing();
#endif
#if AC_FENCE == ENABLED
    plane.fence.auto_disable_fence_for_landing();
#endif
}

void ModeQLand::update()
{
    plane.mode_qstabilize.update();
}

void ModeQLand::run()
{
    plane.mode_qloiter.run();
}
