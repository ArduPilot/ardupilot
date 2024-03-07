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

    // clear precland timestamp
    last_target_loc_set_ms = 0;

    return true;
}

void ModeQLand::update()
{
    plane.mode_qstabilize.update();
}

void ModeQLand::run()
{
    /*
      see if precision landing is active with an override of the
      target location
     */
    const uint32_t last_pos_set_ms = last_target_loc_set_ms;
    const uint32_t last_vel_set_ms = quadplane.poscontrol.last_velocity_match_ms;
    const uint32_t now_ms = AP_HAL::millis();

    if (last_pos_set_ms != 0 && now_ms - last_pos_set_ms < 500) {
        // we have an active landing target override
        Vector2f rel_origin;
        if (plane.next_WP_loc.get_vector_xy_from_origin_NE(rel_origin)) {
            quadplane.pos_control->set_pos_target_xy_cm(rel_origin.x, rel_origin.y);
        }
    }

    // allow for velocity override as well
    if (last_vel_set_ms != 0 && now_ms - last_vel_set_ms < 500) {
        // we have an active landing velocity override
        Vector2f target_accel;
        Vector2f target_speed_xy_cms{quadplane.poscontrol.velocity_match.x*100, quadplane.poscontrol.velocity_match.y*100};
        quadplane.pos_control->input_vel_accel_xy(target_speed_xy_cms, target_accel);
    }

    /*
      use QLOITER to do the main control
     */
    plane.mode_qloiter.run();
}

#endif
