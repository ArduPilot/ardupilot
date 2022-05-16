#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeQRTL::_enter()
{
    // treat QRTL as QLAND if we are in guided wait takeoff state, to cope
    // with failsafes during GUIDED->AUTO takeoff sequence
    if (plane.quadplane.guided_wait_takeoff_on_mode_enter) {
       plane.set_mode(plane.mode_qland, ModeReason::QLAND_INSTEAD_OF_RTL);
       return true;
    }

    // use do_RTL() to setup next_WP_loc
    plane.do_RTL(plane.home.alt + quadplane.qrtl_alt*100UL);
    plane.prev_WP_loc = plane.current_loc;
    pos_control->set_accel_desired_xy_cmss(Vector2f());
    pos_control->init_xy_controller();
    quadplane.poscontrol_init_approach();
    float dist = plane.next_WP_loc.get_distance(plane.current_loc);
    const float radius = MAX(fabsf(plane.aparm.loiter_radius), fabsf(plane.g.rtl_radius));
    if (dist < 1.5*radius &&
        quadplane.motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED) {
        // we're close to destination and already running VTOL motors, don't transition
        gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 d=%.1f r=%.1f", dist, radius);
        poscontrol.set_state(QuadPlane::QPOS_POSITION1);
    }
    int32_t from_alt;
    int32_t to_alt;
    if (plane.current_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,from_alt) && plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE,to_alt)) {
        poscontrol.slow_descent = from_alt > to_alt;
        return true;
    }
    // defualt back to old method
    poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
    return true;
}

void ModeQRTL::update()
{
    plane.mode_qstabilize.update();
}

/*
  handle QRTL mode
 */
void ModeQRTL::run()
{
    quadplane.vtol_position_controller();
    if (poscontrol.get_state() > QuadPlane::QPOS_POSITION2) {
        // change target altitude to home alt
        plane.next_WP_loc.alt = plane.home.alt;
    }
    if (poscontrol.get_state() >= QuadPlane::QPOS_POSITION2) {
        // start landing logic
        quadplane.verify_vtol_land();
    }

    // when in approach allow stick mixing
    if (quadplane.poscontrol.get_state() == QuadPlane::QPOS_AIRBRAKE ||
        quadplane.poscontrol.get_state() == QuadPlane::QPOS_APPROACH) {
        plane.stabilize_stick_mixing_fbw();
    }
}

/*
  update target altitude for QRTL profile
 */
bool ModeQRTL::update_target_altitude()
{
    /*
      update height target in approach
     */
    if (plane.quadplane.poscontrol.get_state() != QuadPlane::QPOS_APPROACH) {
        return false;
    }

    /*
      initially approach at RTL_ALT_CM, then drop down to QRTL_ALT based on maximum sink rate from TECS,
      giving time to lose speed before we transition
     */
    const float radius = MAX(fabsf(plane.aparm.loiter_radius), fabsf(plane.g.rtl_radius));
    const float rtl_alt_delta = MAX(0, plane.g.RTL_altitude_cm*0.01 - plane.quadplane.qrtl_alt);
    const float sink_time = rtl_alt_delta / MAX(0.6*plane.TECS_controller.get_max_sinkrate(), 1);
    const float sink_dist = plane.aparm.airspeed_cruise_cm * 0.01 * sink_time;
    const float dist = plane.auto_state.wp_distance;
    const float rad_min = 2*radius;
    const float rad_max = 20*radius;
    float alt = linear_interpolate(0, rtl_alt_delta,
                                   dist,
                                   rad_min, MAX(rad_min, MIN(rad_max, rad_min+sink_dist)));
    Location loc = plane.next_WP_loc;
    loc.alt += alt*100;
    plane.set_target_altitude_location(loc);
    return true;
}

// only nudge during approach
bool ModeQRTL::allows_throttle_nudging() const
{
    return plane.quadplane.poscontrol.get_state() == QuadPlane::QPOS_APPROACH;
}

#endif
