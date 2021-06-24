#include "mode.h"
#include "Plane.h"

bool ModeQRTL::_enter()
{
    return plane.mode_qstabilize._enter();
}

void ModeQRTL::update()
{
    plane.mode_qstabilize.update();
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
    const float sink_time = rtl_alt_delta / MAX(0.6*plane.SpdHgt_Controller->get_max_sinkrate(), 1);
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

