#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeLoiterAltQLand::_enter()
{
    if (plane.previous_mode->is_vtol_mode() || plane.quadplane.in_vtol_mode()) {
        plane.set_mode(plane.mode_qland, ModeReason::LOITER_ALT_IN_VTOL);
        return true;
    }

    // If we were already in a loiter then use that waypoint. Else, use the current point
    const bool already_in_a_loiter = plane.nav_controller->reached_loiter_target() && !plane.nav_controller->data_is_stale();
    const Location loiter_wp = already_in_a_loiter ? plane.next_WP_loc : plane.current_loc;

    ModeLoiter::_enter();

    handle_guided_request(loiter_wp);

    switch_qland();

    return true;
}

void ModeLoiterAltQLand::navigate()
{
    switch_qland();

    ModeLoiter::navigate();
}

void ModeLoiterAltQLand::switch_qland()
{
    ftype dist;
    if ((!plane.current_loc.get_height_above(plane.next_WP_loc, dist) || is_negative(dist)) && plane.nav_controller->reached_loiter_target()) {
        plane.set_mode(plane.mode_qland, ModeReason::LOITER_ALT_REACHED_QLAND);
    }
}

bool ModeLoiterAltQLand::handle_guided_request(Location target_loc)
{
    // setup altitude
#if AP_TERRAIN_AVAILABLE
    if (plane.terrain_enabled_in_mode(Mode::Number::QLAND)) {
        target_loc.set_alt_m(quadplane.qrtl_alt_m, Location::AltFrame::ABOVE_TERRAIN);
    } else {
        target_loc.set_alt_m(quadplane.qrtl_alt_m, Location::AltFrame::ABOVE_HOME);
    }
#else
    target_loc.set_alt_m(quadplane.qrtl_alt_m, Location::AltFrame::ABOVE_HOME);
#endif

    plane.set_guided_WP(target_loc);

    return true;
}

#endif
