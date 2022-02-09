#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

bool ModeLoiterAltQLand::_enter()
{
    if (plane.previous_mode->is_vtol_mode() || plane.quadplane.in_vtol_mode()) {
        plane.set_mode(plane.mode_qland, ModeReason::LOITER_ALT_IN_VTOL);
        return true;
    }

    ModeLoiter::_enter();

#if AP_TERRAIN_AVAILABLE
    if (plane.terrain_enabled_in_mode(Mode::Number::QLAND)) {
        plane.next_WP_loc.set_alt_cm(quadplane.qrtl_alt*100UL, Location::AltFrame::ABOVE_TERRAIN);
    } else {
        plane.next_WP_loc.set_alt_cm(quadplane.qrtl_alt*100UL, Location::AltFrame::ABOVE_HOME);
    }
#else
    plane.next_WP_loc.set_alt_cm(quadplane.qrtl_alt*100UL, Location::AltFrame::ABOVE_HOME);
#endif

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
    if ((!plane.current_loc.get_alt_distance(plane.next_WP_loc, dist) || is_negative(dist)) && plane.nav_controller->reached_loiter_target()) {
        plane.set_mode(plane.mode_qland, ModeReason::LOITER_ALT_REACHED_QLAND);
    }
}

bool ModeLoiterAltQLand::handle_guided_request(Location target_loc)
{
    // setup altitude
#if AP_TERRAIN_AVAILABLE
    if (plane.terrain_enabled_in_mode(Mode::Number::QLAND)) {
        target_loc.set_alt_cm(quadplane.qrtl_alt*100UL, Location::AltFrame::ABOVE_TERRAIN);
    } else {
        target_loc.set_alt_cm(quadplane.qrtl_alt*100UL, Location::AltFrame::ABOVE_HOME);
    }
#else
    target_loc.set_alt_cm(quadplane.qrtl_alt*100UL, Location::AltFrame::ABOVE_HOME);
#endif

    plane.set_guided_WP(target_loc);

    return true;
}

#endif
