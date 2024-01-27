#include "mode.h"
#include "Plane.h"

bool ModeLoiter::_enter()
{
    plane.do_loiter_at_location();
    plane.setup_terrain_target_alt(plane.next_WP_loc);

    // make sure the local target altitude is the same as the nav target used for loiter nav
    // this allows us to do FBWB style stick control
    /*IGNORE_RETURN(plane.next_WP_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, plane.target_altitude.amsl_cm));*/
    if (plane.stick_mixing_enabled() && (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL))) {
        plane.set_target_altitude_current();
    }

    plane.loiter_angle_reset();

    return true;
}

void ModeLoiter::update()
{
    plane.calc_nav_roll();
    if (plane.stick_mixing_enabled() && plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL)) {
        plane.update_fbwb_speed_height();
    } else {
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }

#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // while a trick is running we reset altitude
        plane.set_target_altitude_current();
        plane.next_WP_loc.set_alt_cm(plane.target_altitude.amsl_cm, Location::AltFrame::ABSOLUTE);
    }
#endif
}

bool ModeLoiter::isHeadingLinedUp(const Location loiterCenterLoc, const Location targetLoc)
{
    // Return true if current heading is aligned to vector to targetLoc.

    const float targetDistanceFromCenter = loiterCenterLoc.get_distance(targetLoc);
    if (targetDistanceFromCenter < 0.99f * fabsf(plane.loiter.radius)) {
        /* Whenever next waypoint is within the loiter radius (damped slightly for rounding error),
           maintaining loiter would prevent us from ever pointing toward the next waypoint.
           Hence break out of loiter immediately
         */
        return true;
    }
    // Bearing in centi-degrees
    const int32_t bearing_cd = plane.current_loc.get_bearing_to(targetLoc);
    if (targetDistanceFromCenter < 1.05f * fabsf(plane.loiter.radius)) {
        /*  Target Waypoint is between [radius, 1.05*radius) to center so we can
            point toward the next waypoint, but there may be overshooting. Hence,
            tolerance increased to 20 degrees on initial check to reduce loop-backs.
        */
        return isHeadingLinedUp_cd(bearing_cd, 2000);
    }
    // Target waypoint is far enough from center, base margin tolerance is acceptable.
    return isHeadingLinedUp_cd(bearing_cd);

}


bool ModeLoiter::isHeadingLinedUp_cd(const int32_t bearing_cd, const int32_t initial_bearing_margin_cd)
{
    // Return true if current heading is aligned to bearing_cd.
    // Tolerance is initially "initial_bearing_margin_cd" and grows at 10 degrees for each loiter circle completed.

    // get current heading.
    const int32_t heading_cd = (wrap_360(degrees(ahrs.groundspeed_vector().angle())))*100;

    const int32_t heading_err_cd = wrap_180_cd(bearing_cd - heading_cd);

    /*
      Check to see if the the plane is heading toward the land waypoint.
      We use 2*initial_bearing_margin_cd (+/- initial_bearing_margin_cd) so
      that we can handle 20*initial_bearing_margin_cd degrees/second of yaw.

      After every full circle, extend acceptance criteria to ensure aircraft
      will not loop forever in case high winds are forcing it beyond
      20*initial_bearing_margin_cd deg/sec when passing the desired exit course
    */

    // Use integer division to get discrete steps
    const int32_t expanded_acceptance = 1000 * (labs(plane.loiter.sum_cd) / 36000);

    if (labs(heading_err_cd) <= initial_bearing_margin_cd + expanded_acceptance) {
        // Want to head in a straight line from _here_ to the next waypoint instead of center of loiter wp

        // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        if (plane.next_WP_loc.loiter_xtrack) {
            plane.next_WP_loc = plane.current_loc;
        }
        return true;
    }
    return false;
}

void ModeLoiter::navigate()
{
    if (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL)) {
        // update the WP alt from the global target adjusted by update_fbwb_speed_height
        plane.next_WP_loc.set_alt_cm(plane.target_altitude.amsl_cm, Location::AltFrame::ABSOLUTE);
    }

#if AP_SCRIPTING_ENABLED
    if (plane.nav_scripting_active()) {
        // don't try to navigate while running trick
        return;
    }
#endif

    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}

void ModeLoiter::update_target_altitude()
{
    if (plane.stick_mixing_enabled() && (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL))) {
        return;
    }
    Mode::update_target_altitude();
}
