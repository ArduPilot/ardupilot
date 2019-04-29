#include "mode.h"
#include "Rover.h"

bool ModeRTL::_enter()
{
    // refuse RTL if home has not been set
    if (!AP::ahrs().home_is_set()) {
        return false;
    }

    // initialise waypoint speed
    if (is_positive(g2.rtl_speed)) {
        g2.wp_nav.set_desired_speed(g2.rtl_speed);
    } else {
        g2.wp_nav.set_desired_speed_to_default();
    }

    // set target to the closest rally point or home
#if AP_RALLY == ENABLED
    g2.wp_nav.set_desired_location(g2.rally.calc_best_rally_or_home_location(rover.current_loc, ahrs.get_home().alt));
#else
    // set destination
    g2.wp_nav.set_desired_location(rover.home);
#endif

    sent_notification = false;

    return true;
}

void ModeRTL::update()
{
    // determine if we should keep navigating
    if (!g2.wp_nav.reached_destination() || rover.is_boat()) {
        // update navigation controller
        navigate_to_waypoint();
    } else {
        // send notification
        if (!sent_notification) {
            sent_notification = true;
            gcs().send_text(MAV_SEVERITY_INFO, "Reached destination");
        }

        // we've reached destination so stop
        stop_vehicle();

        // update distance to destination
        _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
    }
}

bool ModeRTL::reached_destination() const
{
    return g2.wp_nav.reached_destination();
}
