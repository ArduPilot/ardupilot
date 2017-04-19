#include "Copter.h"

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
void Copter::run_nav_updates(void)
{
    // calculate distance and bearing for reporting and autopilot decisions
    calc_distance_and_bearing();

    // run autopilot to make high level decisions about control modes
    run_autopilot();
}

// calc_distance_and_bearing - calculate distance and bearing to next waypoint and home
void Copter::calc_distance_and_bearing()
{
    calc_wp_distance();
    calc_wp_bearing();
    calc_home_distance_and_bearing();
}

// calc_wp_distance - calculate distance to next waypoint for reporting and autopilot decisions
void Copter::calc_wp_distance()
{
    // get target from loiter or wpinav controller
    switch (control_mode) {
    case LOITER:
    case CIRCLE:
        wp_distance = wp_nav->get_loiter_distance_to_target();
        break;

    case AUTO:
    case RTL:
        wp_distance = wp_nav->get_wp_distance_to_destination();
        break;

    case GUIDED:
        if (guided_mode == Guided_WP) {
            wp_distance = wp_nav->get_wp_distance_to_destination();
            break;
        }
        // no break
    default:
        wp_distance = 0;
        break;
    }
}

// calc_wp_bearing - calculate bearing to next waypoint for reporting and autopilot decisions
void Copter::calc_wp_bearing()
{
    // get target from loiter or wpinav controller
    switch (control_mode) {
    case LOITER:
    case CIRCLE:
        wp_bearing = wp_nav->get_loiter_bearing_to_target();
        break;

    case AUTO:
    case RTL:
        wp_bearing = wp_nav->get_wp_bearing_to_destination();
        break;

    case GUIDED:
        if (guided_mode == Guided_WP) {
            wp_bearing = wp_nav->get_wp_bearing_to_destination();
            break;
        }
        // no break
    default:
        wp_bearing = 0;
        break;
    }
}

// calc_home_distance_and_bearing - calculate distance and bearing to home for reporting and autopilot decisions
void Copter::calc_home_distance_and_bearing()
{
    // calculate home distance and bearing
    if (position_ok()) {
        Vector3f home = pv_location_to_vector(ahrs.get_home());
        Vector3f curr = inertial_nav.get_position();
        home_distance = pv_get_horizontal_distance_cm(curr, home);
        home_bearing = pv_get_bearing_cd(curr,home);

        // update super simple bearing (if required) because it relies on home_bearing
        update_super_simple_bearing(false);
    }
}

// run_autopilot - highest level call to process mission commands
void Copter::run_autopilot()
{
    if (control_mode == AUTO) {
        // update state of mission
        // may call commands_process.pde's start_command and verify_command functions
        mission.update();
    }
}
