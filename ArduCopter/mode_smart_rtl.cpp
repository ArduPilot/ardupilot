#include "Copter.h"

/*
 * Init and run calls for Smart_RTL flight mode
 *
 * This code uses the SmartRTL path that is already in memory, and feeds it into WPNav, one point at a time.
 * Once the copter is close to home, it will run a standard land controller.
 */

bool Copter::ModeSmartRTL::init(bool ignore_checks)
{
    if ((copter.position_ok() || ignore_checks) && g2.smart_rtl.is_active()) {
        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();

        // set current target to a reasonable stopping point
        Vector3f stopping_point;
        pos_control->get_stopping_point_xy(stopping_point);
        pos_control->get_stopping_point_z(stopping_point);
        wp_nav->set_wp_destination(stopping_point);

        // initialise yaw to obey user parameter
        copter.set_auto_yaw_mode(copter.get_default_auto_yaw_mode(true));

        // wait for cleanup of return path
        smart_rtl_state = SmartRTL_WaitForPathCleanup;
        return true;
    }

    return false;
}

// perform cleanup required when leaving smart_rtl
void Copter::ModeSmartRTL::exit()
{
    g2.smart_rtl.cancel_request_for_thorough_cleanup();
}

void Copter::ModeSmartRTL::run()
{
    // initialize smoothing gain
    attitude_control->set_smoothing_gain(get_smoothing_gain());

    switch (smart_rtl_state) {
        case SmartRTL_WaitForPathCleanup:
            wait_cleanup_run();
            break;
        case SmartRTL_PathFollow:
            path_follow_run();
            break;
        case SmartRTL_PreLandPosition:
            pre_land_position_run();
            break;
        case SmartRTL_Descend:
            descent_run(); // Re-using the descend method from normal rtl mode.
            break;
        case SmartRTL_Land:
            land_run(true); // Re-using the land method from normal rtl mode.
            break;
    }
}

void Copter::ModeSmartRTL::wait_cleanup_run()
{
    // hover at current target position
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(),true);

    // check if return path is computed and if yes, begin journey home
    if (g2.smart_rtl.request_thorough_cleanup()) {
        smart_rtl_state = SmartRTL_PathFollow;
    }
}

void Copter::ModeSmartRTL::path_follow_run()
{
    // if we are close to current target point, switch the next point to be our target.
    if (wp_nav->reached_wp_destination()) {
        Vector3f next_point;
        if (g2.smart_rtl.pop_point(next_point)) {
            bool fast_waypoint = true;
            if (g2.smart_rtl.get_num_points() == 0) {
                // this is the very last point, add 2m to the target alt and move to pre-land state
                next_point.z -= 2.0f;
                smart_rtl_state = SmartRTL_PreLandPosition;
                fast_waypoint = false;
            }
            // send target to waypoint controller
            wp_nav->set_wp_destination_NED(next_point);
            wp_nav->set_fast_waypoint(fast_waypoint);
        } else {
            // this can only happen if we fail to get the semaphore which should never happen but just in case, land
            smart_rtl_state = SmartRTL_PreLandPosition;
        }
    }

    // update controllers
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), 0);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(),true);
    }
}

void Copter::ModeSmartRTL::pre_land_position_run()
{
    // if we are close to 2m above start point, we are ready to land.
    if (wp_nav->reached_wp_destination()) {
        // choose descend and hold, or land based on user parameter rtl_alt_final
        if (g.rtl_alt_final <= 0 || copter.failsafe.radio) {
            land_start();
            smart_rtl_state = SmartRTL_Land;
        } else {
            set_descent_target_alt(copter.g.rtl_alt_final);
            descent_start();
            smart_rtl_state = SmartRTL_Descend;
        }
    }

    // update controllers
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(), true);
}

// save current position for use by the smart_rtl flight mode
void Copter::ModeSmartRTL::save_position()
{
    const bool should_save_position = motors->armed() && (copter.control_mode != SMART_RTL);

    copter.g2.smart_rtl.update(copter.position_ok(), should_save_position);
}
