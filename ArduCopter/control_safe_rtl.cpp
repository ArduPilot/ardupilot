#include "Copter.h"

/*
 * Init and run calls for Safe_RTL flight mode
 *
 * This code uses the SafeRTL path that is already in memory, and feeds it into WPNav, one point at a time.
 * Once the copter is close to home, it will run a standard land controller.
 */

bool Copter::safe_rtl_init(bool ignore_checks)
{
    if ((position_ok() || ignore_checks) && g2.safe_rtl.is_active()) {
        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();

        // set current target to a reasonable stopping point
        Vector3f stopping_point;
        pos_control->get_stopping_point_xy(stopping_point);
        pos_control->get_stopping_point_z(stopping_point);
        wp_nav->set_wp_destination(stopping_point);

        // initialise yaw to obey user parameter
        set_auto_yaw_mode(get_default_auto_yaw_mode(true));

        // wait for cleanup of return path
        safe_rtl_state = SafeRTL_WaitForPathCleanup;
        return true;
    } else {
        return false;
    }
}

void Copter::safe_rtl_run()
{
    switch(safe_rtl_state){
        case SafeRTL_WaitForPathCleanup:
            safe_rtl_wait_cleanup_run();
            break;
        case SafeRTL_PathFollow:
            safe_rtl_path_follow_run();
            break;
        case SafeRTL_PreLandPosition:
            safe_rtl_pre_land_position_run();
            break;
        case SafeRTL_Descend:
            rtl_descent_run(); // Re-using the descend method from normal rtl mode.
            break;
        case SafeRTL_Land:
            rtl_land_run(); // Re-using the land method from normal rtl mode.
            break;
    }
}

void Copter::safe_rtl_wait_cleanup_run()
{
    // hover at current target position
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(),true, get_smoothing_gain());

    // check if return path is computed and if yes, begin journey home
    if (g2.safe_rtl.thorough_cleanup()) {
        safe_rtl_state = SafeRTL_PathFollow;
    }
}

void Copter::safe_rtl_path_follow_run()
{
    // if we are close to current target point, switch the next point to be our target.
    if (wp_nav->reached_wp_destination()) {
        Vector3f next_point;
        if (g2.safe_rtl.pop_point(next_point)) {
            // this is the very last point, add 2m to the target alt and move to pre-land state
            if (g2.safe_rtl.get_num_points() == 0) {
                next_point.z -= 2.0f;
                safe_rtl_state = SafeRTL_PreLandPosition;
            }
            // send target to waypoint controller
            wp_nav->set_wp_destination_NED(next_point);
        } else {
            // this should never happen but just in case, land
            safe_rtl_state = SafeRTL_PreLandPosition;
        }
    }

    // update controllers
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), 0, get_smoothing_gain());
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(),true, get_smoothing_gain());
    }
}

void Copter::safe_rtl_pre_land_position_run()
{
    // if we are close to 2m above start point, we are ready to land.
    if (wp_nav->reached_wp_destination()) {
        // choose descend and hold, or land based on user parameter rtl_alt_final
        if (g.rtl_alt_final <= 0 || failsafe.radio) {
            rtl_land_start();
            safe_rtl_state = SafeRTL_Land;
        } else {
            rtl_path.descent_target.alt = g.rtl_alt_final;
            rtl_descent_start();
            safe_rtl_state = SafeRTL_Descend;
        }
    }

    // update controllers
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(), true, get_smoothing_gain());
}

// save current position for use by the safe_rtl flight mode
void Copter::safe_rtl_save_position()
{
    bool save_position = motors->armed() && (control_mode != SAFE_RTL);

    g2.safe_rtl.update(position_ok(), save_position);
}
