#include "Copter.h"

/*
 * Init and run calls for Safe_RTL flight mode
 *
 * This code uses the SafeRTL path that is already in memory, and feeds it into WPNav, one point at a time.
 * Once the copter is close to home, it will run a standard land controller.
 */

bool Copter::safe_rtl_init(bool ignore_checks)
{
    Vector3f current_pos;
    if ((position_ok() || ignore_checks) && safe_rtl_path.is_active() && ahrs.get_relative_position_NED_origin(current_pos)) {
        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();
        wp_nav->init_loiter_target();

        // if we're within 4m of the landing point, just skip right to pre-land-positioning
        // otherwise, begin a normal safertl procedure from the start
        if (HYPOT(current_pos, safe_rtl_path.get_point(0)) <= 4.0f) {
            current_pos[2] -= 2.0f; // go to the point two meters above the landing spot, then start landing.
            safe_rtl_state = SafeRTL_PreLandPosition;
        } else {
            safe_rtl_state = SafeRTL_WaitForPathCleanup;
        }

        // set current position as the target point.
        DataFlash.Log_Write_SRTL(DataFlash_Class::SRTL_POINT_GOTO, current_pos);
        wp_nav->set_wp_destination_NED_origin(current_pos);

        // initialise yaw to obey user parameter
        set_auto_yaw_mode(get_default_auto_yaw_mode(true));

        // tell library to stop accepting new breadcrumbs
        safe_rtl_path.accepting_new_points(false);
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
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(),true, get_smoothing_gain());

    if (safe_rtl_path.cleanup_ready()) {
        safe_rtl_path.thorough_cleanup();
        safe_rtl_state = SafeRTL_PathFollow;
    }
}

void Copter::safe_rtl_path_follow_run()
{
    // if we are close to current target point, switch the next point to be our target.
    if (wp_nav->reached_wp_destination()) {
        Vector3f next_point;
        bool last_point = safe_rtl_path.pop_point(next_point);
        if (!last_point) {
            // go to next point along path
            wp_nav->set_wp_destination_NED_origin(next_point);
        } else {   // if this is the last point, we should prepare to land
            // go to the point that is 2m above the point, instead of directly home.
            next_point[2] -= 2.0f;
            wp_nav->set_wp_destination_NED_origin(next_point);
            safe_rtl_state = SafeRTL_PreLandPosition;
        }
        DataFlash.Log_Write_SRTL(DataFlash_Class::SRTL_POINT_GOTO, next_point);
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
            DataFlash.Log_Write_SRTL(DataFlash_Class::SRTL_LAND, {0.0f, 0.0f, 0.0f});
            rtl_land_start();
            safe_rtl_state = SafeRTL_Land;
        } else {
            DataFlash.Log_Write_SRTL(DataFlash_Class::SRTL_DESCEND, {0.0f, 0.0f, 0.0f});
            rtl_path.descent_target.alt = g.rtl_alt_final;
            rtl_descent_start();
            safe_rtl_state = SafeRTL_Descend;
        }

    }
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(),true, get_smoothing_gain());
}

/**
*   This method might take longer than 1ms. It should be run as often as possible,
*   ideally not in the main loop.
*/
void Copter::safe_rtl_background_cleanup()
{
    if (!safe_rtl_path.is_active()) {
        return;
    }

    safe_rtl_path.detect_simplifications();
    safe_rtl_path.detect_loops();
}
