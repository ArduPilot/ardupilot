#include "Copter.h"

#if MODE_SMARTRTL_ENABLED == ENABLED

/*
 * Init and run calls for Smart_RTL flight mode
 *
 * This code uses the SmartRTL path that is already in memory, and feeds it into WPNav, one point at a time.
 * Once the copter is close to home, it will run a standard land controller.
 */

bool ModeSmartRTL::init(bool ignore_checks)
{
    if (g2.smart_rtl.is_active()) {
        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();

        // set current target to a reasonable stopping point
        Vector3p stopping_point;
        pos_control->get_stopping_point_xy_cm(stopping_point.xy());
        pos_control->get_stopping_point_z_cm(stopping_point.z);
        wp_nav->set_wp_destination(stopping_point.tofloat());

        // initialise yaw to obey user parameter
        auto_yaw.set_mode_to_default(true);

        // wait for cleanup of return path
        smart_rtl_state = SubMode::WAIT_FOR_PATH_CLEANUP;
        return true;
    }

    return false;
}

// perform cleanup required when leaving smart_rtl
void ModeSmartRTL::exit()
{
    // restore last point if we hadn't reached it
    if (smart_rtl_state == SubMode::PATH_FOLLOW && !dest_NED_backup.is_zero()) {
        if (!g2.smart_rtl.add_point(dest_NED_backup)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SmartRTL: lost one point");
        }
    }
    dest_NED_backup.zero();

    g2.smart_rtl.cancel_request_for_thorough_cleanup();
}

void ModeSmartRTL::run()
{
    switch (smart_rtl_state) {
        case SubMode::WAIT_FOR_PATH_CLEANUP:
            wait_cleanup_run();
            break;
        case SubMode::PATH_FOLLOW:
            path_follow_run();
            break;
        case SubMode::PRELAND_POSITION:
            pre_land_position_run();
            break;
        case SubMode::DESCEND:
            descent_run(); // Re-using the descend method from normal rtl mode.
            break;
        case SubMode::LAND:
            land_run(true); // Re-using the land method from normal rtl mode.
            break;
    }
}

bool ModeSmartRTL::is_landing() const
{
    return smart_rtl_state == SubMode::LAND;
}

void ModeSmartRTL::wait_cleanup_run()
{
    // hover at current target position
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    // check if return path is computed and if yes, begin journey home
    if (g2.smart_rtl.request_thorough_cleanup()) {
        path_follow_last_pop_fail_ms = 0;
        smart_rtl_state = SubMode::PATH_FOLLOW;
    }
}

void ModeSmartRTL::path_follow_run()
{
    // if we are close to current target point, switch the next point to be our target.
    if (wp_nav->reached_wp_destination()) {

        // clear destination backup so that it cannot be restored
        dest_NED_backup.zero();

        // this pop_point can fail if the IO task currently has the
        // path semaphore.
        Vector3f dest_NED;
        if (g2.smart_rtl.pop_point(dest_NED)) {
            // backup destination in case we exit smart_rtl mode and need to restore it to the path
            dest_NED_backup = dest_NED;
            path_follow_last_pop_fail_ms = 0;
            if (g2.smart_rtl.get_num_points() == 0) {
                // this is the very last point, add 2m to the target alt and move to pre-land state
                dest_NED.z -= 2.0f;
                smart_rtl_state = SubMode::PRELAND_POSITION;
                wp_nav->set_wp_destination_NED(dest_NED);
            } else {
                // peek at the next point.  this can fail if the IO task currently has the path semaphore
                Vector3f next_dest_NED;
                if (g2.smart_rtl.peek_point(next_dest_NED)) {
                    wp_nav->set_wp_destination_NED(dest_NED);
                    if (g2.smart_rtl.get_num_points() == 1) {
                        // this is the very last point, add 2m to the target alt
                        next_dest_NED.z -= 2.0f;
                    }
                    wp_nav->set_wp_destination_next_NED(next_dest_NED);
                } else {
                    // this can only happen if peek failed to take the semaphore
                    // send next point anyway which will cause the vehicle to slow at the next point
                    wp_nav->set_wp_destination_NED(dest_NED);
                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                }
            }
        } else if (g2.smart_rtl.get_num_points() == 0) {
            // We should never get here; should always have at least
            // two points and the "zero points left" is handled above.
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            smart_rtl_state = SubMode::PRELAND_POSITION;
        } else if (path_follow_last_pop_fail_ms == 0) {
            // first time we've failed to pop off (ever, or after a success)
            path_follow_last_pop_fail_ms = AP_HAL::millis();
        } else if (AP_HAL::millis() - path_follow_last_pop_fail_ms > 10000) {
            // we failed to pop a point off for 10 seconds.  This is
            // almost certainly a bug.
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            smart_rtl_state = SubMode::PRELAND_POSITION;
        }
    }

    // update controllers
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

void ModeSmartRTL::pre_land_position_run()
{
    // if we are close to 2m above start point, we are ready to land.
    if (wp_nav->reached_wp_destination()) {
        // choose descend and hold, or land based on user parameter rtl_alt_final
        if (g.rtl_alt_final <= 0 || copter.failsafe.radio) {
            land_start();
            smart_rtl_state = SubMode::LAND;
        } else {
            set_descent_target_alt(copter.g.rtl_alt_final);
            descent_start();
            smart_rtl_state = SubMode::DESCEND;
        }
    }

    // update controllers
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// save current position for use by the smart_rtl flight mode
void ModeSmartRTL::save_position()
{
    const bool should_save_position = motors->armed() && (copter.flightmode->mode_number() != Mode::Number::SMART_RTL);

    copter.g2.smart_rtl.update(copter.position_ok(), should_save_position);
}

bool ModeSmartRTL::get_wp(Location& destination) const
{
    // provide target in states which use wp_nav
    switch (smart_rtl_state) {
    case SubMode::WAIT_FOR_PATH_CLEANUP:
    case SubMode::PATH_FOLLOW:
    case SubMode::PRELAND_POSITION:
    case SubMode::DESCEND:
        return wp_nav->get_wp_destination_loc(destination);
    case SubMode::LAND:
        return false;
    }

    // we should never get here but just in case
    return false;
}

uint32_t ModeSmartRTL::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

int32_t ModeSmartRTL::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
}

bool ModeSmartRTL::use_pilot_yaw() const
{
    const bool land_repositioning = g.land_repositioning && (smart_rtl_state == SubMode::DESCEND);
    const bool final_landing = smart_rtl_state == SubMode::LAND;
    return g2.smart_rtl.use_pilot_yaw() || land_repositioning || final_landing;
}

#endif
