/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_guided.pde - init and run calls for guided flight mode
 */

#ifndef GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM
 # define GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM     500     // point nose at target if it is more than 5m away
#endif

static bool guided_pilot_yaw_override_yaw = false;

// guided_init - initialise guided controller
static bool guided_init(bool ignore_checks)
{
    if (GPS_ok() || ignore_checks) {

        // initialise waypoint and spline controller
        wp_nav.wp_and_spline_init();

        // initialise wpnav to stopping point at current altitude
        // To-Do: set to current location if disarmed?
        // To-Do: set to stopping point altitude?
        Vector3f stopping_point;
        stopping_point.z = inertial_nav.get_altitude();
        wp_nav.get_wp_stopping_point_xy(stopping_point);
        wp_nav.set_wp_destination(stopping_point);
        guided_pilot_yaw_override_yaw = false;
        // initialise yaw to hold at current heading (reset to point at waypoint in guided_set_destination)
        set_auto_yaw_mode(AUTO_YAW_HOLD);
        return true;
    }else{
        return false;
    }
}

// guided_set_destination - sets guided mode's target destination
static void guided_set_destination(const Vector3f& destination)
{
    if (control_mode == GUIDED) {
        wp_nav.set_wp_destination(destination);
        if (!guided_pilot_yaw_override_yaw) {
            // get default yaw mode
            set_auto_yaw_mode(get_default_auto_yaw_mode(false));
        }
    }
}

// guided_run - runs the guided controller
// should be called at 100hz or more
static void guided_run()
{
    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // To-Do: reset waypoint controller?
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        // To-Do: handle take-offs - these may not only be immediately after auto_armed becomes true
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
        if (target_yaw_rate != 0) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(),true);
    }
}
