/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_auto.pde - init and run calls for auto flight mode
 *
 * This file contains the implementation for Land, Waypoint navigation and Takeoff from Auto mode
 * Command execution code (i.e. command_logic.pde) should:
 *      a) switch to Auto flight mode with set_mode() function.  This will cause auto_init to be called
 *      b) call one of the three auto initialisation functions: auto_wp_start(), auto_takeoff_start(), auto_land_start()
 *      c) call one of the verify functions auto_wp_verify(), auto_takeoff_verify, auto_land_verify repeated to check if the command has completed
 * The main loop (i.e. fast loop) will call update_flight_modes() which will in turn call auto_run() which, based upon the auto_mode variable will call
 *      correct auto_wp_run, auto_takeoff_run or auto_land_run to actually implement the feature
 */

/*
 *  While in the auto flight mode, navigation or do/now commands can be run.
 *  Code in this file implements the navigation commands
 */

// auto_init - initialise auto controller
static bool auto_init(bool ignore_checks)
{
    if ((GPS_ok() && inertial_nav.position_ok() && g.command_total > 1) || ignore_checks) {
        // stop ROI from carrying over from previous runs of the mission
        // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
        if (auto_yaw_mode == AUTO_YAW_ROI) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
        // clear the command queues. will be reloaded when "run_autopilot" calls "update_commands" function
        init_commands();
        return true;
    }else{
        return false;
    }
}

// auto_run - runs the auto controller
//      should be called at 100hz or more
//      relies on run_autopilot being called at 10hz which handles decision making and non-navigation related commands
static void auto_run()
{
    // call the correct auto controller
    switch (auto_mode) {

    case Auto_TakeOff:
        auto_takeoff_run();
        break;

    case Auto_WP:
        auto_wp_run();
        break;

    case Auto_Land:
        auto_land_run();
        break;

    case Auto_RTL:
        auto_rtl_run();
        break;

    case Auto_Circle:
        auto_circle_run();
        break;
    }
}

// auto_takeoff_start - initialises waypoint controller to implement take-off
static void auto_takeoff_start(float final_alt)
{
    auto_mode = Auto_TakeOff;

    // initialise wpnav destination
    Vector3f target_pos = inertial_nav.get_position();
    target_pos.z = final_alt;
    wp_nav.set_wp_destination(target_pos);

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    // tell motors to do a slow start
    motors.slow_start(true);
}

// auto_takeoff_run - takeoff in auto mode
//      called by auto_run at 100hz or more
static void auto_takeoff_run()
{
    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // reset attitude control targets
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
        // tell motors to do a slow start
        motors.slow_start(true);
        // To-Do: re-initialise wpnav targets
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
static void auto_wp_start(const Vector3f& destination)
{
    auto_mode = Auto_WP;

    // initialise wpnav
    wp_nav.set_wp_destination(destination);

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    }
}

// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
static void auto_wp_run()
{
    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // To-Do: reset waypoint origin to current location because copter is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
        // tell motors to do a slow start
        motors.slow_start(true);
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

// auto_land_start - initialises controller to implement a landing
static void auto_land_start()
{
    // set target to stopping point
    Vector3f stopping_point;
    wp_nav.get_loiter_stopping_point_xy(stopping_point);

    // call location specific land start function
    auto_land_start(stopping_point);
}

// auto_land_start - initialises controller to implement a landing
static void auto_land_start(const Vector3f& destination)
{
    auto_mode = Auto_Land;

    // initialise loiter target destination
    wp_nav.set_loiter_target(destination);

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// auto_land_run - lands in auto mode
//      called by auto_run at 100hz or more
static void auto_land_run()
{
    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
        // set target to current position
        wp_nav.init_loiter_target();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    // run loiter controller
    wp_nav.update_loiter();

    // call z-axis position controller
    pos_control.set_alt_target_from_climb_rate(get_throttle_land(), G_Dt);
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
}

// auto_rtl_start - initialises RTL in AUTO flight mode
static void auto_rtl_start()
{
    auto_mode = Auto_RTL;

    // call regular rtl flight mode initialisation and ask it to ignore checks
    rtl_init(true);
}

// auto_rtl_run - rtl in AUTO flight mode
//      called by auto_run at 100hz or more
void auto_rtl_run()
{
    // call regular rtl flight mode run function
    rtl_run();
}

// auto_circle_start - initialises controller to fly a circle in AUTO flight mode
static void auto_circle_start(const Vector3f& center)
{
    auto_mode = Auto_Circle;

    // set circle center
    circle_nav.set_center(center);
}

// auto_circle_run - circle in AUTO flight mode
//      called by auto_run at 100hz or more
void auto_circle_run()
{
    // call circle controller
    circle_nav.update();

    // call z-axis position controller
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), circle_nav.get_yaw(),true);
}

// get_default_auto_yaw_mode - returns auto_yaw_mode based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
uint8_t get_default_auto_yaw_mode(bool rtl)
{
    switch (g.wp_yaw_behavior) {

        case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
            if (rtl) {
                return AUTO_YAW_HOLD;
            }else{
                return AUTO_YAW_LOOK_AT_NEXT_WP;
            }
            break;

        case WP_YAW_BEHAVIOR_LOOK_AHEAD:
            return AUTO_YAW_LOOK_AHEAD;
            break;

        case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
        default:
            return AUTO_YAW_LOOK_AT_NEXT_WP;
            break;
    }
}

// set_auto_yaw_mode - sets the yaw mode for auto
void set_auto_yaw_mode(uint8_t yaw_mode)
{
    // return immediately if no change
    if (auto_yaw_mode == yaw_mode) {
        return;
    }
    auto_yaw_mode = yaw_mode;

    // perform initialisation
    switch (auto_yaw_mode) {

    case AUTO_YAW_LOOK_AT_NEXT_WP:
        // original_wp_bearing will be set by do_nav_wp or other nav command initialisation functions so no init required
        break;

    case AUTO_YAW_ROI:
        // point towards a location held in yaw_look_at_WP
        yaw_look_at_WP_bearing = ahrs.yaw_sensor;
        break;

    case AUTO_YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading with no pilot input allowed
        yaw_look_at_heading = ahrs.yaw_sensor;
        break;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        yaw_look_ahead_bearing = ahrs.yaw_sensor;
        break;

    case AUTO_YAW_RESETTOARMEDYAW:
        // initial_armed_bearing will be set during arming so no init required
        break;
    }
}

// get_auto_heading - returns target heading depending upon auto_yaw_mode
// 100hz update rate
float get_auto_heading(void)
{
    switch(auto_yaw_mode) {

    case AUTO_YAW_ROI:
        // point towards a location held in roi_WP
        return get_roi_yaw();
        break;

    case AUTO_YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading with no pilot input allowed
        return yaw_look_at_heading;
        break;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        return get_look_ahead_yaw();
        break;

    case AUTO_YAW_RESETTOARMEDYAW:
        // changes yaw to be same as when quad was armed
        return initial_armed_bearing;
        break;

    case AUTO_YAW_LOOK_AT_NEXT_WP:
    default:
        // point towards next waypoint.
        // we don't use wp_bearing because we don't want the copter to turn too much during flight
        return original_wp_bearing;
        break;
    }
}

