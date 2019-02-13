#include "Copter.h"

#if MODE_AUTO_ENABLED == ENABLED

/*
 * Init and run calls for auto flight mode
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
bool Copter::ModeAuto::init(bool ignore_checks)
{
    if ((copter.position_ok() && mission.num_commands() > 1) || ignore_checks) {
        _mode = Auto_Loiter;

        // reject switching to auto mode if landed with motors armed but first command is not a takeoff (reduce chance of flips)
        if (motors->armed() && ap.land_complete && !mission.starts_with_takeoff_cmd()) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto: Missing Takeoff Cmd");
            return false;
        }

        // stop ROI from carrying over from previous runs of the mission
        // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
        if (auto_yaw.mode() == AUTO_YAW_ROI) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }

        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();

        // clear guided limits
        copter.mode_guided.limit_clear();

        // start/resume the mission (based on MIS_RESTART parameter)
        mission.start_or_resume();
        return true;
    } else {
        return false;
    }
}

// auto_run - runs the auto controller
//      should be called at 100hz or more
//      relies on run_autopilot being called at 10hz which handles decision making and non-navigation related commands
void Copter::ModeAuto::run()
{
    // call the correct auto controller
    switch (_mode) {

    case Auto_TakeOff:
        takeoff_run();
        break;

    case Auto_WP:
    case Auto_CircleMoveToEdge:
        wp_run();
        break;

    case Auto_Land:
        land_run();
        break;

    case Auto_RTL:
        rtl_run();
        break;

    case Auto_Circle:
        circle_run();
        break;

    case Auto_Spline:
        spline_run();
        break;

    case Auto_NavGuided:
#if NAV_GUIDED == ENABLED
        nav_guided_run();
#endif
        break;

    case Auto_Loiter:
        loiter_run();
        break;

    case Auto_LoiterToAlt:
        loiter_to_alt_run();
        break;

    case Auto_NavPayloadPlace:
        payload_place_run();
        break;
    }
}

// auto_loiter_start - initialises loitering in auto mode
//  returns success/failure because this can be called by exit_mission
bool Copter::ModeAuto::loiter_start()
{
    // return failure if GPS is bad
    if (!copter.position_ok()) {
        return false;
    }
    _mode = Auto_Loiter;

    // calculate stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);

    // initialise waypoint controller target to stopping point
    wp_nav->set_wp_destination(stopping_point);

    // hold yaw at current heading
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    return true;
}

// auto_rtl_start - initialises RTL in AUTO flight mode
void Copter::ModeAuto::rtl_start()
{
    _mode = Auto_RTL;

    // call regular rtl flight mode initialisation and ask it to ignore checks
    copter.mode_rtl.init(true);
}

// auto_takeoff_start - initialises waypoint controller to implement take-off
void Copter::ModeAuto::takeoff_start(const Location& dest_loc)
{
    _mode = Auto_TakeOff;

    Location dest(dest_loc);

    // set horizontal target
    dest.lat = copter.current_loc.lat;
    dest.lng = copter.current_loc.lng;

    // get altitude target
    int32_t alt_target;
    if (!dest.get_alt_cm(Location::ALT_FRAME_ABOVE_HOME, alt_target)) {
        // this failure could only happen if take-off alt was specified as an alt-above terrain and we have no terrain data
        copter.Log_Write_Error(ERROR_SUBSYSTEM_TERRAIN, ERROR_CODE_MISSING_TERRAIN_DATA);
        // fall back to altitude above current altitude
        alt_target = copter.current_loc.alt + dest.alt;
    }

    // sanity check target
    if (alt_target < copter.current_loc.alt) {
        dest.set_alt_cm(copter.current_loc.alt, Location::ALT_FRAME_ABOVE_HOME);
    }
    // Note: if taking off from below home this could cause a climb to an unexpectedly high altitude
    if (alt_target < 100) {
        dest.set_alt_cm(100, Location::ALT_FRAME_ABOVE_HOME);
    }

    // set waypoint controller target
    if (!wp_nav->set_wp_destination(dest)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // get initial alt for WP_NAVALT_MIN
    auto_takeoff_set_start_alt();
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void Copter::ModeAuto::wp_start(const Vector3f& destination)
{
    _mode = Auto_WP;

    // initialise wpnav (no need to check return status because terrain data is not used)
    wp_nav->set_wp_destination(destination, false);

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode_to_default(false);
    }
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void Copter::ModeAuto::wp_start(const Location& dest_loc)
{
    _mode = Auto_WP;

    // send target to waypoint controller
    if (!wp_nav->set_wp_destination(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode_to_default(false);
    }
}

// auto_land_start - initialises controller to implement a landing
void Copter::ModeAuto::land_start()
{
    // set target to stopping point
    Vector3f stopping_point;
    loiter_nav->get_stopping_point_xy(stopping_point);

    // call location specific land start function
    land_start(stopping_point);
}

// auto_land_start - initialises controller to implement a landing
void Copter::ModeAuto::land_start(const Vector3f& destination)
{
    _mode = Auto_Land;

    // initialise loiter target destination
    loiter_nav->init_target(destination);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target(inertial_nav.get_altitude());
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

// auto_circle_movetoedge_start - initialise waypoint controller to move to edge of a circle with it's center at the specified location
//  we assume the caller has performed all required GPS_ok checks
void Copter::ModeAuto::circle_movetoedge_start(const Location &circle_center, float radius_m)
{
    // convert location to vector from ekf origin
    Vector3f circle_center_neu;
    if (!circle_center.get_vector_from_origin_NEU(circle_center_neu)) {
        // default to current position and log error
        circle_center_neu = inertial_nav.get_position();
        copter.Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_FAILED_CIRCLE_INIT);
    }
    copter.circle_nav->set_center(circle_center_neu);

    // set circle radius
    if (!is_zero(radius_m)) {
        copter.circle_nav->set_radius(radius_m * 100.0f);
    }

    // check our distance from edge of circle
    Vector3f circle_edge_neu;
    copter.circle_nav->get_closest_point_on_circle(circle_edge_neu);
    float dist_to_edge = (inertial_nav.get_position() - circle_edge_neu).length();

    // if more than 3m then fly to edge
    if (dist_to_edge > 300.0f) {
        // set the state to move to the edge of the circle
        _mode = Auto_CircleMoveToEdge;

        // convert circle_edge_neu to Location
        Location circle_edge(circle_edge_neu);

        // convert altitude to same as command
        circle_edge.set_alt_cm(circle_center.alt, circle_center.get_alt_frame());

        // initialise wpnav to move to edge of circle
        if (!wp_nav->set_wp_destination(circle_edge)) {
            // failure to set destination can only be because of missing terrain data
            copter.failsafe_terrain_on_event();
        }

        // if we are outside the circle, point at the edge, otherwise hold yaw
        const Vector3f &curr_pos = inertial_nav.get_position();
        float dist_to_center = norm(circle_center_neu.x - curr_pos.x, circle_center_neu.y - curr_pos.y);
        // initialise yaw
        // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
        if (auto_yaw.mode() != AUTO_YAW_ROI) {
            if (dist_to_center > copter.circle_nav->get_radius() && dist_to_center > 500) {
                auto_yaw.set_mode_to_default(false);
            } else {
                // vehicle is within circle so hold yaw to avoid spinning as we move to edge of circle
                auto_yaw.set_mode(AUTO_YAW_HOLD);
            }
        }
    } else {
        circle_start();
    }
}

// auto_circle_start - initialises controller to fly a circle in AUTO flight mode
//   assumes that circle_nav object has already been initialised with circle center and radius
void Copter::ModeAuto::circle_start()
{
    _mode = Auto_Circle;

    // initialise circle controller
    copter.circle_nav->init(copter.circle_nav->get_center());
}

// auto_spline_start - initialises waypoint controller to implement flying to a particular destination using the spline controller
//  seg_end_type can be SEGMENT_END_STOP, SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE.  If Straight or Spline the next_destination should be provided
void Copter::ModeAuto::spline_start(const Location& destination, bool stopped_at_start,
                               AC_WPNav::spline_segment_end_type seg_end_type, 
                               const Location& next_destination)
{
    _mode = Auto_Spline;

    // initialise wpnav
    if (!wp_nav->set_spline_destination(destination, stopped_at_start, seg_end_type, next_destination)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode_to_default(false);
    }
}

#if NAV_GUIDED == ENABLED
// auto_nav_guided_start - hand over control to external navigation controller in AUTO mode
void Copter::ModeAuto::nav_guided_start()
{
    _mode = Auto_NavGuided;

    // call regular guided flight mode initialisation
    copter.mode_guided.init(true);

    // initialise guided start time and position as reference for limit checking
    copter.mode_guided.limit_init_time_and_pos();
}
#endif //NAV_GUIDED

bool Copter::ModeAuto::landing_gear_should_be_deployed() const
{
    switch(_mode) {
    case Auto_Land:
        return true;
    case Auto_RTL:
        return copter.mode_rtl.landing_gear_should_be_deployed();
    default:
        return false;
    }
    return false;
}

// auto_payload_place_start - initialises controller to implement a placing
void Copter::ModeAuto::payload_place_start()
{
    // set target to stopping point
    Vector3f stopping_point;
    loiter_nav->get_stopping_point_xy(stopping_point);

    // call location specific place start function
    payload_place_start(stopping_point);

}

// start_command - this function will be called when the ap_mission lib wishes to start a new command
bool Copter::ModeAuto::start_command(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: logging when new commands start/end
    if (copter.should_log(MASK_LOG_CMD)) {
        copter.logger.Write_Mission_Cmd(mission, cmd);
    }

    switch(cmd.id) {

    ///
    /// navigation commands
    ///
    case MAV_CMD_NAV_TAKEOFF:                   // 22
        do_takeoff(cmd);
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        do_land(cmd);
        break;

    case MAV_CMD_NAV_PAYLOAD_PLACE:              // 94 place at Waypoint
        do_payload_place(cmd);
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // 17 Loiter indefinitely
        do_loiter_unlimited(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              //18 Loiter N Times
        do_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:              // 19
        do_loiter_time(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:
        do_loiter_to_alt(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:             //20
        do_RTL();
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:           // 82  Navigate to Waypoint using spline
        do_spline_wp(cmd);
        break;

#if NAV_GUIDED == ENABLED
    case MAV_CMD_NAV_GUIDED_ENABLE:             // 92  accept navigation commands from external nav computer
        do_nav_guided_enable(cmd);
        break;
#endif

    case MAV_CMD_NAV_DELAY:                    // 94 Delay the next navigation command
        do_nav_delay(cmd);
        break;

    //
    // conditional commands
    //
    case MAV_CMD_CONDITION_DELAY:             // 112
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:             // 114
        do_within_distance(cmd);
        break;

    case MAV_CMD_CONDITION_YAW:             // 115
        do_yaw(cmd);
        break;

    ///
    /// do commands
    ///
    case MAV_CMD_DO_CHANGE_SPEED:             // 178
        do_change_speed(cmd);
        break;

    case MAV_CMD_DO_SET_HOME:             // 179
        do_set_home(cmd);
        break;

    case MAV_CMD_DO_SET_ROI:                // 201
        // point the copter and camera at a region of interest (ROI)
        do_roi(cmd);
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:          // 205
        // point the camera to a specified angle
        do_mount_control(cmd);
        break;
    
    case MAV_CMD_DO_FENCE_ENABLE:
#if AC_FENCE == ENABLED
        if (cmd.p1 == 0) { //disable
            copter.fence.enable(false);
            gcs().send_text(MAV_SEVERITY_INFO, "Fence Disabled");
        } else { //enable fence
            copter.fence.enable(true);
            gcs().send_text(MAV_SEVERITY_INFO, "Fence Enabled");
        }
#endif //AC_FENCE == ENABLED
        break;

#if NAV_GUIDED == ENABLED
    case MAV_CMD_DO_GUIDED_LIMITS:                      // 220  accept guided mode limits
        do_guided_limits(cmd);
        break;
#endif

#if WINCH_ENABLED == ENABLED
    case MAV_CMD_DO_WINCH:                             // Mission command to control winch
        do_winch(cmd);
        break;
#endif

    default:
        // unable to use the command, allow the vehicle to try the next command
        return false;
    }

    // always return success
    return true;
}

// exit_mission - function that is called once the mission completes
void Copter::ModeAuto::exit_mission()
{
    // play a tone
    AP_Notify::events.mission_complete = 1;
    // if we are not on the ground switch to loiter or land
    if (!ap.land_complete) {
        // try to enter loiter but if that fails land
        if (!loiter_start()) {
            set_mode(LAND, MODE_REASON_MISSION_END);
        }
    } else {
        // if we've landed it's safe to disarm
        copter.init_disarm_motors();
    }
}

// do_guided - start guided mode
bool Copter::ModeAuto::do_guided(const AP_Mission::Mission_Command& cmd)
{
    // only process guided waypoint if we are in guided mode
    if (copter.control_mode != GUIDED && !(copter.control_mode == AUTO && mode() == Auto_NavGuided)) {
        return false;
    }

    // switch to handle different commands
    switch (cmd.id) {

        case MAV_CMD_NAV_WAYPOINT:
        {
            // set wp_nav's destination
            Location dest(cmd.content.location);
            return copter.mode_guided.set_destination(dest);
        }

        case MAV_CMD_CONDITION_YAW:
            do_yaw(cmd);
            return true;

        default:
            // reject unrecognised command
            return false;
    }

    return true;
}

uint32_t Copter::ModeAuto::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

int32_t Copter::ModeAuto::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
}

bool Copter::ModeAuto::get_wp(Location& destination)
{
    switch (_mode) {
    case Auto_NavGuided:
        return copter.mode_guided.get_wp(destination);
    case Auto_WP:
        return wp_nav->get_wp_destination(destination);
    default:
        return false;
    }
}

// update mission
void Copter::ModeAuto::run_autopilot()
{
    mission.update();
}

/*******************************************************************************
Verify command Handlers

Each type of mission element has a "verify" operation. The verify
operation returns true when the mission element has completed and we
should move onto the next mission element.
Return true if we do not recognize the command so that we move on to the next command
*******************************************************************************/

// verify_command - callback function called from ap-mission at 10hz or higher when a command is being run
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
bool Copter::ModeAuto::verify_command(const AP_Mission::Mission_Command& cmd)
{
    if (copter.flightmode != &copter.mode_auto) {
        return false;
    }

    bool cmd_complete = false;

    switch (cmd.id) {
    //
    // navigation commands
    //
    case MAV_CMD_NAV_TAKEOFF:
        cmd_complete = verify_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:
        cmd_complete = verify_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:
        cmd_complete = verify_land();
        break;

    case MAV_CMD_NAV_PAYLOAD_PLACE:
        cmd_complete = verify_payload_place();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:
        cmd_complete = verify_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:
        cmd_complete = verify_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        cmd_complete = verify_loiter_time();
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:
        return verify_loiter_to_alt();

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        cmd_complete = verify_RTL();
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:
        cmd_complete = verify_spline_wp(cmd);
        break;

#if NAV_GUIDED == ENABLED
    case MAV_CMD_NAV_GUIDED_ENABLE:
        cmd_complete = verify_nav_guided_enable(cmd);
        break;
#endif

     case MAV_CMD_NAV_DELAY:
        cmd_complete = verify_nav_delay(cmd);
        break;

    ///
    /// conditional commands
    ///
    case MAV_CMD_CONDITION_DELAY:
        cmd_complete = verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        cmd_complete = verify_within_distance();
        break;

    case MAV_CMD_CONDITION_YAW:
        cmd_complete = verify_yaw();
        break;

    // do commands (always return true)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_MOUNT_CONTROL:
    case MAV_CMD_DO_GUIDED_LIMITS:
    case MAV_CMD_DO_FENCE_ENABLE:
    case MAV_CMD_DO_WINCH:
        cmd_complete = true;
        break;

    default:
        // error message
        gcs().send_text(MAV_SEVERITY_WARNING,"Skipping invalid cmd #%i",cmd.id);
        // return true if we do not recognize the command so that we move on to the next command
        cmd_complete = true;
        break;
    }


    // send message to GCS
    if (cmd_complete) {
        gcs().send_mission_item_reached_message(cmd.index);
    }

    return cmd_complete;
}

// auto_takeoff_run - takeoff in auto mode
//      called by auto_run at 100hz or more
void Copter::ModeAuto::takeoff_run()
{
    auto_takeoff_run();
    if (wp_nav->reached_wp_destination()) {
        const Vector3f target = wp_nav->get_wp_destination();
        wp_start(target);
    }
}

// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void Copter::ModeAuto::wp_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
        // To-Do: reset waypoint origin to current location because copter is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
        zero_throttle_and_relax_ac();
        // clear i term when we're taking off
        set_throttle_takeoff();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}

// auto_spline_run - runs the auto spline controller
//      called by auto_run at 100hz or more
void Copter::ModeAuto::spline_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
        // To-Do: reset waypoint origin to current location because copter is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
        zero_throttle_and_relax_ac();
        // clear i term when we're taking off
        set_throttle_takeoff();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rat
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    wp_nav->update_spline();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}

// auto_land_run - lands in auto mode
//      called by auto_run at 100hz or more
void Copter::ModeAuto::land_run()
{
    // if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || ap.land_complete || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        // set target to current position
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    
    land_run_horizontal_control();
    land_run_vertical_control();
}

// auto_rtl_run - rtl in AUTO flight mode
//      called by auto_run at 100hz or more
void Copter::ModeAuto::rtl_run()
{
    // call regular rtl flight mode run function
    copter.mode_rtl.run(false);
}

// auto_circle_run - circle in AUTO flight mode
//      called by auto_run at 100hz or more
void Copter::ModeAuto::circle_run()
{
    // call circle controller
    copter.circle_nav->update();

    // call z-axis position controller
    pos_control->update_z_controller();

    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_yaw(copter.circle_nav->get_roll(), copter.circle_nav->get_pitch(), copter.circle_nav->get_yaw(), true);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(copter.circle_nav->get_roll(), copter.circle_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}

#if NAV_GUIDED == ENABLED
// auto_nav_guided_run - allows control by external navigation controller
//      called by auto_run at 100hz or more
void Copter::ModeAuto::nav_guided_run()
{
    // call regular guided flight mode run function
    copter.mode_guided.run();
}
#endif  // NAV_GUIDED

// auto_loiter_run - loiter in AUTO flight mode
//      called by auto_run at 100hz or more
void Copter::ModeAuto::loiter_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || ap.land_complete || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // accept pilot input of yaw
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint and z-axis position controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
}

// auto_loiter_run - loiter to altitude in AUTO flight mode
//      called by auto_run at 100hz or more
void Copter::ModeAuto::loiter_to_alt_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || ap.land_complete || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // possibly just run the waypoint controller:
    if (!loiter_to_alt.reached_destination_xy) {
        loiter_to_alt.reached_destination_xy = wp_nav->reached_wp_destination_xy();
        if (!loiter_to_alt.reached_destination_xy) {
            wp_run();
            return;
        }
    }

    if (!loiter_to_alt.loiter_start_done) {
        loiter_nav->init_target();
        _mode = Auto_LoiterToAlt;
        loiter_to_alt.loiter_start_done = true;
    }
    const float alt_error_cm = copter.current_loc.alt - loiter_to_alt.alt;
    if (fabsf(alt_error_cm) < 5.0) { // random numbers R US
        loiter_to_alt.reached_alt = true;
    } else if (alt_error_cm * loiter_to_alt.alt_error_cm < 0) {
        // we were above and are now below, or vice-versa
        loiter_to_alt.reached_alt = true;
    }
    loiter_to_alt.alt_error_cm = alt_error_cm;

    // loiter...

    land_run_horizontal_control();

    // Compute a vertical velocity demand such that the vehicle
    // approaches the desired altitude.
    float target_climb_rate = AC_AttitudeControl::sqrt_controller(
        -alt_error_cm,
        pos_control->get_pos_z_p().kP(),
        pos_control->get_max_accel_z(),
        G_Dt);

    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    pos_control->update_z_controller();
}

// auto_payload_place_start - initialises controller to implement placement of a load
void Copter::ModeAuto::payload_place_start(const Vector3f& destination)
{
    _mode = Auto_NavPayloadPlace;
    nav_payload_place.state = PayloadPlaceStateType_Calibrating_Hover_Start;

    // initialise loiter target destination
    loiter_nav->init_target(destination);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target(inertial_nav.get_altitude());
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

// auto_payload_place_run - places an object in auto mode
//      called by auto_run at 100hz or more
void Copter::ModeAuto::payload_place_run()
{
    if (!payload_place_run_should_run()) {
        zero_throttle_and_relax_ac();
        // set target to current position
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    switch (nav_payload_place.state) {
    case PayloadPlaceStateType_FlyToLocation:
        return wp_run();
    case PayloadPlaceStateType_Calibrating_Hover_Start:
    case PayloadPlaceStateType_Calibrating_Hover:
        return payload_place_run_loiter();
    case PayloadPlaceStateType_Descending_Start:
    case PayloadPlaceStateType_Descending:
        return payload_place_run_descend();
    case PayloadPlaceStateType_Releasing_Start:
    case PayloadPlaceStateType_Releasing:
    case PayloadPlaceStateType_Released:
    case PayloadPlaceStateType_Ascending_Start:
    case PayloadPlaceStateType_Ascending:
    case PayloadPlaceStateType_Done:
        return payload_place_run_loiter();
    }
}

bool Copter::ModeAuto::payload_place_run_should_run()
{
    // muts be armed
    if (!motors->armed()) {
        return false;
    }
    // muts be auto-armed
    if (!ap.auto_armed) {
        return false;
    }
    // must not be landed
    if (ap.land_complete) {
        return false;
    }
    // interlock must be enabled (i.e. unsafe)
    if (!motors->get_interlock()) {
        return false;
    }

    return true;
}

void Copter::ModeAuto::payload_place_run_loiter()
{
    // loiter...
    land_run_horizontal_control();

    // run loiter controller
    loiter_nav->update();

    // call attitude controller
    const float target_yaw_rate = 0;
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);

    // call position controller
    pos_control->update_z_controller();
}

void Copter::ModeAuto::payload_place_run_descend()
{
    land_run_horizontal_control();
    land_run_vertical_control();
}

// terrain_adjusted_location: returns a Location with lat/lon from cmd
// and altitude from our current altitude adjusted for location
Location Copter::ModeAuto::terrain_adjusted_location(const AP_Mission::Mission_Command& cmd) const
{
    // convert to location class
    Location target_loc(cmd.content.location);

    // decide if we will use terrain following
    int32_t curr_terr_alt_cm, target_terr_alt_cm;
    if (copter.current_loc.get_alt_cm(Location::ALT_FRAME_ABOVE_TERRAIN, curr_terr_alt_cm) &&
        target_loc.get_alt_cm(Location::ALT_FRAME_ABOVE_TERRAIN, target_terr_alt_cm)) {
        curr_terr_alt_cm = MAX(curr_terr_alt_cm,200);
        // if using terrain, set target altitude to current altitude above terrain
        target_loc.set_alt_cm(curr_terr_alt_cm, Location::ALT_FRAME_ABOVE_TERRAIN);
    } else {
        // set target altitude to current altitude above home
        target_loc.set_alt_cm(copter.current_loc.alt, Location::ALT_FRAME_ABOVE_HOME);
    }
    return target_loc;
}

/********************************************************************************/
//	Nav (Must) commands
/********************************************************************************/

// do_takeoff - initiate takeoff navigation command
void Copter::ModeAuto::do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    // Set wp navigation target to safe altitude above current position
    takeoff_start(cmd.content.location);
}

Location Copter::ModeAuto::loc_from_cmd(const AP_Mission::Mission_Command& cmd) const
{
    Location ret(cmd.content.location);

    // use current lat, lon if zero
    if (ret.lat == 0 && ret.lng == 0) {
        ret.lat = copter.current_loc.lat;
        ret.lng = copter.current_loc.lng;
    }
    // use current altitude if not provided
    if (ret.alt == 0) {
        // set to current altitude but in command's alt frame
        int32_t curr_alt;
        if (copter.current_loc.get_alt_cm(ret.get_alt_frame(),curr_alt)) {
            ret.set_alt_cm(curr_alt, ret.get_alt_frame());
        } else {
            // default to current altitude as alt-above-home
            ret.set_alt_cm(copter.current_loc.alt,
                           copter.current_loc.get_alt_frame());
        }
    }
    return ret;
}

// do_nav_wp - initiate move to next waypoint
void Copter::ModeAuto::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    Location target_loc = loc_from_cmd(cmd);

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;

    // Set wp navigation target
    wp_start(target_loc);

    // if no delay as well as not final waypoint set the waypoint as "fast"
    AP_Mission::Mission_Command temp_cmd;
    if (loiter_time_max == 0 && mission.get_next_nav_cmd(cmd.index+1, temp_cmd)) {
        copter.wp_nav->set_fast_waypoint(true);
    }
}

// do_land - initiate landing procedure
void Copter::ModeAuto::do_land(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: check if we have already landed

    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // set state to fly to location
        land_state = LandStateType_FlyToLocation;

        const Location target_loc = terrain_adjusted_location(cmd);

        wp_start(target_loc);
    } else {
        // set landing state
        land_state = LandStateType_Descending;

        // initialise landing controller
        land_start();
    }
}

// do_loiter_unlimited - start loitering with no end conditions
// note: caller should set yaw_mode
void Copter::ModeAuto::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    // convert back to location
    Location target_loc(cmd.content.location);

    // use current location if not provided
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        // To-Do: make this simpler
        Vector3f temp_pos;
        copter.wp_nav->get_wp_stopping_point_xy(temp_pos);
        const Location temp_loc(temp_pos);
        target_loc.lat = temp_loc.lat;
        target_loc.lng = temp_loc.lng;
    }

    // use current altitude if not provided
    // To-Do: use z-axis stopping point instead of current alt
    if (target_loc.alt == 0) {
        // set to current altitude but in command's alt frame
        int32_t curr_alt;
        if (copter.current_loc.get_alt_cm(target_loc.get_alt_frame(),curr_alt)) {
            target_loc.set_alt_cm(curr_alt, target_loc.get_alt_frame());
        } else {
            // default to current altitude as alt-above-home
            target_loc.set_alt_cm(copter.current_loc.alt,
                                  copter.current_loc.get_alt_frame());
        }
    }

    // start way point navigator and provide it the desired location
    wp_start(target_loc);
}

// do_circle - initiate moving in a circle
void Copter::ModeAuto::do_circle(const AP_Mission::Mission_Command& cmd)
{
    const Location circle_center = loc_from_cmd(cmd);

    // calculate radius
    uint8_t circle_radius_m = HIGHBYTE(cmd.p1); // circle radius held in high byte of p1

    // move to edge of circle (verify_circle) will ensure we begin circling once we reach the edge
    circle_movetoedge_start(circle_center, circle_radius_m);
}

// do_loiter_time - initiate loitering at a point for a given time period
// note: caller should set yaw_mode
void Copter::ModeAuto::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    // re-use loiter unlimited
    do_loiter_unlimited(cmd);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = cmd.p1;     // units are (seconds)
}

// do_loiter_alt - initiate loitering at a point until a given altitude is reached
// note: caller should set yaw_mode
void Copter::ModeAuto::do_loiter_to_alt(const AP_Mission::Mission_Command& cmd)
{
    // re-use loiter unlimited
    do_loiter_unlimited(cmd);
    _mode = Auto_LoiterToAlt;

    // if we aren't navigating to a location then we have to adjust
    // altitude for current location
    Location target_loc(cmd.content.location);
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        target_loc.lat = copter.current_loc.lat;
        target_loc.lng = copter.current_loc.lng;
    }

    if (!target_loc.get_alt_cm(Location::ALT_FRAME_ABOVE_HOME, loiter_to_alt.alt)) {
        loiter_to_alt.reached_destination_xy = true;
        loiter_to_alt.reached_alt = true;
        gcs().send_text(MAV_SEVERITY_INFO, "bad do_loiter_to_alt");
        return;
    }
    loiter_to_alt.reached_destination_xy = false;
    loiter_to_alt.loiter_start_done = false;
    loiter_to_alt.reached_alt = false;
    loiter_to_alt.alt_error_cm = 0;

    pos_control->set_max_accel_z(wp_nav->get_accel_z());
    pos_control->set_max_speed_z(wp_nav->get_default_speed_down(),
                                 wp_nav->get_default_speed_up());

    if (pos_control->is_active_z()) {
        pos_control->freeze_ff_z();
    }
}

// do_spline_wp - initiate move to next waypoint
void Copter::ModeAuto::do_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    const Location target_loc = loc_from_cmd(cmd);

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;

    // determine segment start and end type
    bool stopped_at_start = true;
    AC_WPNav::spline_segment_end_type seg_end_type = AC_WPNav::SEGMENT_END_STOP;
    AP_Mission::Mission_Command temp_cmd;

    // if previous command was a wp_nav command with no delay set stopped_at_start to false
    // To-Do: move processing of delay into wp-nav controller to allow it to determine the stopped_at_start value itself?
    uint16_t prev_cmd_idx = mission.get_prev_nav_cmd_index();
    if (prev_cmd_idx != AP_MISSION_CMD_INDEX_NONE) {
        if (mission.read_cmd_from_storage(prev_cmd_idx, temp_cmd)) {
            if ((temp_cmd.id == MAV_CMD_NAV_WAYPOINT || temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT) && temp_cmd.p1 == 0) {
                stopped_at_start = false;
            }
        }
    }

    // if there is no delay at the end of this segment get next nav command
    Location next_loc;
    if (cmd.p1 == 0 && mission.get_next_nav_cmd(cmd.index+1, temp_cmd)) {
        next_loc = temp_cmd.content.location;
        // default lat, lon to first waypoint's lat, lon
        if (next_loc.lat == 0 && next_loc.lng == 0) {
            next_loc.lat = target_loc.lat;
            next_loc.lng = target_loc.lng;
        }
        // default alt to first waypoint's alt but in next waypoint's alt frame
        if (next_loc.alt == 0) {
            int32_t next_alt;
            if (target_loc.get_alt_cm(next_loc.get_alt_frame(), next_alt)) {
                next_loc.set_alt_cm(next_alt, next_loc.get_alt_frame());
            } else {
                // default to first waypoints altitude
                next_loc.set_alt_cm(target_loc.alt, target_loc.get_alt_frame());
            }
        }
        // if the next nav command is a waypoint set end type to spline or straight
        if (temp_cmd.id == MAV_CMD_NAV_WAYPOINT) {
            seg_end_type = AC_WPNav::SEGMENT_END_STRAIGHT;
        } else if (temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT) {
            seg_end_type = AC_WPNav::SEGMENT_END_SPLINE;
        }
    }

    // set spline navigation target
    spline_start(target_loc, stopped_at_start, seg_end_type, next_loc);
}

#if NAV_GUIDED == ENABLED
// do_nav_guided_enable - initiate accepting commands from external nav computer
void Copter::ModeAuto::do_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 > 0) {
        // initialise guided limits
        copter.mode_guided.limit_init_time_and_pos();

        // set spline navigation target
        nav_guided_start();
    }
}

// do_guided_limits - pass guided limits to guided controller
void Copter::ModeAuto::do_guided_limits(const AP_Mission::Mission_Command& cmd)
{
    copter.mode_guided.limit_set(
        cmd.p1 * 1000, // convert seconds to ms
        cmd.content.guided_limits.alt_min * 100.0f,    // convert meters to cm
        cmd.content.guided_limits.alt_max * 100.0f,    // convert meters to cm
        cmd.content.guided_limits.horiz_max * 100.0f); // convert meters to cm
}
#endif  // NAV_GUIDED

// do_nav_delay - Delay the next navigation command
void Copter::ModeAuto::do_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    nav_delay_time_start = millis();

    if (cmd.content.nav_delay.seconds > 0) {
        // relative delay
        nav_delay_time_max = cmd.content.nav_delay.seconds * 1000; // convert seconds to milliseconds
    } else {
        // absolute delay to utc time
        nav_delay_time_max = AP::rtc().get_time_utc(cmd.content.nav_delay.hour_utc, cmd.content.nav_delay.min_utc, cmd.content.nav_delay.sec_utc, 0);
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Delaying %u sec",(unsigned int)(nav_delay_time_max/1000));
}

/********************************************************************************/
//	Condition (May) commands
/********************************************************************************/

void Copter::ModeAuto::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value = cmd.content.delay.seconds * 1000;     // convert seconds to milliseconds
}

void Copter::ModeAuto::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters * 100;
}

void Copter::ModeAuto::do_yaw(const AP_Mission::Mission_Command& cmd)
{
	auto_yaw.set_fixed_yaw(
		cmd.content.yaw.angle_deg,
		cmd.content.yaw.turn_rate_dps,
		cmd.content.yaw.direction,
		cmd.content.yaw.relative_angle > 0);
}

/********************************************************************************/
//	Do (Now) commands
/********************************************************************************/



void Copter::ModeAuto::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.speed.target_ms > 0) {
        if (cmd.content.speed.speed_type == 2)  {
            copter.wp_nav->set_speed_up(cmd.content.speed.target_ms * 100.0f);
        } else if (cmd.content.speed.speed_type == 3)  {
            copter.wp_nav->set_speed_down(cmd.content.speed.target_ms * 100.0f);
        } else {
            copter.wp_nav->set_speed_xy(cmd.content.speed.target_ms * 100.0f);
        }
    }
}

void Copter::ModeAuto::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 || (cmd.content.location.lat == 0 && cmd.content.location.lng == 0 && cmd.content.location.alt == 0)) {
        copter.set_home_to_current_location(false);
    } else {
        copter.set_home(cmd.content.location, false);
    }
}

// do_roi - starts actions required by MAV_CMD_DO_SET_ROI
//          this involves either moving the camera to point at the ROI (region of interest)
//          and possibly rotating the copter to point at the ROI if our mount type does not support a yaw feature
//	TO-DO: add support for other features of MAV_CMD_DO_SET_ROI including pointing at a given waypoint
void Copter::ModeAuto::do_roi(const AP_Mission::Mission_Command& cmd)
{
    auto_yaw.set_roi(cmd.content.location);
}

// point the camera to a specified angle
void Copter::ModeAuto::do_mount_control(const AP_Mission::Mission_Command& cmd)
{
#if MOUNT == ENABLED
    if (!copter.camera_mount.has_pan_control()) {
        auto_yaw.set_fixed_yaw(cmd.content.mount_control.yaw,0.0f,0,0);
    }
    copter.camera_mount.set_angle_targets(cmd.content.mount_control.roll, cmd.content.mount_control.pitch, cmd.content.mount_control.yaw);
#endif
}

#if WINCH_ENABLED == ENABLED
// control winch based on mission command
void Copter::ModeAuto::do_winch(const AP_Mission::Mission_Command& cmd)
{
    // Note: we ignore the gripper num parameter because we only support one gripper
    switch (cmd.content.winch.action) {
        case WINCH_RELAXED:
            g2.winch.relax();
            Log_Write_Event(DATA_WINCH_RELAXED);
            break;
        case WINCH_RELATIVE_LENGTH_CONTROL:
            g2.winch.release_length(cmd.content.winch.release_length, cmd.content.winch.release_rate);
            Log_Write_Event(DATA_WINCH_LENGTH_CONTROL);
            break;
        case WINCH_RATE_CONTROL:
            g2.winch.set_desired_rate(cmd.content.winch.release_rate);
            Log_Write_Event(DATA_WINCH_RATE_CONTROL);
            break;
        default:
            // do nothing
            break;
    }
}
#endif

// do_payload_place - initiate placing procedure
void Copter::ModeAuto::do_payload_place(const AP_Mission::Mission_Command& cmd)
{
    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // set state to fly to location
        nav_payload_place.state = PayloadPlaceStateType_FlyToLocation;

        const Location target_loc = terrain_adjusted_location(cmd);

        wp_start(target_loc);
    } else {
        nav_payload_place.state = PayloadPlaceStateType_Calibrating_Hover_Start;

        // initialise placing controller
        payload_place_start();
    }
    nav_payload_place.descend_max = cmd.p1;
}

// do_RTL - start Return-to-Launch
void Copter::ModeAuto::do_RTL(void)
{
    // start rtl in auto flight mode
    rtl_start();
}

/********************************************************************************/
//	Verify Nav (Must) commands
/********************************************************************************/

// verify_takeoff - check if we have completed the takeoff
bool Copter::ModeAuto::verify_takeoff()
{
    // have we reached our target altitude?
    return copter.wp_nav->reached_wp_destination();
}

// verify_land - returns true if landing has been completed
bool Copter::ModeAuto::verify_land()
{
    bool retval = false;

    switch (land_state) {
        case LandStateType_FlyToLocation:
            // check if we've reached the location
            if (copter.wp_nav->reached_wp_destination()) {
                // get destination so we can use it for loiter target
                Vector3f dest = copter.wp_nav->get_wp_destination();

                // initialise landing controller
                land_start(dest);

                // advance to next state
                land_state = LandStateType_Descending;
            }
            break;

        case LandStateType_Descending:
            // rely on THROTTLE_LAND mode to correctly update landing status
            retval = ap.land_complete;
            break;

        default:
            // this should never happen
            // TO-DO: log an error
            retval = true;
            break;
    }

    // true is returned if we've successfully landed
    return retval;
}

#define NAV_PAYLOAD_PLACE_DEBUGGING 0

#if NAV_PAYLOAD_PLACE_DEBUGGING
#include <stdio.h>
#define debug(fmt, args ...)  do {::fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

// verify_payload_place - returns true if placing has been completed
bool Copter::ModeAuto::verify_payload_place()
{
    const uint16_t hover_throttle_calibrate_time = 2000; // milliseconds
    const uint16_t descend_throttle_calibrate_time = 2000; // milliseconds
    const float hover_throttle_placed_fraction = 0.7; // i.e. if throttle is less than 70% of hover we have placed
    const float descent_throttle_placed_fraction = 0.9; // i.e. if throttle is less than 90% of descent throttle we have placed
    const uint16_t placed_time = 500; // how long we have to be below a throttle threshold before considering placed

    const float current_throttle_level = motors->get_throttle();
    const uint32_t now =  AP_HAL::millis();

    // if we discover we've landed then immediately release the load:
    if (ap.land_complete) {
        switch (nav_payload_place.state) {
        case PayloadPlaceStateType_FlyToLocation:
        case PayloadPlaceStateType_Calibrating_Hover_Start:
        case PayloadPlaceStateType_Calibrating_Hover:
        case PayloadPlaceStateType_Descending_Start:
        case PayloadPlaceStateType_Descending:
            gcs().send_text(MAV_SEVERITY_INFO, "NAV_PLACE: landed");
            nav_payload_place.state = PayloadPlaceStateType_Releasing_Start;
            break;
        case PayloadPlaceStateType_Releasing_Start:
        case PayloadPlaceStateType_Releasing:
        case PayloadPlaceStateType_Released:
        case PayloadPlaceStateType_Ascending_Start:
        case PayloadPlaceStateType_Ascending:
        case PayloadPlaceStateType_Done:
            break;
        }
    }

    switch (nav_payload_place.state) {
    case PayloadPlaceStateType_FlyToLocation:
        if (!copter.wp_nav->reached_wp_destination()) {
            return false;
        }
        payload_place_start();
        return false;
    case PayloadPlaceStateType_Calibrating_Hover_Start:
        // hover for 1 second to get an idea of what our hover
        // throttle looks like
        debug("Calibrate start");
        nav_payload_place.hover_start_timestamp = now;
        nav_payload_place.state = PayloadPlaceStateType_Calibrating_Hover;
        FALLTHROUGH;
    case PayloadPlaceStateType_Calibrating_Hover: {
        if (now - nav_payload_place.hover_start_timestamp < hover_throttle_calibrate_time) {
            // still calibrating...
            debug("Calibrate Timer: %d", now - nav_payload_place.hover_start_timestamp);
            return false;
        }
        // we have a valid calibration.  Hopefully.
        nav_payload_place.hover_throttle_level = current_throttle_level;
        const float hover_throttle_delta = fabsf(nav_payload_place.hover_throttle_level - motors->get_throttle_hover());
        gcs().send_text(MAV_SEVERITY_INFO, "hover throttle delta: %f", static_cast<double>(hover_throttle_delta));
        nav_payload_place.state = PayloadPlaceStateType_Descending_Start;
        }
        FALLTHROUGH;
    case PayloadPlaceStateType_Descending_Start:
        nav_payload_place.descend_start_timestamp = now;
        nav_payload_place.descend_start_altitude = inertial_nav.get_altitude();
        nav_payload_place.descend_throttle_level = 0;
        nav_payload_place.state = PayloadPlaceStateType_Descending;
        FALLTHROUGH;
    case PayloadPlaceStateType_Descending:
        // make sure we don't descend too far:
        debug("descended: %f cm (%f cm max)", (nav_payload_place.descend_start_altitude - inertial_nav.get_altitude()), nav_payload_place.descend_max);
        if (!is_zero(nav_payload_place.descend_max) &&
            nav_payload_place.descend_start_altitude - inertial_nav.get_altitude()  > nav_payload_place.descend_max) {
            nav_payload_place.state = PayloadPlaceStateType_Ascending;
            gcs().send_text(MAV_SEVERITY_WARNING, "Reached maximum descent");
            return false; // we'll do any cleanups required next time through the loop
        }
        // see if we've been descending long enough to calibrate a descend-throttle-level:
        if (is_zero(nav_payload_place.descend_throttle_level) &&
            now - nav_payload_place.descend_start_timestamp > descend_throttle_calibrate_time) {
            nav_payload_place.descend_throttle_level = current_throttle_level;
        }
        // watch the throttle to determine whether the load has been placed
        // debug("hover ratio: %f   descend ratio: %f\n", current_throttle_level/nav_payload_place.hover_throttle_level, ((nav_payload_place.descend_throttle_level == 0) ? -1.0f : current_throttle_level/nav_payload_place.descend_throttle_level));
        if (current_throttle_level/nav_payload_place.hover_throttle_level > hover_throttle_placed_fraction &&
            (is_zero(nav_payload_place.descend_throttle_level) ||
             current_throttle_level/nav_payload_place.descend_throttle_level > descent_throttle_placed_fraction)) {
            // throttle is above both threshold ratios (or above hover threshold ration and descent threshold ratio not yet valid)
            nav_payload_place.place_start_timestamp = 0;
            return false;
        }
        if (nav_payload_place.place_start_timestamp == 0) {
            // we've only just now hit the correct throttle level
            nav_payload_place.place_start_timestamp = now;
            return false;
        } else if (now - nav_payload_place.place_start_timestamp < placed_time) {
            // keep going down....
            debug("Place Timer: %d", now - nav_payload_place.place_start_timestamp);
            return false;
        }
        nav_payload_place.state = PayloadPlaceStateType_Releasing_Start;
        FALLTHROUGH;
    case PayloadPlaceStateType_Releasing_Start:
#if GRIPPER_ENABLED == ENABLED
        if (g2.gripper.valid()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Releasing the gripper");
            g2.gripper.release();
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Gripper not valid");
            nav_payload_place.state = PayloadPlaceStateType_Ascending_Start;
            break;
        }
#else
        gcs().send_text(MAV_SEVERITY_INFO, "Gripper code disabled");
#endif
        nav_payload_place.state = PayloadPlaceStateType_Releasing;
        FALLTHROUGH;
    case PayloadPlaceStateType_Releasing:
#if GRIPPER_ENABLED == ENABLED
        if (g2.gripper.valid() && !g2.gripper.released()) {
            return false;
        }
#endif
        nav_payload_place.state = PayloadPlaceStateType_Released;
        FALLTHROUGH;
    case PayloadPlaceStateType_Released: {
        nav_payload_place.state = PayloadPlaceStateType_Ascending_Start;
        }
        FALLTHROUGH;
    case PayloadPlaceStateType_Ascending_Start: {
        Location target_loc = inertial_nav.get_position();
        target_loc.alt = nav_payload_place.descend_start_altitude;
        wp_start(target_loc);
        nav_payload_place.state = PayloadPlaceStateType_Ascending;
        }
        FALLTHROUGH;
    case PayloadPlaceStateType_Ascending:
        if (!copter.wp_nav->reached_wp_destination()) {
            return false;
        }
        nav_payload_place.state = PayloadPlaceStateType_Done;
        FALLTHROUGH;
    case PayloadPlaceStateType_Done:
        return true;
    default:
        // this should never happen
        // TO-DO: log an error
        return true;
    }
    // should never get here
    return true;
}
#undef debug

bool Copter::ModeAuto::verify_loiter_unlimited()
{
    return false;
}

// verify_loiter_time - check if we have loitered long enough
bool Copter::ModeAuto::verify_loiter_time()
{
    // return immediately if we haven't reached our destination
    if (!copter.wp_nav->reached_wp_destination()) {
        return false;
    }

    // start our loiter timer
    if ( loiter_time == 0 ) {
        loiter_time = millis();
    }

    // check if loiter timer has run out
    return (((millis() - loiter_time) / 1000) >= loiter_time_max);
}

// verify_loiter_to_alt - check if we have reached both destination
// (roughly) and altitude (precisely)
bool Copter::ModeAuto::verify_loiter_to_alt()
{
    if (loiter_to_alt.reached_destination_xy &&
        loiter_to_alt.reached_alt) {
        return true;
    }
    return false;
}

// verify_RTL - handles any state changes required to implement RTL
// do_RTL should have been called once first to initialise all variables
// returns true with RTL has completed successfully
bool Copter::ModeAuto::verify_RTL()
{
    return (copter.mode_rtl.state_complete() && (copter.mode_rtl.state() == RTL_FinalDescent || copter.mode_rtl.state() == RTL_Land));
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

bool Copter::ModeAuto::verify_wait_delay()
{
    if (millis() - condition_start > (uint32_t)MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

bool Copter::ModeAuto::verify_within_distance()
{
    if (wp_distance() < (uint32_t)MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

// verify_yaw - return true if we have reached the desired heading
bool Copter::ModeAuto::verify_yaw()
{
    // set yaw mode if it has been changed (the waypoint controller often retakes control of yaw as it executes a new waypoint command)
    if (auto_yaw.mode() != AUTO_YAW_FIXED) {
        auto_yaw.set_mode(AUTO_YAW_FIXED);
    }

    // check if we are within 2 degrees of the target heading
    return (fabsf(wrap_180_cd(ahrs.yaw_sensor-auto_yaw.yaw())) <= 200);
}

// verify_nav_wp - check if we have reached the next way point
bool Copter::ModeAuto::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if ( !copter.wp_nav->reached_wp_destination() ) {
        return false;
    }

    // start timer if necessary
    if (loiter_time == 0) {
        loiter_time = millis();
		if (loiter_time_max > 0) {
			// play a tone
			AP_Notify::events.waypoint_complete = 1;
			}
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
		if (loiter_time_max == 0) {
			// play a tone
			AP_Notify::events.waypoint_complete = 1;
			}
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    } else {
        return false;
    }
}

// verify_circle - check if we have circled the point enough
bool Copter::ModeAuto::verify_circle(const AP_Mission::Mission_Command& cmd)
{
    // check if we've reached the edge
    if (mode() == Auto_CircleMoveToEdge) {
        if (copter.wp_nav->reached_wp_destination()) {
            Vector3f circle_center;
            if (!cmd.content.location.get_vector_from_origin_NEU(circle_center)) {
                // should never happen
                return true;
            }
            const Vector3f curr_pos = copter.inertial_nav.get_position();
            // set target altitude if not provided
            if (is_zero(circle_center.z)) {
                circle_center.z = curr_pos.z;
            }

            // set lat/lon position if not provided
            if (cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
                circle_center.x = curr_pos.x;
                circle_center.y = curr_pos.y;
            }

            // start circling
            circle_start();
        }
        return false;
    }

    // check if we have completed circling
    return fabsf(copter.circle_nav->get_angle_total()/M_2PI) >= LOWBYTE(cmd.p1);
}

// verify_spline_wp - check if we have reached the next way point using spline
bool Copter::ModeAuto::verify_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if ( !copter.wp_nav->reached_wp_destination() ) {
        return false;
    }

    // start timer if necessary
    if (loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    } else {
        return false;
    }
}

#if NAV_GUIDED == ENABLED
// verify_nav_guided - check if we have breached any limits
bool Copter::ModeAuto::verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    // if disabling guided mode then immediately return true so we move to next command
    if (cmd.p1 == 0) {
        return true;
    }

    // check time and position limits
    return copter.mode_guided.limit_check();
}
#endif  // NAV_GUIDED

// verify_nav_delay - check if we have waited long enough
bool Copter::ModeAuto::verify_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    if (millis() - nav_delay_time_start > (uint32_t)MAX(nav_delay_time_max,0)) {
        nav_delay_time_max = 0;
        return true;
    }
    return false;
}

#endif
