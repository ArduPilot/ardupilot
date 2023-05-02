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
bool ModeAuto::init(bool ignore_checks)
{
    auto_RTL = false;
    if (mission.num_commands() > 1 || ignore_checks) {
        _mode = SubMode::LOITER;

        // reject switching to auto mode if landed with motors armed but first command is not a takeoff (reduce chance of flips)
        if (motors->armed() && copter.ap.land_complete && !mission.starts_with_takeoff_cmd()) {
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

        // set flag to start mission
        waiting_to_start = true;

        // initialise mission change check (ignore results)
        IGNORE_RETURN(mis_change_detector.check_for_mission_change());

        // clear guided limits
        copter.mode_guided.limit_clear();

        // reset flag indicating if pilot has applied roll or pitch inputs during landing
        copter.ap.land_repo_active = false;

#if PRECISION_LANDING == ENABLED
        // initialise precland state machine
        copter.precland_statemachine.init();
#endif

        return true;
    } else {
        return false;
    }
}

// stop mission when we leave auto mode
void ModeAuto::exit()
{
    if (copter.mode_auto.mission.state() == AP_Mission::MISSION_RUNNING) {
        copter.mode_auto.mission.stop();
    }
#if HAL_MOUNT_ENABLED
    copter.camera_mount.set_mode_to_default();
#endif  // HAL_MOUNT_ENABLED

    auto_RTL = false;
}

// auto_run - runs the auto controller
//      should be called at 100hz or more
void ModeAuto::run()
{
    // start or update mission
    if (waiting_to_start) {
        // don't start the mission until we have an origin
        Location loc;
        if (copter.ahrs.get_origin(loc)) {
            // start/resume the mission (based on MIS_RESTART parameter)
            mission.start_or_resume();
            waiting_to_start = false;

            // initialise mission change check (ignore results)
            IGNORE_RETURN(mis_change_detector.check_for_mission_change());
        }
    } else {
        // check for mission changes
        if (mis_change_detector.check_for_mission_change()) {
            // if mission is running restart the current command if it is a waypoint or spline command
            if ((mission.state() == AP_Mission::MISSION_RUNNING) && (_mode == SubMode::WP)) {
                if (mission.restart_current_nav_cmd()) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto mission changed, restarted command");
                } else {
                    // failed to restart mission for some reason
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto mission changed but failed to restart command");
                }
            }
        }

        mission.update();
    }

    // call the correct auto controller
    switch (_mode) {

    case SubMode::TAKEOFF:
        takeoff_run();
        break;

    case SubMode::WP:
    case SubMode::CIRCLE_MOVE_TO_EDGE:
        wp_run();
        break;

    case SubMode::LAND:
        land_run();
        break;

    case SubMode::RTL:
        rtl_run();
        break;

    case SubMode::CIRCLE:
        circle_run();
        break;

    case SubMode::NAVGUIDED:
    case SubMode::NAV_SCRIPT_TIME:
#if NAV_GUIDED == ENABLED || AP_SCRIPTING_ENABLED
        nav_guided_run();
#endif
        break;

    case SubMode::LOITER:
        loiter_run();
        break;

    case SubMode::LOITER_TO_ALT:
        loiter_to_alt_run();
        break;

    case SubMode::NAV_PAYLOAD_PLACE:
        payload_place_run();
        break;
    }

    // only pretend to be in auto RTL so long as mission still thinks its in a landing sequence or the mission has completed
    if (auto_RTL && (!(mission.get_in_landing_sequence_flag() || mission.state() == AP_Mission::mission_state::MISSION_COMPLETE))) {
        auto_RTL = false;
        // log exit from Auto RTL
        copter.logger.Write_Mode((uint8_t)copter.flightmode->mode_number(), ModeReason::AUTO_RTL_EXIT);
    }
}

bool ModeAuto::allows_arming(AP_Arming::Method method) const
{
    return ((copter.g2.auto_options & (uint32_t)Options::AllowArming) != 0) && !auto_RTL;
};

// Go straight to landing sequence via DO_LAND_START, if succeeds pretend to be Auto RTL mode
bool ModeAuto::jump_to_landing_sequence_auto_RTL(ModeReason reason)
{
    if (mission.jump_to_landing_sequence()) {
        mission.set_force_resume(true);
        // if not already in auto switch to auto
        if ((copter.flightmode == &copter.mode_auto) || set_mode(Mode::Number::AUTO, reason)) {
            auto_RTL = true;
            // log entry into AUTO RTL
            copter.logger.Write_Mode((uint8_t)copter.flightmode->mode_number(), reason);

            // make happy noise
            if (copter.ap.initialised) {
                AP_Notify::events.user_mode_change = 1;
            }
            return true;
        }
        // mode change failed, revert force resume flag
        mission.set_force_resume(false);

        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to AUTO RTL failed");
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to AUTO RTL failed: No landing sequence found");
    }

    AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(Number::AUTO_RTL));
    // make sad noise
    if (copter.ap.initialised) {
        AP_Notify::events.user_mode_change_failed = 1;
    }
    return false;
}

// lua scripts use this to retrieve the contents of the active command
bool ModeAuto::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2)
{
#if AP_SCRIPTING_ENABLED
    if (_mode == SubMode::NAV_SCRIPT_TIME) {
        id = nav_scripting.id;
        cmd = nav_scripting.command;
        arg1 = nav_scripting.arg1;
        arg2 = nav_scripting.arg2;
        return true;
    }
#endif
    return false;
}

// lua scripts use this to indicate when they have complete the command
void ModeAuto::nav_script_time_done(uint16_t id)
{
#if AP_SCRIPTING_ENABLED
    if ((_mode == SubMode::NAV_SCRIPT_TIME) && (id == nav_scripting.id)) {
        nav_scripting.done = true;
    }
#endif
}

// auto_loiter_start - initialises loitering in auto mode
//  returns success/failure because this can be called by exit_mission
bool ModeAuto::loiter_start()
{
    // return failure if GPS is bad
    if (!copter.position_ok()) {
        return false;
    }
    _mode = SubMode::LOITER;

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
void ModeAuto::rtl_start()
{
    _mode = SubMode::RTL;

    // call regular rtl flight mode initialisation and ask it to ignore checks
    copter.mode_rtl.init(true);
}

// auto_takeoff_start - initialises waypoint controller to implement take-off
void ModeAuto::takeoff_start(const Location& dest_loc)
{
    _mode = SubMode::TAKEOFF;

    if (!copter.current_loc.initialised()) {
        // vehicle doesn't know where it is ATM.  We should not
        // initialise our takeoff destination without knowing this!
        return;
    }

    // calculate current and target altitudes
    // by default current_alt_cm and alt_target_cm are alt-above-EKF-origin
    int32_t alt_target_cm;
    bool alt_target_terrain = false;
    float current_alt_cm = inertial_nav.get_position_z_up_cm();
    float terrain_offset;   // terrain's altitude in cm above the ekf origin
    if ((dest_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) && wp_nav->get_terrain_offset(terrain_offset)) {
        // subtract terrain offset to convert vehicle's alt-above-ekf-origin to alt-above-terrain
        current_alt_cm -= terrain_offset;

        // specify alt_target_cm as alt-above-terrain
        alt_target_cm = dest_loc.alt;
        alt_target_terrain = true;
    } else {
        // set horizontal target
        Location dest(dest_loc);
        dest.lat = copter.current_loc.lat;
        dest.lng = copter.current_loc.lng;

        // get altitude target above EKF origin
        if (!dest.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
            // this failure could only happen if take-off alt was specified as an alt-above terrain and we have no terrain data
            AP::logger().Write_Error(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            // fall back to altitude above current altitude
            alt_target_cm = current_alt_cm + dest.alt;
        }
    }

    // sanity check target
    int32_t alt_target_min_cm = current_alt_cm + (copter.ap.land_complete ? 100 : 0);
    alt_target_cm = MAX(alt_target_cm, alt_target_min_cm);

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff_start(alt_target_cm, alt_target_terrain);
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void ModeAuto::wp_start(const Location& dest_loc)
{
    // init wpnav and set origin if transitioning from takeoff
    if (!wp_nav->is_active()) {
        Vector3f stopping_point;
        if (_mode == SubMode::TAKEOFF) {
            Vector3p takeoff_complete_pos;
            if (auto_takeoff_get_position(takeoff_complete_pos)) {
                stopping_point = takeoff_complete_pos.tofloat();
            }
        }
        wp_nav->wp_and_spline_init(0, stopping_point);
    }

    // send target to waypoint controller
    if (!wp_nav->set_wp_destination_loc(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    _mode = SubMode::WP;

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode_to_default(false);
    }
}

// auto_land_start - initialises controller to implement a landing
void ModeAuto::land_start()
{
    _mode = SubMode::LAND;

    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialise the vertical position controller
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

#if LANDING_GEAR_ENABLED == ENABLED
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif

#if AC_FENCE == ENABLED
    // disable the fence on landing
    copter.fence.auto_disable_fence_for_landing();
#endif

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.land_repo_active = false;

    // this will be set true if prec land is later active
    copter.ap.prec_land_active = false;
}

// auto_circle_movetoedge_start - initialise waypoint controller to move to edge of a circle with it's center at the specified location
//  we assume the caller has performed all required GPS_ok checks
void ModeAuto::circle_movetoedge_start(const Location &circle_center, float radius_m)
{
    // set circle center
    copter.circle_nav->set_center(circle_center);

    // set circle radius
    if (!is_zero(radius_m)) {
        copter.circle_nav->set_radius(radius_m * 100.0f);
    }

    // check our distance from edge of circle
    Vector3f circle_edge_neu;
    copter.circle_nav->get_closest_point_on_circle(circle_edge_neu);
    float dist_to_edge = (inertial_nav.get_position_neu_cm() - circle_edge_neu).length();

    // if more than 3m then fly to edge
    if (dist_to_edge > 300.0f) {
        // set the state to move to the edge of the circle
        _mode = SubMode::CIRCLE_MOVE_TO_EDGE;

        // convert circle_edge_neu to Location
        Location circle_edge(circle_edge_neu, Location::AltFrame::ABOVE_ORIGIN);

        // convert altitude to same as command
        circle_edge.set_alt_cm(circle_center.alt, circle_center.get_alt_frame());

        // initialise wpnav to move to edge of circle
        if (!wp_nav->set_wp_destination_loc(circle_edge)) {
            // failure to set destination can only be because of missing terrain data
            copter.failsafe_terrain_on_event();
        }

        // if we are outside the circle, point at the edge, otherwise hold yaw
        const float dist_to_center = get_horizontal_distance_cm(inertial_nav.get_position_xy_cm().topostype(), copter.circle_nav->get_center().xy());
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
void ModeAuto::circle_start()
{
    _mode = SubMode::CIRCLE;

    // initialise circle controller
    copter.circle_nav->init(copter.circle_nav->get_center(), copter.circle_nav->center_is_terrain_alt());

    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode(AUTO_YAW_CIRCLE);
    }
}

#if NAV_GUIDED == ENABLED
// auto_nav_guided_start - hand over control to external navigation controller in AUTO mode
void ModeAuto::nav_guided_start()
{
    _mode = SubMode::NAVGUIDED;

    // call regular guided flight mode initialisation
    copter.mode_guided.init(true);

    // initialise guided start time and position as reference for limit checking
    copter.mode_guided.limit_init_time_and_pos();
}
#endif //NAV_GUIDED

bool ModeAuto::is_landing() const
{
    switch(_mode) {
    case SubMode::LAND:
        return true;
    case SubMode::RTL:
        return copter.mode_rtl.is_landing();
    default:
        return false;
    }
    return false;
}

bool ModeAuto::is_taking_off() const
{
    return ((_mode == SubMode::TAKEOFF) && !wp_nav->reached_wp_destination());
}

// auto_payload_place_start - initialises controller to implement a placing
void ModeAuto::payload_place_start()
{
    _mode = SubMode::NAV_PAYLOAD_PLACE;
    nav_payload_place.state = PayloadPlaceStateType_Calibrating_Hover_Start;

    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialise the vertical position controller
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

// returns true if pilot's yaw input should be used to adjust vehicle's heading
bool ModeAuto::use_pilot_yaw(void) const
{
    return (copter.g2.auto_options.get() & uint32_t(Options::IgnorePilotYaw)) == 0;
}

// start_command - this function will be called when the ap_mission lib wishes to start a new command
bool ModeAuto::start_command(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: logging when new commands start/end
    if (copter.should_log(MASK_LOG_CMD)) {
        copter.logger.Write_Mission_Cmd(mission, cmd);
    }

    switch(cmd.id) {

    ///
    /// navigation commands
    ///
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_TAKEOFF:                   // 22
        do_takeoff(cmd);
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        do_land(cmd);
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

    case MAV_CMD_NAV_DELAY:                    // 93 Delay the next navigation command
        do_nav_delay(cmd);
        break;

    case MAV_CMD_NAV_PAYLOAD_PLACE:              // 94 place at Waypoint
        do_payload_place(cmd);
        break;

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        do_nav_script_time(cmd);
        break;
#endif

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

    case MAV_CMD_DO_LAND_START:
        break;

    default:
        // unable to use the command, allow the vehicle to try the next command
        return false;
    }

    // always return success
    return true;
}

// exit_mission - function that is called once the mission completes
void ModeAuto::exit_mission()
{
    // play a tone
    AP_Notify::events.mission_complete = 1;
    // if we are not on the ground switch to loiter or land
    if (!copter.ap.land_complete) {
        // try to enter loiter but if that fails land
        if (!loiter_start()) {
            set_mode(Mode::Number::LAND, ModeReason::MISSION_END);
        }
    } else {
        // if we've landed it's safe to disarm
        copter.arming.disarm(AP_Arming::Method::MISSIONEXIT);
    }
}

// do_guided - start guided mode
bool ModeAuto::do_guided(const AP_Mission::Mission_Command& cmd)
{
    // only process guided waypoint if we are in guided mode
    if (copter.flightmode->mode_number() != Mode::Number::GUIDED && !(copter.flightmode->mode_number() == Mode::Number::AUTO && mode() == SubMode::NAVGUIDED)) {
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

uint32_t ModeAuto::wp_distance() const
{
    switch (_mode) {
    case SubMode::CIRCLE:
        return copter.circle_nav->get_distance_to_target();
    case SubMode::WP:
    case SubMode::CIRCLE_MOVE_TO_EDGE:
    default:
        return wp_nav->get_wp_distance_to_destination();
    }
}

int32_t ModeAuto::wp_bearing() const
{
    switch (_mode) {
    case SubMode::CIRCLE:
        return copter.circle_nav->get_bearing_to_target();
    case SubMode::WP:
    case SubMode::CIRCLE_MOVE_TO_EDGE:
    default:
        return wp_nav->get_wp_bearing_to_destination();
    }
}

bool ModeAuto::get_wp(Location& destination) const
{
    switch (_mode) {
    case SubMode::NAVGUIDED:
        return copter.mode_guided.get_wp(destination);
    case SubMode::WP:
        return wp_nav->get_oa_wp_destination(destination);
    case SubMode::RTL:
        return copter.mode_rtl.get_wp(destination);
    default:
        return false;
    }
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
bool ModeAuto::verify_command(const AP_Mission::Mission_Command& cmd)
{
    if (copter.flightmode != &copter.mode_auto) {
        return false;
    }

    bool cmd_complete = false;

    switch (cmd.id) {
    //
    // navigation commands
    //
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_TAKEOFF:
        cmd_complete = verify_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:
        cmd_complete = verify_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_VTOL_LAND:
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
        cmd_complete = verify_loiter_time(cmd);
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

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        cmd_complete = verify_nav_script_time();
        break;
#endif

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
    case MAV_CMD_DO_LAND_START:
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

// takeoff_run - takeoff in auto mode
//      called by auto_run at 100hz or more
void ModeAuto::takeoff_run()
{
    // if the user doesn't want to raise the throttle we can set it automatically
    // note that this can defeat the disarm check on takeoff
    if ((copter.g2.auto_options & (int32_t)Options::AllowTakeOffWithoutRaisingThrottle) != 0) {
        copter.set_auto_armed(true);
    }
    auto_takeoff_run();
}

// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void ModeAuto::wp_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(wp_nav->get_thrust_vector(), target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_thrust_vector_heading(wp_nav->get_thrust_vector(), auto_yaw.yaw(), auto_yaw.rate_cds());
    }
}

// auto_land_run - lands in auto mode
//      called by auto_run at 100hz or more
void ModeAuto::land_run()
{

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run normal landing or precision landing (if enabled)
    land_run_normal_or_precland();
}

// auto_rtl_run - rtl in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeAuto::rtl_run()
{
    // call regular rtl flight mode run function
    copter.mode_rtl.run(false);
}

// auto_circle_run - circle in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeAuto::circle_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // call circle controller
    copter.failsafe_terrain_set_status(copter.circle_nav->update());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(copter.circle_nav->get_thrust_vector(), target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_thrust_vector_heading(copter.circle_nav->get_thrust_vector(), auto_yaw.yaw());
    }
}

#if NAV_GUIDED == ENABLED || AP_SCRIPTING_ENABLED
// auto_nav_guided_run - allows control by external navigation controller
//      called by auto_run at 100hz or more
void ModeAuto::nav_guided_run()
{
    // call regular guided flight mode run function
    copter.mode_guided.run();
}
#endif  // NAV_GUIDED || AP_SCRIPTING_ENABLED

// auto_loiter_run - loiter in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeAuto::loiter_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // accept pilot input of yaw
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint and z-axis position controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_rate_heading(wp_nav->get_thrust_vector(), target_yaw_rate);
}

// auto_loiter_run - loiter to altitude in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeAuto::loiter_to_alt_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (is_disarmed_or_landed() || !motors->get_interlock()) {
        make_safe_ground_handling();
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
        // set horizontal speed and acceleration limits
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }

        _mode = SubMode::LOITER_TO_ALT;
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
    float target_climb_rate = sqrt_controller(
        -alt_error_cm,
        pos_control->get_pos_z_p().kP(),
        pos_control->get_max_accel_z_cmss(),
        G_Dt);
    target_climb_rate = constrain_float(target_climb_rate, pos_control->get_max_speed_down_cms(), pos_control->get_max_speed_up_cms());

    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    // update the vertical offset based on the surface measurement
    copter.surface_tracking.update_surface_offset();

    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);

    pos_control->update_z_controller();
}

// auto_payload_place_run - places an object in auto mode
//      called by auto_run at 100hz or more
void ModeAuto::payload_place_run()
{
    if (!payload_place_run_should_run()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

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
        return payload_place_run_loiter();
    case PayloadPlaceStateType_Ascending:
    case PayloadPlaceStateType_Done:
        return wp_run();
    }
}

bool ModeAuto::payload_place_run_should_run()
{
    // must be armed
    if (!motors->armed()) {
        return false;
    }
    // must be auto-armed
    if (!copter.ap.auto_armed) {
        return false;
    }
    // must not be landed
    if (copter.ap.land_complete) {
        return false;
    }
    // interlock must be enabled (i.e. unsafe)
    if (!motors->get_interlock()) {
        return false;
    }

    return true;
}

void ModeAuto::payload_place_run_loiter()
{
    // loiter...
    land_run_horizontal_control();

    // call position controller
    pos_control->update_z_controller();
}

void ModeAuto::payload_place_run_descend()
{
    land_run_horizontal_control();
    land_run_vertical_control();
}

// sets the target_loc's alt to the vehicle's current alt but does not change target_loc's frame
// in the case of terrain altitudes either the terrain database or the rangefinder may be used
// returns true on success, false on failure
bool ModeAuto::shift_alt_to_current_alt(Location& target_loc) const
{
    // if terrain alt using rangefinder is being used then set alt to current rangefinder altitude
    if ((target_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) &&
        (wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER)) {
        int32_t curr_rngfnd_alt_cm;
        if (copter.get_rangefinder_height_interpolated_cm(curr_rngfnd_alt_cm)) {
            // wp_nav is using rangefinder so use current rangefinder alt
            target_loc.set_alt_cm(MAX(curr_rngfnd_alt_cm, 200), Location::AltFrame::ABOVE_TERRAIN);
            return true;
        }
        return false;
    }

    // take copy of current location and change frame to match target
    Location currloc = copter.current_loc;
    if (!currloc.change_alt_frame(target_loc.get_alt_frame())) {
        // this could fail due missing terrain database alt
        return false;
    }

    // set target_loc's alt
    target_loc.set_alt_cm(currloc.alt, currloc.get_alt_frame());
    return true;
}

/********************************************************************************/
// Nav (Must) commands
/********************************************************************************/

// do_takeoff - initiate takeoff navigation command
void ModeAuto::do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    // Set wp navigation target to safe altitude above current position
    takeoff_start(cmd.content.location);
}

Location ModeAuto::loc_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc) const
{
    Location ret(cmd.content.location);

    // use current lat, lon if zero
    if (ret.lat == 0 && ret.lng == 0) {
        ret.lat = default_loc.lat;
        ret.lng = default_loc.lng;
    }
    // use default altitude if not provided in cmd
    if (ret.alt == 0) {
        // set to default_loc's altitude but in command's alt frame
        // note that this may use the terrain database
        int32_t default_alt;
        if (default_loc.get_alt_cm(ret.get_alt_frame(), default_alt)) {
            ret.set_alt_cm(default_alt, ret.get_alt_frame());
        } else {
            // default to default_loc's altitude and frame
            ret.set_alt_cm(default_loc.alt, default_loc.get_alt_frame());
        }
    }
    return ret;
}

// do_nav_wp - initiate move to next waypoint
void ModeAuto::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // calculate default location used when lat, lon or alt is zero
    Location default_loc = copter.current_loc;
    if (wp_nav->is_active() && wp_nav->reached_wp_destination()) {
        if (!wp_nav->get_wp_destination_loc(default_loc)) {
            // this should never happen
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }

    // init wpnav and set origin if transitioning from takeoff
    if (!wp_nav->is_active()) {
        Vector3f stopping_point;
        if (_mode == SubMode::TAKEOFF) {
            Vector3p takeoff_complete_pos;
            if (auto_takeoff_get_position(takeoff_complete_pos)) {
                stopping_point = takeoff_complete_pos.tofloat();
            }
        }
        wp_nav->wp_and_spline_init(0, stopping_point);
    }

    // get waypoint's location from command and send to wp_nav
    const Location dest_loc = loc_from_cmd(cmd, default_loc);
    if (!wp_nav->set_wp_destination_loc(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    _mode = SubMode::WP;

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;

    // set next destination if necessary
    if (!set_next_wp(cmd, dest_loc)) {
        // failure to set next destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode_to_default(false);
    }
}

// checks the next mission command and adds it as a destination if necessary
// supports both straight line and spline waypoints
// cmd should be the current command
// default_loc should be the destination from the current_cmd but corrected for cases where user set lat, lon or alt to zero
// returns true on success, false on failure which should only happen due to a failure to retrieve terrain data
bool ModeAuto::set_next_wp(const AP_Mission::Mission_Command& current_cmd, const Location &default_loc)
{
    // do not add next wp if current command has a delay meaning the vehicle will stop at the destination
    if (current_cmd.p1 > 0) {
        return true;
    }

    // do not add next wp if there are no more navigation commands
    AP_Mission::Mission_Command next_cmd;
    if (!mission.get_next_nav_cmd(current_cmd.index+1, next_cmd)) {
        return true;
    }

    // whether vehicle should stop at the target position depends upon the next command
    switch (next_cmd.id) {
    case MAV_CMD_NAV_WAYPOINT:
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TIME: {
        const Location dest_loc = loc_from_cmd(current_cmd, default_loc);
        const Location next_dest_loc = loc_from_cmd(next_cmd, dest_loc);
        return wp_nav->set_wp_destination_next_loc(next_dest_loc);
    }
    case MAV_CMD_NAV_SPLINE_WAYPOINT: {
        // get spline's location and next location from command and send to wp_nav
        Location next_dest_loc, next_next_dest_loc;
        bool next_next_dest_loc_is_spline;
        get_spline_from_cmd(next_cmd, default_loc, next_dest_loc, next_next_dest_loc, next_next_dest_loc_is_spline);
        return wp_nav->set_spline_destination_next_loc(next_dest_loc, next_next_dest_loc, next_next_dest_loc_is_spline);
    }
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:
        // stop because we may change between rel,abs and terrain alt types
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_TAKEOFF:
        // always stop for RTL and takeoff commands
    default:
        // for unsupported commands it is safer to stop
        break;
    }

    return true;
}

// do_land - initiate landing procedure
void ModeAuto::do_land(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: check if we have already landed

    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // set state to fly to location
        state = State::FlyToLocation;

        // convert cmd to location class
        Location target_loc(cmd.content.location);
        if (!shift_alt_to_current_alt(target_loc)) {
            // this can only fail due to missing terrain database alt or rangefinder alt
            // use current alt-above-home and report error
            target_loc.set_alt_cm(copter.current_loc.alt, Location::AltFrame::ABOVE_HOME);
            AP::logger().Write_Error(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Land: no terrain data, using alt-above-home");
        }

        wp_start(target_loc);
    } else {
        // set landing state
        state = State::Descending;

        // initialise landing controller
        land_start();
    }
}

// do_loiter_unlimited - start loitering with no end conditions
// note: caller should set yaw_mode
void ModeAuto::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    // convert back to location
    Location target_loc(cmd.content.location);

    // use current location if not provided
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        // To-Do: make this simpler
        Vector3f temp_pos;
        copter.wp_nav->get_wp_stopping_point_xy(temp_pos.xy());
        const Location temp_loc(temp_pos, Location::AltFrame::ABOVE_ORIGIN);
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
void ModeAuto::do_circle(const AP_Mission::Mission_Command& cmd)
{
    const Location circle_center = loc_from_cmd(cmd, copter.current_loc);

    // calculate radius
    uint16_t circle_radius_m = HIGHBYTE(cmd.p1); // circle radius held in high byte of p1
    if (cmd.id == MAV_CMD_NAV_LOITER_TURNS &&
        cmd.type_specific_bits & (1U << 0)) {
        // special storage handling allows for larger radii
        circle_radius_m *= 10;
    }

    // move to edge of circle (verify_circle) will ensure we begin circling once we reach the edge
    circle_movetoedge_start(circle_center, circle_radius_m);
}

// do_loiter_time - initiate loitering at a point for a given time period
// note: caller should set yaw_mode
void ModeAuto::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    // re-use loiter unlimited
    do_loiter_unlimited(cmd);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = cmd.p1;     // units are (seconds)
}

// do_loiter_alt - initiate loitering at a point until a given altitude is reached
// note: caller should set yaw_mode
void ModeAuto::do_loiter_to_alt(const AP_Mission::Mission_Command& cmd)
{
    // re-use loiter unlimited
    do_loiter_unlimited(cmd);
    _mode = SubMode::LOITER_TO_ALT;

    // if we aren't navigating to a location then we have to adjust
    // altitude for current location
    Location target_loc(cmd.content.location);
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        target_loc.lat = copter.current_loc.lat;
        target_loc.lng = copter.current_loc.lng;
    }

    if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, loiter_to_alt.alt)) {
        loiter_to_alt.reached_destination_xy = true;
        loiter_to_alt.reached_alt = true;
        gcs().send_text(MAV_SEVERITY_INFO, "bad do_loiter_to_alt");
        return;
    }
    loiter_to_alt.reached_destination_xy = false;
    loiter_to_alt.loiter_start_done = false;
    loiter_to_alt.reached_alt = false;
    loiter_to_alt.alt_error_cm = 0;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
}

// do_spline_wp - initiate move to next waypoint
void ModeAuto::do_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    // calculate default location used when lat, lon or alt is zero
    Location default_loc = copter.current_loc;
    if (wp_nav->is_active() && wp_nav->reached_wp_destination()) {
        if (!wp_nav->get_wp_destination_loc(default_loc)) {
            // this should never happen
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }

    // get spline's location and next location from command and send to wp_nav
    Location dest_loc, next_dest_loc;
    bool next_dest_loc_is_spline;
    get_spline_from_cmd(cmd, default_loc, dest_loc, next_dest_loc, next_dest_loc_is_spline);
    if (!wp_nav->set_spline_destination_loc(dest_loc, next_dest_loc, next_dest_loc_is_spline)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    _mode = SubMode::WP;

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;

    // set next destination if necessary
    if (!set_next_wp(cmd, dest_loc)) {
        // failure to set next destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode_to_default(false);
    }
}

// calculate locations required to build a spline curve from a mission command
// dest_loc is populated from cmd's location using default_loc in cases where the lat and lon or altitude is zero
// next_dest_loc and nest_dest_loc_is_spline is filled in with the following navigation command's location if it exists.  If it does not exist it is set to the dest_loc and false
void ModeAuto::get_spline_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc, Location& dest_loc, Location& next_dest_loc, bool& next_dest_loc_is_spline)
{
    dest_loc = loc_from_cmd(cmd, default_loc);

    // if there is no delay at the end of this segment get next nav command
    AP_Mission::Mission_Command temp_cmd;
    if (cmd.p1 == 0 && mission.get_next_nav_cmd(cmd.index+1, temp_cmd)) {
        next_dest_loc = loc_from_cmd(temp_cmd, dest_loc);
        next_dest_loc_is_spline = temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT;
    } else {
        next_dest_loc = dest_loc;
        next_dest_loc_is_spline = false;
    }
}

#if NAV_GUIDED == ENABLED
// do_nav_guided_enable - initiate accepting commands from external nav computer
void ModeAuto::do_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 > 0) {
        // start guided within auto
        nav_guided_start();
    }
}

// do_guided_limits - pass guided limits to guided controller
void ModeAuto::do_guided_limits(const AP_Mission::Mission_Command& cmd)
{
    copter.mode_guided.limit_set(
        cmd.p1 * 1000, // convert seconds to ms
        cmd.content.guided_limits.alt_min * 100.0f,    // convert meters to cm
        cmd.content.guided_limits.alt_max * 100.0f,    // convert meters to cm
        cmd.content.guided_limits.horiz_max * 100.0f); // convert meters to cm
}
#endif  // NAV_GUIDED

// do_nav_delay - Delay the next navigation command
void ModeAuto::do_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    nav_delay_time_start_ms = millis();

    if (cmd.content.nav_delay.seconds > 0) {
        // relative delay
        nav_delay_time_max_ms = cmd.content.nav_delay.seconds * 1000; // convert seconds to milliseconds
    } else {
        // absolute delay to utc time
        nav_delay_time_max_ms = AP::rtc().get_time_utc(cmd.content.nav_delay.hour_utc, cmd.content.nav_delay.min_utc, cmd.content.nav_delay.sec_utc, 0);
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Delaying %u sec", (unsigned)(nav_delay_time_max_ms/1000));
}

#if AP_SCRIPTING_ENABLED
// start accepting position, velocity and acceleration targets from lua scripts
void ModeAuto::do_nav_script_time(const AP_Mission::Mission_Command& cmd)
{
    // call regular guided flight mode initialisation
    if (copter.mode_guided.init(true)) {
        _mode = SubMode::NAV_SCRIPT_TIME;
        nav_scripting.done = false;
        nav_scripting.id++;
        nav_scripting.start_ms = millis();
        nav_scripting.command = cmd.content.nav_script_time.command;
        nav_scripting.timeout_s = cmd.content.nav_script_time.timeout_s;
        nav_scripting.arg1 = cmd.content.nav_script_time.arg1;
        nav_scripting.arg2 = cmd.content.nav_script_time.arg2;
    } else {
        // for safety we set nav_scripting to done to protect against the mission getting stuck
        nav_scripting.done = true;
    }
}
#endif


/********************************************************************************/
// Condition (May) commands
/********************************************************************************/

void ModeAuto::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value = cmd.content.delay.seconds * 1000;     // convert seconds to milliseconds
}

void ModeAuto::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters * 100;
}

void ModeAuto::do_yaw(const AP_Mission::Mission_Command& cmd)
{
    auto_yaw.set_fixed_yaw(
        cmd.content.yaw.angle_deg,
        cmd.content.yaw.turn_rate_dps,
        cmd.content.yaw.direction,
        cmd.content.yaw.relative_angle > 0);
}

/********************************************************************************/
// Do (Now) commands
/********************************************************************************/



void ModeAuto::do_change_speed(const AP_Mission::Mission_Command& cmd)
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

void ModeAuto::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 || (cmd.content.location.lat == 0 && cmd.content.location.lng == 0 && cmd.content.location.alt == 0)) {
        if (!copter.set_home_to_current_location(false)) {
            // ignore failure
        }
    } else {
        if (!copter.set_home(cmd.content.location, false)) {
            // ignore failure
        }
    }
}

// do_roi - starts actions required by MAV_CMD_DO_SET_ROI
//          this involves either moving the camera to point at the ROI (region of interest)
//          and possibly rotating the copter to point at the ROI if our mount type does not support a yaw feature
// TO-DO: add support for other features of MAV_CMD_DO_SET_ROI including pointing at a given waypoint
void ModeAuto::do_roi(const AP_Mission::Mission_Command& cmd)
{
    auto_yaw.set_roi(cmd.content.location);
}

// point the camera to a specified angle
void ModeAuto::do_mount_control(const AP_Mission::Mission_Command& cmd)
{
#if HAL_MOUNT_ENABLED
    // if vehicle has a camera mount but it doesn't do pan control then yaw the entire vehicle instead
    if ((copter.camera_mount.get_mount_type() != copter.camera_mount.MountType::Mount_Type_None) &&
        !copter.camera_mount.has_pan_control()) {
        auto_yaw.set_yaw_angle_rate(cmd.content.mount_control.yaw,0.0f);
    }
    // pass the target angles to the camera mount
    copter.camera_mount.set_angle_targets(cmd.content.mount_control.roll, cmd.content.mount_control.pitch, cmd.content.mount_control.yaw);
#endif
}

#if WINCH_ENABLED == ENABLED
// control winch based on mission command
void ModeAuto::do_winch(const AP_Mission::Mission_Command& cmd)
{
    // Note: we ignore the gripper num parameter because we only support one gripper
    switch (cmd.content.winch.action) {
        case WINCH_RELAXED:
            g2.winch.relax();
            break;
        case WINCH_RELATIVE_LENGTH_CONTROL:
            g2.winch.release_length(cmd.content.winch.release_length);
            break;
        case WINCH_RATE_CONTROL:
            g2.winch.set_desired_rate(cmd.content.winch.release_rate);
            break;
        default:
            // do nothing
            break;
    }
}
#endif

// do_payload_place - initiate placing procedure
void ModeAuto::do_payload_place(const AP_Mission::Mission_Command& cmd)
{
    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // set state to fly to location
        nav_payload_place.state = PayloadPlaceStateType_FlyToLocation;

        // convert cmd to location class
        Location target_loc(cmd.content.location);
        if (!shift_alt_to_current_alt(target_loc)) {
            // this can only fail due to missing terrain database alt or rangefinder alt
            // use current alt-above-home and report error
            target_loc.set_alt_cm(copter.current_loc.alt, Location::AltFrame::ABOVE_HOME);
            AP::logger().Write_Error(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "PayloadPlace: no terrain data, using alt-above-home");
        }

        wp_start(target_loc);
    } else {
        nav_payload_place.state = PayloadPlaceStateType_Calibrating_Hover_Start;

        // initialise placing controller
        payload_place_start();
    }
    nav_payload_place.descend_max = cmd.p1;
}

// do_RTL - start Return-to-Launch
void ModeAuto::do_RTL(void)
{
    // start rtl in auto flight mode
    rtl_start();
}

/********************************************************************************/
// Verify Nav (Must) commands
/********************************************************************************/

// verify_takeoff - check if we have completed the takeoff
bool ModeAuto::verify_takeoff()
{
#if LANDING_GEAR_ENABLED == ENABLED
    // if we have reached our destination
    if (auto_takeoff_complete) {
        // retract the landing gear
        copter.landinggear.retract_after_takeoff();
    }
#endif

    return auto_takeoff_complete;
}

// verify_land - returns true if landing has been completed
bool ModeAuto::verify_land()
{
    bool retval = false;

    switch (state) {
        case State::FlyToLocation:
            // check if we've reached the location
            if (copter.wp_nav->reached_wp_destination()) {
                // initialise landing controller
                land_start();

                // advance to next state
                state = State::Descending;
            }
            break;

        case State::Descending:
            // rely on THROTTLE_LAND mode to correctly update landing status
            retval = copter.ap.land_complete && (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE);
            if (retval && !mission.continue_after_land_check_for_takeoff() && copter.motors->armed()) {
                /*
                  we want to stop mission processing on land
                  completion. Disarm now, then return false. This
                  leaves mission state machine in the current NAV_LAND
                  mission item. After disarming the mission will reset
                */
                copter.arming.disarm(AP_Arming::Method::LANDED);
                retval = false;
            }
            break;

        default:
            // this should never happen
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
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
bool ModeAuto::verify_payload_place()
{
    const uint16_t hover_throttle_calibrate_time = 2000; // milliseconds
    const uint16_t descend_throttle_calibrate_time = 2000; // milliseconds
    const float hover_throttle_placed_fraction = 0.7; // i.e. if throttle is less than 70% of hover we have placed
    const float descent_throttle_placed_fraction = 0.9; // i.e. if throttle is less than 90% of descent throttle we have placed
    const uint16_t placed_time = 500; // how long we have to be below a throttle threshold before considering placed

    const float current_throttle_level = motors->get_throttle();
    const uint32_t now = AP_HAL::millis();

    // if we discover we've landed then immediately release the load:
    if (copter.ap.land_complete) {
        switch (nav_payload_place.state) {
        case PayloadPlaceStateType_FlyToLocation:
        case PayloadPlaceStateType_Calibrating_Hover_Start:
        case PayloadPlaceStateType_Calibrating_Hover:
        case PayloadPlaceStateType_Descending_Start:
        case PayloadPlaceStateType_Descending:
            gcs().send_text(MAV_SEVERITY_INFO, "PayloadPlace: landed");
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
        nav_payload_place.descend_start_altitude = inertial_nav.get_position_z_up_cm();
        nav_payload_place.descend_throttle_level = 0;
        nav_payload_place.state = PayloadPlaceStateType_Descending;
        FALLTHROUGH;
    case PayloadPlaceStateType_Descending:
        // make sure we don't descend too far:
        debug("descended: %f cm (%f cm max)", (nav_payload_place.descend_start_altitude - inertial_nav.get_position_z_up_cm()), nav_payload_place.descend_max);
        if (!is_zero(nav_payload_place.descend_max) &&
            nav_payload_place.descend_start_altitude - inertial_nav.get_position_z_up_cm()  > nav_payload_place.descend_max) {
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
        Location target_loc(inertial_nav.get_position_neu_cm(), Location::AltFrame::ABOVE_ORIGIN);
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
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return true;
    }
    // should never get here
    return true;
}
#undef debug

bool ModeAuto::verify_loiter_unlimited()
{
    return false;
}

// verify_loiter_time - check if we have loitered long enough
bool ModeAuto::verify_loiter_time(const AP_Mission::Mission_Command& cmd)
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
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    }

    return false;
}

// verify_loiter_to_alt - check if we have reached both destination
// (roughly) and altitude (precisely)
bool ModeAuto::verify_loiter_to_alt() const
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
bool ModeAuto::verify_RTL()
{
    return (copter.mode_rtl.state_complete() && 
            (copter.mode_rtl.state() == ModeRTL::SubMode::FINAL_DESCENT || copter.mode_rtl.state() == ModeRTL::SubMode::LAND) &&
            (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE));
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

bool ModeAuto::verify_wait_delay()
{
    if (millis() - condition_start > (uint32_t)MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

bool ModeAuto::verify_within_distance()
{
    if (wp_distance() < (uint32_t)MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

// verify_yaw - return true if we have reached the desired heading
bool ModeAuto::verify_yaw()
{
    // make sure still in fixed yaw mode, the waypoint controller often retakes control of yaw as it executes a new waypoint command
    auto_yaw.set_mode(AUTO_YAW_FIXED);

    // check if we are within 2 degrees of the target heading
    return auto_yaw.fixed_yaw_slew_finished() && (fabsf(wrap_180_cd(ahrs.yaw_sensor-auto_yaw.yaw())) <= 200);
}

// verify_nav_wp - check if we have reached the next way point
bool ModeAuto::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
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
    }
    return false;
}

// verify_circle - check if we have circled the point enough
bool ModeAuto::verify_circle(const AP_Mission::Mission_Command& cmd)
{
    // check if we've reached the edge
    if (mode() == SubMode::CIRCLE_MOVE_TO_EDGE) {
        if (copter.wp_nav->reached_wp_destination()) {
            // start circling
            circle_start();
        }
        return false;
    }

    // check if we have completed circling
    return fabsf(copter.circle_nav->get_angle_total()/M_2PI) >= LOWBYTE(cmd.p1);
}

// verify_spline_wp - check if we have reached the next way point using spline
bool ModeAuto::verify_spline_wp(const AP_Mission::Mission_Command& cmd)
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
    }
    return false;
}

#if NAV_GUIDED == ENABLED
// verify_nav_guided - check if we have breached any limits
bool ModeAuto::verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
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
bool ModeAuto::verify_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    if (millis() - nav_delay_time_start_ms > nav_delay_time_max_ms) {
        nav_delay_time_max_ms = 0;
        return true;
    }
    return false;
}

#if AP_SCRIPTING_ENABLED
// check if verify_nav_script_time command has completed
bool ModeAuto::verify_nav_script_time()
{
    // if done or timeout then return true
    if (nav_scripting.done ||
        ((nav_scripting.timeout_s > 0) &&
         (AP_HAL::millis() - nav_scripting.start_ms) > (nav_scripting.timeout_s * 1000))) {
        return true;
    }
    return false;
}
#endif

// pause - Prevent aircraft from progressing along the track
bool ModeAuto::pause()
{
    // do not pause if already paused or not in the WP sub mode or already reached to the destination
    if(wp_nav->paused() || _mode != SubMode::WP || wp_nav->reached_wp_destination()) {
        return false;
    }

    wp_nav->set_pause();
    return true;
}

// resume - Allow aircraft to progress along the track
bool ModeAuto::resume()
{
    // do not resume if not paused before
    if(!wp_nav->paused()) {
        return false;
    }

    wp_nav->set_resume();
    return true;
}

#endif
