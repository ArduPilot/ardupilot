#include "Copter.h"

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
    if ((_copter.position_ok() && mission.num_commands() > 1) || ignore_checks) {
        _mode = Auto_Loiter;

        // reject switching to auto mode if landed with motors armed but first command is not a takeoff (reduce chance of flips)
        if (motors->armed() && ap.land_complete && !mission.starts_with_takeoff_cmd()) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto: Missing Takeoff Cmd");
            return false;
        }

        // stop ROI from carrying over from previous runs of the mission
        // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
        if (_copter.auto_yaw_mode == AUTO_YAW_ROI) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }

        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();

        // clear guided limits
        _copter.mode_guided.limit_clear();

        // start/resume the mission (based on MIS_RESTART parameter)
        mission.start_or_resume();
        return true;
    }else{
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

    case Auto_NavPayloadPlace:
        payload_place_run();
        break;
    }
}

// auto_takeoff_start - initialises waypoint controller to implement take-off
void Copter::ModeAuto::takeoff_start(const Location& dest_loc)
{
    _mode = Auto_TakeOff;

    // convert location to class
    Location_Class dest(dest_loc);

    // set horizontal target
    dest.lat = _copter.current_loc.lat;
    dest.lng = _copter.current_loc.lng;

    // get altitude target
    int32_t alt_target;
    if (!dest.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_HOME, alt_target)) {
        // this failure could only happen if take-off alt was specified as an alt-above terrain and we have no terrain data
        _copter.Log_Write_Error(ERROR_SUBSYSTEM_TERRAIN, ERROR_CODE_MISSING_TERRAIN_DATA);
        // fall back to altitude above current altitude
        alt_target = _copter.current_loc.alt + dest.alt;
    }

    // sanity check target
    if (alt_target < _copter.current_loc.alt) {
        dest.set_alt_cm(_copter.current_loc.alt, Location_Class::ALT_FRAME_ABOVE_HOME);
    }
    // Note: if taking off from below home this could cause a climb to an unexpectedly high altitude
    if (alt_target < 100) {
        dest.set_alt_cm(100, Location_Class::ALT_FRAME_ABOVE_HOME);
    }

    // set waypoint controller target
    if (!wp_nav->set_wp_destination(dest)) {
        // failure to set destination can only be because of missing terrain data
        _copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // get initial alt for WP_NAVALT_MIN
    _copter.auto_takeoff_set_start_alt();
}

// auto_takeoff_run - takeoff in auto mode
//      called by auto_run at 100hz or more
void Copter::ModeAuto::takeoff_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
        // initialise wpnav targets
        wp_nav->shift_wp_origin_to_current_pos();
        zero_throttle_and_relax_ac();
        // clear i term when we're taking off
        set_throttle_takeoff();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!_copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

#if FRAME_CONFIG == HELI_FRAME
    // helicopters stay in landed state until rotor speed runup has finished
    if (motors->rotor_runup_complete()) {
        set_land_complete(false);
    } else {
        // initialise wpnav targets
        wp_nav->shift_wp_origin_to_current_pos();
    }
#else
    set_land_complete(false);
#endif

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    _copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    _copter.auto_takeoff_attitude_run(target_yaw_rate);
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void Copter::ModeAuto::wp_start(const Vector3f& destination)
{
    _mode = Auto_WP;

    // initialise wpnav (no need to check return status because terrain data is not used)
    wp_nav->set_wp_destination(destination, false);

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (_copter.auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(_copter.get_default_auto_yaw_mode(false));
    }
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void Copter::ModeAuto::wp_start(const Location_Class& dest_loc)
{
    _mode = Auto_WP;

    // send target to waypoint controller
    if (!wp_nav->set_wp_destination(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        _copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (_copter.auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(_copter.get_default_auto_yaw_mode(false));
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
    if (!_copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    _copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (_copter.auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(),true);
    }
}

// auto_spline_start - initialises waypoint controller to implement flying to a particular destination using the spline controller
//  seg_end_type can be SEGMENT_END_STOP, SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE.  If Straight or Spline the next_destination should be provided
void Copter::ModeAuto::spline_start(const Location_Class& destination, bool stopped_at_start,
                               AC_WPNav::spline_segment_end_type seg_end_type, 
                               const Location_Class& next_destination)
{
    _mode = Auto_Spline;

    // initialise wpnav
    if (!wp_nav->set_spline_destination(destination, stopped_at_start, seg_end_type, next_destination)) {
        // failure to set destination can only be because of missing terrain data
        _copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (_copter.auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(_copter.get_default_auto_yaw_mode(false));
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
    if (!_copter.failsafe.radio) {
        // get pilot's desired yaw rat
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    wp_nav->update_spline();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (_copter.auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(), true);
    }
}

// auto_land_start - initialises controller to implement a landing
void Copter::ModeAuto::land_start()
{
    // set target to stopping point
    Vector3f stopping_point;
    wp_nav->get_loiter_stopping_point_xy(stopping_point);

    // call location specific land start function
    land_start(stopping_point);
}

// auto_land_start - initialises controller to implement a landing
void Copter::ModeAuto::land_start(const Vector3f& destination)
{
    _mode = Auto_Land;

    // initialise loiter target destination
    wp_nav->init_loiter_target(destination);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target(inertial_nav.get_altitude());
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// auto_land_run - lands in auto mode
//      called by auto_run at 100hz or more
void Copter::ModeAuto::land_run()
{
    // if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || ap.land_complete || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        // set target to current position
        wp_nav->init_loiter_target();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    
    _copter.land_run_horizontal_control();
    _copter.land_run_vertical_control();
}

bool Copter::ModeAuto::landing_gear_should_be_deployed() const
{
    switch(_mode) {
    case Auto_Land:
        return true;
    case Auto_RTL:
        return _copter.mode_rtl.landing_gear_should_be_deployed();
    default:
        return false;
    }
    return false;
}

// auto_rtl_start - initialises RTL in AUTO flight mode
void Copter::ModeAuto::rtl_start()
{
    _mode = Auto_RTL;

    // call regular rtl flight mode initialisation and ask it to ignore checks
    _copter.mode_rtl.init(true);
}

// auto_rtl_run - rtl in AUTO flight mode
//      called by auto_run at 100hz or more
void Copter::ModeAuto::rtl_run()
{
    // call regular rtl flight mode run function
    _copter.mode_rtl.run(false);
}

// auto_circle_movetoedge_start - initialise waypoint controller to move to edge of a circle with it's center at the specified location
//  we assume the caller has performed all required GPS_ok checks
void Copter::ModeAuto::circle_movetoedge_start(const Location_Class &circle_center, float radius_m)
{
    // convert location to vector from ekf origin
    Vector3f circle_center_neu;
    if (!circle_center.get_vector_from_origin_NEU(circle_center_neu)) {
        // default to current position and log error
        circle_center_neu = inertial_nav.get_position();
        _copter.Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_FAILED_CIRCLE_INIT);
    }
    circle_nav->set_center(circle_center_neu);

    // set circle radius
    if (!is_zero(radius_m)) {
        circle_nav->set_radius(radius_m * 100.0f);
    }

    // check our distance from edge of circle
    Vector3f circle_edge_neu;
    circle_nav->get_closest_point_on_circle(circle_edge_neu);
    float dist_to_edge = (inertial_nav.get_position() - circle_edge_neu).length();

    // if more than 3m then fly to edge
    if (dist_to_edge > 300.0f) {
        // set the state to move to the edge of the circle
        _mode = Auto_CircleMoveToEdge;

        // convert circle_edge_neu to Location_Class
        Location_Class circle_edge(circle_edge_neu);

        // convert altitude to same as command
        circle_edge.set_alt_cm(circle_center.alt, circle_center.get_alt_frame());

        // initialise wpnav to move to edge of circle
        if (!wp_nav->set_wp_destination(circle_edge)) {
            // failure to set destination can only be because of missing terrain data
            _copter.failsafe_terrain_on_event();
        }

        // if we are outside the circle, point at the edge, otherwise hold yaw
        const Vector3f &curr_pos = inertial_nav.get_position();
        float dist_to_center = norm(circle_center_neu.x - curr_pos.x, circle_center_neu.y - curr_pos.y);
        if (dist_to_center > circle_nav->get_radius() && dist_to_center > 500) {
            set_auto_yaw_mode(_copter.get_default_auto_yaw_mode(false));
        } else {
            // vehicle is within circle so hold yaw to avoid spinning as we move to edge of circle
            set_auto_yaw_mode(AUTO_YAW_HOLD);
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
    circle_nav->init(circle_nav->get_center());
}

// auto_circle_run - circle in AUTO flight mode
//      called by auto_run at 100hz or more
void Copter::ModeAuto::circle_run()
{
    // call circle controller
    circle_nav->update();

    // call z-axis position controller
    pos_control->update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_yaw(circle_nav->get_roll(), circle_nav->get_pitch(), circle_nav->get_yaw(),true);
}

#if NAV_GUIDED == ENABLED
// auto_nav_guided_start - hand over control to external navigation controller in AUTO mode
void Copter::ModeAuto::nav_guided_start()
{
    _mode = Auto_NavGuided;

    // call regular guided flight mode initialisation
    _copter.mode_guided.init(true);

    // initialise guided start time and position as reference for limit checking
    _copter.mode_guided.limit_init_time_and_pos();
}

// auto_nav_guided_run - allows control by external navigation controller
//      called by auto_run at 100hz or more
void Copter::ModeAuto::nav_guided_run()
{
    // call regular guided flight mode run function
    _copter.mode_guided.run();
}
#endif  // NAV_GUIDED

// auto_loiter_start - initialises loitering in auto mode
//  returns success/failure because this can be called by exit_mission
bool Copter::ModeAuto::loiter_start()
{
    // return failure if GPS is bad
    if (!_copter.position_ok()) {
        return false;
    }
    _mode = Auto_Loiter;

    // calculate stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);

    // initialise waypoint controller target to stopping point
    wp_nav->set_wp_destination(stopping_point);

    // hold yaw at current heading
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    return true;
}

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
    if(!_copter.failsafe.radio) {
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint and z-axis position controller
    _copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    pos_control->update_z_controller();
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
}

// get_default_auto_yaw_mode - returns auto_yaw_mode based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
uint8_t Copter::get_default_auto_yaw_mode(bool rtl)
{
    switch (g.wp_yaw_behavior) {

        case WP_YAW_BEHAVIOR_NONE:
            return AUTO_YAW_HOLD;

        case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
            if (rtl) {
                return AUTO_YAW_HOLD;
            }else{
                return AUTO_YAW_LOOK_AT_NEXT_WP;
            }

        case WP_YAW_BEHAVIOR_LOOK_AHEAD:
            return AUTO_YAW_LOOK_AHEAD;

        case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
        default:
            return AUTO_YAW_LOOK_AT_NEXT_WP;
    }
}

// set_auto_yaw_mode - sets the yaw mode for auto
void Copter::set_auto_yaw_mode(uint8_t yaw_mode)
{
    // return immediately if no change
    if (auto_yaw_mode == yaw_mode) {
        return;
    }
    auto_yaw_mode = yaw_mode;

    // perform initialisation
    switch (auto_yaw_mode) {

    case AUTO_YAW_LOOK_AT_NEXT_WP:
        // wpnav will initialise heading when wpnav's set_destination method is called
        break;

    case AUTO_YAW_ROI:
        // point towards a location held in yaw_look_at_WP
        yaw_look_at_WP_bearing = ahrs.yaw_sensor;
        break;

    case AUTO_YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading
        // caller should set the yaw_look_at_heading
        break;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        yaw_look_ahead_bearing = ahrs.yaw_sensor;
        break;

    case AUTO_YAW_RESETTOARMEDYAW:
        // initial_armed_bearing will be set during arming so no init required
        break;

    case AUTO_YAW_RATE:
        // initialise target yaw rate to zero
        auto_yaw_rate_cds = 0.0f;
        break;
    }
}

// set_auto_yaw_look_at_heading - sets the yaw look at heading for auto mode
void Copter::set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, bool relative_angle)
{
    // get current yaw target
    int32_t curr_yaw_target = attitude_control->get_att_target_euler_cd().z;

    // calculate final angle as relative to vehicle heading or absolute
    if (!relative_angle) {
        // absolute angle
        yaw_look_at_heading = wrap_360_cd(angle_deg * 100);
    } else {
        // relative angle
        if (direction < 0) {
            angle_deg = -angle_deg;
        }
        yaw_look_at_heading = wrap_360_cd((angle_deg * 100) + curr_yaw_target);
    }

    // get turn speed
    if (is_zero(turn_rate_dps)) {
        // default to regular auto slew rate
        yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
    }else{
        int32_t turn_rate = (wrap_180_cd(yaw_look_at_heading - curr_yaw_target) / 100) / turn_rate_dps;
        yaw_look_at_heading_slew = constrain_int32(turn_rate, 1, 360);    // deg / sec
    }

    // set yaw mode
    set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);

    // TO-DO: restore support for clockwise and counter clockwise rotation held in cmd.content.yaw.direction.  1 = clockwise, -1 = counterclockwise
}

// set_auto_yaw_roi - sets the yaw to look at roi for auto mode
void Copter::set_auto_yaw_roi(const Location &roi_location)
{
    // if location is zero lat, lon and altitude turn off ROI
    if (roi_location.alt == 0 && roi_location.lat == 0 && roi_location.lng == 0) {
        // set auto yaw mode back to default assuming the active command is a waypoint command.  A more sophisticated method is required to ensure we return to the proper yaw control for the active command
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
#if MOUNT == ENABLED
        // switch off the camera tracking if enabled
        if (camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
            camera_mount.set_mode_to_default();
        }
#endif  // MOUNT == ENABLED
    }else{
#if MOUNT == ENABLED
        // check if mount type requires us to rotate the quad
        if(!camera_mount.has_pan_control()) {
            roi_WP = pv_location_to_vector(roi_location);
            set_auto_yaw_mode(AUTO_YAW_ROI);
        }
        // send the command to the camera mount
        camera_mount.set_roi_target(roi_location);

        // TO-DO: expand handling of the do_nav_roi to support all modes of the MAVLink.  Currently we only handle mode 4 (see below)
        //      0: do nothing
        //      1: point at next waypoint
        //      2: point at a waypoint taken from WP# parameter (2nd parameter?)
        //      3: point at a location given by alt, lon, lat parameters
        //      4: point at a target given a target id (can't be implemented)
#else
        // if we have no camera mount aim the quad at the location
        roi_WP = pv_location_to_vector(roi_location);
        set_auto_yaw_mode(AUTO_YAW_ROI);
#endif  // MOUNT == ENABLED
    }
}

// set auto yaw rate in centi-degrees per second
void Copter::set_auto_yaw_rate(float turn_rate_cds)
{
    set_auto_yaw_mode(AUTO_YAW_RATE);
    auto_yaw_rate_cds = turn_rate_cds;
}

// get_auto_heading - returns target heading depending upon auto_yaw_mode
// 100hz update rate
float Copter::get_auto_heading(void)
{
    switch(auto_yaw_mode) {

    case AUTO_YAW_ROI:
        // point towards a location held in roi_WP
        return get_roi_yaw();

    case AUTO_YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading with no pilot input allowed
        return yaw_look_at_heading;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        return get_look_ahead_yaw();

    case AUTO_YAW_RESETTOARMEDYAW:
        // changes yaw to be same as when quad was armed
        return initial_armed_bearing;

    case AUTO_YAW_LOOK_AT_NEXT_WP:
    default:
        // point towards next waypoint.
        // we don't use wp_bearing because we don't want the copter to turn too much during flight
        return wp_nav->get_yaw();
    }
}

// returns yaw rate held in auto_yaw_rate and normally set by SET_POSITION_TARGET mavlink messages (positive it clockwise, negative is counter clockwise)
float Copter::get_auto_yaw_rate_cds(void)
{
    if (auto_yaw_mode == AUTO_YAW_RATE) {
        return auto_yaw_rate_cds;
    }

    // return zero turn rate (this should never happen)
    return 0.0f;
}

// auto_payload_place_start - initialises controller to implement a placing
void Copter::ModeAuto::payload_place_start()
{
    // set target to stopping point
    Vector3f stopping_point;
    wp_nav->get_loiter_stopping_point_xy(stopping_point);

    // call location specific place start function
    payload_place_start(stopping_point);

}

// auto_payload_place_start - initialises controller to implement placement of a load
void Copter::ModeAuto::payload_place_start(const Vector3f& destination)
{
    _mode = Auto_NavPayloadPlace;
    nav_payload_place.state = PayloadPlaceStateType_Calibrating_Hover_Start;

    // initialise loiter target destination
    wp_nav->init_loiter_target(destination);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target(inertial_nav.get_altitude());
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
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

// auto_payload_place_run - places an object in auto mode
//      called by auto_run at 100hz or more
void Copter::ModeAuto::payload_place_run()
{
    if (!payload_place_run_should_run()) {
        zero_throttle_and_relax_ac();
        // set target to current position
        wp_nav->init_loiter_target();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    switch (nav_payload_place.state) {
    case PayloadPlaceStateType_FlyToLocation:
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

void Copter::ModeAuto::payload_place_run_loiter()
{
    // loiter...
    _copter.land_run_horizontal_control();

    // run loiter controller
    wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    const float target_yaw_rate = 0;
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);

    // call position controller
    pos_control->update_z_controller();
}

void Copter::ModeAuto::payload_place_run_descend()
{
    _copter.land_run_horizontal_control();
    _copter.land_run_vertical_control();
}
