#include "Sub.h"

/*
 * control_auto.cpp
 *  Contains the mission, waypoint navigation and NAV_CMD item implementation
 *
 *  While in the auto flight mode, navigation or do/now commands can be run.
 *  Code in this file implements the navigation commands
 */

// auto_init - initialise auto controller
bool Sub::auto_init()
{
    if (!position_ok() || mission.num_commands() < 2) {
        return false;
    }

    auto_mode = Auto_Loiter;

    // stop ROI from carrying over from previous runs of the mission
    // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
    if (auto_yaw_mode == AUTO_YAW_ROI) {
        set_auto_yaw_mode(AUTO_YAW_HOLD);
    }

    // initialise waypoint and spline controller
    wp_nav.wp_and_spline_init();

    // clear guided limits
    guided_limit_clear();

    // start/resume the mission (based on MIS_RESTART parameter)
    mission.start_or_resume();
    return true;
}

// auto_run - runs the appropriate auto controller
// according to the current auto_mode
// should be called at 100hz or more
void Sub::auto_run()
{
    mission.update();

    // call the correct auto controller
    switch (auto_mode) {

    case Auto_WP:
    case Auto_CircleMoveToEdge:
        auto_wp_run();
        break;

    case Auto_Circle:
        auto_circle_run();
        break;

    case Auto_Spline:
        auto_spline_run();
        break;

    case Auto_NavGuided:
#if NAV_GUIDED == ENABLED
        auto_nav_guided_run();
#endif
        break;

    case Auto_Loiter:
        auto_loiter_run();
        break;

    case Auto_TerrainRecover:
        auto_terrain_recover_run();
        break;
    }
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void Sub::auto_wp_start(const Vector3f& destination)
{
    auto_mode = Auto_WP;

    // initialise wpnav (no need to check return status because terrain data is not used)
    wp_nav.set_wp_destination(destination, false);

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    }
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void Sub::auto_wp_start(const Location& dest_loc)
{
    auto_mode = Auto_WP;

    // send target to waypoint controller
    if (!wp_nav.set_wp_destination(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    }
}

// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void Sub::auto_wp_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        // To-Do: reset waypoint origin to current location because vehicle is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
        // call attitude controller
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        motors.set_desired_spool_state(AP_Motors::DESIRED_GROUND_IDLE);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.pilot_input) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    // TODO logic for terrain tracking target going below fence limit
    // TODO implement waypoint radius individually for each waypoint based on cmd.p2
    // TODO fix auto yaw heading to switch to something appropriate when mission complete and switches to loiter
    failsafe_terrain_set_status(wp_nav.update_wpnav());

    ///////////////////////
    // update xy outputs //

    float lateral_out, forward_out;
    translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    ////////////////////////////
    // update attitude output //

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, get_auto_heading(), true);
    }
}

// auto_spline_start - initialises waypoint controller to implement flying to a particular destination using the spline controller
//  seg_end_type can be SEGMENT_END_STOP, SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE.  If Straight or Spline the next_destination should be provided
void Sub::auto_spline_start(const Location& destination, bool stopped_at_start,
                            AC_WPNav::spline_segment_end_type seg_end_type,
                            const Location& next_destination)
{
    auto_mode = Auto_Spline;

    // initialise wpnav
    if (!wp_nav.set_spline_destination(destination, stopped_at_start, seg_end_type, next_destination)) {
        // failure to set destination can only be because of missing terrain data
        failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    }
}

// auto_spline_run - runs the auto spline controller
//      called by auto_run at 100hz or more
void Sub::auto_spline_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        // To-Do: reset waypoint origin to current location because vehicle is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        motors.set_desired_spool_state(AP_Motors::DESIRED_GROUND_IDLE);

        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.pilot_input) {
        // get pilot's desired yaw rat
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    wp_nav.update_spline();

    float lateral_out, forward_out;
    translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, get_auto_heading(), true);
    }
}

// auto_circle_movetoedge_start - initialise waypoint controller to move to edge of a circle with it's center at the specified location
//  we assume the caller has set the circle's circle with circle_nav.set_center()
//  we assume the caller has performed all required GPS_ok checks
void Sub::auto_circle_movetoedge_start(const Location &circle_center, float radius_m)
{
    // convert location to vector from ekf origin
    Vector3f circle_center_neu;
    if (!circle_center.get_vector_from_origin_NEU(circle_center_neu)) {
        // default to current position and log error
        circle_center_neu = inertial_nav.get_position();
        Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_FAILED_CIRCLE_INIT);
    }
    circle_nav.set_center(circle_center_neu);

    // set circle radius
    if (!is_zero(radius_m)) {
        circle_nav.set_radius(radius_m * 100.0f);
    }

    // check our distance from edge of circle
    Vector3f circle_edge_neu;
    circle_nav.get_closest_point_on_circle(circle_edge_neu);
    float dist_to_edge = (inertial_nav.get_position() - circle_edge_neu).length();

    // if more than 3m then fly to edge
    if (dist_to_edge > 300.0f) {
        // set the state to move to the edge of the circle
        auto_mode = Auto_CircleMoveToEdge;

        // convert circle_edge_neu to Location
        Location circle_edge(circle_edge_neu);

        // convert altitude to same as command
        circle_edge.set_alt_cm(circle_center.alt, circle_center.get_alt_frame());

        // initialise wpnav to move to edge of circle
        if (!wp_nav.set_wp_destination(circle_edge)) {
            // failure to set destination can only be because of missing terrain data
            failsafe_terrain_on_event();
        }

        // if we are outside the circle, point at the edge, otherwise hold yaw
        const Vector3f &curr_pos = inertial_nav.get_position();
        float dist_to_center = norm(circle_center_neu.x - curr_pos.x, circle_center_neu.y - curr_pos.y);
        if (dist_to_center > circle_nav.get_radius() && dist_to_center > 500) {
            set_auto_yaw_mode(get_default_auto_yaw_mode(false));
        } else {
            // vehicle is within circle so hold yaw to avoid spinning as we move to edge of circle
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    } else {
        auto_circle_start();
    }
}

// auto_circle_start - initialises controller to fly a circle in AUTO flight mode
//   assumes that circle_nav object has already been initialised with circle center and radius
void Sub::auto_circle_start()
{
    auto_mode = Auto_Circle;

    // initialise circle controller
    circle_nav.init(circle_nav.get_center());
}

// auto_circle_run - circle in AUTO flight mode
//      called by auto_run at 100hz or more
void Sub::auto_circle_run()
{
    // call circle controller
    circle_nav.update();

    float lateral_out, forward_out;
    translate_circle_nav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // call z-axis position controller
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.input_euler_angle_roll_pitch_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), circle_nav.get_yaw(), true);
}

#if NAV_GUIDED == ENABLED
// auto_nav_guided_start - hand over control to external navigation controller in AUTO mode
void Sub::auto_nav_guided_start()
{
    auto_mode = Auto_NavGuided;

    // call regular guided flight mode initialisation
    guided_init(true);

    // initialise guided start time and position as reference for limit checking
    guided_limit_init_time_and_pos();
}

// auto_nav_guided_run - allows control by external navigation controller
//      called by auto_run at 100hz or more
void Sub::auto_nav_guided_run()
{
    // call regular guided flight mode run function
    guided_run();
}
#endif  // NAV_GUIDED

// auto_loiter_start - initialises loitering in auto mode
//  returns success/failure because this can be called by exit_mission
bool Sub::auto_loiter_start()
{
    // return failure if GPS is bad
    if (!position_ok()) {
        return false;
    }
    auto_mode = Auto_Loiter;

    Vector3f origin = inertial_nav.get_position();

    // calculate stopping point
    Vector3f stopping_point;
    pos_control.get_stopping_point_xy(stopping_point);
    pos_control.get_stopping_point_z(stopping_point);

    // initialise waypoint controller target to stopping point
    wp_nav.set_wp_origin_and_destination(origin, stopping_point);

    // hold yaw at current heading
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    return true;
}

// auto_loiter_run - loiter in AUTO flight mode
//      called by auto_run at 100hz or more
void Sub::auto_loiter_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

        return;
    }

    // accept pilot input of yaw
    float target_yaw_rate = 0;
    if (!failsafe.pilot_input) {
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint and z-axis position controller
    failsafe_terrain_set_status(wp_nav.update_wpnav());

    ///////////////////////
    // update xy outputs //
    float lateral_out, forward_out;
    translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
}

// get_default_auto_yaw_mode - returns auto_yaw_mode based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
uint8_t Sub::get_default_auto_yaw_mode(bool rtl)
{
    switch (g.wp_yaw_behavior) {

    case WP_YAW_BEHAVIOR_NONE:
        return AUTO_YAW_HOLD;
        break;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
        if (rtl) {
            return AUTO_YAW_HOLD;
        } else {
            return AUTO_YAW_LOOK_AT_NEXT_WP;
        }
        break;

    case WP_YAW_BEHAVIOR_LOOK_AHEAD:
        return AUTO_YAW_LOOK_AHEAD;
        break;

    case WP_YAW_BEHAVIOR_CORRECT_XTRACK:
        return AUTO_YAW_CORRECT_XTRACK;
        break;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
    default:
        return AUTO_YAW_LOOK_AT_NEXT_WP;
        break;
    }
}

// set_auto_yaw_mode - sets the yaw mode for auto
void Sub::set_auto_yaw_mode(uint8_t yaw_mode)
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
    }
}

// set_auto_yaw_look_at_heading - sets the yaw look at heading for auto mode
void Sub::set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, uint8_t relative_angle)
{
    // get current yaw target
    int32_t curr_yaw_target = attitude_control.get_att_target_euler_cd().z;

    // get final angle, 1 = Relative, 0 = Absolute
    if (relative_angle == 0) {
        // absolute angle
        yaw_look_at_heading = wrap_360_cd(angle_deg * 100);
    } else {
        // relative angle
        if (direction < 0) {
            angle_deg = -angle_deg;
        }
        yaw_look_at_heading = wrap_360_cd((angle_deg*100+curr_yaw_target));
    }

    // get turn speed
    // TODO actually implement this, right now, yaw_look_at_heading_slew is unused
    // see AP_Float _slew_yaw in AC_AttitudeControl
    if (is_zero(turn_rate_dps)) {
        // default to regular auto slew rate
        yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
    } else {
        int32_t turn_rate = (wrap_180_cd(yaw_look_at_heading - curr_yaw_target) / 100) / turn_rate_dps;
        yaw_look_at_heading_slew = constrain_int32(turn_rate, 1, 360);    // deg / sec
    }

    // set yaw mode
    set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);

    // TO-DO: restore support for clockwise and counter clockwise rotation held in cmd.content.yaw.direction.  1 = clockwise, -1 = counterclockwise
}

// set_auto_yaw_roi - sets the yaw to look at roi for auto mode
void Sub::set_auto_yaw_roi(const Location &roi_location)
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
    } else {
#if MOUNT == ENABLED
        // check if mount type requires us to rotate the quad
        if (!camera_mount.has_pan_control()) {
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

// get_auto_heading - returns target heading depending upon auto_yaw_mode
// 100hz update rate
float Sub::get_auto_heading()
{
    switch (auto_yaw_mode) {

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

    case AUTO_YAW_CORRECT_XTRACK: {
        // TODO return current yaw if not in appropriate mode
        // Bearing of current track (centidegrees)
        float track_bearing = get_bearing_cd(wp_nav.get_wp_origin(), wp_nav.get_wp_destination());

        // Bearing from current position towards intermediate position target (centidegrees)
        float desired_angle = pos_control.get_bearing_to_target();

        float angle_error = wrap_180_cd(desired_angle - track_bearing);
        float angle_limited = constrain_float(angle_error, -g.xtrack_angle_limit * 100.0f, g.xtrack_angle_limit * 100.0f);
        return wrap_360_cd(track_bearing + angle_limited);
    }
    break;

    case AUTO_YAW_LOOK_AT_NEXT_WP:
    default:
        // point towards next waypoint.
        // we don't use wp_bearing because we don't want the vehicle to turn too much during flight
        return wp_nav.get_yaw();
        break;
    }
}

// Return true if it is possible to recover from a rangefinder failure
bool Sub::auto_terrain_recover_start()
{
    // Check rangefinder status to see if recovery is possible
    switch (rangefinder.status_orient(ROTATION_PITCH_270)) {

    case RangeFinder::RangeFinder_OutOfRangeLow:
    case RangeFinder::RangeFinder_OutOfRangeHigh:

        // RangeFinder_Good if just one valid sample was obtained recently, but ::rangefinder_state.alt_healthy
        // requires several consecutive valid readings for wpnav to accept rangefinder data
    case RangeFinder::RangeFinder_Good:
        auto_mode = Auto_TerrainRecover;
        break;

        // Not connected or no data
    default:
        return false; // Rangefinder is not connected, or has stopped responding
    }

    // Initialize recovery timeout time
    fs_terrain_recover_start_ms = AP_HAL::millis();

    // Stop mission
    mission.stop();

    // Reset xy target
    loiter_nav.clear_pilot_desired_acceleration();
    loiter_nav.init_target();

    // Reset z axis controller
    pos_control.relax_alt_hold_controllers(motors.get_throttle_hover());

    // initialize vertical speeds and leash lengths
    pos_control.set_max_speed_z(wp_nav.get_speed_down(), wp_nav.get_speed_up());
    pos_control.set_max_accel_z(wp_nav.get_accel_z());

    // Reset vertical position and velocity targets
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    gcs().send_text(MAV_SEVERITY_WARNING, "Attempting auto failsafe recovery");
    return true;
}

// Attempt recovery from terrain failsafe
// If recovery is successful resume mission
// If recovery fails revert to failsafe action
void Sub::auto_terrain_recover_run()
{
    float target_climb_rate = 0;
    static uint32_t rangefinder_recovery_ms = 0;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_GROUND_IDLE);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    switch (rangefinder.status_orient(ROTATION_PITCH_270)) {

    case RangeFinder::RangeFinder_OutOfRangeLow:
        target_climb_rate = wp_nav.get_speed_up();
        rangefinder_recovery_ms = 0;
        break;

    case RangeFinder::RangeFinder_OutOfRangeHigh:
        target_climb_rate = wp_nav.get_speed_down();
        rangefinder_recovery_ms = 0;
        break;

    case RangeFinder::RangeFinder_Good: // exit on success (recovered rangefinder data)

        target_climb_rate = 0; // Attempt to hold current depth

        if (rangefinder_state.alt_healthy) {

            // Start timer as soon as rangefinder is healthy
            if (rangefinder_recovery_ms == 0) {
                rangefinder_recovery_ms = AP_HAL::millis();
                pos_control.relax_alt_hold_controllers(motors.get_throttle_hover()); // Reset alt hold targets
            }

            // 1.5 seconds of healthy rangefinder means we can resume mission with terrain enabled
            if (AP_HAL::millis() > rangefinder_recovery_ms + 1500) {
                gcs().send_text(MAV_SEVERITY_INFO, "Terrain failsafe recovery successful!");
                failsafe_terrain_set_status(true); // Reset failsafe timers
                failsafe.terrain = false; // Clear flag
                auto_mode = Auto_Loiter; // Switch back to loiter for next iteration
                mission.resume(); // Resume mission
                rangefinder_recovery_ms = 0; // Reset for subsequent recoveries
            }

        }
        break;

        // Not connected, or no data
    default:
        // Terrain failsafe recovery has failed, terrain data is not available
        // and rangefinder is not connected, or has stopped responding
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Terrain failsafe recovery failure: No Rangefinder!");
        failsafe_terrain_act();
        rangefinder_recovery_ms = 0;
        return;
    }

    // exit on failure (timeout)
    if (AP_HAL::millis() > fs_terrain_recover_start_ms + FS_TERRAIN_RECOVER_TIMEOUT_MS) {
        // Recovery has failed, revert to failsafe action
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Terrain failsafe recovery timeout!");
        failsafe_terrain_act();
    }

    // run loiter controller
    loiter_nav.update();

    ///////////////////////
    // update xy targets //
    float lateral_out, forward_out;
    translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    /////////////////////
    // update z target //
    pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, true);
    pos_control.update_z_controller();

    ////////////////////////////
    // update angular targets //
    float target_roll = 0;
    float target_pitch = 0;

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    float target_yaw_rate = 0;

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
}
