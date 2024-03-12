#include "Sub.h"

/*
 * control_auto.cpp
 *  Contains the mission, waypoint navigation and NAV_CMD item implementation
 *
 *  While in the auto flight mode, navigation or do/now commands can be run.
 *  Code in this file implements the navigation commands
 */
bool ModeAuto::init(bool ignore_checks) {
     if (!sub.position_ok() || sub.mission.num_commands() < 2) {
        return false;
    }

    sub.auto_mode = Auto_Loiter;

    // stop ROI from carrying over from previous runs of the mission
    // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
    if (sub.auto_yaw_mode == AUTO_YAW_ROI) {
        set_auto_yaw_mode(AUTO_YAW_HOLD);
    }

    // initialise waypoint controller
    sub.wp_nav.wp_and_spline_init();

    // clear guided limits
    guided_limit_clear();

    // start/resume the mission (based on MIS_RESTART parameter)
    sub.mission.start_or_resume();
    return true;
}

// auto_run - runs the appropriate auto controller
// according to the current auto_mode
void ModeAuto::run()
{
    sub.mission.update();

    // call the correct auto controller
    switch (sub.auto_mode) {

    case Auto_WP:
    case Auto_CircleMoveToEdge:
        auto_wp_run();
        break;

    case Auto_Circle:
        auto_circle_run();
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
void ModeAuto::auto_wp_start(const Vector3f& destination)
{
    sub.auto_mode = Auto_WP;

    // initialise wpnav (no need to check return status because terrain data is not used)
    sub.wp_nav.set_wp_destination(destination, false);

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (sub.auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    }
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void ModeAuto::auto_wp_start(const Location& dest_loc)
{
    sub.auto_mode = Auto_WP;

    // send target to waypoint controller
    if (!sub.wp_nav.set_wp_destination_loc(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        sub.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (sub.auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    }
}

// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void ModeAuto::auto_wp_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        // To-Do: reset waypoint origin to current location because vehicle is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
        // call attitude controller
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        sub.wp_nav.wp_and_spline_init();                                                // Reset xy target
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!sub.failsafe.pilot_input) {
        // get pilot's desired yaw rate
        target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    // TODO logic for terrain tracking target going below fence limit
    // TODO implement waypoint radius individually for each waypoint based on cmd.p2
    // TODO fix auto yaw heading to switch to something appropriate when mission complete and switches to loiter
    sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());

    ///////////////////////
    // update xy outputs //

    float lateral_out, forward_out;
    sub.translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    position_control->update_z_controller();

    ////////////////////////////
    // update attitude output //

    // get pilot desired lean angles
    float target_roll, target_pitch;
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, sub.aparm.angle_max);

    // call attitude controller
    if (sub.auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch & yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    } else {
        // roll, pitch from pilot, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, get_auto_heading(), true);
    }
}

// auto_circle_movetoedge_start - initialise waypoint controller to move to edge of a circle with it's center at the specified location
//  we assume the caller has set the circle's circle with sub.circle_nav.set_center()
//  we assume the caller has performed all required GPS_ok checks
void ModeAuto::auto_circle_movetoedge_start(const Location &circle_center, float radius_m, bool ccw_turn)
{
    // set circle center
    sub.circle_nav.set_center(circle_center);

    // set circle radius
    if (!is_zero(radius_m)) {
        sub.circle_nav.set_radius_cm(radius_m * 100.0f);
    }

     // set circle direction by using rate
    float current_rate = sub.circle_nav.get_rate();
    current_rate = ccw_turn ? -fabsf(current_rate) : fabsf(current_rate);
    sub.circle_nav.set_rate(current_rate);

    // check our distance from edge of circle
    Vector3f circle_edge_neu;
    sub.circle_nav.get_closest_point_on_circle(circle_edge_neu);
    float dist_to_edge = (inertial_nav.get_position_neu_cm() - circle_edge_neu).length();

    // if more than 3m then fly to edge
    if (dist_to_edge > 300.0f) {
        // set the state to move to the edge of the circle
        sub.auto_mode = Auto_CircleMoveToEdge;

        // convert circle_edge_neu to Location
        Location circle_edge(circle_edge_neu, Location::AltFrame::ABOVE_ORIGIN);

        // convert altitude to same as command
        circle_edge.set_alt_cm(circle_center.alt, circle_center.get_alt_frame());

        // initialise wpnav to move to edge of circle
        if (!sub.wp_nav.set_wp_destination_loc(circle_edge)) {
            // failure to set destination can only be because of missing terrain data
            sub.failsafe_terrain_on_event();
        }

        // if we are outside the circle, point at the edge, otherwise hold yaw
        float dist_to_center = get_horizontal_distance_cm(inertial_nav.get_position_xy_cm().topostype(), sub.circle_nav.get_center().xy());
        if (dist_to_center > sub.circle_nav.get_radius() && dist_to_center > 500) {
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
void ModeAuto::auto_circle_start()
{
    sub.auto_mode = Auto_Circle;

    // initialise circle controller
    sub.circle_nav.init(sub.circle_nav.get_center(), sub.circle_nav.center_is_terrain_alt(), sub.circle_nav.get_rate());
}

// auto_circle_run - circle in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeAuto::auto_circle_run()
{
    // call circle controller
    sub.failsafe_terrain_set_status(sub.circle_nav.update());

    float lateral_out, forward_out;
    sub.translate_circle_nav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    position_control->update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), sub.circle_nav.get_yaw(), true);
}

#if NAV_GUIDED == ENABLED
// auto_nav_guided_start - hand over control to external navigation controller in AUTO mode
void ModeAuto::auto_nav_guided_start()
{
    sub.mode_guided.init(true);
    sub.auto_mode = Auto_NavGuided;
    // initialise guided start time and position as reference for limit checking
    sub.mode_auto.guided_limit_init_time_and_pos();
}

// auto_nav_guided_run - allows control by external navigation controller
//      called by auto_run at 100hz or more
void ModeAuto::auto_nav_guided_run()
{
    // call regular guided flight mode run function
    sub.mode_guided.run();
}
#endif  // NAV_GUIDED

// auto_loiter_start - initialises loitering in auto mode
//  returns success/failure because this can be called by exit_mission
bool ModeAuto::auto_loiter_start()
{
    // return failure if GPS is bad
    if (!sub.position_ok()) {
        return false;
    }
    sub.auto_mode = Auto_Loiter;

    // calculate stopping point
    Vector3f stopping_point;
    sub.wp_nav.get_wp_stopping_point(stopping_point);

    // initialise waypoint controller target to stopping point
    sub.wp_nav.set_wp_destination(stopping_point);

    // hold yaw at current heading
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    return true;
}

// auto_loiter_run - loiter in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeAuto::auto_loiter_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();

        sub.wp_nav.wp_and_spline_init();                                                // Reset xy target
        return;
    }

    // accept pilot input of yaw
    float target_yaw_rate = 0;
    if (!sub.failsafe.pilot_input) {
        target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint and z-axis position controller
    sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());

    ///////////////////////
    // update xy outputs //
    float lateral_out, forward_out;
    sub.translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    position_control->update_z_controller();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, sub.aparm.angle_max);

    // roll & pitch & yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
}


// set_auto_yaw_look_at_heading - sets the yaw look at heading for auto mode
void ModeAuto::set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, uint8_t relative_angle)
{
    // get current yaw
    int32_t curr_yaw_target = attitude_control->get_att_target_euler_cd().z;

    // get final angle, 1 = Relative, 0 = Absolute
    if (relative_angle == 0) {
        // absolute angle
        sub.yaw_look_at_heading = wrap_360_cd(angle_deg * 100);
    } else {
        // relative angle
        if (direction < 0) {
            angle_deg = -angle_deg;
        }
        sub.yaw_look_at_heading = wrap_360_cd((angle_deg * 100 + curr_yaw_target));
    }

    // get turn speed
    if (is_zero(turn_rate_dps)) {
        // default to regular auto slew rate
        sub.yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
    } else {
        sub.yaw_look_at_heading_slew = MIN(turn_rate_dps, AUTO_YAW_SLEW_RATE);    // deg / sec
    }

    // set yaw mode
    set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);

    // TO-DO: restore support for clockwise and counter clockwise rotation held in cmd.content.yaw.direction.  1 = clockwise, -1 = counterclockwise
}


// sets the desired yaw rate
void ModeAuto::set_yaw_rate(float turn_rate_dps)
{    
    // set sub to desired yaw rate
    sub.yaw_look_at_heading_slew = MIN(turn_rate_dps, AUTO_YAW_SLEW_RATE);    // deg / sec

    // set yaw mode
    set_auto_yaw_mode(AUTO_YAW_RATE);
}

// set_auto_yaw_roi - sets the yaw to look at roi for auto mode
void ModeAuto::set_auto_yaw_roi(const Location &roi_location)
{
    // if location is zero lat, lon and altitude turn off ROI
    if (roi_location.alt == 0 && roi_location.lat == 0 && roi_location.lng == 0) {
        // set auto yaw mode back to default assuming the active command is a waypoint command.  A more sophisticated method is required to ensure we return to the proper yaw control for the active command
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
#if HAL_MOUNT_ENABLED
        // switch off the camera tracking if enabled
        sub.camera_mount.clear_roi_target();
#endif  // HAL_MOUNT_ENABLED
    } else {
#if HAL_MOUNT_ENABLED
        // check if mount type requires us to rotate the sub
        if (!sub.camera_mount.has_pan_control()) {
            if (roi_location.get_vector_from_origin_NEU(sub.roi_WP)) {
                set_auto_yaw_mode(AUTO_YAW_ROI);
            }
        }
        // send the command to the camera mount
        sub.camera_mount.set_roi_target(roi_location);

        // TO-DO: expand handling of the do_nav_roi to support all modes of the MAVLink.  Currently we only handle mode 4 (see below)
        //      0: do nothing
        //      1: point at next waypoint
        //      2: point at a waypoint taken from WP# parameter (2nd parameter?)
        //      3: point at a location given by alt, lon, lat parameters
        //      4: point at a target given a target id (can't be implemented)
#else
        // if we have no camera mount aim the sub at the location
        if (roi_location.get_vector_from_origin_NEU(sub.roi_WP)) {
            set_auto_yaw_mode(AUTO_YAW_ROI);
        }
#endif  // HAL_MOUNT_ENABLED
    }
}

// Return true if it is possible to recover from a rangefinder failure
bool ModeAuto::auto_terrain_recover_start()
{
    // Check rangefinder status to see if recovery is possible
    switch (sub.rangefinder.status_orient(ROTATION_PITCH_270)) {

    case RangeFinder::Status::OutOfRangeLow:
    case RangeFinder::Status::OutOfRangeHigh:

        // RangeFinder::Good if just one valid sample was obtained recently, but ::rangefinder_state.alt_healthy
        // requires several consecutive valid readings for wpnav to accept rangefinder data
    case RangeFinder::Status::Good:
        sub.auto_mode = Auto_TerrainRecover;
        break;

        // Not connected or no data
    default:
        return false; // Rangefinder is not connected, or has stopped responding
    }

    // Initialize recovery timeout time
    sub.fs_terrain_recover_start_ms = AP_HAL::millis();

    // Stop mission
    sub.mission.stop();

    // Reset xy target
    sub.loiter_nav.clear_pilot_desired_acceleration();
    sub.loiter_nav.init_target();

    // Reset z axis controller
    position_control->relax_z_controller(motors.get_throttle_hover());

    // initialize vertical maximum speeds and acceleration
    position_control->set_max_speed_accel_z(sub.wp_nav.get_default_speed_down(), sub.wp_nav.get_default_speed_up(), sub.wp_nav.get_accel_z());
    position_control->set_correction_speed_accel_z(sub.wp_nav.get_default_speed_down(), sub.wp_nav.get_default_speed_up(), sub.wp_nav.get_accel_z());

    gcs().send_text(MAV_SEVERITY_WARNING, "Attempting auto failsafe recovery");
    return true;
}

// Attempt recovery from terrain failsafe
// If recovery is successful resume mission
// If recovery fails revert to failsafe action
void ModeAuto::auto_terrain_recover_run()
{
    float target_climb_rate = 0;
    static uint32_t rangefinder_recovery_ms = 0;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();

        sub.loiter_nav.init_target();                                                   // Reset xy target
        position_control->relax_z_controller(motors.get_throttle_hover());                // Reset z axis controller
        return;
    }

    switch (sub.rangefinder.status_orient(ROTATION_PITCH_270)) {

    case RangeFinder::Status::OutOfRangeLow:
        target_climb_rate = sub.wp_nav.get_default_speed_up();
        rangefinder_recovery_ms = 0;
        break;

    case RangeFinder::Status::OutOfRangeHigh:
        target_climb_rate = sub.wp_nav.get_default_speed_down();
        rangefinder_recovery_ms = 0;
        break;

    case RangeFinder::Status::Good: // exit on success (recovered rangefinder data)

        target_climb_rate = 0; // Attempt to hold current depth

        if (sub.rangefinder_state.alt_healthy) {

            // Start timer as soon as rangefinder is healthy
            if (rangefinder_recovery_ms == 0) {
                rangefinder_recovery_ms = AP_HAL::millis();
                position_control->relax_z_controller(motors.get_throttle_hover()); // Reset alt hold targets
            }

            // 1.5 seconds of healthy rangefinder means we can resume mission with terrain enabled
            if (AP_HAL::millis() > rangefinder_recovery_ms + 1500) {
                gcs().send_text(MAV_SEVERITY_INFO, "Terrain failsafe recovery successful!");
                sub.failsafe_terrain_set_status(true); // Reset failsafe timers
                sub.failsafe.terrain = false; // Clear flag
                sub.auto_mode = Auto_Loiter; // Switch back to loiter for next iteration
                sub.mission.resume(); // Resume mission
                rangefinder_recovery_ms = 0; // Reset for subsequent recoveries
            }

        }
        break;

        // Not connected, or no data
    default:
        // Terrain failsafe recovery has failed, terrain data is not available
        // and rangefinder is not connected, or has stopped responding
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Terrain failsafe recovery failure: No Rangefinder!");
        sub.failsafe_terrain_act();
        rangefinder_recovery_ms = 0;
        return;
    }

    // exit on failure (timeout)
    if (AP_HAL::millis() > sub.fs_terrain_recover_start_ms + FS_TERRAIN_RECOVER_TIMEOUT_MS) {
        // Recovery has failed, revert to failsafe action
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Terrain failsafe recovery timeout!");
        sub.failsafe_terrain_act();
    }

    // run loiter controller
    sub.loiter_nav.update();

    ///////////////////////
    // update xy targets //
    float lateral_out, forward_out;
    sub.translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    /////////////////////
    // update z target //
    position_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
    position_control->update_z_controller();

    ////////////////////////////
    // update angular targets //
    float target_roll = 0;
    float target_pitch = 0;

    // convert pilot input to lean angles
    // To-Do: convert sub.get_pilot_desired_lean_angles to return angles as floats
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, sub.aparm.angle_max);

    float target_yaw_rate = 0;

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
}
