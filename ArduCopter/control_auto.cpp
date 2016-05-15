/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


// auto_init - initialise auto controller
static bool auto_init(bool ignore_checks)
{
    if ((GPS_ok() && inertial_nav.position_ok() && mission.num_commands() > 1) || ignore_checks) {
        // stop ROI from carrying over from previous runs of the mission
        // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
        if (auto_yaw_mode == AUTO_YAW_ROI) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }

        // initialise waypoint and spline controller
        wp_nav.wp_and_spline_init();

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
static void auto_run()
{
    // call the correct auto controller
    switch (auto_mode) {

    case Auto_TakeOff:
        auto_takeoff_run();
        break;

    case Auto_WP:
    case Auto_CircleMoveToEdge:
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

    //prevents compiler warnings
    case Auto_Spline:
        break;

    //prevents compiler warnings
    case Auto_NavGuided:
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

    // BEV ensure smooth spin up
    attitude_control.relax_bf_rate_controller();
    attitude_control.set_yaw_target_to_current_heading();
    attitude_control.set_throttle_out(0, false);
}

// auto_takeoff_run - takeoff in auto mode
//      called by auto_run at 100hz or more
static void auto_takeoff_run()
{
    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // initialise wpnav targets
        wp_nav.shift_wp_origin_to_current_pos();
        // reset attitude control targets
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        // tell motors to do a slow start
        motors.slow_start(true);
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (yaw_input_valid()) {
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

    //BEV set the plane controllers waypoint target
    //if we're within 100m of the previous waypoint us it as the starting location
    //otherwise use the present location. This helps get precise cross tracking for mapping missions
    if( (wp_distance < 10000) && (wp_distance != 0) ) { //!=0 is a check for uninitialization
        prev_WP_loc = next_WP_loc;
    } else {
        prev_WP_loc = current_loc;
    }
    next_WP_loc = pv_vector_to_location(wp_nav.get_destination());
    //determine distance to waypoint (in cm)
    wp_distance = get_distance(current_loc, next_WP_loc)*100;
    alt_hold_gs_des_alt = next_WP_loc.alt;
    setup_turn_angle();
}

// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
static void auto_wp_run()
{
    int16_t target_roll, target_pitch;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // To-Do: reset waypoint origin to current location because copter is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        // tell motors to do a slow start
        motors.slow_start(true);
        return;
    }

    //pass target roll and pitch to is_transitioning. Will be set to desired values if indeed transitioning.
    if(is_transitioning_get_angles(target_roll, target_pitch)) {
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, 0, get_smoothing_gain());
    } else {
        // run waypoint controller
        wp_nav.update_wpnav();
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(), true);
    }

    // call z-axis position controller (wpnav should have already updated its alt target)
    pos_control.update_z_controller();

    //BEV for logging
    desired_climb_rate = pos_control.get_vel_target_z();
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
    //drop the gear
    gear_lower();

    auto_mode = Auto_Land;

    // initialise loiter target destination
    wp_nav.init_loiter_target(destination);

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    //BEV set the plane controllers waypoint target
    prev_WP_loc = current_loc;
    next_WP_loc = pv_vector_to_location(wp_nav.get_destination());
    alt_hold_gs_des_alt = next_WP_loc.alt;
    setup_turn_angle();

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// auto_land_run - lands in auto mode
//      called by auto_run at 100hz or more
static void auto_land_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;

    if(near_land_point_transition_copter()) {
        transition_to_copter();
    }

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        // set target to current position
        wp_nav.init_loiter_target();
        return;
    }

    // relax loiter targets if we might be landed
    if (land_complete_maybe()) {
        wp_nav.loiter_soften_for_landing();
    }

    // process pilot's input
    if (roll_pitch_input_valid()) {
        //BEV hardcoded in land repositioning
        // process pilot's roll and pitch input
        roll_control = g.rc_1.control_in;
        pitch_control = g.rc_2.control_in;
    }
    if(yaw_input_valid()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    //pass target roll and pitch to is_transitioning. Will be set to desired values if indeed transitioning.
    if(is_transitioning_get_angles(roll_control, pitch_control)) {
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(roll_control, pitch_control, 0, get_smoothing_gain());
    } else {
        // process roll, pitch inputs
        wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);
        //run loiter controller
        wp_nav.update_loiter();
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }

    // call z-axis position controller
    pos_control.set_alt_target_from_climb_rate(get_throttle_land(), G_Dt);
    pos_control.update_z_controller();
}

// auto_rtl_start - initialises RTL in AUTO flight mode
static void auto_rtl_start()
{
    //BEV set mode to RTL
    set_mode(RTL);
}

// auto_rtl_run - rtl in AUTO flight mode
//      called by auto_run at 100hz or more
void auto_rtl_run()
{
    // call regular rtl flight mode run function
    rtl_run();
}

// auto_circle_movetoedge_start - initialise waypoint controller to move to edge of a circle with it's center at the specified location
//  we assume the caller has set the circle's circle with circle_nav.set_center()
//  we assume the caller has performed all required GPS_ok checks
static void auto_circle_movetoedge_start()
{
    // check our distance from edge of circle
    Vector3f circle_edge;
    circle_nav.get_closest_point_on_circle(circle_edge);

    // set the state to move to the edge of the circle
    auto_mode = Auto_CircleMoveToEdge;

    // initialise wpnav to move to edge of circle
    wp_nav.set_wp_destination(circle_edge);

    // if we are outside the circle, point at the edge, otherwise hold yaw
    const Vector3f &curr_pos = inertial_nav.get_position();
    const Vector3f &circle_center = circle_nav.get_center();
    float dist_to_center = pythagorous2(circle_center.x - curr_pos.x, circle_center.y - curr_pos.y);
    if (dist_to_center > circle_nav.get_radius() && dist_to_center > 500) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    } else {
        // vehicle is within circle so hold yaw to avoid spinning as we move to edge of circle
        set_auto_yaw_mode(AUTO_YAW_HOLD);
    }
}

// auto_circle_start - initialises controller to fly a circle in AUTO flight mode
static void auto_circle_start()
{
    auto_mode = Auto_Circle;

    // initialise circle controller
    // center was set in do_circle so initialise with current center
    circle_nav.init(circle_nav.get_center());
}

// auto_circle_run - circle in AUTO flight mode
//      called by auto_run at 100hz or more
void auto_circle_run()
{
    int16_t target_roll, target_pitch;

    //pass target roll and pitch to is_transitioning. Will be set to desired values if indeed transitioning.
    if(is_transitioning_get_angles(target_roll, target_pitch)) {
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, 0, get_smoothing_gain());
    } else {
        // call circle controller
        circle_nav.update();
        if(circle_nav.get_radius() > 0) {
            //BEV always look tangential to the circle in the direction of motion. THis should be smoother than relying on GPS velocity vector
            attitude_control.angle_ef_roll_pitch_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), wrap_180_cd(circle_nav.get_yaw() - 9000), true);
        } else {
            //BEV always look tangential to the circle in the direction of motion. THis should be smoother than relying on GPS velocity vector
            attitude_control.angle_ef_roll_pitch_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), wrap_180_cd(circle_nav.get_yaw() + 9000), true);
        }
    }

    // call z-axis position controller
    pos_control.update_z_controller();
}

#if NAV_GUIDED == ENABLED
// auto_nav_guided_start - hand over control to external navigation controller in AUTO mode
void auto_nav_guided_start()
{
    auto_mode = Auto_NavGuided;

    // call regular guided flight mode initialisation
    guided_init(true);
}

// auto_nav_guided_run - allows control by external navigation controller
//      called by auto_run at 100hz or more
void auto_nav_guided_run()
{
    // call regular guided flight mode run function
    guided_run();
}
#endif  // NAV_GUIDED

// get_default_auto_yaw_mode - returns auto_yaw_mode based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
uint8_t get_default_auto_yaw_mode(bool rtl)
{
    //BEV always look at next waypoint unless RTL
    if(rtl) {
        //don't turn
        yaw_look_at_heading = ahrs.yaw_sensor;
        return AUTO_YAW_LOOK_AT_HEADING;
    }else{
        //always point at next waypoint
        return AUTO_YAW_LOOK_AT_NEXT_WP;
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
    // wpnav will initialise heading when wpnav's set_destination method is called
}

// set_auto_yaw_look_at_heading - sets the yaw look at heading for auto mode 
static void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, uint8_t relative_angle)
{
    // get current yaw target
    int32_t curr_yaw_target = attitude_control.angle_ef_targets().z;

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
    if ( fabs(turn_rate_dps) < 0.1 ) {
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
static void set_auto_yaw_roi(const Location &roi_location)
{
    // if location is zero lat, lon and altitude turn off ROI
    if (auto_yaw_mode == AUTO_YAW_ROI && (roi_location.alt == 0 && roi_location.lat == 0 && roi_location.lng == 0)) {
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
        if(camera_mount.get_mount_type() != AP_Mount::k_pan_tilt && camera_mount.get_mount_type() != AP_Mount::k_pan_tilt_roll) {
            roi_WP = pv_location_to_vector(roi_location);
            set_auto_yaw_mode(AUTO_YAW_ROI);
        }
        // send the command to the camera mount
        camera_mount.set_roi_cmd(&roi_location);

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
        return wp_nav.get_yaw();
        break;
    }
}

