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

// forward declarations to make compiler happy
static void do_takeoff(const AP_Mission::Mission_Command& cmd);
static void do_nav_wp(const AP_Mission::Mission_Command& cmd);
static void do_land(const AP_Mission::Mission_Command& cmd);
static void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
static void do_circle(const AP_Mission::Mission_Command& cmd);
static void do_loiter_time(const AP_Mission::Mission_Command& cmd);
static void do_set_roi(const AP_Mission::Mission_Command& cmd);
static void do_set_servos(const AP_Mission::Mission_Command& cmd);
static bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
static bool verify_circle(const AP_Mission::Mission_Command& cmd);

// start_command - this function will be called when the ap_mission lib wishes to start a new command
static bool start_command(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: logging when new commands start/end
    if (should_log(MASK_LOG_CMD)) {
        Log_Write_Cmd(cmd);
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

    case MAV_CMD_NAV_LOITER_UNLIM:              // 17 Loiter indefinitely
        do_loiter_unlimited(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              //18 Loiter N Times
        do_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:              // 19
        do_loiter_time(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:             //20
        do_RTL();
        break;
    case MAV_CMD_DO_SET_SERVO:                  //183
        do_set_servos(cmd);
        break;

    //BEV our do commands
    case MAV_CMD_DO_TRANSITION_TOGGLE:
        transition_toggle();
        break;
    case MAV_CMD_DO_GEAR_TOGGLE:
        gear_toggle();
        break;
    case MAV_CMD_DO_SET_ROI:
        do_set_roi(cmd);
        break;

    //camera stuff
#if CAMERA == ENABLED
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        camera.set_trigger_distance(cmd.content.cam_trigg_dist.meters);
        break;
#endif

    default:
        // do nothing with unrecognized MAVLink messages
        break;
    }

    // always return success
    return true;
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

// verify_command - this will be called repeatedly by ap_mission lib to ensure the active commands are progressing
//  should return true once the active navigation command completes successfully
//  called at 10hz or higher
static bool verify_command(const AP_Mission::Mission_Command& cmd)
{
    switch(cmd.id) {

    //
    // navigation commands
    //
    case MAV_CMD_NAV_TAKEOFF:
        return verify_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:
        return verify_land();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();
        break;

    default:
        // return true if we do not recognise the command so that we move on to the next command
        return true;
        break;
    }
}

// exit_mission - function that is called once the mission completes
static void exit_mission()
{
    // BEV if we are not on the ground switch do a loiter unlimited with home as the target
    if(!ap.land_complete) {
        mission.start_loiter_at_home(get_RTL_alt());
    }else{
#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (g.rc_3.control_in == 0 || !throttle_input_valid()) {
            init_disarm_motors();
        }
#else
        // if we've landed it's safe to disarm
        init_disarm_motors();
#endif
    }
}

/********************************************************************************/
//
/********************************************************************************/

// do_RTL - start Return-to-Launch
static void do_RTL(void)
{
    // start rtl in auto flight mode
    auto_rtl_start();
}

static void do_set_servos(const AP_Mission::Mission_Command& cmd)
{
    //to add
    servos.output(cmd.content.servo.channel, cmd.content.servo.pwm);
}

/********************************************************************************/
//	Nav (Must) commands
/********************************************************************************/

// do_takeoff - initiate takeoff navigation command
static void do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    //keep from doing a takeoff in plane mode, but be very careful not to disarm if in the air
    if( is_plane_nav_active() && is_on_ground_maybe() )
    {
        init_disarm_motors();
    }

    // Set wp navigation target to safe altitude above current position
    float takeoff_alt = cmd.content.location.alt;
    takeoff_alt = max(takeoff_alt,current_loc.alt);
    takeoff_alt = max(takeoff_alt,100.0f);
    auto_takeoff_start(takeoff_alt);
}

// do_nav_wp - initiate move to next waypoint
static void do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    const Vector3f &curr_pos = inertial_nav.get_position();
    Vector3f local_pos = pv_location_to_vector(cmd.content.location);

    // set target altitude to current altitude if not provided
    if (cmd.content.location.alt == 0) {
        local_pos.z = curr_pos.z;
    }

    // set lat/lon position to current position if not provided
    if (cmd.content.location.lat == 0 || cmd.content.location.lng == 0) {
        local_pos.x = curr_pos.x;
        local_pos.y = curr_pos.y;
    }

    //used by plane to mark the desired lat/lon and alt as reached
    pln_nav_position_arrived = false;
    pln_nav_altitude_arrived = false;

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = abs(cmd.p1);

    // Set wp navigation target
    auto_wp_start(local_pos);
    // if no delay set the waypoint as "fast"
    if (loiter_time_max == 0 ) {
        wp_nav.set_fast_waypoint(true);
    }
}

// do_land - initiate landing procedure
static void do_land(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: check if we have already landed
    //BEV update plane desired waypoint accordingly
    if( is_plane_nav_active()) {
        //BEV set plane's target location
        prev_WP_loc = current_loc;
        if (cmd.content.location.lat == 0 || cmd.content.location.lng == 0) {
            //if not navigating to a certian location transition immediately
            transition_to_copter();
        } else {
            //set the desired waypoint as provided
            next_WP_loc = cmd.content.location;
        }
        alt_hold_gs_des_alt = current_loc.alt;
    }

    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 && cmd.content.location.lng != 0) {
        // set state to fly to location
        land_state = LAND_STATE_FLY_TO_LOCATION;

        // calculate and set desired location above landing target
        Vector3f pos = pv_location_to_vector(cmd.content.location);
        pos.z = current_loc.alt;
        auto_wp_start(pos);
    }else{
        // set landing state
        land_state = LAND_STATE_DESCENDING;

        // initialise landing controller
        auto_land_start();
    }
}

// do_loiter_unlimited - start loitering with no end conditions
// note: caller should set yaw_mode
static void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    Vector3f target_pos;

    // get current position
    Vector3f curr_pos = inertial_nav.get_position();

    // default to use position provided
    target_pos = pv_location_to_vector(cmd.content.location);

    // use current location if not provided
    if(cmd.content.location.lat == 0 || cmd.content.location.lng == 0) {
        wp_nav.get_wp_stopping_point_xy(target_pos);
    }

    // use current altitude if not provided
    // To-Do: use z-axis stopping point instead of current alt
    if( cmd.content.location.alt == 0 ) {
        target_pos.z = curr_pos.z;
    }

    // start way point navigator and provide it the desired location
    auto_wp_start(target_pos);
}

// do_circle - initiate moving in a circle
static void do_circle(const AP_Mission::Mission_Command& cmd)
{
    Vector3f curr_pos = inertial_nav.get_position();
    Vector3f circle_center = pv_location_to_vector(cmd.content.location);
    uint8_t circle_radius_m = HIGHBYTE(cmd.p1); // circle radius held in high byte of p1
    bool move_to_edge_required = false;

    // set target altitude if not provided
    if (cmd.content.location.alt == 0) {
        circle_center.z = curr_pos.z;
    } else {
        move_to_edge_required = true;
    }

    // set lat/lon position if not provided
    // To-Do: use previous command's destination if it was a straight line or spline waypoint command
    //BEV changed from && to ||
    if (cmd.content.location.lat == 0 || cmd.content.location.lng == 0) {
        circle_center.x = curr_pos.x;
        circle_center.y = curr_pos.y;
    } else {
        move_to_edge_required = true;
    }

    // set circle controller's center
    circle_nav.set_center(circle_center);

    // set circle radius
    if (circle_radius_m != 0) {
        circle_nav.set_radius((float)circle_radius_m * 100.0f);
    }

    // set plane targets accordingly.
    prev_WP_loc = current_loc;
    next_WP_loc = pv_vector_to_location(circle_center);
    alt_hold_gs_des_alt = next_WP_loc.alt;
    loiter_angle_reset();
    setup_turn_angle();

    if (move_to_edge_required) {
        // move to edge of circle (verify_circle) will ensure we begin circling once we reach the edge
        auto_circle_movetoedge_start();
    } else {
        // start circling
        auto_circle_start();
    }
}

// do_loiter_time - initiate loitering at a point for a given time period
// note: caller should set yaw_mode
static void do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    Vector3f target_pos;

    // get current position
    Vector3f curr_pos = inertial_nav.get_position();

    // default to use position provided
    target_pos = pv_location_to_vector(cmd.content.location);

    // use current location if not provided
    if(cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
        wp_nav.get_wp_stopping_point_xy(target_pos);
    }

    // use current altitude if not provided
    if( cmd.content.location.alt == 0 ) {
        target_pos.z = curr_pos.z;
    }

    // start way point navigator and provide it the desired location
    auto_wp_start(target_pos);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = cmd.p1;     // units are (seconds)
}
//BEV adding ROI command
static void do_set_roi(const AP_Mission::Mission_Command& cmd)
{
    payload_manager.gimbal.point_here(cmd.content.location.lat, cmd.content.location.lng, cmd.content.location.alt);
}

/********************************************************************************/
//	Verify Nav (Must) commands
/********************************************************************************/

// verify_takeoff - check if we have completed the takeoff
static bool verify_takeoff()
{
    //BEV if a takeoff is accidentally inserted in plane mode say it's verified
    //so we can move onto the next waypoint
    if(is_plane_nav_active()) {
        return true;
    }

    // have we reached our target altitude?
    //BEV raise the gear if we have completed takeoff
    if(wp_nav.reached_wp_destination()) {
        gear_raise();
        return true;
    }
    return false;
    //return wp_nav.reached_wp_destination();
}

// verify_land - returns true if landing has been completed
static bool verify_land()
{
    bool retval = false;
    if(is_plane_nav_active()) {
        update_loiter();
        if(near_land_point_transition_copter()) {
            transition_to_copter();
        }
        return false;
    }

    switch( land_state ) {
        case LAND_STATE_FLY_TO_LOCATION:
            // check if we've reached the location
            if (wp_nav.reached_wp_destination()) {
                // get destination so we can use it for loiter target
                Vector3f dest = wp_nav.get_wp_destination();

                // initialise landing controller
                auto_land_start(dest);

                // advance to next state
                land_state = LAND_STATE_DESCENDING;
            }
            break;

        case LAND_STATE_DESCENDING:
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

// verify_nav_wp - check if we have reached the next way point
static bool verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if( !reached_wp() ) {
        return false;
    }

    // start timer if necessary
    if(loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs_send_text_fmt(PSTR("Reached Command #%i"),cmd.index);
        return true;
    }else{
        return false;
    }
}

//BEV added this function to allow switching between plane and copter wp arrival logic
static bool reached_wp()
{
    if(is_plane_nav_active()) {
        //if loiter time is counting, we've already marked this waypoint as reached. Update the loiter controller and return true
        if(loiter_time) {
            update_loiter();
            return true;
        }

        //BEV override altitude arrival check. Just uncomment the below to make plane loiter until reached alt
        pln_nav_altitude_arrived = true;
        //BEV see if arrived at altitude
        //if(!pln_nav_altitude_arrived && (abs(current_loc.alt - next_WP_loc.alt) < 300)) {
        //    pln_nav_altitude_arrived = true;
        //

        //BEV see if arrived at lat / lon, or flown past
        if(!pln_nav_position_arrived) {
            //turn_distance in m. Convert to cm
            if (wp_distance <= nav_controller->turn_distance(g.plane_loiter_radius, auto_state.next_turn_angle)*100) {
                gcs_send_text_fmt(PSTR("Reached Waypoint #%i dist %um"),
                                  (unsigned)mission.get_current_nav_cmd().index,
                                  (unsigned)get_distance(current_loc, next_WP_loc));
                pln_nav_position_arrived = true;
            }

            // have we flown past the waypoint?
            if (location_passed_point(current_loc, prev_WP_loc, next_WP_loc)) {
                gcs_send_text_fmt(PSTR("Passed Waypoint #%i dist %um"),
                              (unsigned)mission.get_current_nav_cmd().index,
                              (unsigned)get_distance(current_loc, next_WP_loc));
                pln_nav_position_arrived = true;
            }
        }

        if(!pln_nav_position_arrived) {
            nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
            return false;
        }
        if(pln_nav_position_arrived && (!pln_nav_altitude_arrived)) {
            update_loiter();
            return false;
        }

        return true;
    } else { //copter ctrl running
        return wp_nav.reached_wp_destination();
    }
}

static bool verify_loiter_unlimited()
{
    if(is_plane_nav_active()) {
        update_loiter();
    }
    return false;
}

// verify_loiter_time - check if we have loitered long enough
static bool verify_loiter_time()
{
    // return immediately if we haven't reached our destination
    if (!reached_wp()) {
        return false;
    }

    // start our loiter timer
    if( loiter_time == 0 ) {
        loiter_time = millis();
    }

    // check if loiter timer has run out
    return (((millis() - loiter_time) / 1000) >= loiter_time_max);
}

// verify_circle - check if we have circled the point enough
static bool verify_circle(const AP_Mission::Mission_Command& cmd)
{
    if(is_plane_nav_active()) { //update plane controllers
        update_loiter();
        return fabsf(loiter.sum_cd) >= (float)LOWBYTE(cmd.p1)*36000;
    } else { //copter
        // check if we've reached the edge
        if (auto_mode == Auto_CircleMoveToEdge) {
            if (wp_nav.reached_wp_destination()) {
                Vector3f curr_pos = inertial_nav.get_position();
                Vector3f circle_center = pv_location_to_vector(cmd.content.location);

                // set target altitude if not provided
                if (fabs(circle_center.z) < 1.0f) {
                    circle_center.z = curr_pos.z;
                }

                // set lat/lon position if not provided
                if (cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
                    circle_center.x = curr_pos.x;
                    circle_center.y = curr_pos.y;
                }

                // start circling
                auto_circle_start();
            }
            return false;
        }

        // check if we have completed circling
        return fabsf(circle_nav.get_angle_total()/(2.0f*(float)M_PI)) >= (float)LOWBYTE(cmd.p1);
    }
}

// externs to remove compiler warning
extern bool rtl_state_complete;

// verify_RTL - handles any state changes required to implement RTL
// do_RTL should have been called once first to initialise all variables
// returns true with RTL has completed successfully
static bool verify_RTL()
{
    //BEV this should never be called as a do_rtl forces into RTL mode
    return false;
}

/********************************************************************************/
//	Do (Now) commands
/********************************************************************************/

// do_guided - start guided mode
static bool do_guided(const AP_Mission::Mission_Command& cmd)
{
    Vector3f pos_or_vel;    // target location or velocity

    // only process guided waypoint if we are in guided mode
    if (control_mode != GUIDED && !(control_mode == AUTO && auto_mode == Auto_NavGuided)) {
        return false;
    }

    // switch to handle different commands
    switch (cmd.id) {

        case MAV_CMD_NAV_WAYPOINT:
            // set wp_nav's destination
            pos_or_vel = pv_location_to_vector(cmd.content.location);
            guided_set_destination(pos_or_vel);
            return true;
            break;
        default:
            // reject unrecognised command
            return false;
            break;
    }

    return true;
}

// do_take_picture - take a picture with the camera library
static void do_take_picture()
{
#if CAMERA == ENABLED
    if(get_key_level() != BEV_Key::KEY_MAPPING) {
        return;
    }
    //use aux6 to trigger camera
    camera.trigger_pic();
    //trigger payload camera
    payload_manager.cameraTrigger.trigger_camera();

    //BEV if the next call is going to start the log, make sure the message is sent prior to starting the log.
    //otherwise there can be a log delay
    if(!DataFlash.logging_started() && !in_log_download) {
        //make sure the payloadCommunicator is ready to receive
        hal.scheduler->delay(41);
        //push the message through
        payload_manager.update();
    }

    if (can_force_log()) {
        DataFlash.Log_Write_Camera(ahrs, gps, current_loc);
    }
#endif
}
