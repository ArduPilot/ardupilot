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

//bev necessary for choosing heading controller
//when false use the roll2yaw ff
bool rtl_use_auto_heading;

// rtl_init - initialise rtl controller
static bool rtl_init(bool ignore_checks)
{
    if (GPS_ok() || ignore_checks) {
        if(rtl_force_wpnav) {
            rtl_return_start();
            rtl_force_wpnav = false;
        } else {
            rtl_climb_start();
        }
        return true;
    }else{
        return false;
    }
}

// rtl_run - runs the return-to-launch controller
// should be called at 100hz or more
static void rtl_run()
{
    //BEV immedidiately reject transitions to plane if close to home
    if( (wp_distance < BEV_RTL_TRANSITION_DISTANCE) ) {
        override_transitions_to_plane();
    }

    // check if we need to move to next state
    if (rtl_state_complete) {
        switch (rtl_state) {
        case InitialClimb:
            rtl_return_start();
            break;
        case ReturnHome:
            rtl_land_start();
            break;
        case Land:
            // do nothing - rtl_land_run will take care of disarming motors
            break;
        }
    }

    // call the correct run function
    switch (rtl_state) {

    case InitialClimb:
        rtl_climb_return_run();
        break;

    case ReturnHome:
        rtl_climb_return_run();
        break;

    case Land:
        rtl_land_run();
        break;
    }
}

// rtl_climb_start - initialise climb to RTL altitude
static void rtl_climb_start()
{
    rtl_state = InitialClimb;
    rtl_state_complete = false;
    rtl_use_auto_heading = true;

    // initialise waypoint and spline controller
    wp_nav.wp_and_spline_init();

    // get horizontal stopping point
    Vector3f destination;
    wp_nav.get_wp_stopping_point_xy(destination);

#if AC_RALLY == ENABLED
    // rally_point.alt will be the altitude of the nearest rally point or the RTL_ALT. uses absolute altitudes
    Location rally_point = rally.calc_best_rally_or_home_location(current_loc, get_RTL_alt()+ahrs.get_home().alt);
    rally_point.alt -= ahrs.get_home().alt; // convert to altitude above home
    rally_point.alt = max(rally_point.alt, current_loc.alt);    // ensure we do not descend before reaching home
    destination.z = rally_point.alt;
#else
    destination.z = get_RTL_alt();
#endif

    // set the destination
    wp_nav.set_wp_destination(destination);
    wp_nav.set_fast_waypoint(true);

    //BEV hold current yaw during initial climb
    set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
    yaw_look_at_heading = ahrs.yaw_sensor;
}

// rtl_return_start - initialise return to home
static void rtl_return_start()
{
    rtl_state = ReturnHome;
    rtl_state_complete = false;
    rtl_use_auto_heading = true;

    Vector3f destination = Vector3f(0,0,get_RTL_alt());

    wp_nav.set_wp_destination(destination);

    // initialise yaw to point home (maybe)
    set_auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP); //BEV the false forces it to use wpnav heading
}

// rtl_climb_return_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
static void rtl_climb_return_run()
{
    int16_t target_roll = 0, target_pitch = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // reset attitude control targets
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        // To-Do: re-initialise wpnav targets
        return;
    }

    //pass target roll and pitch to is_transitioning. Will be set to desired values if indeed transitioning.
    if(is_transitioning_get_angles(target_roll, target_pitch)) {
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, 0, get_smoothing_gain());
    } else {
        // run waypoint controller
        wp_nav.update_wpnav();
        //decide which heading controller to use. Use roll2yawff if it's ever been necessary. Otherwise, auto_heading.
        if(rtl_use_auto_heading) {
            // call attitude controller
            attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(), true);
            if(get_roll2yaw_ff()) {
                //need too much roll to maintian position. Use the roll2yaw feed foward
                rtl_use_auto_heading = false;
            }
        } else {
            attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(wp_nav.get_roll(), wp_nav.get_pitch(), get_roll2yaw_ff(), get_smoothing_gain());
        }
    }

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // check if we've completed this stage of RTL
    rtl_state_complete = wp_nav.reached_wp_destination();
}


// rtl_loiterathome_start - initialise controllers to loiter over home
static void rtl_land_start()
{
    rtl_state = Land;
    rtl_state_complete = false;

    // Set wp navigation target to above home
    wp_nav.init_loiter_target(wp_nav.get_wp_destination());

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    //BEV lower the gear
    gear_lower();
}

// rtl_returnhome_run - return home
//      called by rtl_run at 100hz or more
static void rtl_land_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        // set target to current position
        wp_nav.init_loiter_target();

#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.land_complete && (g.rc_3.control_in == 0 || throttle_input_valid())) {
            init_disarm_motors();
        }
#else
        // disarm when the landing detector says we've landed
        if (ap.land_complete) {
            init_disarm_motors();
        }
#endif

        // check if we've completed this stage of RTL
        rtl_state_complete = ap.land_complete;
        return;
    }

    // relax loiter target if we might be landed
    if (land_complete_maybe()) {
        wp_nav.loiter_soften_for_landing();
    }

    //pass target roll and pitch to is_transitioning. Will be set to desired values if indeed transitioning.
    if(is_transitioning_get_angles(roll_control, pitch_control)) {
        target_yaw_rate = 0.0f;
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(roll_control, pitch_control, target_yaw_rate, get_smoothing_gain());
    } else {
        //handle pilot inputs if any
        if(roll_pitch_input_valid()) {
            roll_control = g.rc_1.control_in;
            pitch_control = g.rc_2.control_in;
        }
        if(yaw_input_valid()) {
            target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
        }
        //process pilot inputs
        wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

        // run waypoint controller
        wp_nav.update_loiter();
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }

    // call z-axis position controller
    float cmb_rate = get_throttle_land();
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt);
    pos_control.update_z_controller();

    // check if we've completed this stage of RTL
    rtl_state_complete = ap.land_complete;
}

// get_RTL_alt - return altitude which vehicle should return home at
//      altitude is in cm above home
static float get_RTL_alt()
{
    // maximum of current altitude and rtl altitude
    return max(current_loc.alt, g.rtl_altitude);
}

