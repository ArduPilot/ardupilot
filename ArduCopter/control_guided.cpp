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

// guided_init - initialise guided controller
static bool guided_init(bool ignore_checks)
{
    if (GPS_ok() || ignore_checks) {
        // initialise yaw
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
        // start in position control mode
        guided_pos_control_start();
        return true;
    }else{
        return false;
    }
}


// guided_takeoff_start - initialises waypoint controller to implement take-off
static void guided_takeoff_start(float final_alt)
{
    guided_mode = Guided_TakeOff;
    
    // initialise wpnav destination
    Vector3f target_pos = inertial_nav.get_position();
    target_pos.z = final_alt;
    wp_nav.set_wp_destination(target_pos);

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    // tell motors to do a slow start
    motors.slow_start(true);
}

// initialise guided mode's position controller
void guided_pos_control_start()
{
    // set to position control mode
    guided_mode = Guided_WP;

    // initialise waypoint and spline controller
    wp_nav.wp_and_spline_init();

    // initialise wpnav to stopping point at current altitude
    // To-Do: set to current location if disarmed?
    // To-Do: set to stopping point altitude?
    Vector3f stopping_point;
    stopping_point.z = inertial_nav.get_altitude();
    wp_nav.get_wp_stopping_point_xy(stopping_point);
    wp_nav.set_wp_destination(stopping_point);

    //BEV always look at the desired wp
    set_auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP);
}

// guided_set_destination - sets guided mode's target destination
static void guided_set_destination(const Vector3f& destination)
{
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

    wp_nav.set_wp_destination(destination);

    //BEV set the plane's destination too. This function can be called when 100% plane
    prev_WP_loc = current_loc;
    next_WP_loc = pv_vector_to_location(destination);
    alt_hold_gs_des_alt = destination.z;
}

#if NAV_GUIDED == ENABLED
// guided_set_velocity - sets guided mode's target velocity
static void guided_set_velocity(const Vector3f& velocity)
{
    // check we are in velocity control mode
    if (guided_mode != Guided_Velocity) {
        guided_vel_control_start();
    }

    // set position controller velocity target
    pos_control.set_desired_velocity(velocity);
}
#endif

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

    // call the correct auto controller
    switch (guided_mode) {

    case Guided_TakeOff:
        // run takeoff controller
        guided_takeoff_run();
        break;

    case Guided_WP:
        // run position controller
        guided_pos_control_run();
        break;

#if NAV_GUIDED == ENABLED
    case Guided_Velocity:
        // run velocity controller
        guided_vel_control_run();
        break;
#endif
    }
 }

// guided_takeoff_run - takeoff in guided mode
//      called by guided_run at 100hz or more
static void guided_takeoff_run()
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

// guided_pos_control_run - runs the guided position controller
// called from guided_run
static void guided_pos_control_run()
{
    int16_t target_roll, target_pitch;

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

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();
}
