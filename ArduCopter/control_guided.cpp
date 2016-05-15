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

// initialise guided mode's position controller
void guided_pos_control_start()
{
    //bev since it's setting desired postion to local we've arrived
    guided_arrived = true;

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
    //BEV haven't arrived at destination yet
    guided_arrived = false;

    wp_nav.set_wp_destination(destination);

    //BEV set the plane's destination too. This function can be called when 100% plane
    prev_WP_loc = current_loc;
    next_WP_loc = pv_vector_to_location(destination);
    alt_hold_gs_des_alt = destination.z;

    //BEV just to be sure
    set_auto_yaw_mode(AUTO_YAW_LOOK_AT_NEXT_WP);
}

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

    // run position controller
    guided_pos_control_run();
 }

// guided_pos_control_run - runs the guided position controller
// called from guided_run
static void guided_pos_control_run()
{
    if( !guided_arrived && (wp_nav.get_wp_distance_to_destination() < 500.0f) ) {
        guided_arrived = true;
    }

    int16_t target_roll, target_pitch;

    //pass target roll and pitch to is_transitioning. Will be set to desired values if indeed transitioning.
    if(is_transitioning_get_angles(target_roll, target_pitch)) {
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, 0, get_smoothing_gain());
    } else {
        // run waypoint controller
        wp_nav.update_wpnav();
        //use get_auto_heading if navigating to target. Otherwise, use roll2yaw feed forward
        if(guided_arrived) {
            //use feed forward to set yaw rate
            attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(wp_nav.get_roll(), wp_nav.get_pitch(), get_roll2yaw_ff(), get_smoothing_gain());
        } else {
            //otherwise use get auto heading
            attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(), true);
        }
    }

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();
}
