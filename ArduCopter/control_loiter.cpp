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

// loiter_init - initialise loiter controller
static bool loiter_init(bool ignore_checks)
{
    if (GPS_ok() || ignore_checks) {

        // set target to current position
        wp_nav.init_loiter_target();

        // initialize vertical speed and accelerationj
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        // initialise altitude target to stopping point
        pos_control.set_target_to_stopping_point_z();
        alt_hold_gs_des_alt = pos_control.get_alt_target();

        return true;
    }else{
        return false;
    }
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
static void loiter_run()
{
    int16_t target_roll = 0, target_pitch = 0;
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed || !inertial_nav.position_ok()) {
        wp_nav.init_loiter_target();
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        pos_control.set_alt_target_to_current_alt();
        return;
    }

    // process pilot inputs
    if (roll_pitch_input_valid()) {
        // process pilot's roll and pitch input
        wp_nav.set_pilot_desired_acceleration(g.rc_1.control_in, g.rc_2.control_in);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        wp_nav.clear_pilot_desired_acceleration();
    }
    if(yaw_input_valid()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }
    if(throttle_input_valid()) {
        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }
    }

    // relax loiter target if we might be landed
    if (land_complete_maybe()) {
        wp_nav.loiter_soften_for_landing();
    }

    // when landed reset targets and output zero throttle
    if (ap.land_complete) {
        wp_nav.init_loiter_target();
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(g.rc_3.control_in), false);
        pos_control.set_alt_target_to_current_alt();
        //BEV let it auto disarm on the ground
#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (g.rc_3.control_in == 0 || !throttle_input_valid()){
            init_disarm_motors();
        }
#else
        // disarm when the landing detector says we've landed
        init_disarm_motors();
#endif
    }else{
        //pass target roll and pitch to is_transitioning. Will be set to desired values if indeed transitioning.
        if(is_transitioning_get_angles(target_roll, target_pitch)) {
            target_yaw_rate = 0.0f;
            // call attitude controller
            attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        } else {
            // run loiter controller
            wp_nav.update_loiter();
            // call attitude controller
            attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
        }

        //BEV only override the current desired altitude if the pilot is commanding a climb. Otherwise, maintain the last requested altitude
        if(fabs(target_climb_rate) > 0.05) {
            // call position controller
            pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
            pos_control.update_z_controller();
            alt_hold_gs_des_alt = pos_control.get_alt_target();
        } else {
            pos_control.set_alt_target_with_slew(alt_hold_gs_des_alt, G_Dt);
            pos_control.update_z_controller();
        }
    }
}
