#include "Copter.h"

#if MODE_ZIGZAG_ENABLED == ENABLED

/*
* Init and run calls for zigzag flight mode
*/

#define ZIGZAG_WP_RADIUS_SQUARED  9

// init - initialise zigzag controller
bool Copter::ModeZigZag::init(bool ignore_checks)
{
    if (!copter.position_ok() && !ignore_checks) {
        return false;
    }

    // initialize's loiter position and velocity on xy-axes from current pos and velocity
    loiter_nav->init_target();

    // initialise position_z and desired velocity_z
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise waypoint state
    zigzag_is_between_A_and_B = false;
    zigzag_judge_moving.is_keeping_time = false;
    stage = REQUIRE_A;     
    return true;   
}

// run - runs the zigzag controller
// should be called at 100hz or more
void Copter::ModeZigZag::run()
{
    // initialize vertical speed and acceleration's range
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        zero_throttle_and_relax_ac();
        return;
    }

    // manual control activated when point A B is not defined
    if (stage == REQUIRE_A || stage == REQUIRE_B || stage == MANUAL_REGAIN) {
        // receive pilot's inputs, do position and attitude control
        manual_control();
    } else { 
        // auto flight
        // judge if the vehicle has arrived at the current destination
        // if yes, go to the manual control stage
        // else, fly to current destination
        if (has_arr_at_dest()) {  // if the vehicle has arrived at the current destination
            stage = MANUAL_REGAIN;
            loiter_nav->init_target();
            AP_Notify::events.waypoint_complete = 1;    // play a tone
        } else {
            auto_control();
        }
    }
}

// auto_control - guide the vehicle to fly to current destination
void Copter::ModeZigZag::auto_control()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller to update xy
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);        
}

// manual_control - process manual control
void Copter::ModeZigZag::manual_control()
{
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        // make sure the climb rate is in the given range, prevent floating point errors
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we
        // do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    // run loiter controller
    loiter_nav->update(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(),
            loiter_nav->get_pitch(), target_yaw_rate);

    // adjust climb rate using rangefinder
    target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);

    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    // adjusts target up or down using a climb rate

    pos_control->update_z_controller();
}

// has_arr_at_next_dest - judge if the vehicle is within a small area around the current destination
bool Copter::ModeZigZag::has_arr_at_dest()
{
    if (!zigzag_judge_moving.is_keeping_time) {
        zigzag_judge_moving.is_keeping_time = true;
        zigzag_judge_moving.last_judge_pos_time = AP_HAL::millis();
        zigzag_judge_moving.last_pos = inertial_nav.get_position();
        return false;
    }
    if ((AP_HAL::millis() - zigzag_judge_moving.last_judge_pos_time) < 1000) {
        return false;
    }
    Vector3f cur_pos = inertial_nav.get_position();
    const float dist_x = cur_pos.x - zigzag_judge_moving.last_pos.x;
    const float dist_y = cur_pos.y - zigzag_judge_moving.last_pos.y;
    if ((sq(dist_x) + sq(dist_y)) < ZIGZAG_WP_RADIUS_SQUARED) {
        return true;
    }
    zigzag_judge_moving.last_judge_pos_time = AP_HAL::millis();
    zigzag_judge_moving.last_pos = inertial_nav.get_position();
    return false;
}

// calculate_next_dest - calculate next destination according to vector A-B and current position
bool Copter::ModeZigZag::calculate_next_dest(Vector3f& next_dest, RC_Channel::aux_switch_pos_t next_A_or_B) const
{
    // calculate difference between A and B - vector AB and its direction
    Vector2f pos_diff = zigzag_waypoint.B_pos - zigzag_waypoint.A_pos;
    // get current position
    Vector3f cur_pos = inertial_nav.get_position();
    if (!zigzag_is_between_A_and_B) {
        // if the drone's position is on the side of A or B
        if (next_A_or_B != zigzag_waypoint.switch_pos_B && next_A_or_B != zigzag_waypoint.switch_pos_A) {
            return false; // if next_dest not initialised, return false
        }
        if (next_A_or_B == zigzag_waypoint.switch_pos_B) {
            next_dest.x = cur_pos.x + pos_diff.x;
            next_dest.y = cur_pos.y + pos_diff.y;
            next_dest.z = cur_pos.z;
            return true;
        } 
        // can only be the case when (next_A_or_B == zigzag_waypoint.switch_pos_A)
        next_dest.x = cur_pos.x - pos_diff.x;
        next_dest.y = cur_pos.y - pos_diff.y;
        next_dest.z = cur_pos.z;
        return true;
    }
    // used to check if the drone is outside A-B scale
    int8_t next_dir = 1;
    // if the drone's position is between A and B
    const Vector2f cur_pos_2d{cur_pos.x, cur_pos.y};
    const Vector2f AB = zigzag_waypoint.B_pos - zigzag_waypoint.A_pos;
    const Vector2f P_on_AB = Vector2f::closest_point(cur_pos_2d, zigzag_waypoint.A_pos, zigzag_waypoint.B_pos);
    float dist_AB = AB.length();
    float dist_from_AB_squared = (P_on_AB - cur_pos_2d).length_squared();
    next_dest.z = cur_pos.z;
    if (is_zero(dist_AB)) {   // protection against division by zero
        return false;
    }
    if (next_A_or_B != zigzag_waypoint.switch_pos_B && next_A_or_B != zigzag_waypoint.switch_pos_A) {
        return false; // if next_dest not initialised, return false
    }
    if (next_A_or_B == zigzag_waypoint.switch_pos_B) {
        // calculate next B
        Vector2f pos_diff_BC = cur_pos_2d - zigzag_waypoint.B_pos;
        if ((pos_diff_BC.x*pos_diff.x + pos_diff_BC.y*pos_diff.y) > 0) {
            next_dir = -1;
        }
        float dist_CB_squared = (cur_pos_2d - zigzag_waypoint.B_pos).length_squared();
        float dist_BE = sqrtf(dist_CB_squared - dist_from_AB_squared);
        float dist_ratio = dist_BE / dist_AB;
        next_dest.x = cur_pos.x + next_dir*dist_ratio*pos_diff.x;
        next_dest.y = cur_pos.y + next_dir*dist_ratio*pos_diff.y;
        return true;
    }
    // can only be the case when (next_A_or_B == zigzag_waypoint.switch_pos_A)
    // calculate next A
    Vector2f pos_diff_AC = cur_pos_2d - zigzag_waypoint.A_pos;
    if ((pos_diff_AC.x*pos_diff.x + pos_diff_AC.y*pos_diff.y) < 0) {
        next_dir = -1;
    }
    float dist_CA_squared = (cur_pos_2d - zigzag_waypoint.A_pos).length_squared();
    float dist_AE = sqrtf(dist_CA_squared - dist_from_AB_squared);
    float dist_ratio = dist_AE / dist_AB;
    next_dest.x = cur_pos.x - next_dir*dist_ratio*pos_diff.x;
    next_dest.y = cur_pos.y - next_dir*dist_ratio*pos_diff.y;
    return true;
}

// called by ZIGZAG case in RC_Channel.cpp
// used to record point A, B and give the signal to fly to next destination automatically
void Copter::ModeZigZag::receive_signal_from_auxsw(RC_Channel::aux_switch_pos_t aux_switch_position)
{
    // define point A and B
    if (stage == REQUIRE_A || stage == REQUIRE_B) {
        if (aux_switch_position != RC_Channel::aux_switch_pos_t::MIDDLE) {
            Vector3f cur_pos = inertial_nav.get_position();
            set_destination(cur_pos, aux_switch_position);
            return;
        }
    } else {      
        // A and B have been defined
        if (aux_switch_position != RC_Channel::aux_switch_pos_t::MIDDLE) { // switch position in HIGH or LOW
            // calculate next point A or B
            // need to judge if the drone's position is between A and B
            Vector3f next_dest;
            if (calculate_next_dest(next_dest, aux_switch_position)) {
                // initialise waypoint and spline controller
                wp_nav->wp_and_spline_init();
                set_destination(next_dest, aux_switch_position);
                // initialise yaw
                auto_yaw.set_mode_to_default(false);
                stage = AUTO;
                zigzag_is_between_A_and_B = false;
            }
        } else {      //switch in middle position, regain the control
            if (stage == AUTO) {
                stage = MANUAL_REGAIN;
                loiter_nav->init_target();
                zigzag_is_between_A_and_B = true;
            } else {
                zigzag_is_between_A_and_B = false;
            }
        }
    }
}

// set_destination - sets zigzag mode's target destination
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
bool Copter::ModeZigZag::set_destination(const Vector3f& destination, RC_Channel::aux_switch_pos_t aux_switch_position)
{

#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    Location_Class dest_loc(destination);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        copter.Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_DEST_OUTSIDE_FENCE);
        return false;
    }
#endif
    switch (stage) {
    case REQUIRE_A:
        // define point A
        zigzag_waypoint.A_pos.x = destination.x;
        zigzag_waypoint.A_pos.y = destination.y;
        zigzag_waypoint.switch_pos_A = aux_switch_position;
        stage = REQUIRE_B;     // next need to define point B
        gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: point A stored");
        copter.Log_Write_Event(DATA_ZIGZAG_STORE_A);
        return true;
    case REQUIRE_B:
        // point B will only be defined after A is defined
        // if user toggle to the switch position that were previously defined as A
        // exit the function and do nothing
        if (aux_switch_position == zigzag_waypoint.switch_pos_A) {
            return true;
        }
        // define point B
        zigzag_waypoint.B_pos.x = destination.x;
        zigzag_waypoint.B_pos.y = destination.y;
        zigzag_waypoint.switch_pos_B = aux_switch_position;
        stage = MANUAL_REGAIN;  // user is still in manual control until he/she returns the switch again to point A position
        gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: point B stored");
        copter.Log_Write_Event(DATA_ZIGZAG_STORE_B);
        return true;
    default:
        // when both A and B are defined and switch in not in middle position, set waypoint destination
        // no need to check return status because terrain data is not used
        wp_nav->set_wp_destination(destination, false);
        return true;
    }

}
#endif // MODE_ZIGZAG_ENABLED == ENABLED
