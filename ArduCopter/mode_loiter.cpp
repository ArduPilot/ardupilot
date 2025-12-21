#include "Copter.h"

#if MODE_LOITER_ENABLED

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeLoiter::init(bool ignore_checks)
{
    float target_roll_rad, target_pitch_rad;
    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, loiter_nav->get_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

    // process pilot's roll and pitch input
    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, target_pitch_rad);

    loiter_nav->init_target();

    // initialise the vertical position controller
    if (!pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());
    pos_control->D_set_correction_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

#if AC_PRECLAND_ENABLED
    _precision_loiter_active = false;
#endif

    return true;
}

#if AC_PRECLAND_ENABLED
bool ModeLoiter::do_precision_loiter()
{
    if (!_precision_loiter_enabled) {
        return false;
    }
    if (copter.ap.land_complete_maybe) {
        return false;        // don't move on the ground
    }
    // if the pilot *really* wants to move the vehicle, let them....
    if (loiter_nav->get_pilot_desired_acceleration_NE_mss().length() > 0.5) {
        return false;
    }
    if (!copter.precland.target_acquired()) {
        return false; // we don't have a good vector
    }
    return true;
}

void ModeLoiter::precision_loiter_xy()
{
    loiter_nav->clear_pilot_desired_acceleration();
    Vector2p target_pos_ne_m;
    Vector2f target_vel_ne_ms;
    if (!copter.precland.get_target_position_m(target_pos_ne_m)) {
        target_pos_ne_m = pos_control->get_pos_estimate_NED_m().xy();
    }
    // get the velocity of the target
    copter.precland.get_target_velocity_ms(pos_control->get_vel_estimate_NED_ms().xy(), target_vel_ne_ms);

    Vector2f zero;
    // target vel will remain zero if landing target is stationary
    pos_control->input_pos_vel_accel_NE_m(target_pos_ne_m, target_vel_ne_ms, zero);
    // run pos controller
    pos_control->NE_update_controller();
}
#endif

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeLoiter::run()
{
    float target_roll_rad, target_pitch_rad;
    float target_yaw_rate_rads = 0.0f;
    float target_climb_rate_ms = 0.0f;

    // set vertical speed and acceleration limits
    pos_control->D_set_max_speed_accel_m(get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_D_mss());

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, loiter_nav->get_angle_max_rad(), attitude_control->get_althold_lean_angle_max_rad());

    // process pilot's roll and pitch input
    loiter_nav->set_pilot_desired_acceleration_rad(target_roll_rad, target_pitch_rad);

    // get pilot's desired yaw rate
    target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    // get pilot desired climb rate
    target_climb_rate_ms = get_pilot_desired_climb_rate_ms();
    target_climb_rate_ms = constrain_float(target_climb_rate_ms, -get_pilot_speed_dn_ms(), get_pilot_speed_up_ms());

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    AltHoldModeState loiter_state = get_alt_hold_state_D_ms(target_climb_rate_ms);

    // Loiter State Machine
    switch (loiter_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->D_relax_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        pos_control->D_relax_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHoldModeState::Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start_m(constrain_float(g.pilot_takeoff_alt_cm * 0.01, 0.0, 10.0));
        }

        // get avoidance adjusted climb rate
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff_ms(target_climb_rate_ms);

        // run loiter controller
        loiter_nav->update();
        break;

    case AltHoldModeState::Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_PRECLAND_ENABLED
        bool precision_loiter_old_state = _precision_loiter_active;
        if (do_precision_loiter()) {
            precision_loiter_xy();
            _precision_loiter_active = true;
        } else {
            _precision_loiter_active = false;
        }
        if (precision_loiter_old_state && !_precision_loiter_active) {
            // prec loiter was active, not any more, let's init again as user takes control
            loiter_nav->init_target();
        }
        // run loiter controller if we are not doing prec loiter
        if (!_precision_loiter_active) {
            loiter_nav->update();
        }
#else
        loiter_nav->update();
#endif


        // get avoidance adjusted climb rate
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif

        // Send the commanded climb rate to the position controller
        pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);
        break;
    }

    // call attitude controller
    attitude_control->input_thrust_vector_rate_heading_rads(loiter_nav->get_thrust_vector(), target_yaw_rate_rads, false);
    // run the vertical position controller and set output throttle
    pos_control->D_update_controller();
}

float ModeLoiter::wp_distance_m() const
{
    return loiter_nav->get_distance_to_target_m();
}

float ModeLoiter::wp_bearing_deg() const
{
    return degrees(loiter_nav->get_bearing_to_target_rad());
}

#endif
