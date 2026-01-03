#include "Copter.h"

Mode::_TakeOff Mode::takeoff;
_AutoTakeoff Mode::auto_takeoff;

// This file contains the high-level takeoff logic for Loiter, PosHold, AltHold, Sport modes.
//   The take-off can be initiated from a GCS NAV_TAKEOFF command which includes a takeoff altitude
//   A safe takeoff speed is calculated and used to calculate a time_ms
//   the pos_control target is then slowly increased until time_ms expires

bool Mode::do_user_takeoff_start_m(float takeoff_alt_m)
{
    copter.flightmode->takeoff.start_m(takeoff_alt_m);
    return true;
}

// initiate user takeoff - called when MAVLink TAKEOFF command is received
bool Mode::do_user_takeoff_U_m(float takeoff_alt_m, bool must_navigate)
{
    if (!copter.motors->armed()) {
        return false;
    }
    if (!copter.ap.land_complete) {
        // can't takeoff again!
        return false;
    }
    if (!has_user_takeoff(must_navigate)) {
        // this mode doesn't support user takeoff
        return false;
    }
    if (takeoff_alt_m <= copter.current_loc.alt * 0.01) {
        // can't takeoff downwards...
        return false;
    }

    // Vehicles using motor interlock should return false if motor interlock is disabled.
    // Interlock must be enabled to allow the controller to spool up the motor(s) for takeoff.
    if (!motors->get_interlock() && copter.ap.using_interlock) {
        return false;
    }

    if (!do_user_takeoff_start_m(takeoff_alt_m)) {
        return false;
    }

    copter.set_auto_armed(true);
    return true;
}

// start takeoff to specified altitude above home in meters
void Mode::_TakeOff::start_m(float alt_m)
{
    // initialise takeoff state
    _running = true;
    take_off_start_alt_m = copter.pos_control->get_pos_estimate_U_m();
    take_off_complete_alt_m  = take_off_start_alt_m + alt_m;
}

// stop takeoff
void Mode::_TakeOff::stop()
{
    _running = false;
    // Check if we have progressed far enough through the takeoff process that the
    // aircraft may have left the ground but not yet detected the climb.
    if (copter.attitude_control->get_throttle_in() > copter.get_non_takeoff_throttle()) {
        copter.set_land_complete(false);
    }
}

// do_pilot_takeoff_ms - controls the vertical position controller during the process of taking off
//  take off is complete when the vertical target reaches the take off altitude.
//  climb is cancelled if pilot_climb_rate_ms becomes negative
//  sets take off to complete when target altitude is within 1% of the take off altitude
void Mode::_TakeOff::do_pilot_takeoff_ms(float pilot_climb_rate_ms)
{
    // return pilot_climb_rate if take-off inactive
    if (!_running) {
        return;
    }

    if (copter.ap.land_complete) {
        // send throttle to attitude controller with angle boost
        float throttle_norm = constrain_float(copter.attitude_control->get_throttle_in() + copter.G_Dt / copter.g2.takeoff_throttle_slew_time, 0.0, 1.0);
        copter.attitude_control->set_throttle_out(throttle_norm, true, 0.0);
        // tell position controller to reset alt target and reset I terms
        copter.pos_control->D_init_controller();
        if (throttle_norm >= MIN(copter.g2.takeoff_throttle_max, 0.9) || 
            (copter.pos_control->get_estimated_accel_U_mss() >= 0.5 * copter.pos_control->D_get_max_accel_mss()) ||
            (copter.pos_control->get_vel_estimate_U_ms() >= constrain_float(pilot_climb_rate_ms, copter.pos_control->get_max_speed_up_ms() * 0.1, copter.pos_control->get_max_speed_up_ms() * 0.5)) || 
            (is_positive(take_off_complete_alt_m - take_off_start_alt_m) && copter.pos_control->get_pos_estimate_U_m() - take_off_start_alt_m > 0.5 * (take_off_complete_alt_m - take_off_start_alt_m))) {
            // throttle > 90%
            // acceleration > 50% maximum acceleration
            // velocity > 10% maximum velocity && commanded climb rate
            // velocity > 50% maximum velocity
            // altitude change greater than half complete alt from start off alt
            copter.set_land_complete(false);
        }
    } else {
        float pos_d_m = -take_off_complete_alt_m;
        float vel_d_ms = -pilot_climb_rate_ms;

        // command the aircraft to the take off altitude and current pilot climb rate
        copter.pos_control->input_pos_vel_accel_D_m(pos_d_m, vel_d_ms, 0.0);

        // stop take off early and return if negative climb rate is commanded or we are within 0.1% of our take off altitude
        if (is_negative(pilot_climb_rate_ms) ||
            (take_off_complete_alt_m  - take_off_start_alt_m) * 0.999f < copter.pos_control->get_pos_desired_U_m() - take_off_start_alt_m) {
            stop();
        }
    }
}

// auto_takeoff_run - controls the vertical position controller during the process of taking off in auto modes
// auto_takeoff_complete set to true when target altitude is within 10% of the take off altitude and less than 50% max climb rate
void _AutoTakeoff::run()
{
    const auto &g2 = copter.g2;
    const auto &wp_nav = copter.wp_nav;
    auto *motors = copter.motors;
    auto *pos_control = copter.pos_control;
    auto *attitude_control = copter.attitude_control;

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || !copter.ap.auto_armed) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        copter.flightmode->make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        // update auto_takeoff_no_nav_alt_m
        no_nav_alt_m = pos_control->get_pos_estimate_U_m() + g2.wp_navalt_min_m;
        return;
    }

    // get terrain offset
    float terrain_u_m = 0.0f;
    if (is_terrain_alt && !wp_nav->get_terrain_U_m(terrain_u_m)) {
        // trigger terrain failsafe
        copter.failsafe_terrain_on_event();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // aircraft stays in landed state until rotor speed run up has finished
    if (motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        // motors have not completed spool up yet so relax navigation and position controllers
        pos_control->NE_relax_velocity_controller();
        pos_control->NE_update_controller();
        pos_control->D_relax_controller(0.0f);   // forces throttle output to decay to zero
        pos_control->D_update_controller();
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(), 0.0);
        // update auto_takeoff_no_nav_alt_m
        no_nav_alt_m = pos_control->get_pos_estimate_U_m() + g2.wp_navalt_min_m;
        return;
    }
    
    // aircraft stays in landed state until vertical movement is detected or 90% throttle is reached
    if (copter.ap.land_complete) {
        // send throttle to attitude controller with angle boost
        float throttle = constrain_float(copter.attitude_control->get_throttle_in() + copter.G_Dt / copter.g2.takeoff_throttle_slew_time, 0.0, 1.0);
        copter.attitude_control->set_throttle_out(throttle, true, 0.0);
        // tell position controller to reset alt target and reset I terms
        copter.pos_control->D_init_controller();
        pos_control->NE_relax_velocity_controller();
        pos_control->NE_update_controller();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(), 0.0);
        if (throttle >= MIN(copter.g2.takeoff_throttle_max, 0.9) || 
            (copter.pos_control->get_estimated_accel_U_mss() >= 0.5 * copter.pos_control->D_get_max_accel_mss()) ||
            (copter.pos_control->get_vel_estimate_U_ms() >= 0.1 * copter.pos_control->get_max_speed_up_ms()) || 
            ( no_nav_active && (pos_control->get_pos_estimate_U_m() >= no_nav_alt_m))) {
            // throttle > 90%
            // acceleration > 50% maximum acceleration
            // velocity > 10% maximum velocity
            // altitude change greater than half auto_takeoff_no_nav_alt_cm
            copter.set_land_complete(false);
        }
        return;
    }

    // check if we are not navigating because of low altitude
    if (no_nav_active) {
        // check if vehicle has reached no_nav_alt threshold
        if (pos_control->get_pos_estimate_U_m() >= no_nav_alt_m) {
            no_nav_active = false;
        }
        pos_control->NE_relax_velocity_controller();
    } else {
        Vector2f vel_zero;
        Vector2f accel_zero;
        pos_control->input_vel_accel_NE_m(vel_zero, accel_zero);
    }
    pos_control->NE_update_controller();

    // command the aircraft to the take off altitude
    float pos_d_m = -(complete_alt_m + terrain_u_m);
    float vel_zero = 0.0;
    copter.pos_control->input_pos_vel_accel_D_m(pos_d_m, vel_zero, 0.0);
    
    // run the vertical position controller and set output throttle
    pos_control->D_update_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), copter.flightmode->auto_yaw.get_heading());

    // takeoff complete when we are less than 1% of the stopping distance from the target altitude
    // and 10% our maximum climb rate
    const float vel_threshold_fraction = 0.1;
    // stopping distance from vel_threshold_fraction * max velocity
    const float stop_distance_m = 0.5 * sq(vel_threshold_fraction * copter.pos_control->get_max_speed_up_ms()) / copter.pos_control->D_get_max_accel_mss();
    bool reached_altitude = copter.pos_control->get_pos_desired_U_m() >= -pos_d_m - stop_distance_m;
    bool reached_climb_rate = copter.pos_control->get_vel_desired_U_ms() < copter.pos_control->get_max_speed_up_ms() * vel_threshold_fraction;
    complete = reached_altitude && reached_climb_rate;

    // calculate completion for location in case it is needed for a smooth transition to wp_nav
    if (complete) {
        const Vector3p& _complete_pos_ned_m = copter.pos_control->get_pos_desired_NED_m();
        complete_pos_ned_m = Vector3p{_complete_pos_ned_m.x, _complete_pos_ned_m.y, pos_d_m};
    }
}

void _AutoTakeoff::start_m(float _complete_alt_m, bool _is_terrain_alt)
{
    // auto_takeoff_complete_alt_m is a problem if equal to auto_takeoff_start_alt_m
    complete_alt_m = _complete_alt_m;
    is_terrain_alt = _is_terrain_alt;
    complete = false;
    // initialise auto_takeoff_no_nav_alt_m
    const auto &g2 = copter.g2;
    no_nav_alt_m = copter.pos_control->get_pos_estimate_U_m() + g2.wp_navalt_min_m;
    if ((g2.wp_navalt_min_m > 0) && (copter.flightmode->is_disarmed_or_landed() || !copter.motors->get_interlock())) {
        // we are not flying, climb with no navigation to current alt-above-ekf-origin + wp_navalt_min_m
        no_nav_active = true;
    } else {
        no_nav_active = false;
    }
}

// return takeoff final target position in m from the EKF origin if takeoff has completed successfully
bool _AutoTakeoff::get_completion_pos_ned_m(Vector3p& pos_ned_m)
{
    // only provide location if takeoff has completed
    if (!complete) {
        return false;
    }

    pos_ned_m = complete_pos_ned_m;
    return true;
}

bool Mode::is_taking_off() const
{
    if (!has_user_takeoff(false)) {
        return false;
    }
    return takeoff.running();
}
