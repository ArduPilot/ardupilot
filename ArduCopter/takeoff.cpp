#include "Copter.h"

Mode::_TakeOff Mode::takeoff;

bool Mode::auto_takeoff_no_nav_active = false;
float Mode::auto_takeoff_no_nav_alt_cm = 0;

// This file contains the high-level takeoff logic for Loiter, PosHold, AltHold, Sport modes.
//   The take-off can be initiated from a GCS NAV_TAKEOFF command which includes a takeoff altitude
//   A safe takeoff speed is calculated and used to calculate a time_ms
//   the pos_control target is then slowly increased until time_ms expires

bool Mode::do_user_takeoff_start(float takeoff_alt_cm)
{
    copter.flightmode->takeoff.start(takeoff_alt_cm);
    return true;
}

// initiate user takeoff - called when MAVLink TAKEOFF command is received
bool Mode::do_user_takeoff(float takeoff_alt_cm, bool must_navigate)
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
    if (takeoff_alt_cm <= copter.current_loc.alt) {
        // can't takeoff downwards...
        return false;
    }

    // Vehicles using motor interlock should return false if motor interlock is disabled.
    // Interlock must be enabled to allow the controller to spool up the motor(s) for takeoff.
    if (!motors->get_interlock() && copter.ap.using_interlock) {
        return false;
    }

    if (!do_user_takeoff_start(takeoff_alt_cm)) {
        return false;
    }

    copter.set_auto_armed(true);
    return true;
}

// start takeoff to specified altitude above home in centimeters
void Mode::_TakeOff::start(float alt_cm)
{
    // indicate we are taking off
    copter.set_land_complete(false);
    // tell position controller to reset alt target and reset I terms
    copter.flightmode->set_throttle_takeoff();

    // initialise takeoff state
    _running = true;
    take_off_start_alt = copter.pos_control->get_pos_target_z_cm();
    take_off_complete_alt  = take_off_start_alt + alt_cm;
}

// stop takeoff
void Mode::_TakeOff::stop()
{
    _running = false;
}

// do_pilot_takeoff - controls the vertical position controller during the process of taking off
//  take off is complete when the vertical target reaches the take off altitude.
//  climb is cancelled if pilot_climb_rate_cm becomes negative
//  sets take off to complete when target altitude is within 1% of the take off altitude
void Mode::_TakeOff::do_pilot_takeoff(float& pilot_climb_rate_cm)
{
    // return pilot_climb_rate if take-off inactive
    if (!_running) {
        return;
    }

    float pos_z = take_off_complete_alt;
    float vel_z = pilot_climb_rate_cm;

    // command the aircraft to the take off altitude and current pilot climb rate
    copter.pos_control->input_pos_vel_accel_z(pos_z, vel_z, 0);

    // stop take off early and return if negative climb rate is commanded or we are within 0.1% of our take off altitude
    if (is_negative(pilot_climb_rate_cm) ||
        (take_off_complete_alt  - take_off_start_alt) * 0.999f < copter.pos_control->get_pos_target_z_cm() - take_off_start_alt) {
        stop();
    }
}

void Mode::auto_takeoff_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || !copter.ap.auto_armed) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        wp_nav->shift_wp_origin_and_destination_to_current_pos_xy();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // aircraft stays in landed state until rotor speed runup has finished
    if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        set_land_complete(false);
    } else {
        // motors have not completed spool up yet so relax navigation and position controllers
        wp_nav->shift_wp_origin_and_destination_to_current_pos_xy();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        pos_control->update_z_controller();
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
        return;
    }

    // check if we are not navigating because of low altitude
    if (auto_takeoff_no_nav_active) {
        // check if vehicle has reached no_nav_alt threshold
        if (inertial_nav.get_altitude() >= auto_takeoff_no_nav_alt_cm) {
            auto_takeoff_no_nav_active = false;
            wp_nav->shift_wp_origin_and_destination_to_stopping_point_xy();
        } else {
            // shift the navigation target horizontally to our current position
            wp_nav->shift_wp_origin_and_destination_to_current_pos_xy();
        }
        // tell the position controller that we have limited roll/pitch demand to prevent integrator buildup
        pos_control->set_externally_limited_xy();
    }

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    Vector3f thrustvector{0, 0, -GRAVITY_MSS * 100.0f};
    if (!auto_takeoff_no_nav_active) {
        thrustvector = wp_nav->get_thrust_vector();
    }

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    copter.pos_control->update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_thrust_vector_rate_heading(thrustvector, target_yaw_rate);
}

void Mode::auto_takeoff_set_start_alt(void)
{
    if ((g2.wp_navalt_min > 0) && (is_disarmed_or_landed() || !motors->get_interlock())) {
        // we are not flying, climb with no navigation to current alt-above-ekf-origin + wp_navalt_min
        auto_takeoff_no_nav_alt_cm = inertial_nav.get_altitude() + g2.wp_navalt_min * 100;
        auto_takeoff_no_nav_active = true;
    } else {
        auto_takeoff_no_nav_active = false;
    }
}

bool Mode::is_taking_off() const
{
    if (!has_user_takeoff(false)) {
        return false;
    }
    if (copter.ap.land_complete) {
        return false;
    }
    return takeoff.running();
}
