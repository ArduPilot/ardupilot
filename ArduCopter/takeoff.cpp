#include "Copter.h"

Mode::_TakeOff Mode::takeoff;

bool Mode::auto_takeoff_no_nav_active = false;
float Mode::auto_takeoff_no_nav_alt_cm = 0;
float Mode::auto_takeoff_start_alt_cm = 0;
float Mode::auto_takeoff_complete_alt_cm = 0;
bool Mode::auto_takeoff_terrain_alt = false;
bool Mode::auto_takeoff_complete = false;
Vector3p Mode::auto_takeoff_complete_pos;

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
    // initialise takeoff state
    _running = true;
    take_off_start_alt = copter.pos_control->get_pos_target_z_cm();
    take_off_complete_alt  = take_off_start_alt + alt_cm;
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

    if (copter.ap.land_complete) {
        // send throttle to attitude controller with angle boost
        float throttle = constrain_float(copter.attitude_control->get_throttle_in() + copter.G_Dt / copter.g2.takeoff_throttle_slew_time, 0.0, 1.0);
        copter.attitude_control->set_throttle_out(throttle, true, 0.0);
        // tell position controller to reset alt target and reset I terms
        copter.pos_control->init_z_controller();
        if (throttle >= 0.9 || 
            (copter.pos_control->get_z_accel_cmss() >= 0.5 * copter.pos_control->get_max_accel_z_cmss()) ||
            (copter.pos_control->get_vel_desired_cms().z >= constrain_float(pilot_climb_rate_cm, copter.pos_control->get_max_speed_up_cms() * 0.1, copter.pos_control->get_max_speed_up_cms() * 0.5)) || 
            (is_positive(take_off_complete_alt - take_off_start_alt) && copter.pos_control->get_pos_target_z_cm() - take_off_start_alt > 0.5 * (take_off_complete_alt - take_off_start_alt))) {
            // throttle > 90%
            // acceleration > 50% maximum acceleration
            // velocity > 10% maximum velocity && commanded climb rate
            // velocity > 50% maximum velocity
            // altitude change greater than half complete alt from start off alt
            copter.set_land_complete(false);
        }
    } else {
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
}

// auto_takeoff_run - controls the vertical position controller during the process of taking off in auto modes
// auto_takeoff_complete set to true when target altitude is within 10% of the take off altitude and less than 50% max climb rate
void Mode::auto_takeoff_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || !copter.ap.auto_armed) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // get terrain offset
    float terr_offset = 0.0f;
    if (auto_takeoff_terrain_alt && !wp_nav->get_terrain_offset(terr_offset)) {
        // trigger terrain failsafe
        copter.failsafe_terrain_on_event();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // aircraft stays in landed state until rotor speed run up has finished
    if (motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        // motors have not completed spool up yet so relax navigation and position controllers
        pos_control->relax_velocity_controller_xy();
        pos_control->update_xy_controller();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        pos_control->update_z_controller();
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0);
        return;
    }
    
    // aircraft stays in landed state until vertical movement is detected or 90% throttle is reached
    if (copter.ap.land_complete) {
        // send throttle to attitude controller with angle boost
        float throttle = constrain_float(copter.attitude_control->get_throttle_in() + copter.G_Dt / copter.g2.takeoff_throttle_slew_time, 0.0, 1.0);
        copter.attitude_control->set_throttle_out(throttle, true, 0.0);
        // tell position controller to reset alt target and reset I terms
        copter.pos_control->init_z_controller();
        pos_control->relax_velocity_controller_xy();
        pos_control->update_xy_controller();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0);
        if (throttle >= 0.9 || 
            (copter.pos_control->get_z_accel_cmss() >= 0.5 * copter.pos_control->get_max_accel_z_cmss()) ||
            (copter.pos_control->get_vel_desired_cms().z >= 0.1 * copter.pos_control->get_max_speed_up_cms()) || 
            ( auto_takeoff_no_nav_active && (inertial_nav.get_position_z_up_cm() >= auto_takeoff_no_nav_alt_cm))) {
            // throttle > 90%
            // acceleration > 50% maximum acceleration
            // velocity > 10% maximum velocity
            // altitude change greater than half auto_takeoff_no_nav_alt_cm
            copter.set_land_complete(false);
        }
        return;
    }

    // check if we are not navigating because of low altitude
    if (auto_takeoff_no_nav_active) {
        // check if vehicle has reached no_nav_alt threshold
        if (inertial_nav.get_position_z_up_cm() >= auto_takeoff_no_nav_alt_cm) {
            auto_takeoff_no_nav_active = false;
        }
        pos_control->relax_velocity_controller_xy();
    } else {
        Vector2f vel;
        Vector2f accel;
        pos_control->input_vel_accel_xy(vel, accel);
    }
    pos_control->update_xy_controller();

    // command the aircraft to the take off altitude
    float pos_z = auto_takeoff_complete_alt_cm + terr_offset;
    float vel_z = 0.0;
    copter.pos_control->input_pos_vel_accel_z(pos_z, vel_z, 0.0);
    
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    // handle takeoff completion
    bool reached_altitude = (copter.pos_control->get_pos_target_z_cm() - auto_takeoff_start_alt_cm) >= ((auto_takeoff_complete_alt_cm + terr_offset - auto_takeoff_start_alt_cm) * 0.90);
    bool reached_climb_rate = copter.pos_control->get_vel_desired_cms().z < copter.pos_control->get_max_speed_up_cms() * 0.1;
    auto_takeoff_complete = reached_altitude && reached_climb_rate;

    // calculate completion for location in case it is needed for a smooth transition to wp_nav
    if (auto_takeoff_complete) {
        const Vector3p& complete_pos = copter.pos_control->get_pos_target_cm();
        auto_takeoff_complete_pos = Vector3p{complete_pos.x, complete_pos.y, pos_z};
    }
}

void Mode::auto_takeoff_start(float complete_alt_cm, bool terrain_alt)
{
    auto_takeoff_start_alt_cm = inertial_nav.get_position_z_up_cm();
    auto_takeoff_complete_alt_cm = complete_alt_cm;
    auto_takeoff_terrain_alt = terrain_alt;
    auto_takeoff_complete = false;
    if ((g2.wp_navalt_min > 0) && (is_disarmed_or_landed() || !motors->get_interlock())) {
        // we are not flying, climb with no navigation to current alt-above-ekf-origin + wp_navalt_min
        auto_takeoff_no_nav_alt_cm = auto_takeoff_start_alt_cm + g2.wp_navalt_min * 100;
        auto_takeoff_no_nav_active = true;
    } else {
        auto_takeoff_no_nav_active = false;
    }
}

// return takeoff final position if takeoff has completed successfully
bool Mode::auto_takeoff_get_position(Vector3p& complete_pos)
{
    // only provide location if takeoff has completed
    if (!auto_takeoff_complete) {
        return false;
    }

    complete_pos = auto_takeoff_complete_pos;
    return true;
}

bool Mode::is_taking_off() const
{
    if (!has_user_takeoff(false)) {
        return false;
    }
    return takeoff.running();
}
