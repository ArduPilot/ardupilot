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

    // Helicopters should return false if MAVlink takeoff command is received while the rotor is not spinning
    if (motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED && copter.ap.using_interlock) {
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

    // calculate climb rate
    const float speed = MIN(copter.wp_nav->get_default_speed_up(), MAX(copter.g.pilot_speed_up*2.0f/3.0f, copter.g.pilot_speed_up-50.0f));

    // sanity check speed and target
    if (speed <= 0.0f || alt_cm <= 0.0f) {
        return;
    }

    // initialise takeoff state
    _running = true;
    max_speed = speed;
    start_ms = millis();
    alt_delta = alt_cm;
}

// stop takeoff
void Mode::_TakeOff::stop()
{
    _running = false;
    start_ms = 0;
}

// returns pilot and takeoff climb rates
//  pilot_climb_rate is both an input and an output
//  takeoff_climb_rate is only an output
//  has side-effect of turning takeoff off when timeout as expired
void Mode::_TakeOff::get_climb_rates(float& pilot_climb_rate,
                                                  float& takeoff_climb_rate)
{
    // return pilot_climb_rate if take-off inactive
    if (!_running) {
        takeoff_climb_rate = 0.0f;
        return;
    }

    // acceleration of 50cm/s/s
    static constexpr float TAKEOFF_ACCEL = 50.0f;
    const float takeoff_minspeed = MIN(50.0f, max_speed);
    const float time_elapsed = (millis() - start_ms) * 1.0e-3f;
    const float speed = MIN(time_elapsed * TAKEOFF_ACCEL + takeoff_minspeed, max_speed);

    const float time_to_max_speed = (max_speed - takeoff_minspeed) / TAKEOFF_ACCEL;
    float height_gained;
    if (time_elapsed <= time_to_max_speed) {
        height_gained = 0.5f * TAKEOFF_ACCEL * sq(time_elapsed) + takeoff_minspeed * time_elapsed;
    } else {
        height_gained = 0.5f * TAKEOFF_ACCEL * sq(time_to_max_speed) + takeoff_minspeed * time_to_max_speed +
                        (time_elapsed - time_to_max_speed) * max_speed;
    }

    // check if the takeoff is over
    if (height_gained >= alt_delta) {
        stop();
    }

    // if takeoff climb rate is zero return
    if (speed <= 0.0f) {
        takeoff_climb_rate = 0.0f;
        return;
    }

    // default take-off climb rate to maximum speed
    takeoff_climb_rate = speed;

    // if pilot's commands descent
    if (pilot_climb_rate < 0.0f) {
        // if overall climb rate is still positive, move to take-off climb rate
        if (takeoff_climb_rate + pilot_climb_rate > 0.0f) {
            takeoff_climb_rate = takeoff_climb_rate + pilot_climb_rate;
            pilot_climb_rate = 0.0f;
        } else {
            // if overall is negative, move to pilot climb rate
            pilot_climb_rate = pilot_climb_rate + takeoff_climb_rate;
            takeoff_climb_rate = 0.0f;
        }
    } else { // pilot commands climb
        // pilot climb rate is zero until it surpasses the take-off climb rate
        if (pilot_climb_rate > takeoff_climb_rate) {
            pilot_climb_rate = pilot_climb_rate - takeoff_climb_rate;
        } else {
            pilot_climb_rate = 0.0f;
        }
    }
}

void Mode::auto_takeoff_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || !copter.ap.auto_armed) {
        make_safe_spool_down();
        wp_nav->shift_wp_origin_to_current_pos();
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
        wp_nav->shift_wp_origin_to_current_pos();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
        return;
    }

    // check if we are not navigating because of low altitude
    float nav_roll = 0.0f, nav_pitch = 0.0f;
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
        pos_control->set_limit_accel_xy();
    }

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    if (!auto_takeoff_no_nav_active) {
        nav_roll = wp_nav->get_roll();
        nav_pitch = wp_nav->get_pitch();
    }

    // call z-axis position controller (wpnav should have already updated it's alt target)
    copter.pos_control->update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(nav_roll, nav_pitch, target_yaw_rate);
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

//  called when takeoff is complete
void Mode::autoenable_floor_fence(void)
{
#if AC_FENCE == ENABLED
    switch(copter.fence.auto_enabled()) {
        case AC_Fence::AutoEnable::ALWAYS_ENABLED:
        case AC_Fence::AutoEnable::ENABLE_DISABLE_FLOOR_ONLY:
            copter.fence.enable(true);
            break;
        default:
            // fence does not auto-enable in other takeoff conditions
            break;
    }
#endif
}
