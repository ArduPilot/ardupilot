#include "Copter.h"

Copter::Mode::_TakeOff Copter::Mode::takeoff;

// This file contains the high-level takeoff logic for Loiter, PosHold, AltHold, Sport modes.
//   The take-off can be initiated from a GCS NAV_TAKEOFF command which includes a takeoff altitude
//   A safe takeoff speed is calculated and used to calculate a time_ms
//   the pos_control target is then slowly increased until time_ms expires

bool Copter::Mode::do_user_takeoff_start(float takeoff_alt_cm)
{
    copter.flightmode->takeoff.start(takeoff_alt_cm);
    return true;
}

// initiate user takeoff - called when MAVLink TAKEOFF command is received
bool Copter::Mode::do_user_takeoff(float takeoff_alt_cm, bool must_navigate)
{
    if (!copter.motors->armed()) {
        return false;
    }
    if (!ap.land_complete) {
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

#if FRAME_CONFIG == HELI_FRAME
    // Helicopters should return false if MAVlink takeoff command is received while the rotor is not spinning
    if (!copter.motors->rotor_runup_complete()) {
        return false;
    }
#endif

    if (!do_user_takeoff_start(takeoff_alt_cm)) {
        return false;
    }

    copter.set_auto_armed(true);
    return true;
}

// start takeoff to specified altitude above home in centimeters
void Copter::Mode::_TakeOff::start(float alt_cm)
{
    // calculate climb rate
    const float speed = MIN(copter.wp_nav->get_default_speed_up(), MAX(copter.g.pilot_speed_up*2.0f/3.0f, copter.g.pilot_speed_up-50.0f));

    // sanity check speed and target
    if (running() || speed <= 0.0f || alt_cm <= 0.0f) {
        return;
    }

    // initialise takeoff state
    _running = true;
    max_speed = speed;
    start_ms = millis();
    alt_delta = alt_cm;
}

// stop takeoff
void Copter::Mode::_TakeOff::stop()
{
    _running = false;
    start_ms = 0;
}

// returns pilot and takeoff climb rates
//  pilot_climb_rate is both an input and an output
//  takeoff_climb_rate is only an output
//  has side-effect of turning takeoff off when timeout as expired
void Copter::Mode::_TakeOff::get_climb_rates(float& pilot_climb_rate,
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

void Copter::Mode::auto_takeoff_set_start_alt(void)
{
    // start with our current altitude
    auto_takeoff_no_nav_alt_cm = inertial_nav.get_altitude();
    
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        // we are not flying, add the wp_navalt_min
        auto_takeoff_no_nav_alt_cm += g2.wp_navalt_min * 100;
    }
}


/*
  call attitude controller for automatic takeoff, limiting roll/pitch
  if below wp_navalt_min
 */
void Copter::Mode::auto_takeoff_attitude_run(float target_yaw_rate)
{
    float nav_roll, nav_pitch;
    
    if (g2.wp_navalt_min > 0 && inertial_nav.get_altitude() < auto_takeoff_no_nav_alt_cm) {
        // we haven't reached the takeoff navigation altitude yet
        nav_roll = 0;
        nav_pitch = 0;
#if FRAME_CONFIG == HELI_FRAME
        // prevent hover roll starting till past specified altitude
        copter.hover_roll_trim_scalar_slew = 0;        
#endif
        // tell the position controller that we have limited roll/pitch demand to prevent integrator buildup
        pos_control->set_limit_accel_xy();
    } else {
        nav_roll = wp_nav->get_roll();
        nav_pitch = wp_nav->get_pitch();
    }
    
    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(nav_roll, nav_pitch, target_yaw_rate);
}
