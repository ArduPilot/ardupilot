// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

// This file contains the high-level takeoff logic for Loiter, PosHold, AltHold, Sport modes.
//   The take-off can be initiated from a GCS NAV_TAKEOFF command which includes a takeoff altitude
//   A safe takeoff speed is calculated and used to calculate a time_ms
//   the pos_control target is then slowly increased until time_ms expires

// return true if this flight mode supports user takeoff
//  must_nagivate is true if mode must also control horizontal position
bool Copter::current_mode_has_user_takeoff(bool must_navigate)
{
    switch (control_mode) {
        case GUIDED:
        case LOITER:
        case POSHOLD:
            return true;
        case ALT_HOLD:
        case SPORT:
            return !must_navigate;
        default:
            return false;
    }
}

// initiate user takeoff - called when MAVLink TAKEOFF command is received
bool Copter::do_user_takeoff(float takeoff_alt_cm, bool must_navigate)
{
    if (motors.armed() && ap.land_complete && current_mode_has_user_takeoff(must_navigate) && takeoff_alt_cm > current_loc.alt) {

#if FRAME_CONFIG == HELI_FRAME
        // Helicopters should return false if MAVlink takeoff command is received while the rotor is not spinning
        if (!motors.rotor_runup_complete()) {
            return false;
        }
#endif

        switch(control_mode) {
            case GUIDED:
                set_auto_armed(true);
                guided_takeoff_start(takeoff_alt_cm);
                return true;
            case LOITER:
            case POSHOLD:
            case ALT_HOLD:
            case SPORT:
                set_auto_armed(true);
                takeoff_timer_start(takeoff_alt_cm);
                return true;
        }
    }
    return false;
}

// start takeoff to specified altitude above home in centimeters
void Copter::takeoff_timer_start(float alt_cm)
{
    // calculate climb rate
    float speed = min(wp_nav.get_speed_up(), max(g.pilot_velocity_z_max*2.0f/3.0f, g.pilot_velocity_z_max-50.0f));

    // sanity check speed and target
    if (takeoff_state.running || speed <= 0.0f || alt_cm <= 0.0f) {
        return;
    }

    // initialise takeoff state
    takeoff_state.running = true;
    takeoff_state.speed = speed;
    takeoff_state.start_ms = millis();
    takeoff_state.time_ms = (alt_cm/takeoff_state.speed) * 1.0e3f;
}

// stop takeoff
void Copter::takeoff_stop()
{
    takeoff_state.running = false;
    takeoff_state.start_ms = 0;
}

// returns pilot and takeoff climb rates
//  pilot_climb_rate is both an input and an output
//  takeoff_climb_rate is only an output
//  has side-effect of turning takeoff off when timeout as expired
void Copter::takeoff_get_climb_rates(float& pilot_climb_rate, float& takeoff_climb_rate)
{
    // return pilot_climb_rate if take-off inactive
    if (!takeoff_state.running) {
        takeoff_climb_rate = 0.0f;
        return;
    }

    // check if timeout as expired
    if ((millis()-takeoff_state.start_ms) >= takeoff_state.time_ms) {
        takeoff_stop();
        takeoff_climb_rate = 0.0f;
        return;
    }

    // if takeoff climb rate is zero return
    if (takeoff_state.speed <= 0.0f) {
        takeoff_climb_rate = 0.0f;
        return;
    }

    // default take-off climb rate to maximum speed
    takeoff_climb_rate = takeoff_state.speed;

    // if pilot's commands descent
    if (pilot_climb_rate < 0.0f) {
        // if overall climb rate is still positive, move to take-off climb rate
        if (takeoff_climb_rate + pilot_climb_rate > 0.0f) {
            takeoff_climb_rate = takeoff_climb_rate + pilot_climb_rate;
            pilot_climb_rate = 0;
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
