#include "Copter.h"

/*
 * Init and run calls for brake flight mode
 */

// brake_init - initialise brake controller
bool Copter::ModeBrake::init(bool ignore_checks)
{
    if (_copter.position_ok() || ignore_checks) {

        // set desired acceleration to zero
        wp_nav->clear_pilot_desired_acceleration();

        // set target to current position
        wp_nav->init_brake_target(BRAKE_MODE_DECEL_RATE);

        // initialize vertical speed and acceleration
        pos_control->set_speed_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z);
        pos_control->set_accel_z(BRAKE_MODE_DECEL_RATE);

        // initialise position and desired velocity
        if (!pos_control->is_active_z()) {
            pos_control->set_alt_target_to_current_alt();
            pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
        }

        _timeout_ms = 0;

        return true;
    }else{
        return false;
    }
}

// brake_run - runs the brake controller
// should be called at 100hz or more
void Copter::ModeBrake::run()
{
    // initialize smoothing gain
    attitude_control->set_smoothing_gain(get_smoothing_gain());

    // if not auto armed set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
        wp_nav->init_brake_target(BRAKE_MODE_DECEL_RATE);
        zero_throttle_and_relax_ac();
        pos_control->relax_alt_hold_controllers(0.0f);
        return;
    }

    // relax stop target if we might be landed
    if (ap.land_complete_maybe) {
        wp_nav->loiter_soften_for_landing();
    }

    // if landed immediately disarm
    if (ap.land_complete) {
        _copter.init_disarm_motors();
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run brake controller
    wp_nav->update_brake(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), 0.0f);

    // body-frame rate controller is run directly from 100hz loop

    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate_ff(0.0f, G_Dt, false);
    pos_control->update_z_controller();

    if (_timeout_ms != 0 && millis()-_timeout_start >= _timeout_ms) {
        if (!_copter.set_mode(LOITER, MODE_REASON_BRAKE_TIMEOUT)) {
            _copter.set_mode(ALT_HOLD, MODE_REASON_BRAKE_TIMEOUT);
        }
    }
}

void Copter::ModeBrake::timeout_to_loiter_ms(uint32_t timeout_ms)
{
    _timeout_start = millis();
    _timeout_ms = timeout_ms;
}
