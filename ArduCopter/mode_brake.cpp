#include "Copter.h"

#if MODE_BRAKE_ENABLED == ENABLED

/*
 * Init and run calls for brake flight mode
 */

// init_brake_target - initializes stop position from current position and velocity
    void Copter::ModeBrake::init_brake_target(float accel_cmss)
{
    const Vector3f& curr_vel = inertial_nav.get_velocity();
    Vector3f stopping_point;

    // initialise position controller
    pos_control->set_desired_velocity_xy(0.0f,0.0f);
    pos_control->set_desired_accel_xy(0.0f,0.0f);
    pos_control->init_xy_controller();
    // initialis-> pos controller speed->and acceleration
    pos_control->set_max_speed_xy(curr_vel.length());
    pos_control->set_max_accel_xy(accel_cmss);
    pos_control->calc_leash_length_xy();

    // set target position
    pos_control->get_stopping_point_xy(stopping_point);
    pos_control->set_xy_target(stopping_point.x, stopping_point.y);
}


// update_brake - run the stop controller - gets called at 400hz
void Copter::ModeBrake::update_brake()
{
    // send adjusted feed forward velocity back to position controller
    pos_control->set_desired_velocity_xy(0.0f, 0.0f);
    pos_control->update_xy_controller();
}


// brake_init - initialise brake controller
bool Copter::ModeBrake::init(bool ignore_checks)
{
    // set target to current position
    init_brake_target(BRAKE_MODE_DECEL_RATE);

    // initialize vertical speed and acceleration
    pos_control->set_max_speed_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z);
    pos_control->set_max_accel_z(BRAKE_MODE_DECEL_RATE);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    _timeout_ms = 0;

    return true;
}



// brake_run - runs the brake controller
// should be called at 100hz or more
void Copter::ModeBrake::run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        init_brake_target(BRAKE_MODE_DECEL_RATE);
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // relax stop target if we might be landed
    if (ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // run brake controller
    update_brake();

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), 0.0f);

    // update altitude target and call position controller
    // protects heli's from inflight motor interlock disable
    if (motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::GROUND_IDLE && !ap.land_complete) {
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
    } else {
        pos_control->set_alt_target_from_climb_rate_ff(0.0f, G_Dt, false);
    }
    pos_control->update_z_controller();

    if (_timeout_ms != 0 && millis()-_timeout_start >= _timeout_ms) {
        if (!copter.set_mode(LOITER, MODE_REASON_BRAKE_TIMEOUT)) {
            copter.set_mode(ALT_HOLD, MODE_REASON_BRAKE_TIMEOUT);
        }
    }
}

void Copter::ModeBrake::timeout_to_loiter_ms(uint32_t timeout_ms)
{
    _timeout_start = millis();
    _timeout_ms = timeout_ms;
}

#endif
