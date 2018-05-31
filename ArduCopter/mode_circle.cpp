#include "Copter.h"

#if MODE_CIRCLE_ENABLED == ENABLED

/*
 * Init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool Copter::ModeCircle::init(bool ignore_checks)
{
    if (copter.position_ok() || ignore_checks) {
        pilot_yaw_override = false;

        // initialize speeds and accelerations
        pos_control->set_max_speed_xy(wp_nav->get_speed_xy());
        pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
        pos_control->set_max_accel_z(g.pilot_accel_z);

        // initialise circle controller including setting the circle center based on vehicle speed
        copter.circle_nav->init();

        return true;
    }else{
        return false;
    }
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void Copter::ModeCircle::run()
{
    // initialize speeds and accelerations
    pos_control->set_max_speed_xy(wp_nav->get_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // get pilot's desired yaw rate (or zero if in radio failsafe)
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    if (!is_zero(target_yaw_rate)) {
        pilot_yaw_override = true;
    }

    // get pilot desired climb rate (or zero if in radio failsafe)
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    // adjust climb rate using rangefinder
    if (copter.rangefinder_alt_ok()) {
        // if rangefinder is ok, use surface tracking
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
    }

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        pos_control->relax_alt_hold_controllers(0.0f);
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        return;
    }

    // if landed, spool down motors and disarm
    if (ap.land_complete) {
        zero_throttle_and_hold_attitude();
        pos_control->relax_alt_hold_controllers(0.0f);
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        if (motors->get_spool_mode() == AP_Motors::SPIN_WHEN_ARMED) {
            copter.init_disarm_motors();
        }
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run circle controller
    copter.circle_nav->update();

    // call attitude controller
    if (pilot_yaw_override) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(copter.circle_nav->get_roll(),
                                                                      copter.circle_nav->get_pitch(),
                                                                      target_yaw_rate);
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw(copter.circle_nav->get_roll(),
                                                           copter.circle_nav->get_pitch(),
                                                           copter.circle_nav->get_yaw(), true);
    }

    // update altitude target and call position controller
    pos_control->set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
    pos_control->update_z_controller();
}

uint32_t Copter::ModeCircle::wp_distance() const
{
    return copter.circle_nav->get_distance_to_target();
}

int32_t Copter::ModeCircle::wp_bearing() const
{
    return copter.circle_nav->get_bearing_to_target();
}

#endif
