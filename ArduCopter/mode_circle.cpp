#include "Copter.h"

#if MODE_CIRCLE_ENABLED == ENABLED

/*
 * Init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool ModeCircle::init(bool ignore_checks)
{
    pilot_yaw_override = false;
    speed_changing = false;

    // initialize speeds and accelerations
    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // initialise circle controller including setting the circle center based on vehicle speed
    copter.circle_nav->init();

    return true;
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void ModeCircle::run()
{
    // initialize speeds and accelerations
    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // get pilot's desired yaw rate (or zero if in radio failsafe)
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    if (!is_zero(target_yaw_rate)) {
        pilot_yaw_override = true;
    }

    // pilot changes to circle rate and radius
    // skip if in radio failsafe
    if (!copter.failsafe.radio && copter.circle_nav->pilot_control_enabled()) {
        // update the circle controller's radius target based on pilot pitch stick inputs
        const float radius_current = copter.circle_nav->get_radius();           // circle controller's radius target, which begins as the circle_radius parameter
        const float pitch_stick = channel_pitch->norm_input_dz();               // pitch stick normalized -1 to 1
        const float nav_speed = copter.wp_nav->get_default_speed_xy();          // copter WP_NAV parameter speed
        const float radius_pilot_change = (pitch_stick * nav_speed) * G_Dt;     // rate of change (pitch stick up reduces the radius, as in moving forward)
        const float radius_new = MAX(radius_current + radius_pilot_change,0);   // new radius target

        if (!is_equal(radius_current, radius_new)) {
            copter.circle_nav->set_radius(radius_new);
        }

        // update the orbicular rate target based on pilot roll stick inputs
        // skip if using CH6 tuning knob for circle rate
        if (g.radio_tuning != TUNING_CIRCLE_RATE) {
            const float roll_stick = channel_roll->norm_input_dz();         // roll stick normalized -1 to 1

            if (is_zero(roll_stick)) {
                // no speed change, so reset speed changing flag
                speed_changing = false;
            } else {
                const float rate = copter.circle_nav->get_rate();           // circle controller's rate target, which begins as the circle_rate parameter
                const float rate_current = copter.circle_nav->get_rate_current(); // current adjusted rate target, which is probably different from _rate
                const float rate_pilot_change = (roll_stick * G_Dt);        // rate of change from 0 to 1 degrees per second
                float rate_new = rate_current;                              // new rate target
                if (is_positive(rate)) {
                    // currently moving clockwise, constrain 0 to 90
                    rate_new = constrain_float(rate_current + rate_pilot_change, 0, 90);

                } else if (is_negative(rate)) {
                    // currently moving counterclockwise, constrain -90 to 0
                    rate_new = constrain_float(rate_current + rate_pilot_change, -90, 0);

                } else if (is_zero(rate) && !speed_changing) {
                    // Stopped, pilot has released the roll stick, and pilot now wants to begin moving with the roll stick
                    rate_new = rate_pilot_change;
                }

                speed_changing = true;
                copter.circle_nav->set_rate(rate_new);
            }
        }
    }

    // get pilot desired climb rate (or zero if in radio failsafe)
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    // adjust climb rate using rangefinder
    if (copter.rangefinder_alt_ok()) {
        // if rangefinder is ok, use surface tracking
        target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run circle controller
    copter.failsafe_terrain_set_status(copter.circle_nav->update());

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
    // protects heli's from inflight motor interlock disable
    if (motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::GROUND_IDLE && !copter.ap.land_complete) {
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
    } else {
        pos_control->set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
    }
    pos_control->update_z_controller();
}

uint32_t ModeCircle::wp_distance() const
{
    return copter.circle_nav->get_distance_to_target();
}

int32_t ModeCircle::wp_bearing() const
{
    return copter.circle_nav->get_bearing_to_target();
}

#endif
