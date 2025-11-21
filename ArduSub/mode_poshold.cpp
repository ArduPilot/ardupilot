// ArduSub position hold flight mode
// GPS required
// Jacob Walser August 2016

#include "Sub.h"

#if POSHOLD_ENABLED

// poshold_init - initialise PosHold controller
bool ModePoshold::init(bool ignore_checks)
{
    // fail to initialise PosHold mode if no GPS lock
    if (!sub.position_ok()) {
        return false;
    }

    // initialize vertical speeds and acceleration
    // All limits must be positive
    position_control->NE_set_max_speed_accel_cm(g.pilot_speed, g.pilot_accel_z);
    position_control->NE_set_correction_speed_accel_cm(g.pilot_speed, g.pilot_accel_z);
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->D_set_correction_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise position and desired velocity
    position_control->NE_init_controller_stopping_point();
    position_control->D_init_controller();

    // Stop all thrusters
    attitude_control->set_throttle_out(0.5f ,true, g.throttle_filt);
    attitude_control->relax_attitude_controllers();
    position_control->D_relax_controller(0.5f);

    sub.last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
void ModePoshold::run()
{
    uint32_t tnow = AP_HAL::millis();
    // When unarmed, disable motors and stabilization
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control->set_throttle_out(0.5f ,true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->NE_init_controller_stopping_point();
        position_control->D_relax_controller(0.5f);
        sub.last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    /////////////////////
    // Update attitude //

    // get pilot's desired yaw rate
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * sub.gain, channel_yaw->get_radio_trim());
    float target_yaw_rate = sub.get_pilot_desired_yaw_rate(yaw_input);

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    float target_roll, target_pitch;
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, sub.aparm.angle_max);

    // update attitude controller targets
    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
        sub.last_pilot_heading = ahrs.yaw_sensor;
        sub.last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw manoeuvres
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < sub.last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0; // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
            sub.last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute bearing
            attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll, target_pitch, sub.last_pilot_heading, true);
        }
    }

    // update z axis
    control_depth();

    // update xy axis
    // call this after Sub::get_pilot_desired_climb_rate is called so that THR_DZ is reasonable
    control_horizontal();
}

void ModePoshold::control_horizontal() {
    float lateral_out = 0;
    float forward_out = 0;

    // get desired rates in the body frame
    Vector2f body_rates_cms = {
        sub.get_pilot_desired_horizontal_rate(channel_forward),
        sub.get_pilot_desired_horizontal_rate(channel_lateral)
    };

    if (sub.position_ok()) {
        if (!position_control->NE_is_active()) {
            // the xy controller timed out, re-initialize
            position_control->NE_init_controller_stopping_point();
        }

        // convert to the earth frame and set target rates
        auto earth_rates_cms = ahrs.body_to_earth2D(body_rates_cms);
        position_control->input_vel_accel_NE_cm(earth_rates_cms, {0, 0});

        // convert pos control roll and pitch angles back to lateral and forward efforts
        sub.translate_pos_control_rp(lateral_out, forward_out);

        // update the xy controller
        position_control->NE_update_controller();
    } else if (g.pilot_speed > 0) {
        // allow the pilot to reposition manually
        forward_out = body_rates_cms.x / (float)g.pilot_speed;
        lateral_out = body_rates_cms.y / (float)g.pilot_speed;
    }

    motors.set_forward(forward_out);
    motors.set_lateral(lateral_out);
}
#endif  // POSHOLD_ENABLED
