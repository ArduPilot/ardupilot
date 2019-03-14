#include "Sub.h"


/*
 * control_althold.pde - init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Sub::althold_init()
{
    if(!control_check_barometer()) {
        return false;
    }

    // initialize vertical speeds and leash lengths
    // sets the maximum speed up and down returned by position controller
    pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control.set_max_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Sub::althold_run()
{
    uint32_t tnow = AP_HAL::millis();

    // initialize vertical speeds and acceleration
    pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control.set_max_accel_z(g.pilot_accel_z);

    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(motors.get_throttle_hover());
        last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // get pilot desired lean angles
    float target_roll, target_pitch;

    // Check if set_attitude_target_no_gps is valid
    if (tnow - sub.set_attitude_target_no_gps.last_message_ms < 5000) {
        float target_yaw;
        Quaternion(
            set_attitude_target_no_gps.packet.q
        ).to_euler(
            target_roll,
            target_pitch,
            target_yaw
        );
        target_roll = degrees(target_roll);
        target_pitch = degrees(target_pitch);
        target_yaw = degrees(target_yaw);

        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll * 1e2f, target_pitch * 1e2f, target_yaw * 1e2f, true);
        return;
    }

    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // call attitude controller
    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        last_pilot_heading = ahrs.yaw_sensor;
        last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0; // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute absolute bearing
            attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, last_pilot_heading, true);
        }
    }

    if (fabsf(channel_throttle->norm_input()-0.5f) > 0.05f) { // Throttle input above 5%
        // output pilot's throttle
        attitude_control.set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);
        // reset z targets to current values
        pos_control.relax_alt_hold_controllers(motors.get_throttle_hover());
    } else { // hold z

        if (ap.at_bottom) {
            pos_control.relax_alt_hold_controllers(motors.get_throttle_hover()); // clear velocity and position targets, and integrator
            pos_control.set_alt_target(inertial_nav.get_altitude() + 10.0f); // set target to 10 cm above bottom
        } else if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            float target_climb_rate = get_surface_tracking_climb_rate(0, pos_control.get_alt_target(), G_Dt);
            pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        }
        pos_control.update_z_controller();
    }

    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
