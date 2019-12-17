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

    pos_control.set_target_to_stopping_point_z();
    holding_depth = true;

    last_roll = ahrs.roll_sensor;
    last_pitch = ahrs.pitch_sensor;
    last_yaw = ahrs.yaw_sensor;
    last_input_ms = AP_HAL::millis();

    return true;
}


void Sub::handle_attitude()
{
    uint32_t tnow = AP_HAL::millis();

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get pilot desired lean angles
    float target_roll, target_pitch, target_yaw;

    // Check if set_attitude_target_no_gps is valid
    if (tnow - sub.set_attitude_target_no_gps.last_message_ms < 5000) {
        Quaternion(
            set_attitude_target_no_gps.packet.q
        ).to_euler(
            target_roll,
            target_pitch,
            target_yaw
        );
        target_roll = 100 * degrees(target_roll);
        target_pitch = 100 * degrees(target_pitch);
        target_yaw = 100 * degrees(target_yaw);
        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
    } else {
        // If we don't have a mavlink attitude target, we use the pilot's input instead
        get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());
        target_yaw = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (abs(target_roll) > 50 || abs(target_pitch) > 50) {
            last_roll = ahrs.roll_sensor;
            last_pitch = ahrs.pitch_sensor;
            last_yaw = ahrs.yaw_sensor;
            last_input_ms = tnow;
            attitude_control.input_rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);
        } else if (abs(target_yaw) > 50) {
            // if only yaw is being controlled, don't update pitch and roll
            attitude_control.input_rate_bf_roll_pitch_yaw(0, 0, target_yaw);
            last_yaw = ahrs.yaw_sensor;
            last_input_ms = tnow;
        } else if (tnow < last_input_ms + 250) {
            // just brake for a few mooments so we don't bounce
            last_yaw = ahrs.yaw_sensor;
            attitude_control.input_rate_bf_roll_pitch_yaw(0, 0, 0);
        } else {
            // Lock attitude
            attitude_control.input_euler_angle_roll_pitch_yaw(last_roll, last_pitch, last_yaw, true);
        }
    }
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Sub::althold_run()
{
    // When unarmed, disable motors and stabilization
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out(0.5 ,true, g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        pos_control.relax_alt_hold_controllers();
        last_roll = ahrs.roll_sensor;
        last_pitch = ahrs.pitch_sensor;
        last_yaw = ahrs.yaw_sensor;
        holding_depth = false;
        return;
    }

    // Vehicle is armed, motors are free to run
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    handle_attitude();

    pos_control.update_z_controller();
    // Read the output of the z controller and rotate it so it always points up
    Vector3f throttle_vehicle_frame = ahrs.get_rotation_body_to_ned().transposed() * Vector3f(0, 0, motors.get_throttle_in_bidirectional());
    // Output the Z controller + pilot input to all motors.

    //TODO: scale throttle with the ammount of thrusters in the given direction
    motors.set_throttle(0.5+throttle_vehicle_frame.z + channel_throttle->norm_input()-0.5);
    motors.set_forward(-throttle_vehicle_frame.x + channel_forward->norm_input());
    motors.set_lateral(-throttle_vehicle_frame.y + channel_lateral->norm_input());

    // We rotate the RC inputs to the earth frame to check if the user is giving an input that would change the depth.
    Vector3f earth_frame_rc_inputs = ahrs.get_rotation_body_to_ned() * Vector3f(channel_forward->norm_input(), channel_lateral->norm_input(), (2.0f*(-0.5f+channel_throttle->norm_input())));

    if (fabsf(earth_frame_rc_inputs.z) > 0.05f) { // Throttle input  above 5%
       // reset z targets to current values
        holding_depth = false;
        pos_control.relax_alt_hold_controllers();
    } else { // hold z
        if (ap.at_surface) {
            pos_control.set_alt_target(g.surface_depth - 5.0f); // set target to 5cm below surface level
            holding_depth = true;
        } else if (ap.at_bottom) {
            pos_control.set_alt_target(inertial_nav.get_altitude() + 10.0f); // set target to 10 cm above bottom
            holding_depth = true;
        } else if (!holding_depth) {
            pos_control.set_target_to_stopping_point_z();
            holding_depth = true;
        }
    }
}
