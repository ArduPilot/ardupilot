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

    // initialize vertical maximum speeds and acceleration
    // sets the maximum speed up and down returned by position controller
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control.set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.init_z_controller_stopping_point();

    if(prev_control_mode != control_mode_t::STABILIZE) {
        last_roll = 0;
        last_pitch = 0;
    }
    last_pilot_heading = ahrs.yaw_sensor;
    last_input_ms = AP_HAL::millis();

    return true;
}


void Sub::handle_attitude()
{
    uint32_t tnow = AP_HAL::millis();

    // initialize vertical speeds and acceleration
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
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
        last_roll = target_roll;
        last_pitch = target_pitch;
        last_pilot_heading = target_yaw;
        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
    } else {
        // If we don't have a mavlink attitude target, we use the pilot's input instead
        get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());
        target_yaw = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (abs(target_roll) > 50 || abs(target_pitch) > 50 || abs(target_yaw) > 50) {
            last_roll = ahrs.roll_sensor;
            last_pitch = ahrs.pitch_sensor;
            last_pilot_heading = ahrs.yaw_sensor;
            last_input_ms = tnow;
            attitude_control.input_rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);
        } else if (tnow < last_input_ms + 250) {
            // just brake for a few mooments so we don't bounce
            attitude_control.input_rate_bf_roll_pitch_yaw(0, 0, 0);
        } else {
            // Lock attitude
            attitude_control.input_euler_angle_roll_pitch_yaw(last_roll, last_pitch, last_pilot_heading, true);
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
        attitude_control.set_throttle_out(0.5,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        pos_control.relax_z_controller(motors.get_throttle_hover());
        last_roll = 0;
        last_pitch = 0;
        last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    handle_attitude();

    control_depth();
}

void Sub::control_depth() {
    // We rotate the RC inputs to the earth frame to check if the user is giving an input that would change the depth.
    // Output the Z controller + pilot input to all motors.
    Vector3f earth_frame_rc_inputs = ahrs.get_rotation_body_to_ned() * Vector3f(-channel_forward->norm_input(), -channel_lateral->norm_input(), (2.0f*(-0.5f+channel_throttle->norm_input())));
    float target_climb_rate_cm_s = get_pilot_desired_climb_rate(500 + g.pilot_speed_up * earth_frame_rc_inputs.z);

    bool surfacing = ap.at_surface || pos_control.get_pos_target_z_cm() > g.surface_depth;
    float upper_speed_limit = surfacing ? 0 : g.pilot_speed_up;
    float lower_speed_limit = ap.at_bottom ? 0 : -get_pilot_speed_dn();
    target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, lower_speed_limit, upper_speed_limit);
    pos_control.set_pos_target_z_from_climb_rate_cm(target_climb_rate_cm_s);

    if (surfacing) {
        pos_control.set_alt_target_with_slew(MIN(pos_control.get_pos_target_z_cm(), g.surface_depth - 5.0f)); // set target to 5 cm below surface level
    } else if (ap.at_bottom) {
        pos_control.set_alt_target_with_slew(MAX(inertial_nav.get_altitude() + 10.0f, pos_control.get_pos_target_z_cm())); // set target to 10 cm above bottom
    }
    pos_control.update_z_controller();
    // Read the output of the z controller and rotate it so it always points up
    Vector3f throttle_vehicle_frame = ahrs.get_rotation_body_to_ned().transposed() * Vector3f(0, 0, motors.get_throttle_in_bidirectional());
    //TODO: scale throttle with the ammount of thrusters in the given direction
    float raw_throttle_factor = (ahrs.get_rotation_body_to_ned() * Vector3f(0, 0, 1.0)).xy().length();
    motors.set_throttle(throttle_vehicle_frame.z + raw_throttle_factor * channel_throttle->norm_input());
    motors.set_forward(-throttle_vehicle_frame.x + channel_forward->norm_input());
    motors.set_lateral(-throttle_vehicle_frame.y + channel_lateral->norm_input());
}