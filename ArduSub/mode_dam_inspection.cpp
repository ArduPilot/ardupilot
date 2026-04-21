#include "Sub.h"

#define TEST_ROLL_D 10.0f 

bool ModeDamInspection::init(bool ignore_checks) {

    // initialize vertical maximum speeds and acceleration
    // sets the maximum speed up and down returned by position controller
    position_control->set_max_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    sub.inertial_doppler.set_sensor_to_body_rot(Rotation::ROTATION_PITCH_270);
    // initialise position and desired velocity
    position_control->init_zz_controller(g2.zz_desired_m);

    sub.last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

void ModeDamInspection::run()
{
    run_pre();
    //control_depth();
    run_post();
}

bool ModeDamInspection::get_desired_euler_angles_pitch_roll(float& roll_rad, float& pitch_rad)
{
    Vector3f v = sub.inertial_doppler.get_normal_body();
    if (v.is_zero()) {
        return false;
    }
    roll_rad = atan2f(v.y, v.z);
    pitch_rad = -atan2f(v.x, safe_sqrt(sq(v.y) + sq(v.z)));
    return true;
}

void ModeDamInspection::add_ROV_attitude_to_degrees(float& roll, float& pitch, float& yaw)
{
    roll += ahrs.get_roll();
    pitch += ahrs.get_pitch();
    yaw += ahrs.get_yaw();

    roll = degrees(roll);
    pitch = degrees(pitch);
    yaw = degrees(yaw);
}

void ModeDamInspection::run_pre()
{
    uint32_t tnow = AP_HAL::millis();

    // initialize vertical speeds and acceleration
    position_control->set_max_speed_accel_z(-sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control->set_throttle_out(0.5,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->relax_zz_controller(g2.zz_desired_m);
        sub.last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get pilot desired yaw
    float target_roll, target_pitch, target_yaw;

    if (!get_desired_euler_angles_pitch_roll(target_roll, target_pitch))
    {
        target_roll = last_roll;
        target_pitch = last_pitch;
    }
    last_roll = target_roll;
    last_pitch = target_pitch;

    // Check if set_attitude_target_no_gps is valid

    float fake_roll, fake_pitch;
    // I didn't know how to write this code, so I just copied it from the original code
    if (tnow - sub.set_attitude_target_no_gps.last_message_ms < 5000) {
        Quaternion(
            sub.set_attitude_target_no_gps.packet.q
        ).to_euler(
            fake_roll,
            fake_pitch,
            target_yaw
        );
        
        add_ROV_attitude_to_degrees(target_roll, target_pitch, target_yaw);
        attitude_control->input_euler_angle_roll_pitch_yaw(TEST_ROLL_D * 1e2f, target_pitch * 1e2f, target_yaw * 1e2f, true);
        return;
    }

    // get pilot's desired yaw rate
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * sub.gain, channel_yaw->get_radio_trim());
    float target_yaw_rate = sub.get_pilot_desired_yaw_rate(yaw_input);

    add_ROV_attitude_to_degrees(target_roll, target_pitch, target_yaw);

    // call attitude controller
    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(TEST_ROLL_D * 1e2f, target_pitch * 1e2f, target_yaw_rate);
        sub.last_pilot_heading = ahrs.yaw_sensor;
        sub.last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < sub.last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0; // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(TEST_ROLL_D, target_pitch, target_yaw_rate);
            sub.last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute bearing
            attitude_control->input_euler_angle_roll_pitch_yaw(TEST_ROLL_D, target_pitch, sub.last_pilot_heading, true);
        }
    }
}

void ModeDamInspection::run_post()
{
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}

void ModeDamInspection::control_depth() {
    position_control->update_zz_controller(sub.inertial_doppler.get_distance_m(), sub.inertial_doppler.get_z_vel_mps());
}
