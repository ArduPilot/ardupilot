#include "Sub.h"

// stabilize_init - initialise stabilize controller
bool Sub::stabilize_init()
{
    // set target altitude to zero for reporting
    pos_control.set_pos_target_z_cm(0);
    if(prev_control_mode != control_mode_t::ALT_HOLD) {
        last_roll = 0;
        last_pitch = 0;
    }
    last_pilot_heading = ahrs.yaw_sensor;
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Sub::stabilize_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        last_pilot_heading = ahrs.yaw_sensor;
        last_roll = 0;
        last_pitch = 0;
        return;
    }

    handle_attitude();

    // output pilot's throttle
    attitude_control.set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);

    //control_in is range -1000-1000
    //radio_in is raw pwm value
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}


void Sub::handle_attitude()
{
    uint32_t tnow = AP_HAL::millis();
    float desired_roll_rate, desired_pitch_rate, desired_yaw_rate;
    // initialize vertical speeds and acceleration
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    handle_mavlink_attitude_target();

    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), desired_roll_rate, desired_pitch_rate, attitude_control.get_althold_lean_angle_max());
    float yaw_input =  channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * gain, channel_yaw->get_radio_trim());
    desired_yaw_rate = get_pilot_desired_yaw_rate(yaw_input);

    switch (g.control_frame)
    {
        case MAV_FRAME_BODY_FRD:
        {
            if (abs(desired_roll_rate) > 50 || abs(desired_pitch_rate) > 50 || abs(desired_yaw_rate) > 50)
            {
                attitude_control.input_rate_bf_roll_pitch_yaw(desired_roll_rate, desired_pitch_rate, desired_yaw_rate);
                Quaternion attitude_target = attitude_control.get_attitude_target_quat();
                last_roll = degrees(attitude_target.get_euler_roll()) * 100;
                last_pitch = degrees(attitude_target.get_euler_pitch()) * 100;
                last_pilot_heading = degrees(attitude_target.get_euler_yaw()) * 100;
            }
            else
            {
                attitude_control.input_euler_angle_roll_pitch_yaw(last_roll, last_pitch, last_pilot_heading, true);
            }
        }
        break;
        default:
        {
            if (!is_zero(desired_yaw_rate))
            { // call attitude controller with rate yaw determined by pilot input
                attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(last_roll, last_pitch, desired_yaw_rate);
                last_pilot_heading = ahrs.yaw_sensor;
                last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading
            }
            else
            { // hold current heading

                // this check is required to prevent bounce back after very fast yaw maneuvers
                // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
                if (tnow < last_pilot_yaw_input_ms + 250)
                {                         // give 250ms to slow down, then set target heading
                    desired_yaw_rate = 0; // Stop rotation on yaw axis

                    // call attitude controller with target yaw rate = 0 to decelerate onqq yaw axis
                    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(last_roll, last_pitch, desired_yaw_rate);
                    last_pilot_heading = ahrs.yaw_sensor; // update heading to hold
                }
                else
                { // call attitude controller holding absolute absolute bearing
                    attitude_control.input_euler_angle_roll_pitch_yaw(last_roll, last_pitch, last_pilot_heading, true);
                }
            }
        }
    }
}
