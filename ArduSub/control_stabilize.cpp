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
    float desired_roll_rate, desired_pitch_rate, desired_yaw_rate;
    // initialize vertical speeds and acceleration
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    handle_mavlink_attitude_target();

    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), desired_roll_rate, desired_pitch_rate, attitude_control.get_althold_lean_angle_max());
    float yaw_input =  channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * gain, channel_yaw->get_radio_trim());
    desired_yaw_rate = get_pilot_desired_yaw_rate(yaw_input);

    switch (g.control_frame) {
      case MAV_FRAME_BODY_FRD:
      {
        if (abs(desired_roll_rate) > 50 || abs(desired_pitch_rate) > 50 || abs(desired_yaw_rate) > 50) {
            attitude_control.input_rate_bf_roll_pitch_yaw(desired_roll_rate, desired_pitch_rate, desired_yaw_rate);
            Vector3f attitude_target = attitude_control.get_att_target_euler_cd();
            last_roll = attitude_target.x;
            last_pitch = attitude_target.y;
            last_pilot_heading = attitude_target.z;
        } else {
            attitude_control.input_euler_angle_roll_pitch_yaw(last_roll, last_pitch, last_pilot_heading, true);
        }
      }
      break;
      default:
      {
        if (!is_zero(desired_roll_rate) || !is_zero(desired_pitch_rate) || !is_zero(desired_yaw_rate)) {
            attitude_control.input_euler_rate_roll_pitch_yaw(desired_roll_rate, desired_pitch_rate, desired_yaw_rate);
            Vector3f attitude_target = attitude_control.get_att_target_euler_cd();
            last_roll = attitude_target.x;
            last_pitch = attitude_target.y;
            last_pilot_heading = attitude_target.z;
            last_input_ms = AP_HAL::millis();
            return;
        }
        attitude_control.input_euler_angle_roll_pitch_yaw(last_roll, last_pitch, last_pilot_heading, true);
      }
    }
}
