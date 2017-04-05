#include "Sub.h"

void Sub::init_rc_in()
{
    channel_pitch    = RC_Channels::rc_channel(0);
    channel_roll     = RC_Channels::rc_channel(1);
    channel_throttle = RC_Channels::rc_channel(2);
    channel_yaw      = RC_Channels::rc_channel(3);
    channel_forward  = RC_Channels::rc_channel(4);
    channel_lateral  = RC_Channels::rc_channel(5);

    // set rc channel ranges
    channel_roll->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_pitch->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_yaw->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_throttle->set_range(1000);
    channel_forward->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_lateral->set_angle(ROLL_PITCH_INPUT_MAX);

    // set default dead zones
    channel_roll->set_default_dead_zone(30);
    channel_pitch->set_default_dead_zone(30);
    channel_throttle->set_default_dead_zone(30);
    channel_yaw->set_default_dead_zone(40);
    channel_forward->set_default_dead_zone(30);
    channel_lateral->set_default_dead_zone(30);

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    // initialize rc input to 1500 on control channels (rather than 0)
    for (int i = 0; i < 6; i++) {
        hal.rcin->set_override(i, 1500);
    }
#endif
}

// init_rc_out -- initialise motors and check if pilot wants to perform ESC calibration
void Sub::init_rc_out()
{
    motors.set_update_rate(g.rc_speed);
    motors.set_loop_rate(scheduler.get_loop_rate_hz());
    motors.init((AP_Motors::motor_frame_class)g.frame_configuration.get(), (AP_Motors::motor_frame_type)0);
    motors.set_throttle_range(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());

    // enable output to motors
    if (arming.rc_check()) {
        enable_motor_output();
    }

    // refresh auxiliary channel to function map
    SRV_Channels::update_aux_servo_function();
}
