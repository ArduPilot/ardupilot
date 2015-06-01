// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_manual.pde - manual control mode
 */

/*
 * update_manual - runs the manual controller
 *  called at 50hz while control_mode is 'MANUAL'
 */
static void update_manual(void)
{
    // copy yaw and pitch input to output
    channel_yaw.radio_out = constrain_int16(channel_yaw.radio_in, channel_yaw.radio_min, channel_yaw.radio_max);
    channel_pitch.radio_out = constrain_int16(channel_pitch.radio_in, channel_pitch.radio_min, channel_pitch.radio_max);

    // send output to servos
    channel_yaw.output();
    channel_pitch.output();
}
