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
    channel_yaw.radio_out = channel_yaw.radio_in;
    channel_pitch.radio_out = channel_pitch.radio_in;
    channel_yaw.output();
    channel_pitch.output();
}
