// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

/*
 * Manual control mode
 */

/*
 * update_manual - runs the manual controller
 *  called at 50hz while control_mode is 'MANUAL'
 */
void Tracker::update_manual(void)
{
    // copy yaw and pitch input to output
    channel_yaw.set_radio_out(constrain_int16(channel_yaw.get_radio_in(), channel_yaw.get_radio_min(), channel_yaw.get_radio_max()));
    channel_pitch.set_radio_out(constrain_int16(channel_pitch.get_radio_in(), channel_pitch.get_radio_min(), channel_pitch.get_radio_max()));

    // send output to servos
    channel_yaw.output();
    channel_pitch.output();
}
