#include "Tracker.h"

/*
 * Manual control mode
 */

/*
 * update_manual - runs the manual controller
 *  called at 50hz while control mode is 'MANUAL'
 */
void ModeManual::update()
{
    // copy yaw and pitch input to output
    SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_yaw, RC_Channels::rc_channel(CH_YAW)->get_radio_in());
    SRV_Channels::constrain_pwm(SRV_Channel::k_tracker_yaw);

    SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_pitch, RC_Channels::rc_channel(CH_PITCH)->get_radio_in());
    SRV_Channels::constrain_pwm(SRV_Channel::k_tracker_pitch);
}
