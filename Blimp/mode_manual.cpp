#include "Blimp.h"
/*
 * Init and run calls for manual flight mode
 */

// Runs the main manual controller
void ModeManual::run()
{
    motors->right_out = channel_right->get_control_in() / float(RC_SCALE);
    motors->front_out = channel_front->get_control_in() / float(RC_SCALE);
    motors->yaw_out = channel_yaw->get_control_in() / float(RC_SCALE);
    motors->down_out = channel_down->get_control_in() / float(RC_SCALE);
}
