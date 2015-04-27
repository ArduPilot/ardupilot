#include "MsgHandler_AHR2.h"

void MsgHandler_AHR2::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    attitude_from_msg(msg, ahr2_attitude, "Roll", "Pitch", "Yaw");
}
