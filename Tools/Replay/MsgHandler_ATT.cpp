#include "MsgHandler_ATT.h"

void MsgHandler_ATT::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    attitude_from_msg(msg, attitude, "Roll", "Pitch", "Yaw");
}
