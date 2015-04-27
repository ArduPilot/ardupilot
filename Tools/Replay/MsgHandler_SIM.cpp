#include "MsgHandler_SIM.h"

void MsgHandler_SIM::process_message(uint8_t *msg)
{
    wait_timestamp_from_msg(msg);
    attitude_from_msg(msg, sim_attitude, "Roll", "Pitch", "Yaw");
}
