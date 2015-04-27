#include "MsgHandler_IMU.h"

void MsgHandler_IMU::process_message(uint8_t *msg)
{
    update_from_msg_imu(0, msg);
}
