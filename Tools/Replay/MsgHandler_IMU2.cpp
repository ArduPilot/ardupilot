#include "MsgHandler_IMU2.h"

void MsgHandler_IMU2::process_message(uint8_t *msg)
{
  update_from_msg_imu(1, msg);
}
