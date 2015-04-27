#include "MsgHandler_IMU3.h"

void MsgHandler_IMU3::process_message(uint8_t *msg)
{
  update_from_msg_imu(2, msg);
}
