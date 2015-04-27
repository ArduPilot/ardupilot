#include "MsgHandler_GPS.h"

void MsgHandler_GPS::process_message(uint8_t *msg)
{
    update_from_msg_gps(0, msg, true);
}
