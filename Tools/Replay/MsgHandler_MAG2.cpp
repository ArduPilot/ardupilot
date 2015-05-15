#include "MsgHandler_MAG2.h"

void MsgHandler_MAG2::process_message(uint8_t *msg)
{
    update_from_msg_compass(1, msg);
}
