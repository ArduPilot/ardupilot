#include "MsgHandler_MAG.h"

void MsgHandler_MAG::process_message(uint8_t *msg)
{
    update_from_msg_compass(0, msg);
}
