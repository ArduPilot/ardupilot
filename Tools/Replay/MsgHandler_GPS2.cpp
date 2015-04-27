#include <MsgHandler_GPS2.h>

void MsgHandler_GPS2::process_message(uint8_t *msg)
{
    // only LOG_GPS_MSG gives us relative altitude.  We still log
    // the relative altitude when we get a LOG_GPS2_MESSAGE - but
    // the value we use (probably) comes from the most recent
    // LOG_GPS_MESSAGE message!
    update_from_msg_gps(1, msg, false);
}
