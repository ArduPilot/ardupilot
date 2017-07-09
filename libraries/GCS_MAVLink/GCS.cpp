#include "GCS.h"

extern const AP_HAL::HAL& hal;

/*
  send a text message to all GCS
 */
void GCS::send_text(MAV_SEVERITY severity, const char *fmt, ...)
{
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1] {};
    va_list arg_list;
    va_start(arg_list, fmt);
    hal.util->vsnprintf((char *)text, sizeof(text)-1, fmt, arg_list);
    va_end(arg_list);
    text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = 0;
    send_statustext(severity, GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask(), text);
}
