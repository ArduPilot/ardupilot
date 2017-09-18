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

#define FOR_EACH_ACTIVE_CHANNEL(methodcall)          \
    do {                                             \
        for (uint8_t i=0; i<num_gcs(); i++) {        \
            if (!chan(i).initialised) {              \
                continue;                            \
            }                                        \
            if (!(GCS_MAVLINK::active_channel_mask() & (1 << (chan(i).get_chan()-MAVLINK_COMM_0)))) { \
                continue;                            \
            }                                        \
            chan(i).methodcall;                      \
        }                                            \
    } while (0)

void GCS::send_home(const Location &home) const
{
    FOR_EACH_ACTIVE_CHANNEL(send_home(home));
}

void GCS::send_ekf_origin(const Location &ekf_origin) const
{
    FOR_EACH_ACTIVE_CHANNEL(send_ekf_origin(ekf_origin));
}

#undef FOR_EACH_ACTIVE_CHANNEL
