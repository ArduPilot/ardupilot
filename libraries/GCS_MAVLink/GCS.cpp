#include "GCS.h"

extern const AP_HAL::HAL& hal;

/*
  send a text message to all GCS
 */
void GCS::send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list)
{
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1] {};
    hal.util->vsnprintf((char *)text, sizeof(text)-1, fmt, arg_list);
    send_statustext(severity, GCS_MAVLINK::active_channel_mask() | GCS_MAVLINK::streaming_channel_mask(), text);
}

void GCS::send_text(MAV_SEVERITY severity, const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    send_textv(severity, fmt, arg_list);
    va_end(arg_list);
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

void GCS::send_named_float(const char *name, float value) const
{
    FOR_EACH_ACTIVE_CHANNEL(send_named_float(name, value));
}

void GCS::send_home() const
{
    FOR_EACH_ACTIVE_CHANNEL(send_home());
}

void GCS::send_ekf_origin() const
{
    FOR_EACH_ACTIVE_CHANNEL(send_ekf_origin());
}

/*
  install an alternative protocol handler. This allows another
  protocol to take over the link if MAVLink goes idle. It is used to
  allow for the AP_BLHeli pass-thru protocols to run on hal.uartA
 */
bool GCS::install_alternative_protocol(mavlink_channel_t c, GCS_MAVLINK::protocol_handler_fn_t handler)
{
    if (c >= num_gcs()) {
        return false;
    }
    if (chan(c).alternative.handler) {
        // already have one installed - we may need to add support for
        // multiple alternative handlers
        return false;
    }
    chan(c).alternative.handler = handler;
    return true;
}


#undef FOR_EACH_ACTIVE_CHANNEL
