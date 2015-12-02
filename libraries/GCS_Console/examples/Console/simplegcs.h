
#ifndef __SIMPLE_GCS_H__
#define __SIMPLE_GCS_H__

#include <GCS_MAVLink/GCS_MAVLink.h>

void send_heartbeat(mavlink_channel_t chan);
bool try_send_message(mavlink_channel_t chan, int msgid);
bool try_send_statustext(mavlink_channel_t chan, const char *text, int len);

void simplegcs_update(mavlink_channel_t chan);
void handle_message(mavlink_channel_t chan, mavlink_message_t* msg);

#endif // __SIMPLE_GCS_H__

