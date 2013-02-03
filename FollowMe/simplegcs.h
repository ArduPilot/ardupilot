
#ifndef __SIMPLE_GCS_H__
#define __SIMPLE_GCS_H__

#include <GCS_MAVLink.h>

typedef void(*simplegcs_handler_t)(mavlink_channel_t, mavlink_message_t*);

void simplegcs_send_heartbeat(mavlink_channel_t chan);
bool simplegcs_try_send_statustext(mavlink_channel_t chan, const char *text, int len);

void simplegcs_update(mavlink_channel_t chan, simplegcs_handler_t);
void handle_message(mavlink_channel_t chan, mavlink_message_t* msg);

void simplegcs_send_console_async(uint32_t ms);
void simplegcs_send_heartbeat_async(uint32_t ms);

#endif // __SIMPLE_GCS_H__

