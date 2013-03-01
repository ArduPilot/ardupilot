
#ifndef __FOLLOWME_UPSTREAM_H__
#define __FOLLOWME_UPSTREAM_H__

#include <GCS_MAVLink.h>

void upstream_handler(mavlink_channel_t from, mavlink_message_t* msg);

#endif // __FOLLOWME_UPSTREAM_H__

