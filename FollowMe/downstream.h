
#ifndef __FOLLOWME_DOWNSTREAM_H__
#define __FOLLOWME_DOWNSTREAM_H__

#include <GCS_MAVLink.h>

void downstream_handler(mavlink_channel_t from, mavlink_message_t* msg);

#endif // __FOLLOWME_DOWNSTREAM_H__

