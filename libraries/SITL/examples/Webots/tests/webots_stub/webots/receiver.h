#ifndef WEBOTS_STUB_RECEIVER_H
#define WEBOTS_STUB_RECEIVER_H
#include "types.h"
void wb_receiver_enable(WbDeviceTag tag, int ms);
void wb_receiver_set_channel(WbDeviceTag tag, int channel);
int wb_receiver_get_queue_length(WbDeviceTag tag);
const void *wb_receiver_get_data(WbDeviceTag tag);
void wb_receiver_next_packet(WbDeviceTag tag);
#endif
