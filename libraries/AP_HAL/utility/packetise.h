#pragma once
#include "RingBuffer.h"
#include <GCS_MAVLink/GCS_config.h>

#ifndef AP_MAVLINK_PACKETISE_ENABLED
#define AP_MAVLINK_PACKETISE_ENABLED HAL_GCS_ENABLED
#endif

/*
  return the number of bytes to send for a packetised connection
*/
uint16_t mavlink_packetise(ByteBuffer &writebuf, uint16_t n);

