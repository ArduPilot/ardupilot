/// @file	GCS_MAVLink.h
/// @brief	One size fits all header for MAVLink integration.
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// we have separate helpers disabled to make it possible
// to select MAVLink 1.0 in the arduino GUI build
#define MAVLINK_SEPARATE_HELPERS
#define MAVLINK_NO_CONVERSION_HELPERS

#define MAVLINK_SEND_UART_BYTES(chan, buf, len) comm_send_buffer(chan, buf, len)

#define MAVLINK_START_UART_SEND(chan, size) comm_send_lock(chan, size)
#define MAVLINK_END_UART_SEND(chan, size) comm_send_unlock(chan)

// allow five telemetry ports
#define MAVLINK_COMM_NUM_BUFFERS 5

#define MAVLINK_GET_CHANNEL_BUFFER 1
#define MAVLINK_GET_CHANNEL_STATUS 1

/*
  The MAVLink protocol code generator does its own alignment, so
  alignment cast warnings can be ignored
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"

#if defined(__GNUC__) && __GNUC__ >= 9
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif

#include "include/mavlink/v2.0/all/version.h"

#define MAVLINK_MAX_PAYLOAD_LEN 255

#include "include/mavlink/v2.0/mavlink_types.h"

/// MAVLink stream used for uartA
extern AP_HAL::UARTDriver	*mavlink_comm_port[MAVLINK_COMM_NUM_BUFFERS];
extern bool gcs_alternative_active[MAVLINK_COMM_NUM_BUFFERS];

/// MAVLink system definition
extern mavlink_system_t mavlink_system;

/// Sanity check MAVLink channel
///
/// @param chan		Channel to send to
static inline bool valid_channel(mavlink_channel_t chan)
{
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wtautological-constant-out-of-range-compare"
    return chan < MAVLINK_COMM_NUM_BUFFERS;
#pragma clang diagnostic pop
}

mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan);
mavlink_status_t* mavlink_get_channel_status(uint8_t chan);

void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len);

/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_txspace(mavlink_channel_t chan);

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "include/mavlink/v2.0/all/mavlink.h"

// lock and unlock a channel, for multi-threaded mavlink send
void comm_send_lock(mavlink_channel_t chan, uint16_t size);
void comm_send_unlock(mavlink_channel_t chan);
HAL_Semaphore &comm_chan_lock(mavlink_channel_t chan);

#pragma GCC diagnostic pop
