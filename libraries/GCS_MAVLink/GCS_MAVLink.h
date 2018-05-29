/// @file	GCS_MAVLink.h
/// @brief	One size fits all header for MAVLink integration.
#pragma once

#include <AP_Param/AP_Param.h>

// we have separate helpers disabled to make it possible
// to select MAVLink 1.0 in the arduino GUI build
#define MAVLINK_SEPARATE_HELPERS
#define MAVLINK_NO_CONVERSION_HELPERS

#define MAVLINK_SEND_UART_BYTES(chan, buf, len) comm_send_buffer(chan, buf, len)

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// allow extra mavlink channels in SITL for:
//    Vicon
#define MAVLINK_COMM_NUM_BUFFERS 6
#else
// allow five telemetry ports
#define MAVLINK_COMM_NUM_BUFFERS 5
#endif

/*
  The MAVLink protocol code generator does its own alignment, so
  alignment cast warnings can be ignored
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"

#include "include/mavlink/v2.0/ardupilotmega/version.h"

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

void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len);

/// Check for available data on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_available(mavlink_channel_t chan);


/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_txspace(mavlink_channel_t chan);

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "include/mavlink/v2.0/ardupilotmega/mavlink.h"

// return a MAVLink variable type given a AP_Param type
uint8_t mav_var_type(enum ap_var_type t);

#pragma GCC diagnostic pop
