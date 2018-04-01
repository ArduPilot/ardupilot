// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	GCS_MAVLink.h
/// @brief	One size fits all header for MAVLink integration.

#ifndef GCS_MAVLink_h
#define GCS_MAVLink_h

#include "compat.h"


#define MAVLINK_COMM_NUM_CHANNELS 1
#define MAVLINK_CRC_EXTRA 1

#include "include/mavlink/v2.0/ardupilotmega/version.h"

#include "include/mavlink/v2.0/mavlink_types.h"

#undef MAVLINK_COMM_NUM_BUFFERS
#define MAVLINK_COMM_NUM_BUFFERS 1

/// MAVLink stream used for HIL interaction
extern AP_HAL::BetterStream	*mavlink_comm_0_port;

/// MAVLink stream used for ground control communication
extern AP_HAL::BetterStream	*mavlink_comm_1_port;

/// MAVLink system definition
extern mavlink_system_t mavlink_system;

extern void osd_queue(uint8_t c);

/// Send a byte to the nominated MAVLink channel
///
/// @param chan		Channel to send to
/// @param ch		Byte to send
///
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    osd_queue(ch);
}

/// Read a byte from the nominated MAVLink channel
///
/// @param chan		Channel to receive on
/// @returns		Byte read
///
static inline uint8_t comm_receive_ch(mavlink_channel_t chan)
{
    return 0;
}

/// Check for available data on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
static inline uint16_t comm_get_available(mavlink_channel_t chan)
{
    return 0;
}


/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available, -1 for error
static inline int comm_get_txspace(mavlink_channel_t chan)
{
    return 255;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align" // yes I know


#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "include/mavlink/v2.0/ardupilotmega/mavlink.h"

#pragma GCC diagnostic pop

uint8_t mavlink_check_target(uint8_t sysid, uint8_t compid);

// return a MAVLink variable type given a AP_Param type
uint8_t mav_var_type(enum ap_var_type t);

#endif // GCS_MAVLink_h
