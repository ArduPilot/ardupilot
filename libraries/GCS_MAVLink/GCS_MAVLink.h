// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	GCS_MAVLink.h
/// @brief	One size fits all header for MAVLink integration.

#ifndef GCS_MAVLink_h
#define GCS_MAVLink_h

#include <BetterStream.h>

#define MAVLINK_SEPARATE_HELPERS

#ifdef MAVLINK10
# include "include_v1.0/ardupilotmega/version.h"
#else
# include "include/ardupilotmega/version.h"
#endif

// this allows us to make mavlink_message_t much smaller
#define MAVLINK_MAX_PAYLOAD_LEN MAVLINK_MAX_DIALECT_PAYLOAD_SIZE

#define MAVLINK_COMM_NUM_BUFFERS 2
#ifdef MAVLINK10
# include "include_v1.0/mavlink_types.h"
#else
# include "include/mavlink_types.h"
#endif

/// MAVLink stream used for HIL interaction
extern BetterStream	*mavlink_comm_0_port;

/// MAVLink stream used for ground control communication
extern BetterStream	*mavlink_comm_1_port;

/// MAVLink system definition
extern mavlink_system_t mavlink_system;

/// Send a byte to the nominated MAVLink channel
///
/// @param chan		Channel to send to
/// @param ch		Byte to send
///
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    switch(chan) {
	case MAVLINK_COMM_0:
		mavlink_comm_0_port->write(ch);
		break;
	case MAVLINK_COMM_1:
		mavlink_comm_1_port->write(ch);
		break;
	default:
		break;
	}
}

/// Read a byte from the nominated MAVLink channel
///
/// @param chan		Channel to receive on
/// @returns		Byte read
///
static inline uint8_t comm_receive_ch(mavlink_channel_t chan)
{
    uint8_t data = 0;

    switch(chan) {
	case MAVLINK_COMM_0:
		data = mavlink_comm_0_port->read();
		break;
	case MAVLINK_COMM_1:
		data = mavlink_comm_1_port->read();
		break;
	default:
		break;
	}
    return data;
}

/// Check for available data on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
static inline uint16_t comm_get_available(mavlink_channel_t chan)
{
    uint16_t bytes = 0;
    switch(chan) {
	case MAVLINK_COMM_0:
		bytes = mavlink_comm_0_port->available();
		break;
	case MAVLINK_COMM_1:
		bytes = mavlink_comm_1_port->available();
		break;
	default:
		break;
	}
    return bytes;
}


/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available, -1 for error
static inline int comm_get_txspace(mavlink_channel_t chan)
{
    switch(chan) {
	case MAVLINK_COMM_0:
		return mavlink_comm_0_port->txspace();
		break;
	case MAVLINK_COMM_1:
		return mavlink_comm_1_port->txspace();
		break;
	default:
		break;
	}
    return -1;
}

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#ifdef MAVLINK10
# include "include_v1.0/ardupilotmega/mavlink.h"
#else
# include "include/ardupilotmega/mavlink.h"
#endif

uint8_t mavlink_check_target(uint8_t sysid, uint8_t compid);

#endif // GCS_MAVLink_h
