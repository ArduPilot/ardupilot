
#include <stdint.h>

// #include <GCS_MAVLink/GCS_MAVLink.h>
#define MAVLINK_MAX_PAYLOAD_LEN 255

#pragma once

#define QURT_MSG_ID_MAVLINK_MSG 1

struct __attribute__((__packed__)) qurt_mavlink_msg {
    uint8_t msg_id{QURT_MSG_ID_MAVLINK_MSG};
    uint16_t data_length;
    uint32_t seq;
    uint8_t mav_msg[MAVLINK_MAX_PAYLOAD_LEN];
};

#define QURT_MAVLINK_MSG_HEADER_LEN 7
