
#include <stdint.h>

#pragma once

#define QURT_MSG_ID_MAVLINK_MSG 1
#define QURT_MSG_ID_REBOOT 2

#define MAX_MAVLINK_INSTANCES 2

struct __attribute__((__packed__)) qurt_rpc_msg {
    uint8_t msg_id;
    uint8_t inst;
    uint16_t data_length;
    uint32_t seq;
    uint8_t data[300];
};

#define QURT_RPC_MSG_HEADER_LEN 8

