
#include <stdint.h>

#pragma once

#define QURT_MSG_ID_MAVLINK_MSG 1
#define QURT_MSG_ID_REBOOT 2
#define QURT_MSG_ID_PRINTF 3
#define QURT_MSG_ID_UART_CONFIG 4
#define QURT_MSG_ID_UART_DATA 5

#define MAX_MAVLINK_INSTANCES 2

// Number of tunneled remote UART ports. Each port has a fixed mapping
// to an apps-processor Linux serial device; the encapsulating packet
// carries a port_id so multiple ports can be multiplexed over the RPC.
#define MAX_REMOTE_UART_INSTANCES 5

struct __attribute__((__packed__)) qurt_rpc_msg {
    uint8_t msg_id;
    uint8_t inst;
    uint16_t data_length;
    uint32_t seq;
    uint8_t data[300];
};

#define QURT_RPC_MSG_HEADER_LEN 8

// Payload for QURT_MSG_ID_UART_CONFIG. device_id maps directly to the
// Linux device path on the apps processor as "/dev/ttyHS<device_id>".
// port_id is the tunneled-port index (0..MAX_REMOTE_UART_INSTANCES-1)
// used to demultiplex data between multiple remote UARTs.
struct __attribute__((__packed__)) qurt_uart_config {
    uint32_t baudrate;
    uint32_t device_id;
    uint8_t  port_id;
    uint8_t  _reserved[3];
};
