#pragma once

#include <AP_Logger/LogStructure.h>
#include "UARTDriver.h"

#define LOG_IDS_FROM_HAL \
    LOG_UART_MSG

// @LoggerMessage: UART
// @Description: UART stats
// @Field: TimeUS: Time since system startup
// @Field: I: instance
// @Field: Tx: transmitted data rate bytes per second
// @Field: Rx: received data rate bytes per second
struct PACKED log_UART {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t instance;
    float tx_rate;
    float rx_rate;
};

#if !HAL_UART_STATS_ENABLED
#define LOG_STRUCTURE_FROM_HAL
#else
#define LOG_STRUCTURE_FROM_HAL                          \
    { LOG_UART_MSG, sizeof(log_UART),                   \
      "UART","QBff","TimeUS,I,Tx,Rx", "s#BB", "F---" },
#endif
